//! Debug wireframe of colliders as line data: [`PhysicsWorld::debug_lines`].
//!
//! Jolt draws each shape through its `DebugRenderer`; joltc's `DebugRendererSimple` subclass
//! turns wireframe triangles into `DrawLine` calls, which oxijolt collects into a
//! [`DebugLines`] buffer. Nothing is drawn on screen.

use std::any::Any;
use std::cell::{Cell, RefCell};
use std::ffi::c_void;
use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::ptr::NonNull;
use std::sync::{Mutex, MutexGuard, OnceLock, PoisonError};

use oxijolt_sys::*;

use crate::body::with_read_locked_body;
use crate::limits;
use crate::owned::{JoltObject, Owned};
use crate::query::{offset_from, rotation_translation};
use crate::{BodyId, ObjectLayer, PhysicsWorld, Quat, QueryError, QueryFilter, RVec3, Real, Vec3};

/// Extra room around the query box of the broad phase, in metres, for the rounding of world
/// positions to `f32` within a bounded local frame (a few hundred metres around the origin).
const BROAD_PHASE_MARGIN: Real = 0.01;

/// Which colliders [`PhysicsWorld::debug_lines`] draws and how many lines it keeps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DebugLineSettings {
    center: RVec3,
    radius: f32,
    max_lines: usize,
}

impl DebugLineSettings {
    /// Draws the colliders whose world bounds lie within `radius` metres of `center`, without a
    /// line cap.
    pub fn new(center: RVec3, radius: f32) -> Self {
        Self {
            center,
            radius,
            max_lines: usize::MAX,
        }
    }

    /// Keeps at most `max_lines` lines; [`DebugLines::is_truncated`] reports when more existed.
    #[must_use]
    pub fn max_lines(self, max_lines: usize) -> Self {
        Self { max_lines, ..self }
    }

    /// Checks the center and the radius.
    fn validate(&self) -> Result<(), QueryError> {
        if !limits::is_in_frame(self.center) {
            return Err(QueryError::InvalidValue(
                "debug line center must be finite and within limits::MAX_POSITION",
            ));
        }
        let radius = Real::from(self.radius);
        if !(radius.is_finite() && (0.0..=2.0 * limits::MAX_POSITION).contains(&radius)) {
            return Err(QueryError::InvalidValue(
                "debug line radius must be between 0 and 2 * limits::MAX_POSITION",
            ));
        }
        Ok(())
    }
}

/// One line segment of a collider's wireframe, in world space.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct DebugLine {
    /// Start point.
    pub from: RVec3,
    /// End point.
    pub to: RVec3,
    /// The body whose collider the line belongs to.
    pub body: BodyId,
    /// The [`CompoundChild::user_data`](crate::CompoundChild::user_data) (the group) of the
    /// top-level compound child the line belongs to; `None` when the body's shape is not a
    /// compound.
    pub child_user_data: Option<u32>,
}

/// A buffer of debug lines, returned by [`PhysicsWorld::debug_lines`] or refilled by
/// [`PhysicsWorld::debug_lines_into`].
///
/// Each `debug_lines_into` call clears the buffer and refills it, keeping its capacity, so drawing
/// every frame allocates only when the line count grows. [`release`](Self::release) or dropping the
/// buffer frees the line storage. Jolt's own per-shape debug geometry (built for heightfields and
/// meshes on their first draw) stays with the shape and is not affected.
#[derive(Debug, Default)]
pub struct DebugLines {
    lines: Vec<DebugLine>,
    truncated: bool,
}

impl DebugLines {
    /// An empty buffer that holds no memory yet.
    pub fn new() -> Self {
        Self::default()
    }

    /// The lines of the last call.
    pub fn lines(&self) -> &[DebugLine] {
        &self.lines
    }

    /// Whether the last call had more lines than its cap and dropped the rest.
    pub fn is_truncated(&self) -> bool {
        self.truncated
    }

    /// Frees the line storage and clears the truncated flag.
    pub fn release(&mut self) {
        self.lines = Vec::new();
        self.truncated = false;
    }
}

impl PhysicsWorld {
    /// The wireframe lines of the colliders near `settings`' center, in a new buffer; see
    /// [`debug_lines_into`](Self::debug_lines_into), which reuses one.
    ///
    /// # Errors
    /// As [`debug_lines_into`](Self::debug_lines_into).
    pub fn debug_lines(
        &self,
        settings: &DebugLineSettings,
        filter: &QueryFilter<'_>,
    ) -> Result<DebugLines, QueryError> {
        let mut lines = DebugLines::new();
        self.debug_lines_into(settings, filter, &mut lines)?;
        Ok(lines)
    }

    /// Collects the wireframe lines of the colliders near `settings`' center into `out`,
    /// replacing what it held.
    ///
    /// A collider is drawn when its world bounding box lies within the radius of the center
    /// (touches the sphere): a whole body, a heightfield as a whole, and each top-level child
    /// of a compound on its own. `filter` selects bodies by object layer, skips its excluded
    /// body and the bodies its body filter rejects, and its child groups select top-level
    /// compound children by user data; each line carries its body and that child user data.
    ///
    /// Lines come in body-id order, then child order, then Jolt's triangle order: three lines
    /// per triangle, so edges shared by triangles repeat. Coordinates are world space, exact to
    /// `f32` rounding of positions within a bounded local frame (a few hundred metres around
    /// the origin). With a line cap, `out` holds exactly the first `max_lines` lines and
    /// [`DebugLines::is_truncated`] is true exactly when more existed; Jolt still finishes the
    /// shape it was drawing, and the lines past the cap are dropped.
    ///
    /// Calls are serialized process-wide, because Jolt's debug renderer is a process
    /// singleton. Jolt keeps a heightfield's or mesh's debug geometry in the shape after the
    /// first draw, until the shape is freed.
    ///
    /// # Cost
    /// There is no level of detail by camera distance: every shape is drawn with Jolt's finest
    /// geometry, so one capsule gives 6528 lines and one cylinder 768, whatever their size and
    /// distance. Each call also builds a new Jolt debug renderer with its unit geometries. Use
    /// the radius and [`DebugLineSettings::max_lines`] to bound the output. Only bodies are
    /// drawn: a character (`CharacterVirtual`) appears only through its inner body, if it has
    /// one.
    ///
    /// # Errors
    /// [`QueryError::InvalidValue`] when the center is not finite or not within
    /// [`limits::MAX_POSITION`](crate::limits::MAX_POSITION), the radius is negative, not finite
    /// or above twice that bound. [`QueryError::UnknownObjectLayer`] or [`QueryError::WrongWorld`]
    /// when `filter` names an object layer this world does not have or a body of another world.
    pub fn debug_lines_into(
        &self,
        settings: &DebugLineSettings,
        filter: &QueryFilter<'_>,
        out: &mut DebugLines,
    ) -> Result<(), QueryError> {
        settings.validate()?;
        filter.validate(self)?;
        out.lines.clear();
        out.truncated = false;

        let bodies = self.debug_bodies(settings, filter);
        // Declared in this order so that the renderer is destroyed before the state it points
        // to, and both before the lock is released, also while unwinding.
        let guard = lock_draw();
        install_procs();
        let state = DrawState::new(&mut out.lines, settings.max_lines);
        {
            let renderer = create_renderer(&state)?;
            for body in &bodies {
                if state.truncated.get() {
                    break;
                }
                self.draw_body(&renderer, &state, body, settings, filter);
            }
        }
        let truncated = state.truncated.get();
        let panic = state.panic.take();
        drop(state);
        drop(guard);
        if let Some(payload) = panic {
            resume_unwind(payload);
        }
        out.truncated = truncated;
        Ok(())
    }

    /// The bodies `filter` keeps whose world bounds lie within the radius, in body-id order.
    fn debug_bodies(
        &self,
        settings: &DebugLineSettings,
        filter: &QueryFilter<'_>,
    ) -> Vec<DebugBody> {
        let reach = Real::from(settings.radius) + BROAD_PHASE_MARGIN;
        let center = settings.center;
        // `Real` is `f32` without the `double-precision` feature, so the cast is a no-op there.
        #[allow(clippy::unnecessary_cast)]
        let corner = |sign: Real| JPH_Vec3 {
            x: (center.x + sign * reach) as f32,
            y: (center.y + sign * reach) as f32,
            z: (center.z + sign * reach) as f32,
        };
        let query = JPH_AABox {
            min: corner(-1.0),
            max: corner(1.0),
        };
        let radius_sq = Real::from(settings.radius) * Real::from(settings.radius);

        let mut bodies = Vec::new();
        for raw in self.broad_phase_bodies(&query) {
            let id = BodyId::new(raw, self.tag);
            if !filter.keeps_body(id) {
                continue;
            }
            let read = with_read_locked_body(self.body_lock_interface, id, |body| {
                let body = body.as_ptr();
                let mut bounds = JPH_AABox {
                    min: Vec3::ZERO.to_jph(),
                    max: Vec3::ZERO.to_jph(),
                };
                let mut position = RVec3::ZERO.to_jph();
                let mut rotation = Quat::IDENTITY.to_jph();
                // SAFETY: `body` is locked for reading for the duration of the closure; the
                // getters only read it and write the live locals.
                let (layer, shape) = unsafe {
                    JPH_Body_GetWorldSpaceBounds(body, &mut bounds);
                    JPH_Body_GetCenterOfMassPosition(body, &mut position);
                    JPH_Body_GetRotation(body, &mut rotation);
                    (JPH_Body_GetObjectLayer(body), JPH_Body_GetShape(body))
                };
                (
                    ObjectLayer::new(layer),
                    bounds,
                    RVec3::from_jph(position),
                    Quat::from_jph(rotation),
                    NonNull::new(shape.cast_mut()),
                )
            });
            let Some((layer, bounds, position, rotation, Some(shape))) = read else {
                continue;
            };
            if filter.keeps_object_layer(layer) && bounds_distance_sq(&bounds, center) <= radius_sq
            {
                bodies.push(DebugBody {
                    id,
                    shape,
                    position,
                    rotation,
                });
            }
        }
        bodies
    }

    /// Draws one body: each compound child that the filter keeps and that lies within the
    /// radius, or the whole shape of any other body.
    fn draw_body(
        &self,
        renderer: &Owned<JPH_DebugRenderer>,
        state: &DrawState<'_>,
        body: &DebugBody,
        settings: &DebugLineSettings,
        filter: &QueryFilter<'_>,
    ) {
        let root = body.shape.as_ptr();
        // SAFETY: the body holds a reference to its shape, and `&PhysicsWorld` forbids removing
        // the body or replacing its shape until a `&mut` call; the getter only reads it.
        let sub_type = unsafe { JPH_Shape_GetSubType(root) };
        if sub_type != JPH_ShapeSubType_StaticCompound
            && sub_type != JPH_ShapeSubType_MutableCompound
        {
            state.current.set(Some((body.id, None)));
            draw_shape(renderer, root, body.rotation, body.position);
            return;
        }

        let compound: *const JPH_CompoundShape = root.cast();
        let radius_sq = Real::from(settings.radius) * Real::from(settings.radius);
        // SAFETY: as above, and the shape is a compound (checked above).
        let count = unsafe { JPH_CompoundShape_GetNumSubShapes(compound) };
        for index in 0..count {
            if state.truncated.get() {
                return;
            }
            let mut child: *const JPH_Shape = std::ptr::null();
            let mut position_com = Vec3::ZERO.to_jph();
            let mut rotation = Quat::IDENTITY.to_jph();
            let mut user_data = 0;
            // SAFETY: as above; `index < count`, and the outputs are live locals.
            unsafe {
                JPH_CompoundShape_GetSubShape(
                    compound,
                    index,
                    &mut child,
                    &mut position_com,
                    &mut rotation,
                    &mut user_data,
                );
            }
            if child.is_null() || !filter.keeps_group(user_data) {
                continue;
            }
            // Jolt `CompoundShape::Draw` with unit scale: the body's centre-of-mass transform
            // times the child's local transform.
            let rotation = body.rotation.product(Quat::from_jph(rotation));
            let offset = body.rotation.rotate(Vec3::from_jph(position_com));
            let position = offset_from(body.position, offset.to_jph());
            if bounds_distance_sq(&shape_bounds(child, rotation, position), settings.center)
                > radius_sq
            {
                continue;
            }
            state.current.set(Some((body.id, Some(user_data))));
            draw_shape(renderer, child, rotation, position);
        }
    }
}

/// A body [`PhysicsWorld::debug_lines`] draws, read under its lock.
struct DebugBody {
    id: BodyId,
    /// The body's root shape; the body keeps it alive while the world is borrowed.
    shape: NonNull<JPH_Shape>,
    /// Centre-of-mass position.
    position: RVec3,
    rotation: Quat,
}

/// The world bounds of `shape` with its centre of mass at `position`.
fn shape_bounds(shape: *const JPH_Shape, rotation: Quat, position: RVec3) -> JPH_AABox {
    let mut transform = rotation_translation(rotation, position);
    let mut scale = Vec3::new(1.0, 1.0, 1.0).to_jph();
    let mut bounds = JPH_AABox {
        min: Vec3::ZERO.to_jph(),
        max: Vec3::ZERO.to_jph(),
    };
    // SAFETY: `shape` is a child of a compound that a body of the borrowed world holds, so it is
    // alive; joltc only reads the shape and the matrix and scale, and writes `bounds`.
    unsafe { JPH_Shape_GetWorldSpaceBounds(shape, &mut transform, &mut scale, &mut bounds) };
    bounds
}

/// Draws `shape` in wireframe, unit scale, with its centre of mass at `position`.
fn draw_shape(
    renderer: &Owned<JPH_DebugRenderer>,
    shape: *const JPH_Shape,
    rotation: Quat,
    position: RVec3,
) {
    let transform = rotation_translation(rotation, position);
    let scale = Vec3::new(1.0, 1.0, 1.0).to_jph();
    // SAFETY: the root shape stays alive because its body holds a reference and
    // `&PhysicsWorld` forbids removing the body or replacing its shape until a `&mut` call;
    // child shapes are held by their compound. No body lock is held while Jolt draws. `DRAW`
    // is locked, so this is the only live `DebugRenderer` that oxijolt creates and the only
    // thread that oxijolt lets write a shape's debug geometry cache; the raw-API contract
    // of `oxijolt-sys` excludes other renderers. The renderer's `DrawLine` callback runs
    // synchronously on this thread with the live `DrawState` it was created with.
    unsafe {
        JPH_Shape_Draw(
            shape,
            renderer.as_ptr(),
            &transform,
            &scale,
            0xFFFF_FFFF,
            false,
            true,
        );
    }
}

/// Squared distance from `center` to the closest point of `bounds`; zero inside.
fn bounds_distance_sq(bounds: &JPH_AABox, center: RVec3) -> Real {
    let axis = |min: f32, max: f32, value: Real| {
        let closest = value.clamp(Real::from(min), Real::from(max));
        (value - closest) * (value - closest)
    };
    axis(bounds.min.x, bounds.max.x, center.x)
        + axis(bounds.min.y, bounds.max.y, center.y)
        + axis(bounds.min.z, bounds.max.z, center.z)
}

/// A debug renderer created by [`create_renderer`]. joltc's destroy call `delete`s the
/// `ManagedDebugRendererSimple`, which the owner owns entirely (it is not ref-counted); Jolt's
/// destructor clears its `DebugRenderer` singleton.
impl JoltObject for JPH_DebugRenderer {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the renderer (trait contract); no draw call uses it any more.
        unsafe { JPH_DebugRenderer_Destroy(ptr) };
    }
}

/// Wireframe drawing of rigid shapes only produces lines. Soft body shapes draw their faces with
/// `DrawTriangle`, which joltc turns into three `DrawLine` calls when no triangle callback is set
/// (`DebugRendererSimple::DrawTriangle`, `joltc.cpp:9392-9406`).
static DEBUG_RENDERER_PROCS: JPH_DebugRenderer_Procs = JPH_DebugRenderer_Procs {
    DrawLine: Some(draw_line),
    DrawTriangle: None,
    DrawText3D: None,
};

/// Points joltc's global debug renderer proc table at oxijolt' callback, once per process.
fn install_procs() {
    static INSTALLED: OnceLock<()> = OnceLock::new();
    INSTALLED.get_or_init(|| {
        // SAFETY: the table is an immutable static, so the pointer stays valid forever.
        // `SetProcs` only stores the pointer in a plain global; `OnceLock` makes that one write
        // happen before every read by a thread that returns from `get_or_init`, and every
        // renderer is created after this call on the creating thread. oxijolt never calls
        // `SetProcs` again, and the raw-API contract of `oxijolt-sys` forbids other code to.
        unsafe { JPH_DebugRenderer_SetProcs(&DEBUG_RENDERER_PROCS) };
    });
}

/// Serializes debug drawing process-wide: Jolt's `DebugRenderer` is a singleton (its
/// constructor asserts that no other instance is alive), and heightfield and mesh shapes fill
/// an unsynchronized debug geometry cache on their first draw. Shapes may be shared between
/// worlds, so one lock per world would not suffice.
fn lock_draw() -> MutexGuard<'static, ()> {
    static DRAW: Mutex<()> = Mutex::new(());
    DRAW.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Creates the renderer of one [`PhysicsWorld::debug_lines`] call, whose callback writes into
/// `state`.
fn create_renderer(state: &DrawState<'_>) -> Result<Owned<JPH_DebugRenderer>, QueryError> {
    let user_data = (state as *const DrawState<'_>).cast_mut().cast::<c_void>();
    // SAFETY: joltc `new`s a renderer that only stores `user_data` and that the handle owns
    // entirely; the caller keeps `state` alive until the handle is dropped.
    unsafe { Owned::from_raw(JPH_DebugRenderer_Create(user_data)) }
        .ok_or(QueryError::AllocationFailed)
}

/// What the `DrawLine` callback of one [`PhysicsWorld::debug_lines`] call needs, on the stack of
/// that call. joltc calls the callback synchronously on the calling thread inside
/// `JPH_Shape_Draw`; no Jolt job is involved, so `Cell`s suffice.
struct DrawState<'a> {
    lines: RefCell<&'a mut Vec<DebugLine>>,
    max_lines: usize,
    /// Set when a line past `max_lines` was dropped.
    truncated: Cell<bool>,
    /// The body and compound child user data the lines being drawn belong to.
    current: Cell<Option<(BodyId, Option<u32>)>>,
    /// The first panic of the callback, re-raised once the renderer is gone.
    panic: Cell<Option<Box<dyn Any + Send>>>,
}

impl<'a> DrawState<'a> {
    fn new(lines: &'a mut Vec<DebugLine>, max_lines: usize) -> Self {
        Self {
            lines: RefCell::new(lines),
            max_lines,
            truncated: Cell::new(false),
            current: Cell::new(None),
            panic: Cell::new(None),
        }
    }

    fn has_panicked(&self) -> bool {
        let payload = self.panic.take();
        let panicked = payload.is_some();
        self.panic.set(payload);
        panicked
    }

    /// Appends one line, or sets the truncated flag when the cap is reached.
    fn push(&self, from: RVec3, to: RVec3) {
        if self.truncated.get() {
            return;
        }
        let Some((body, child_user_data)) = self.current.get() else {
            return;
        };
        let mut lines = self.lines.borrow_mut();
        if lines.len() < self.max_lines {
            lines.push(DebugLine {
                from,
                to,
                body,
                child_user_data,
            });
        } else {
            self.truncated.set(true);
        }
    }
}

/// `DrawLine` callback of the debug renderer: records the line in the call's [`DrawState`] and
/// never lets a panic unwind into joltc.
///
/// # Safety
/// Called only by joltc during a `JPH_Shape_Draw` of [`PhysicsWorld::debug_lines`], with that
/// call's live `DrawState` as `user_data` and valid `from` and `to`.
unsafe extern "C" fn draw_line(
    user_data: *mut c_void,
    from: *const JPH_RVec3,
    to: *const JPH_RVec3,
    _color: JPH_Color,
) {
    // SAFETY: guaranteed by the caller (function contract); only shared references to the
    // state exist while Jolt draws.
    let (state, from, to) = unsafe {
        (
            &*user_data.cast::<DrawState<'_>>(),
            RVec3::from_jph(*from),
            RVec3::from_jph(*to),
        )
    };
    if state.has_panicked() {
        return;
    }
    let pushed = catch_unwind(AssertUnwindSafe(|| {
        #[cfg(test)]
        tests::panic_if_injected();
        state.push(from, to);
    }));
    if let Err(payload) = pushed {
        state.panic.set(Some(payload));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodySettings, CompoundChild, Shape, WorldSettings};

    thread_local! {
        /// Whether `draw_line` panics on this thread. Jolt calls it on the drawing thread, so
        /// tests running in parallel do not see each other's choice.
        static INJECTED_PANIC: Cell<bool> = const { Cell::new(false) };
    }

    /// Panics when the running test asked for it.
    pub(super) fn panic_if_injected() {
        if INJECTED_PANIC.get() {
            panic!("injected draw_line panic");
        }
    }

    fn unit_box() -> JPH_AABox {
        JPH_AABox {
            min: Vec3::new(-1.0, -1.0, -1.0).to_jph(),
            max: Vec3::new(1.0, 1.0, 1.0).to_jph(),
        }
    }

    #[test]
    fn bounds_distance_is_zero_inside_and_on_faces() {
        assert_eq!(bounds_distance_sq(&unit_box(), RVec3::ZERO), 0.0);
        assert_eq!(
            bounds_distance_sq(&unit_box(), RVec3::new(1.0, 0.5, -1.0)),
            0.0
        );
    }

    #[test]
    fn bounds_distance_outside() {
        assert_eq!(
            bounds_distance_sq(&unit_box(), RVec3::new(4.0, 0.0, 0.0)),
            9.0
        );
        assert_eq!(
            bounds_distance_sq(&unit_box(), RVec3::new(-3.0, 3.0, 0.5)),
            8.0
        );
        assert_eq!(
            bounds_distance_sq(&unit_box(), RVec3::new(2.0, -3.0, 4.0)),
            1.0 + 4.0 + 9.0
        );
    }

    #[test]
    fn release_frees_the_line_storage() {
        let mut lines = DebugLines::new();
        assert_eq!(lines.lines.capacity(), 0);
        let world = PhysicsWorld::new(WorldSettings::default()).unwrap();
        let line = DebugLine {
            from: RVec3::ZERO,
            to: RVec3::ZERO,
            body: BodyId::new(0, world.tag),
            child_user_data: None,
        };
        lines.lines.extend([line; 100]);
        lines.truncated = true;
        lines.release();
        assert_eq!(lines.lines.capacity(), 0);
        assert!(lines.lines().is_empty());
        assert!(!lines.is_truncated());
    }

    #[test]
    fn settings_validation() {
        let valid = |settings: DebugLineSettings| settings.validate();
        assert_eq!(valid(DebugLineSettings::new(RVec3::ZERO, 0.0)), Ok(()));
        assert_eq!(valid(DebugLineSettings::new(RVec3::ZERO, 5.0)), Ok(()));
        for radius in [f32::NAN, -1.0, f32::INFINITY] {
            assert!(
                matches!(
                    valid(DebugLineSettings::new(RVec3::ZERO, radius)),
                    Err(QueryError::InvalidValue(_))
                ),
                "{radius}"
            );
        }
        for center in [
            RVec3::new(Real::NAN, 0.0, 0.0),
            RVec3::new(0.0, Real::INFINITY, 0.0),
        ] {
            assert!(matches!(
                valid(DebugLineSettings::new(center, 1.0)),
                Err(QueryError::InvalidValue(_))
            ));
        }
    }

    #[test]
    fn a_panic_in_draw_line_resumes_after_the_renderer_is_gone() {
        let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
        let unit = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
        let compound = Shape::new_compound(&[
            CompoundChild {
                shape: &unit,
                position: Vec3::ZERO,
                rotation: Quat::IDENTITY,
                user_data: 1,
            },
            CompoundChild {
                shape: &unit,
                position: Vec3::new(2.0, 0.0, 0.0),
                rotation: Quat::IDENTITY,
                user_data: 2,
            },
        ])
        .unwrap();
        world
            .create_body(&compound, &BodySettings::new_static())
            .unwrap();
        world
            .create_body(
                &unit,
                &BodySettings::new_static().position(RVec3::new(0.0, 3.0, 0.0)),
            )
            .unwrap();
        let settings = DebugLineSettings::new(RVec3::ZERO, 10.0);
        let filter = QueryFilter::new();

        let mut clean = DebugLines::new();
        world
            .debug_lines_into(&settings, &filter, &mut clean)
            .unwrap();
        assert_eq!(clean.lines().len(), 3 * 36);

        let mut lines = DebugLines::new();
        INJECTED_PANIC.set(true);
        let result = catch_unwind(AssertUnwindSafe(|| {
            world.debug_lines_into(&settings, &filter, &mut lines)
        }));
        INJECTED_PANIC.set(false);
        let payload = result.expect_err("joltc returned and the panic resumed");
        assert_eq!(
            payload.downcast_ref::<&str>(),
            Some(&"injected draw_line panic")
        );

        // A second live renderer would trip Jolt's singleton, and a held lock would deadlock.
        world
            .debug_lines_into(&settings, &filter, &mut lines)
            .unwrap();
        assert_eq!(lines.lines(), clean.lines());
        assert!(!lines.is_truncated());
    }
}
