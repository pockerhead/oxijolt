//! Scene queries: ray casts, shape casts, collide-shape and point queries.
//!
//! Queries take `&PhysicsWorld` and may run on many threads at once while nobody steps the
//! world. They see bodies created, moved and removed through this API immediately, without a
//! step; [`PhysicsWorld::optimize_broad_phase`] only makes them faster after many bodies were
//! added one by one. Every query takes a [`QueryFilter`].
//!
//! Units are metres. Every reported normal is the outward surface normal of the obstacle (the
//! body that was hit) in world space: a floor below the query gives a normal pointing up, a
//! ceiling above it a normal pointing down.

use std::cell::RefCell;
use std::ffi::c_void;
use std::mem::MaybeUninit;
use std::ptr::{null, NonNull};

use oxijolt_sys::*;

use crate::body::with_read_locked_body;
use crate::filter::{with_query_filters, FilterState};
use crate::limits;
use crate::shape::compound_sub_shape_of;
use crate::{
    BodyError, BodyId, CompoundSubShape, ObjectLayer, PhysicsWorld, Quat, QueryError, QueryFilter,
    RVec3, Real, Shape, SubShapeId, Vec3,
};

mod cast;
mod collide;
mod point;

pub use cast::{ShapeCast, ShapeCastHit};
pub use collide::{CollideShape, CollideShapeHit};
pub use point::CollidePointHit;

/// A ray from `origin` along `direction` (Jolt `RRayCast`). The direction's length is the ray's
/// length; hits report the fraction along it, in `[0, 1]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayCast {
    origin: RVec3,
    direction: Vec3,
}

impl RayCast {
    /// A ray from `origin` (world space, metres) covering `direction` (metres). Both are checked
    /// by [`PhysicsWorld::cast_ray`].
    pub fn new(origin: RVec3, direction: Vec3) -> Self {
        Self { origin, direction }
    }

    /// Start of the ray in world space, metres.
    pub fn origin(&self) -> RVec3 {
        self.origin
    }

    /// Direction and length of the ray, metres.
    pub fn direction(&self) -> Vec3 {
        self.direction
    }

    /// The point `origin + direction * fraction`, computed as Jolt does
    /// (`RRayCast::GetPointOnRay`).
    pub fn point_at(&self, fraction: f32) -> RVec3 {
        RVec3::new(
            self.origin.x + Real::from(self.direction.x * fraction),
            self.origin.y + Real::from(self.direction.y * fraction),
            self.origin.z + Real::from(self.direction.z * fraction),
        )
    }
}

/// A hit of a [`PhysicsWorld::cast_ray`] or [`PhysicsWorld::cast_ray_all`].
///
/// Convex shapes are solid: a ray that starts inside one hits it at fraction `0.0` exactly.
/// Triangle back faces are hit, so a heightfield is hit from below too.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct RayCastHit {
    /// The body that was hit.
    pub body: BodyId,
    /// Fraction along the ray's direction where it hit, in `[0, 1]`.
    pub fraction: f32,
    /// Path from the body's shape to the leaf shape that was hit.
    pub sub_shape_id: SubShapeId,
    /// Distance from the origin to the hit in metres: `fraction` times the direction's length.
    pub distance: f32,
    /// Outward surface normal of the hit face in world space, unit length.
    ///
    /// For a triangle hit from its back (a heightfield from below) it is still the face's
    /// normal, so it points away from the side the ray came from: `normal . direction > 0`.
    /// For a ray that starts inside a convex shape (`fraction == 0.0`) it is the normal Jolt
    /// computes at the origin, the normal of a nearby face with no unique meaning.
    pub normal: Vec3,
    /// The object layer of the hit body.
    pub object_layer: ObjectLayer,
    /// The compound child that was hit, when the body's shape is a compound.
    pub compound_child: Option<CompoundSubShape>,
}

/// The distance a hit at `fraction` of `direction` lies from the start, in metres. The length
/// is taken in `f64`, so a direction whose `f32` length overflows still gives the finite distance
/// of a near hit.
fn hit_distance(fraction: f32, direction: Vec3) -> f32 {
    let [x, y, z] = [direction.x, direction.y, direction.z].map(f64::from);
    ((x * x + y * y + z * z).sqrt() * f64::from(fraction)) as f32
}

/// Where the result callback of a query collects every hit joltc reports, next to the query's
/// filter state.
pub(crate) struct HitsSlot<'s, T> {
    state: &'s FilterState<'s>,
    hits: RefCell<Vec<T>>,
}

impl<'s, T: Copy> HitsSlot<'s, T> {
    pub(crate) fn new(state: &'s FilterState<'s>) -> Self {
        Self {
            state,
            hits: RefCell::new(Vec::new()),
        }
    }

    pub(crate) fn as_user_data(&self) -> *mut c_void {
        (self as *const Self).cast_mut().cast()
    }

    /// Appends a copy of `*result`.
    ///
    /// # Safety
    /// `user_data` points to a live `HitsSlot<T>` and `result` to a live `T`.
    pub(crate) unsafe fn push(user_data: *mut c_void, result: *const T) {
        // SAFETY: guaranteed by the caller; only shared references to the slot exist.
        let slot = unsafe { &*user_data.cast::<Self>() };
        // SAFETY: guaranteed by the caller.
        let result = unsafe { *result };
        slot.state
            .guarded((), || slot.hits.borrow_mut().push(result));
    }

    pub(crate) fn into_hits(self) -> Vec<T> {
        self.hits.into_inner()
    }
}

/// Result callback of [`PhysicsWorld::cast_ray`] and [`PhysicsWorld::cast_ray_all`].
///
/// # Safety
/// Called only by joltc during a ray cast, with that call's live
/// `HitsSlot<JPH_RayCastResult>` as `user_data` and a live result.
unsafe extern "C" fn push_ray_hit(user_data: *mut c_void, result: *const JPH_RayCastResult) {
    // SAFETY: guaranteed by the caller (function contract).
    unsafe { HitsSlot::<JPH_RayCastResult>::push(user_data, result) }
}

impl PhysicsWorld {
    /// The closest body that `ray` hits among those `filter` selects, if any.
    ///
    /// Convex shapes are solid: a ray that starts inside one hits it at fraction `0.0` exactly.
    /// Triangle back faces are hit, so a heightfield is hit from below. Rays see a box's sharp
    /// faces whatever its convex radius. [`RayCastHit::normal`] is the outward normal of the hit
    /// face.
    ///
    /// When several hits share the closest fraction, the one returned follows Jolt's traversal
    /// and may depend on the broad phase's history; [`cast_ray_all`](Self::cast_ray_all) orders
    /// such hits by body id and sub-shape id.
    ///
    /// The origin must be finite with every component at most [`limits::MAX_POSITION`] in
    /// absolute value and the direction finite and not zero; otherwise
    /// [`QueryError::InvalidValue`] is returned.
    /// A filter that names an object layer this world does not have, or a body of another
    /// world, gives [`QueryError::UnknownObjectLayer`] or [`QueryError::WrongWorld`].
    pub fn cast_ray(
        &self,
        ray: &RayCast,
        filter: &QueryFilter<'_>,
    ) -> Result<Option<RayCastHit>, QueryError> {
        let hits = self.cast_ray_with(ray, filter, JPH_CollisionCollectorType_ClosestHit)?;
        Ok(hits.into_iter().next())
    }

    /// Every hit of `ray` on the bodies `filter` selects, sorted by fraction, then body id, then
    /// sub-shape id, so the order does not depend on the broad phase's history.
    ///
    /// A body is reported once per leaf shape the ray hits: a convex shape once (where the ray
    /// enters it, or at `0.0` when the ray starts inside it), a mesh or heightfield once per
    /// triangle crossed. Hits follow the rules of [`cast_ray`](Self::cast_ray), and the
    /// validation and errors are the same.
    ///
    /// The first hit has the fraction of `cast_ray`'s hit and is the same hit unless several
    /// share that fraction; `cast_ray` then returns one of them.
    pub fn cast_ray_all(
        &self,
        ray: &RayCast,
        filter: &QueryFilter<'_>,
    ) -> Result<Vec<RayCastHit>, QueryError> {
        self.cast_ray_with(ray, filter, JPH_CollisionCollectorType_AllHit)
    }

    /// The hits of `ray` that joltc's `collector` reports, sorted as
    /// [`cast_ray_all`](Self::cast_ray_all) promises.
    fn cast_ray_with(
        &self,
        ray: &RayCast,
        filter: &QueryFilter<'_>,
        collector: JPH_CollisionCollectorType,
    ) -> Result<Vec<RayCastHit>, QueryError> {
        if !limits::is_in_frame(ray.origin) {
            return Err(QueryError::InvalidValue(
                "ray origin must be finite and within limits::MAX_POSITION",
            ));
        }
        if !(ray.direction.is_finite() && ray.direction != Vec3::ZERO) {
            return Err(QueryError::InvalidValue(
                "ray direction must be finite and not zero",
            ));
        }
        filter.validate(self)?;
        let origin = ray.origin.to_jph();
        let direction = ray.direction.to_jph();
        // Solid convex shapes and triangle back faces, as Jolt's closest-hit `CastRay` without
        // settings does.
        let settings = JPH_RayCastSettings {
            backFaceModeTriangles: JPH_BackFaceMode_CollideWithBackFaces,
            backFaceModeConvex: JPH_BackFaceMode_IgnoreBackFaces,
            treatConvexAsSolid: true,
        };
        let mut hits = with_query_filters(self, filter, |raw, state| {
            let slot = HitsSlot::<JPH_RayCastResult>::new(state);
            // SAFETY: the query object lives inside this world's physics system. Jolt's locking
            // narrow-phase query takes body read locks and the broad-phase query lock, and
            // `step` needs `&mut self`, so no update runs meanwhile. `origin`, `direction`,
            // `settings` and `slot` are live locals, `push_ray_hit` matches the slot type and
            // copies the result, and the filters are live or null (Jolt's accept-all defaults).
            unsafe {
                JPH_NarrowPhaseQuery_CastRay3(
                    self.narrow_phase_query.as_ptr(),
                    &origin,
                    &direction,
                    &settings,
                    collector,
                    Some(push_ray_hit),
                    slot.as_user_data(),
                    null(),
                    raw.object_layer,
                    raw.body,
                    raw.shape,
                )
            };
            slot.into_hits()
        })?;
        hits.sort_unstable_by(|a, b| {
            a.fraction
                .total_cmp(&b.fraction)
                .then(a.bodyID.cmp(&b.bodyID))
                .then(a.subShapeID2.cmp(&b.subShapeID2))
        });
        hits.into_iter()
            .map(|hit| self.ray_cast_hit(ray, hit))
            .collect()
    }

    /// The [`RayCastHit`] for a hit joltc reported for `ray`, with the normal and object layer
    /// read from the hit body.
    fn ray_cast_hit(
        &self,
        ray: &RayCast,
        hit: JPH_RayCastResult,
    ) -> Result<RayCastHit, QueryError> {
        let body = BodyId::new(hit.bodyID, self.tag);
        let point = ray.point_at(hit.fraction).to_jph();
        let (normal, object_layer) =
            with_read_locked_body(self.body_lock_interface, body, |locked| {
                let mut normal = Vec3::ZERO.to_jph();
                // SAFETY: `locked` is read-locked for the closure; `point` and `normal` are live
                // locals, and the sub-shape id came from a hit on this body.
                unsafe {
                    JPH_Body_GetWorldSpaceSurfaceNormal(
                        locked.as_ptr(),
                        hit.subShapeID2,
                        &point,
                        &mut normal,
                    );
                }
                // SAFETY: as above.
                let layer = unsafe { JPH_Body_GetObjectLayer(locked.as_ptr()) };
                (Vec3::from_jph(normal), ObjectLayer::new(layer))
            })
            .ok_or(QueryError::AllocationFailed)?;
        let sub_shape_id = SubShapeId::new(hit.subShapeID2);
        Ok(RayCastHit {
            body,
            fraction: hit.fraction,
            sub_shape_id,
            distance: hit_distance(hit.fraction, ray.direction),
            normal,
            object_layer,
            compound_child: self.compound_sub_shape(body, sub_shape_id).ok().flatten(),
        })
    }

    /// The compound child of `body`'s shape that `id` leads to, as
    /// [`Shape::compound_sub_shape`](crate::Shape::compound_sub_shape) does for the body's
    /// shape: `Ok(None)` when the body's shape is not a compound or `id` names none of its
    /// children.
    pub fn compound_sub_shape(
        &self,
        body: BodyId,
        id: SubShapeId,
    ) -> Result<Option<CompoundSubShape>, BodyError> {
        self.check(body)?;
        // SAFETY: the body interface belongs to this live world. joltc returns the pointer
        // after releasing the temporary `RefConst<Shape>` it got from `BodyInterface::GetShape`,
        // so only the body's own shape reference keeps the shape alive. The body cannot be
        // removed and its shape cannot be replaced while `&self` is borrowed: `remove_body` and
        // `BodyMut::set_shape` need `&mut PhysicsWorld`.
        let shape =
            unsafe { JPH_BodyInterface_GetShape(self.body_interface.as_ptr(), body.to_raw()) };
        let Some(shape) = NonNull::new(shape.cast_mut()) else {
            return Ok(None);
        };
        // SAFETY: `shape` stays alive for the call, as argued above.
        Ok(unsafe { compound_sub_shape_of(shape, id) })
    }

    /// The object layer of a body that a query just reported.
    pub(crate) fn object_layer_of(&self, body: BodyId) -> ObjectLayer {
        // SAFETY: the body interface belongs to this live world; the getter locks the body and
        // accepts any id.
        ObjectLayer::new(unsafe {
            JPH_BodyInterface_GetObjectLayer(self.body_interface.as_ptr(), body.to_raw())
        })
    }
}

/// `base + offset`, for contact points Jolt reports relative to a base offset.
pub(crate) fn offset_from(base: RVec3, offset: JPH_Vec3) -> RVec3 {
    RVec3::new(
        base.x + Real::from(offset.x),
        base.y + Real::from(offset.y),
        base.z + Real::from(offset.z),
    )
}

/// Jolt's rotation-translation matrix for a pose (`RMat44::sRotationTranslation`).
pub(crate) fn rotation_translation(rotation: Quat, position: RVec3) -> JPH_RMat4 {
    let rotation = rotation.to_jph();
    let position = position.to_jph();
    let mut matrix = MaybeUninit::<JPH_RMat4>::uninit();
    // SAFETY: joltc writes the whole matrix; `rotation` and `position` are live locals.
    unsafe {
        JPH_RMat4_RotationTranslation(matrix.as_mut_ptr(), &rotation, &position);
        matrix.assume_init()
    }
}

/// Checks a query shape's pose; Jolt only asserts these.
pub(crate) fn validate_pose(
    shape: &Shape,
    position: RVec3,
    rotation: Quat,
) -> Result<(), QueryError> {
    if !limits::is_in_frame(position) {
        return Err(QueryError::InvalidValue(
            "query shape position must be finite and within limits::MAX_POSITION",
        ));
    }
    if !rotation.is_valid_rotation() {
        return Err(QueryError::InvalidValue(
            "query shape rotation must be a finite unit quaternion",
        ));
    }
    if shape.must_be_static() {
        return Err(QueryError::InvalidValue(
            "static-only shapes cannot be query shapes",
        ));
    }
    Ok(())
}
