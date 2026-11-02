//! Query filters: which bodies and compound children a scene query considers.
//!
//! joltc's object-layer, body and shape filters call one process-global proc table per filter
//! type (`JPH_*Filter_SetProcs`). oxijolt installs its tables once and never changes them;
//! everything that differs between queries travels in the filters' `userData`, which points to
//! a [`FilterState`] on the stack of the running query. joltc runs a narrow-phase query and
//! every callback synchronously on the calling thread (no job system is involved), so that
//! state lives exactly as long as the callbacks can see it.

use std::any::Any;
use std::cell::Cell;
use std::ffi::c_void;
use std::fmt;
use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::ptr::{null, NonNull};
use std::sync::OnceLock;

use oxijolt_sys::*;

use crate::owned::{JoltObject, Owned};
use crate::shape::compound_sub_shape_of;
use crate::{BodyId, ObjectLayer, PhysicsWorld, QueryError, SubShapeId};

/// Selects the bodies and compound children a scene query considers. The default accepts
/// everything.
///
/// The parts combine: a body or child is considered only when every part that is set accepts
/// it.
///
/// - [`object_layers`](Self::object_layers) selects whole bodies by their
///   [`ObjectLayer`], compound or not.
/// - [`child_groups`](Self::child_groups) selects children of compound bodies by the
///   [`CompoundChild::user_data`](crate::CompoundChild::user_data) they were created with.
/// - [`exclude_body`](Self::exclude_body) skips one body, for example the querying actor's own.
/// - [`body_filter`](Self::body_filter) asks a closure about each body.
///
/// For collision groups that are per body, give each group its own object layer and select
/// layers; for groups that share one body (the children of a chunk compound), store the group
/// as child user data and select it with a group mask. oxijolt stores no mapping from layers
/// to groups: the caller knows which layer holds which group. These are not the bodies'
/// [`CollisionGroup`](crate::CollisionGroup)s, which filter what the solver collides and which
/// queries ignore.
#[derive(Clone, Copy, Default)]
pub struct QueryFilter<'a> {
    object_layers: Option<&'a [ObjectLayer]>,
    child_groups: Option<u32>,
    excluded_body: Option<BodyId>,
    body_filter: Option<&'a (dyn Fn(BodyId) -> bool + Sync)>,
}

impl fmt::Debug for QueryFilter<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("QueryFilter")
            .field("object_layers", &self.object_layers)
            .field("child_groups", &self.child_groups)
            .field("excluded_body", &self.excluded_body)
            .field("body_filter", &self.body_filter.is_some())
            .finish()
    }
}

impl<'a> QueryFilter<'a> {
    /// A filter that accepts every body and every compound child.
    pub const fn new() -> Self {
        Self {
            object_layers: None,
            child_groups: None,
            excluded_body: None,
            body_filter: None,
        }
    }

    /// Considers only bodies whose object layer is in `layers`. An empty slice selects
    /// nothing. Every layer must exist in the queried world.
    #[must_use]
    pub fn object_layers(mut self, layers: &'a [ObjectLayer]) -> Self {
        self.object_layers = Some(layers);
        self
    }

    /// Considers only those children of compound bodies whose group is in `mask`: a child is
    /// kept when its `user_data` is below 32 and bit `user_data` of `mask` is set, so
    /// `1 << 2 | 1 << 3` selects the children created with user data 2 or 3.
    ///
    /// The group of a child is the user data of the top-level child of the body's compound; a
    /// nested compound's own children inherit it. Bodies whose shape is not a compound are not
    /// affected; select them with [`object_layers`](Self::object_layers).
    ///
    /// For example, a ground query that must see terrain and structures but not the features of
    /// a chunk compound selects the terrain and chunk layers and `1 << STRUCTURE`, where
    /// `STRUCTURE` is the user data the chunk's structure children carry.
    #[must_use]
    pub fn child_groups(mut self, mask: u32) -> Self {
        self.child_groups = Some(mask);
        self
    }

    /// Skips the body `body`. The id must belong to the queried world; the id of a body that was
    /// removed is allowed and excludes nothing.
    #[must_use]
    pub fn exclude_body(mut self, body: BodyId) -> Self {
        self.excluded_body = Some(body);
        self
    }

    /// Considers only bodies for which `keep` returns `true`; it is combined with
    /// [`exclude_body`](Self::exclude_body), which is checked first.
    ///
    /// The query asks `keep` about every candidate body before it tests the body's shape, so a
    /// rejected body does not hide the bodies behind it. `keep` runs on the calling thread
    /// during the query, may be called more than once for one body and in no particular order,
    /// and receives ids of the queried world. It cannot change the world, which the query
    /// borrows, and must not run queries on it: Jolt would take the broad phase's shared lock
    /// twice on one thread. It should decide from data of its own, and for deterministic results
    /// its answer should depend only on the id. It is `Sync` so that one filter can serve
    /// queries on many threads.
    ///
    /// A panic in `keep` reaches the caller of the query. While Jolt runs the query, the query
    /// rejects everything after the panic and resumes it once Jolt has returned, as for a panic
    /// in any other filter callback.
    ///
    /// ```
    /// # use oxijolt::{BodyId, PhysicsWorld, QueryFilter, RVec3, RayCast, Vec3, WorldSettings};
    /// # let world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    /// let ignored: Vec<BodyId> = Vec::new();
    /// let keep = |body: BodyId| !ignored.contains(&body);
    /// let ray = RayCast::new(RVec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -20.0, 0.0));
    /// let hit = world.cast_ray(&ray, &QueryFilter::new().body_filter(&keep))?;
    /// # assert!(hit.is_none());
    /// # Ok::<(), oxijolt::QueryError>(())
    /// ```
    #[must_use]
    pub fn body_filter(mut self, keep: &'a (dyn Fn(BodyId) -> bool + Sync)) -> Self {
        self.body_filter = Some(keep);
        self
    }

    /// Checks the parts that refer to `world`.
    pub(crate) fn validate(&self, world: &PhysicsWorld) -> Result<(), QueryError> {
        let layers = self.object_layers.unwrap_or_default();
        if let Some(&layer) = layers
            .iter()
            .find(|layer| layer.get() >= world.object_layer_count)
        {
            return Err(QueryError::UnknownObjectLayer(layer));
        }
        match self.excluded_body {
            Some(body) if body.world != world.tag => Err(QueryError::WrongWorld(body)),
            _ => Ok(()),
        }
    }

    /// Whether a compound child with `user_data` is in the group mask, if one is set.
    pub(crate) fn keeps_group(&self, user_data: u32) -> bool {
        self.child_groups
            .is_none_or(|mask| user_data < 32 && mask & (1 << user_data) != 0)
    }

    /// Whether a body in `layer` is in the selected object layers, if any are set.
    pub(crate) fn keeps_object_layer(&self, layer: ObjectLayer) -> bool {
        self.object_layers
            .is_none_or(|layers| layers.contains(&layer))
    }

    /// Whether `body` is not the excluded body and the body filter, if any, keeps it.
    pub(crate) fn keeps_body(&self, body: BodyId) -> bool {
        self.excluded_body
            .is_none_or(|excluded| excluded.to_raw() != body.to_raw())
            && self.body_filter.is_none_or(|keep| keep(body))
    }
}

/// An object-layer filter created by [`with_query_filters`]. joltc's destroy call `delete`s the
/// whole object, which the owner owns entirely.
impl JoltObject for JPH_ObjectLayerFilter {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the filter (trait contract); no query uses it any more.
        unsafe { JPH_ObjectLayerFilter_Destroy(ptr) };
    }
}

/// A body filter created by [`with_query_filters`]. joltc's destroy call `delete`s the whole
/// object, which the owner owns entirely.
impl JoltObject for JPH_BodyFilter {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the filter (trait contract); no query uses it any more.
        unsafe { JPH_BodyFilter_Destroy(ptr) };
    }
}

/// A shape filter created by [`with_query_filters`]. joltc's destroy call `delete`s the whole
/// object, which the owner owns entirely.
impl JoltObject for JPH_ShapeFilter {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the filter (trait contract); no query uses it any more.
        unsafe { JPH_ShapeFilter_Destroy(ptr) };
    }
}

static OBJECT_LAYER_PROCS: JPH_ObjectLayerFilter_Procs = JPH_ObjectLayerFilter_Procs {
    ShouldCollide: Some(object_layer_should_collide),
};

// `ShouldCollideLocked` stays unset, so joltc accepts the body: Jolt calls it while it holds the
// body's read lock (`NarrowPhaseQuery.cpp`), where a callback must not lock anything.
static BODY_PROCS: JPH_BodyFilter_Procs = JPH_BodyFilter_Procs {
    ShouldCollide: Some(body_should_collide),
    ShouldCollideLocked: None,
};

static SHAPE_PROCS: JPH_ShapeFilter_Procs = JPH_ShapeFilter_Procs {
    ShouldCollide: Some(shape_should_collide),
    ShouldCollide2: Some(shape_should_collide2),
};

/// Points joltc's three global filter proc tables at oxijolt' callbacks, once per process.
fn install_procs() {
    static INSTALLED: OnceLock<()> = OnceLock::new();
    INSTALLED.get_or_init(|| {
        // SAFETY: the tables are immutable statics, so the pointers stay valid forever.
        // `SetProcs` only stores the pointer in a plain global; `OnceLock` makes that one write
        // happen before every read by a thread that returns from `get_or_init`, and every filter
        // is created after this call on the creating thread. oxijolt never calls `SetProcs`
        // again, and the raw-API contract of `oxijolt-sys` forbids other code to.
        unsafe {
            JPH_ObjectLayerFilter_SetProcs(&OBJECT_LAYER_PROCS);
            JPH_BodyFilter_SetProcs(&BODY_PROCS);
            JPH_ShapeFilter_SetProcs(&SHAPE_PROCS);
        }
    });
}

/// What the filter callbacks of one query need, on the stack of that query for the whole
/// synchronous joltc call.
///
/// `Cell`s suffice: joltc runs the Jolt query and every callback on the calling thread, and the
/// type is not `Sync`.
pub(crate) struct FilterState<'q> {
    world: &'q PhysicsWorld,
    filter: &'q QueryFilter<'q>,
    /// The shape filter of this query, or null; it knows the body whose shape is being tested.
    shape_filter: Cell<*mut JPH_ShapeFilter>,
    /// The last body the shape filter asked about and its root shape.
    root_cache: Cell<Option<(JPH_BodyID, NonNull<JPH_Shape>)>>,
    /// The first panic of a callback, re-raised once joltc has returned.
    panic: Cell<Option<Box<dyn Any + Send>>>,
}

impl<'q> FilterState<'q> {
    fn new(world: &'q PhysicsWorld, filter: &'q QueryFilter<'q>) -> Self {
        Self {
            world,
            filter,
            shape_filter: Cell::new(std::ptr::null_mut()),
            root_cache: Cell::new(None),
            panic: Cell::new(None),
        }
    }

    fn has_panicked(&self) -> bool {
        let payload = self.panic.take();
        let panicked = payload.is_some();
        self.panic.set(payload);
        panicked
    }

    /// Runs `f` unless a callback of this query has panicked before, and returns `reject`
    /// instead of unwinding into joltc when `f` panics. The first panic is kept for
    /// [`with_query_filters`] to re-raise.
    pub(crate) fn guarded<R>(&self, reject: R, f: impl FnOnce() -> R) -> R {
        if self.has_panicked() {
            return reject;
        }
        match catch_unwind(AssertUnwindSafe(f)) {
            Ok(value) => value,
            Err(payload) => {
                self.panic.set(Some(payload));
                reject
            }
        }
    }

    /// The root shape of the body the shape filter is testing now; `None` if the body has no
    /// shape, which cannot happen while the world is borrowed.
    fn current_root_shape(&self) -> Option<NonNull<JPH_Shape>> {
        // SAFETY: the shape filter is live for the whole query, and Jolt sets the body id on it
        // before every shape filter call (`TransformedShape.cpp`).
        let body = unsafe { JPH_ShapeFilter_GetBodyID2(self.shape_filter.get()) };
        if let Some((cached, root)) = self.root_cache.get() {
            if cached == body {
                return Some(root);
            }
        }
        // SAFETY: the body interface belongs to the borrowed world. Jolt has released the body
        // lock before the narrow phase calls the shape filter (`NarrowPhaseQuery.cpp`), so taking
        // it again here does not deadlock. joltc returns the pointer after releasing its
        // temporary reference; the body's own reference keeps the shape alive, because nothing
        // can remove a body or replace its shape while `&PhysicsWorld` is borrowed (`remove_body`
        // and `BodyMut::set_shape` need `&mut PhysicsWorld`).
        let root = unsafe { JPH_BodyInterface_GetShape(self.world.body_interface.as_ptr(), body) };
        let root = NonNull::new(root.cast_mut())?;
        self.root_cache.set(Some((body, root)));
        Some(root)
    }

    /// Whether the shape filter keeps `shape2` with sub-shape id `id2`, a part of the body it is
    /// testing now.
    fn keeps_child(&self, shape2: *const JPH_Shape, id2: JPH_SubShapeID) -> bool {
        if self.filter.child_groups.is_none() {
            return true;
        }
        let Some(root) = self.current_root_shape() else {
            return false;
        };
        // The root-level call carries the empty id, which would decode to the last child when
        // the child count is a power of two.
        if shape2 == root.as_ptr().cast_const() {
            return true;
        }
        // SAFETY: `root` stays alive for the query, as argued in `current_root_shape`.
        let sub_type = unsafe { JPH_Shape_GetSubType(root.as_ptr()) };
        if sub_type != JPH_ShapeSubType_StaticCompound
            && sub_type != JPH_ShapeSubType_MutableCompound
        {
            return true;
        }
        // SAFETY: as above.
        match unsafe { compound_sub_shape_of(root, SubShapeId::new(id2)) } {
            Some(child) => self.filter.keeps_group(child.user_data),
            None => false,
        }
    }
}

/// The joltc filters of one query, null where the filter part is not set (Jolt's accept-all
/// default).
pub(crate) struct RawFilters {
    pub(crate) object_layer: *const JPH_ObjectLayerFilter,
    pub(crate) body: *const JPH_BodyFilter,
    pub(crate) shape: *const JPH_ShapeFilter,
}

/// Creates a joltc filter carrying `user_data` when `wanted`; `None` otherwise.
///
/// # Safety
/// `create` is one of joltc's `JPH_*Filter_Create` functions and returns a new object of type
/// `T` that the caller owns entirely.
unsafe fn create_filter<T: JoltObject>(
    wanted: bool,
    create: unsafe extern "C" fn(*mut c_void) -> *mut T,
    user_data: *mut c_void,
) -> Result<Option<Owned<T>>, QueryError> {
    if !wanted {
        return Ok(None);
    }
    // SAFETY: joltc's create functions only store `user_data`; the caller guarantees that the
    // returned object is a `T` owned entirely by the handle.
    unsafe { Owned::from_raw(create(user_data)) }
        .map(Some)
        .ok_or(QueryError::AllocationFailed)
}

/// Runs `run` with the joltc filters that implement `filter` for one query on `world`.
///
/// The filters live until `run` returns and are destroyed before this returns. A panic in a
/// filter callback (or in a result callback that uses [`FilterState::guarded`]) is re-raised
/// here, after joltc has returned and the filters are gone.
pub(crate) fn with_query_filters<R>(
    world: &PhysicsWorld,
    filter: &QueryFilter<'_>,
    run: impl FnOnce(&RawFilters, &FilterState<'_>) -> R,
) -> Result<R, QueryError> {
    let state = FilterState::new(world, filter);
    let result = {
        install_procs();
        let user_data = (&state as *const FilterState<'_>)
            .cast_mut()
            .cast::<c_void>();
        // SAFETY: the create function returns a new object-layer filter that joltc `new`s and
        // the handle owns entirely; `user_data` points to `state`, which outlives the filter.
        let object_layer = unsafe {
            create_filter(
                filter.object_layers.is_some(),
                JPH_ObjectLayerFilter_Create,
                user_data,
            )
        }?;
        // SAFETY: as above, for a body filter.
        let body = unsafe {
            create_filter(
                filter.excluded_body.is_some() || filter.body_filter.is_some(),
                JPH_BodyFilter_Create,
                user_data,
            )
        }?;
        // SAFETY: as above, for a shape filter.
        let shape = unsafe {
            create_filter(
                filter.child_groups.is_some(),
                JPH_ShapeFilter_Create,
                user_data,
            )
        }?;
        if let Some(shape) = &shape {
            state.shape_filter.set(shape.as_ptr());
        }
        let raw = RawFilters {
            object_layer: object_layer
                .as_ref()
                .map_or(null(), |f| f.as_ptr().cast_const()),
            body: body.as_ref().map_or(null(), |f| f.as_ptr().cast_const()),
            shape: shape.as_ref().map_or(null(), |f| f.as_ptr().cast_const()),
        };
        run(&raw, &state)
    };
    if let Some(payload) = state.panic.take() {
        resume_unwind(payload);
    }
    Ok(result)
}

/// Reads the [`FilterState`] behind a filter's `userData`.
///
/// # Safety
/// `user_data` is the `userData` of a filter created by [`with_query_filters`] whose query is
/// still running, so it points to that query's live `FilterState`.
unsafe fn filter_state<'s>(user_data: *mut c_void) -> &'s FilterState<'s> {
    // SAFETY: the caller guarantees `user_data` points to a live `FilterState`; only shared
    // references to it exist while the query runs.
    unsafe { &*user_data.cast::<FilterState<'s>>() }
}

/// Object-layer filter callback: keeps the layers in [`QueryFilter::object_layers`].
///
/// # Safety
/// Called only by joltc for a filter created by [`with_query_filters`], with that query's live
/// `FilterState` as `user_data`.
unsafe extern "C" fn object_layer_should_collide(
    user_data: *mut c_void,
    layer: JPH_ObjectLayer,
) -> bool {
    // SAFETY: guaranteed by the caller (function contract).
    let state = unsafe { filter_state(user_data) };
    state.guarded(false, || {
        #[cfg(test)]
        tests::panic_if_injected(tests::Callback::ObjectLayer);
        state.filter.keeps_object_layer(ObjectLayer::new(layer))
    })
}

/// Body filter callback: applies [`QueryFilter::exclude_body`] and [`QueryFilter::body_filter`].
///
/// # Safety
/// Called only by joltc for a filter created by [`with_query_filters`], with that query's live
/// `FilterState` as `user_data`.
unsafe extern "C" fn body_should_collide(user_data: *mut c_void, body: JPH_BodyID) -> bool {
    // SAFETY: guaranteed by the caller (function contract).
    let state = unsafe { filter_state(user_data) };
    state.guarded(false, || {
        #[cfg(test)]
        tests::panic_if_injected(tests::Callback::Body);
        state.filter.keeps_body(BodyId::new(body, state.world.tag))
    })
}

/// Shape filter callback of ray casts and point queries: applies [`QueryFilter::child_groups`].
///
/// # Safety
/// Called only by joltc for a filter created by [`with_query_filters`], with that query's live
/// `FilterState` as `user_data`; `sub_shape_id2` points to a live id.
unsafe extern "C" fn shape_should_collide(
    user_data: *mut c_void,
    shape2: *const JPH_Shape,
    sub_shape_id2: *const JPH_SubShapeID,
) -> bool {
    // SAFETY: guaranteed by the caller (function contract).
    let (state, id2) = unsafe { (filter_state(user_data), *sub_shape_id2) };
    state.guarded(false, || {
        #[cfg(test)]
        tests::panic_if_injected(tests::Callback::Shape);
        state.keeps_child(shape2, id2)
    })
}

/// Shape filter callback of shape casts and collide queries: applies
/// [`QueryFilter::child_groups`] to the body's shape, which Jolt passes as shape 2 (also through
/// its `ReversedShapeFilter`). Shape 1 is the query shape and is not filtered.
///
/// # Safety
/// Called only by joltc for a filter created by [`with_query_filters`], with that query's live
/// `FilterState` as `user_data`; `sub_shape_id2` points to a live id.
unsafe extern "C" fn shape_should_collide2(
    user_data: *mut c_void,
    _shape1: *const JPH_Shape,
    _sub_shape_id1: *const JPH_SubShapeID,
    shape2: *const JPH_Shape,
    sub_shape_id2: *const JPH_SubShapeID,
) -> bool {
    // SAFETY: guaranteed by the caller (function contract).
    let (state, id2) = unsafe { (filter_state(user_data), *sub_shape_id2) };
    state.guarded(false, || {
        #[cfg(test)]
        tests::panic_if_injected(tests::Callback::Shape2);
        state.keeps_child(shape2, id2)
    })
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::{
        BodySettings, BroadPhaseLayer, CharacterSettings, CollideShape, CollisionLayers,
        CompoundChild, ExtendedUpdateSettings, Quat, RVec3, RayCast, Shape, ShapeCast, Vec3,
        WorldSettings,
    };

    /// A filter callback that a test can make panic.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) enum Callback {
        ObjectLayer,
        Body,
        Shape,
        Shape2,
    }

    thread_local! {
        /// The callback that panics on this thread. Callbacks run on the querying thread, so
        /// tests running in parallel do not see each other's choice.
        static INJECTED_PANIC: Cell<Option<Callback>> = const { Cell::new(None) };
    }

    /// Makes `callback` panic on this thread from now on, or nothing with `None`.
    pub(crate) fn inject_panic(callback: Option<Callback>) {
        INJECTED_PANIC.set(callback);
    }

    /// Panics when the running test made `callback` panic.
    pub(super) fn panic_if_injected(callback: Callback) {
        if INJECTED_PANIC.get() == Some(callback) {
            panic!("injected {callback:?} panic");
        }
    }

    fn world() -> PhysicsWorld {
        PhysicsWorld::new(WorldSettings::default()).unwrap()
    }

    #[test]
    fn guarded_keeps_the_first_panic_and_short_circuits() {
        let world = world();
        let filter = QueryFilter::new();
        let state = FilterState::new(&world, &filter);
        assert_eq!(state.guarded(0, || 7), 7);
        assert_eq!(state.guarded(0, || panic!("first")), 0);
        let mut ran = false;
        assert_eq!(
            state.guarded(0, || {
                ran = true;
                7
            }),
            0
        );
        assert!(!ran);
        let payload = state.panic.take().expect("the panic is kept");
        assert_eq!(payload.downcast_ref::<&str>(), Some(&"first"));
    }

    #[test]
    fn with_query_filters_re_raises_after_the_query() {
        let world = world();
        let layers = [ObjectLayer::NON_MOVING];
        let filter = QueryFilter::new()
            .object_layers(&layers)
            .child_groups(1)
            .exclude_body(BodyId::new(0, world.tag));
        let result = catch_unwind(AssertUnwindSafe(|| {
            with_query_filters(&world, &filter, |raw, state| {
                assert!(!raw.object_layer.is_null());
                assert!(!raw.body.is_null());
                assert!(!raw.shape.is_null());
                state.guarded((), || panic!("in a callback"));
            })
        }));
        let payload = result.expect_err("the panic is re-raised");
        assert_eq!(payload.downcast_ref::<&str>(), Some(&"in a callback"));
    }

    #[derive(Clone, Copy, Debug)]
    enum Query {
        Ray,
        ShapeCast,
        Collide,
        Point,
    }

    /// The body `query` finds first around the origin, from above, overlapping y 0.5 or
    /// containing a point at y 0.2.
    fn found_body(
        world: &PhysicsWorld,
        filter: &QueryFilter<'_>,
        query: Query,
        ball: &Shape,
    ) -> Option<BodyId> {
        let above = RVec3::new(0.0, 5.0, 0.0);
        let down = Vec3::new(0.0, -10.0, 0.0);
        match query {
            Query::Ray => world
                .cast_ray(&RayCast::new(above, down), filter)
                .unwrap()
                .map(|hit| hit.body),
            Query::ShapeCast => world
                .cast_shape(&ShapeCast::new(ball, above, Quat::IDENTITY, down), filter)
                .unwrap()
                .map(|hit| hit.body),
            Query::Collide => {
                let overlapping = RVec3::new(0.0, 0.5, 0.0);
                world
                    .collide_shape(
                        &CollideShape::new(ball, overlapping, Quat::IDENTITY),
                        filter,
                    )
                    .unwrap()
                    .first()
                    .map(|hit| hit.body)
            }
            Query::Point => world
                .collide_point(RVec3::new(0.0, 0.2, 0.0), filter)
                .unwrap()
                .first()
                .map(|hit| hit.body),
        }
    }

    #[test]
    fn a_panic_in_a_filter_callback_resumes_after_the_query() {
        let mut world = world();
        let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
        let compound = Shape::new_compound(&[
            CompoundChild {
                shape: &unit_box,
                position: Vec3::ZERO,
                rotation: Quat::IDENTITY,
                user_data: 1,
            },
            CompoundChild {
                shape: &unit_box,
                position: Vec3::new(3.0, 0.0, 0.0),
                rotation: Quat::IDENTITY,
                user_data: 2,
            },
        ])
        .unwrap();
        let target = world
            .create_body(&compound, &BodySettings::new_static())
            .unwrap();
        let ball = Shape::new_sphere(0.25).unwrap();
        let excluded = world
            .create_body(
                &ball,
                &BodySettings::new_static().position(RVec3::new(50.0, 0.0, 0.0)),
            )
            .unwrap();
        let layers = [ObjectLayer::NON_MOVING];
        let filter = QueryFilter::new()
            .object_layers(&layers)
            .child_groups(1 << 1)
            .exclude_body(excluded);

        let cases = [
            (Query::Ray, Callback::ObjectLayer),
            (Query::Ray, Callback::Body),
            (Query::Ray, Callback::Shape),
            (Query::ShapeCast, Callback::ObjectLayer),
            (Query::ShapeCast, Callback::Body),
            (Query::ShapeCast, Callback::Shape2),
            (Query::Collide, Callback::ObjectLayer),
            (Query::Collide, Callback::Body),
            (Query::Collide, Callback::Shape2),
            (Query::Point, Callback::ObjectLayer),
            (Query::Point, Callback::Body),
            (Query::Point, Callback::Shape),
        ];
        for (query, callback) in cases {
            INJECTED_PANIC.set(Some(callback));
            let result = catch_unwind(AssertUnwindSafe(|| {
                found_body(&world, &filter, query, &ball)
            }));
            INJECTED_PANIC.set(None);
            let payload = result.expect_err("joltc returned and the panic resumed");
            assert_eq!(
                payload.downcast_ref::<String>().map(String::as_str),
                Some(format!("injected {callback:?} panic").as_str()),
                "{query:?}"
            );
            assert_eq!(
                found_body(&world, &filter, query, &ball),
                Some(target),
                "{query:?} after a {callback:?} panic"
            );
        }

        // No body lock is left held: removing the body takes its write lock.
        world.remove_body(target).unwrap();
        for query in [Query::Ray, Query::ShapeCast, Query::Collide, Query::Point] {
            assert_eq!(found_body(&world, &filter, query, &ball), None, "{query:?}");
        }
    }

    #[test]
    fn a_panic_in_a_filter_callback_resumes_after_a_character_update_or_refresh() {
        let mut world = world();
        let unit_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
        let floor = Shape::new_compound(&[
            CompoundChild {
                shape: &unit_box,
                position: Vec3::new(0.0, -0.5, 0.0),
                rotation: Quat::IDENTITY,
                user_data: 1,
            },
            CompoundChild {
                shape: &unit_box,
                position: Vec3::new(3.0, -0.5, 0.0),
                rotation: Quat::IDENTITY,
                user_data: 2,
            },
        ])
        .unwrap();
        world
            .create_body(&floor, &BodySettings::new_static())
            .unwrap();
        let ball = Shape::new_sphere(0.25).unwrap();
        let excluded = world
            .create_body(
                &ball,
                &BodySettings::new_static().position(RVec3::new(50.0, 0.0, 0.0)),
            )
            .unwrap();
        let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
        let id = world
            .create_character(
                &CharacterSettings::new(&capsule).shape_offset(Vec3::new(0.0, 0.8, 0.0)),
                RVec3::new(0.0, 0.05, 0.0),
                Quat::IDENTITY,
            )
            .unwrap();
        let layers = [ObjectLayer::NON_MOVING];
        let filter = QueryFilter::new()
            .object_layers(&layers)
            .child_groups(1 << 1)
            .exclude_body(excluded);
        let update = |world: &mut PhysicsWorld| {
            world
                .character_mut(id)
                .unwrap()
                .set_linear_velocity(Vec3::new(0.5, -1.0, 0.0))
                .unwrap();
            world.update_character(
                id,
                1.0 / 60.0,
                Vec3::new(0.0, -9.81, 0.0),
                &ExtendedUpdateSettings::default(),
                &filter,
            )
        };

        for callback in [Callback::ObjectLayer, Callback::Body, Callback::Shape2] {
            INJECTED_PANIC.set(Some(callback));
            let result = catch_unwind(AssertUnwindSafe(|| update(&mut world)));
            INJECTED_PANIC.set(None);
            let payload = result.expect_err("joltc returned and the panic resumed");
            assert_eq!(
                payload.downcast_ref::<String>().map(String::as_str),
                Some(format!("injected {callback:?} panic").as_str()),
            );
            assert_eq!(update(&mut world), Ok(()), "after a {callback:?} panic");
        }
        // The same through a contact refresh, the other character call that runs the filters.
        for callback in [Callback::ObjectLayer, Callback::Body, Callback::Shape2] {
            INJECTED_PANIC.set(Some(callback));
            let result = catch_unwind(AssertUnwindSafe(|| {
                world.refresh_character_contacts(id, &filter)
            }));
            INJECTED_PANIC.set(None);
            let payload = result.expect_err("joltc returned and the panic resumed");
            assert_eq!(
                payload.downcast_ref::<String>().map(String::as_str),
                Some(format!("injected {callback:?} panic").as_str()),
            );
            assert_eq!(
                world.refresh_character_contacts(id, &filter),
                Ok(()),
                "after a {callback:?} panic"
            );
        }
        // No body lock is left held: removing a body takes its write lock.
        world.remove_body(excluded).unwrap();
    }

    #[test]
    fn unset_parts_create_no_filters() {
        let world = world();
        let created = with_query_filters(&world, &QueryFilter::new(), |raw, _| {
            [
                raw.object_layer.is_null(),
                raw.body.is_null(),
                raw.shape.is_null(),
            ]
        });
        assert_eq!(created, Ok([true; 3]));
    }

    #[test]
    fn validate_checks_layers_and_worlds() {
        let other = world();
        let mut layers = CollisionLayers::new(1);
        let only = layers.add_object_layer(BroadPhaseLayer::new(0));
        let world = PhysicsWorld::new(WorldSettings::default().layers(layers)).unwrap();

        let known = [only];
        assert_eq!(
            QueryFilter::new().object_layers(&known).validate(&world),
            Ok(())
        );
        let unknown = [ObjectLayer::new(1)];
        assert_eq!(
            QueryFilter::new().object_layers(&unknown).validate(&world),
            Err(QueryError::UnknownObjectLayer(unknown[0]))
        );
        let foreign = BodyId::new(0, other.tag);
        assert_eq!(
            QueryFilter::new().exclude_body(foreign).validate(&world),
            Err(QueryError::WrongWorld(foreign))
        );
        // An id of this world that names no body (removed, or never created) is allowed.
        let removed = BodyId::new(0, world.tag);
        assert!(!world.contains_body(removed));
        assert_eq!(
            QueryFilter::new().exclude_body(removed).validate(&world),
            Ok(())
        );
    }

    #[test]
    fn group_mask_selects_user_data_values() {
        let filter = QueryFilter::new().child_groups(1 << 2 | 1 << 31);
        assert!(filter.keeps_group(2));
        assert!(filter.keeps_group(31));
        assert!(!filter.keeps_group(3));
        assert!(!filter.keeps_group(32));
        assert!(!filter.keeps_group(u32::MAX));
        assert!(QueryFilter::new().keeps_group(u32::MAX));
    }
}
