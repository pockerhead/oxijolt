//! The physics world: a Jolt physics system with its own job system and temp allocator. The job
//! system is Jolt's thread pool, Jolt's single-threaded job system, or a native one that hands the
//! jobs to the caller's [`JobSystem`].
//!
//! joltc keeps a global map of physics systems that creating and destroying a system writes
//! without synchronisation, so both run under one process-wide lock here. Stepping uses the
//! world's own temp allocator and job system and needs no global lock, so independent worlds
//! step in parallel. The joltc functions that read that global map (joltc's own step listener,
//! `JPH_PhysicsStepListener_Create`) or use joltc's shared temp allocator
//! (`JPH_PhysicsSystem_Update` and joltc's own character updates) are not used here; wrapping
//! them requires revisiting this lock. Vehicles are Jolt's own step listeners: they get the
//! system from the step context and touch no joltc global, so worlds with vehicles still step
//! in parallel. Characters are updated with
//! the world's own temp allocator through the `oxijolt-sys` extension
//! (`JPH_CharacterVirtual_ExtendedUpdate2`, `JPH_CharacterVirtual_RefreshContacts2`), which
//! needs `&mut self`. Callbacks that run inside a step must not use the locking body interface,
//! which would deadlock. The event listeners and their callbacks live in the `listener` module
//! ([`PhysicsWorld::set_event_settings`]).
//!
//! [`JobSystem`]: crate::JobSystem

use std::any::Any;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::num::NonZeroU64;
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};

use oxijolt_sys::*;

use crate::character::CharacterEntry;
use crate::constraint::ConstraintEntry;
use crate::job_system::{create_caller_job_system, QueueContext};
use crate::jolt_assert;
use crate::limits;
use crate::listener::Listeners;
use crate::owned::{JoltObject, Owned};
use crate::panic_slot::first_payload;
use crate::ragdoll::RagdollEntry;
use crate::vehicle::VehicleEntry;
use crate::{CharacterContactListener, CollisionLayers, StepError, Vec3, WorldError};

mod rebase;
mod settings;

use settings::JobSystemChoice;
pub use settings::{JobSystemKind, WorldSettings};

/// Installs the assertion handler and runs `JPH_Init` once per process, and returns whether
/// `JPH_Init` succeeded. oxijolt never calls `JPH_Shutdown`: Jolt's global state lives as
/// long as the process.
pub(crate) fn ensure_initialized() -> bool {
    static INIT: OnceLock<bool> = OnceLock::new();
    *INIT.get_or_init(|| {
        jolt_assert::install();
        // SAFETY: `OnceLock` runs the closure exactly once per process, which is the only
        // synchronisation `JPH_Init` (an unsynchronised `bool` guard) needs. The assertion
        // handler is in place before `JPH_Init` registers Jolt's types.
        unsafe { JPH_Init() }
    })
}

/// Serialises `JPH_PhysicsSystem_Create` and `JPH_PhysicsSystem_Destroy`, which write joltc's
/// unsynchronised global map of physics systems.
fn lock_joltc_globals() -> MutexGuard<'static, ()> {
    static GLOBALS: Mutex<()> = Mutex::new(());
    GLOBALS.lock().unwrap_or_else(PoisonError::into_inner)
}

/// A world's temp allocator, owned by the world.
impl JoltObject for JPH_TempAllocator {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the allocator (trait contract); the system that used it is
        // gone or never existed (field order in `PhysicsWorld`).
        unsafe { JPH_TempAllocator_Destroy(ptr) };
    }
}

/// A world's job system: Jolt's thread pool or single-threaded job system, owned by the world, or
/// joltc's callback job system for the caller's [`JobSystem`], owned by the world and by every job
/// still queued.
///
/// [`JobSystem`]: crate::JobSystem
impl JoltObject for JPH_JobSystem {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the job system (trait contract), and no step is running: `step`
        // borrows the world mutably, and the last owner of a callback job system holds no job
        // that Jolt could still run. Destroying a thread pool joins its worker threads; a
        // single-threaded job system has no threads and keeps only its free list of jobs
        // (`JobSystemSingleThreaded.cpp`); a callback job system has no threads and may be
        // destroyed on any thread, also on one of the caller's pool threads.
        unsafe { JPH_JobSystem_Destroy(ptr) };
    }
}

/// The job system a world steps with.
enum WorldJobSystem {
    /// Jolt's thread pool.
    ThreadPool(Owned<JPH_JobSystem>),
    /// Jolt's single-threaded job system.
    SingleThreaded(Owned<JPH_JobSystem>),
    /// joltc's callback job system, which hands jobs to the caller's [`JobSystem`].
    ///
    /// [`JobSystem`]: crate::JobSystem
    Caller(Arc<QueueContext>),
}

impl WorldJobSystem {
    /// Creates Jolt's thread pool with `workers` worker threads, already validated.
    fn thread_pool(workers: u32) -> Result<Self, WorldError> {
        let config = JobSystemThreadPoolConfig {
            maxJobs: 0,
            maxBarriers: 0,
            // `validate` bounds the count to `1..=MAX_WORKER_THREADS`; 0 would mean all hardware
            // threads.
            numThreads: workers as i32,
        };
        // SAFETY: Jolt is initialised; `config` is a live local. Zero job and barrier limits
        // select Jolt's `cMaxPhysicsJobs` and `cMaxPhysicsBarriers`. The handle takes over the
        // returned job system.
        let pool = unsafe { Owned::from_raw(JPH_JobSystemThreadPool_Create(&config)) }
            .ok_or(WorldError::AllocationFailed("job system"))?;
        Ok(Self::ThreadPool(pool))
    }

    /// Creates Jolt's single-threaded job system.
    fn single_threaded() -> Result<Self, WorldError> {
        // SAFETY: Jolt is initialised (`PhysicsWorld::new` checked `ensure_initialized` first). 0
        // selects Jolt's `cMaxPhysicsJobs`, more jobs than one update of a concurrency-1 job
        // system holds at once (docs/limits.md#collision-steps). The handle takes over the
        // returned job system.
        let jobs = unsafe { Owned::from_raw(JPH_JobSystemSingleThreaded_Create(0)) }
            .ok_or(WorldError::AllocationFailed("job system"))?;
        Ok(Self::SingleThreaded(jobs))
    }

    fn as_ptr(&self) -> *mut JPH_JobSystem {
        match self {
            Self::ThreadPool(pool) | Self::SingleThreaded(pool) => pool.as_ptr(),
            Self::Caller(context) => context.as_ptr(),
        }
    }

    /// Finishes a step after the update returned: releases the jobs a caller job system left to
    /// the stepping thread and returns the first panic of its `queue_job` during the step.
    fn finish_update(&self) -> Option<Box<dyn Any + Send>> {
        match self {
            Self::ThreadPool(_) | Self::SingleThreaded(_) => None,
            Self::Caller(context) => context.finish_update(),
        }
    }
}

/// A physics system, owned by its world. Destroying it also deletes its bodies with their
/// shape references and the three layer objects passed to `JPH_PhysicsSystem_Create`
/// (joltc's `JPH_PhysicsSystem_Destroy`).
///
/// Destroying takes the globals lock, so an `Owned<JPH_PhysicsSystem>` must never be dropped
/// while that lock is held: the mutex is not reentrant.
impl JoltObject for JPH_PhysicsSystem {
    unsafe fn destroy(ptr: *mut Self) {
        let _globals = lock_joltc_globals();
        // SAFETY: the owner owns the system (trait contract) and the globals lock is held, so
        // no other thread writes joltc's system map. Nothing borrows the system any more.
        unsafe { JPH_PhysicsSystem_Destroy(ptr) };
    }
}

/// Identifies a world among all worlds of the process, so a [`BodyId`](crate::BodyId) from one
/// world is rejected by another. Never affects simulation results.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub(crate) struct WorldTag(NonZeroU64);

impl WorldTag {
    pub(crate) fn next() -> Self {
        static NEXT_WORLD: AtomicU64 = AtomicU64::new(1);
        let value = NEXT_WORLD.fetch_add(1, Ordering::Relaxed);
        Self(NonZeroU64::new(value).expect("world counter overflowed"))
    }
}

/// Advances a world's structure epoch ([`PhysicsWorld::note_structure_change`]), for views that
/// borrow only part of the world. Overflow cannot happen in practice and fails closed.
pub(crate) fn advance_structure_epoch(epoch: &mut u64) {
    *epoch = epoch.checked_add(1).expect("structure epoch overflowed");
}

/// A Jolt physics system with its collision layers, its own job system and temp allocator.
///
/// The job system is Jolt's thread pool, Jolt's single-threaded job system, or a native one that
/// hands the jobs to the caller's [`JobSystem`] ([`WorldSettings::job_system`]).
///
/// Changing the world, including [`step`](Self::step), takes `&mut self`; reading takes
/// `&self`, so many threads can read one world while nobody steps it. Several worlds are
/// independent and may be stepped on different threads at the same time.
///
/// Bodies added one at a time leave the broad phase unoptimised, which makes queries slower
/// until Jolt has rebuilt it during steps (Jolt docs, "Bodies"); see
/// [`optimize_broad_phase`](Self::optimize_broad_phase).
///
/// [`JobSystem`]: crate::JobSystem
pub struct PhysicsWorld {
    // Field order is drop order, after `Drop for PhysicsWorld` has taken every constraint out of
    // the system and released it, then every vehicle out of the system's step listeners and
    // constraints and every ragdoll out of the system, so `constraints`, `vehicles` and
    // `ragdolls` are empty by then. The characters go first: each destructor removes its inner
    // body through the still-live system. The character collision set follows; it only frees
    // its list of character pointers. The system goes next, before the job system and allocator
    // its steps used, and deletes the layer tables it owns. A caller job system's native object
    // may outlive the world while jobs the caller's pool has not run or dropped yet hold it. The
    // interface and query pointers after them are borrowed from the system and have no
    // destructor.
    /// The characters by Jolt character id.
    pub(crate) characters: BTreeMap<u32, CharacterEntry>,
    /// The vehicles by vehicle id.
    pub(crate) vehicles: BTreeMap<u32, VehicleEntry>,
    /// The ragdolls by ragdoll id.
    pub(crate) ragdolls: BTreeMap<u32, RagdollEntry>,
    /// The constraints by constraint id.
    pub(crate) constraints: BTreeMap<u32, ConstraintEntry>,
    /// Jolt's `CharacterVsCharacterCollisionSimple` of the characters that collide with each
    /// other, created with the first of them.
    pub(crate) character_collision: Option<Owned<JPH_CharacterVsCharacterCollision>>,
    pub(crate) system: Owned<JPH_PhysicsSystem>,
    /// The contact, activation and soft body listeners, detached first in `Drop` and destroyed
    /// after the system.
    pub(crate) listeners: Listeners,
    job_system: WorldJobSystem,
    job_system_kind: JobSystemKind,
    max_concurrency: u32,
    pub(crate) temp_allocator: Owned<JPH_TempAllocator>,
    pub(crate) body_interface: NonNull<JPH_BodyInterface>,
    pub(crate) body_lock_interface: NonNull<JPH_BodyLockInterface>,
    pub(crate) narrow_phase_query: NonNull<JPH_NarrowPhaseQuery>,
    pub(crate) broad_phase_query: NonNull<JPH_BroadPhaseQuery>,
    pub(crate) object_layer_count: u32,
    // What the world was created with and Jolt has no getter for.
    max_body_pairs: u32,
    max_contact_constraints: u32,
    temp_allocator_size: u32,
    layers: CollisionLayers,
    pub(crate) tag: WorldTag,
    /// Raw ids of the characters' inner bodies.
    pub(crate) inner_bodies: BTreeSet<u32>,
    /// The Jolt character id the next character gets; ids start at 1 and are never reused.
    pub(crate) next_character_id: u32,
    /// Raw ids of the vehicles' chassis bodies, with their vehicle ids.
    pub(crate) vehicle_bodies: BTreeMap<u32, u32>,
    /// The id the next vehicle gets; ids start at 1 and are never reused.
    pub(crate) next_vehicle_id: u32,
    /// Raw ids of the ragdolls' part bodies, with their ragdoll ids.
    pub(crate) ragdoll_bodies: BTreeMap<u32, u32>,
    /// The id the next ragdoll gets; ids start at 1 and are never reused.
    pub(crate) next_ragdoll_id: u32,
    /// Raw ids of the bodies constraints use, with the ids of those constraints.
    pub(crate) constraint_bodies: BTreeMap<u32, BTreeSet<u32>>,
    /// The id the next constraint gets; ids start at 1 and are never reused.
    pub(crate) next_constraint_id: u32,
    /// Counts the changes Jolt's saved state cannot express (the body, character, vehicle,
    /// ragdoll and constraint sets, the BodyID allocator, motion types, body shapes, sensor flags,
    /// rebases); a [`WorldState`](crate::WorldState) restores only at the epoch it was saved at.
    pub(crate) structure_epoch: u64,
    /// Raw ids of the bodies whose contact cache the next simulating step invalidates
    /// ([`BodyMut::invalidate_contact_cache`](crate::BodyMut::invalidate_contact_cache),
    /// [`BodyMut::set_shape`](crate::BodyMut::set_shape)); part of a
    /// [`WorldState`](crate::WorldState). Jolt saves no invalidation and its flag cannot be
    /// cleared once set, so the flag is set natively only right before a step that runs the
    /// simulation pass, which clears it at its end (`PhysicsSystem.cpp:2456-2457`), and inside
    /// `set_shape`, which advances the structure epoch.
    pub(crate) pending_cache_invalidations: BTreeSet<u32>,
    /// The listener the character updates and refreshes attach for their duration.
    pub(crate) character_listener: Option<Arc<dyn CharacterContactListener>>,
}

impl fmt::Debug for PhysicsWorld {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PhysicsWorld")
            .field("body_count", &self.body_count())
            .field("constraint_count", &self.constraint_count())
            .finish_non_exhaustive()
    }
}

impl Drop for PhysicsWorld {
    fn drop(&mut self) {
        // First: removing constraints, ragdolls and characters' inner bodies deactivates bodies,
        // which must not reach a listener any more.
        // SAFETY: the system is this world's and no step runs during `drop`.
        unsafe { self.listeners.detach(self.system.as_ptr()) };
        self.remove_all_constraints();
        self.remove_all_vehicles();
        self.remove_all_ragdolls();
    }
}

// SAFETY: the physics system, job system, temp allocator, characters and character collision
// set have no thread affinity. `step` and the character updates need `&mut self`, so the
// allocator and job system serve one call at a time
// (https://jrouwe.github.io/JoltPhysicsDocs/5.6.0/index.html#multi-threaded-access). A caller
// job system is `Send + Sync` by the `JobSystem` bound, and its native callback object is
// `Send + Sync` (see `CallbackJobSystem`). Jolt's single-threaded job system runs jobs on the
// stepping thread and holds no thread-local or thread-bound state, so it has no thread affinity
// either. Vehicle tire max impulse callbacks are `Send + Sync` by their bound.
unsafe impl Send for PhysicsWorld {}
// SAFETY: no `&self` method touches the job system. Every `&self` method only calls Jolt's
// locking body interface, read-only system
// getters, or Jolt's locking narrow-phase queries, which read bodies under body read locks and
// the broad phase under its query lock. Jolt allows all of these from several threads at once.
// Jolt forbids body access only while `PhysicsSystem::Update` runs, and `step` needs `&mut self`
// (https://jrouwe.github.io/JoltPhysicsDocs/5.6.0/index.html#multi-threaded-access). Query filter
// and result callbacks run on the querying thread, read the world only through the locking body
// interface and write only state on that query's stack. Character reads through `&self` are
// joltc getters over const Jolt members (`GetPosition`, `GetGroundState`,
// `GetActiveContacts().at()`, the const `SaveState`); every character change and update takes
// `&mut self`, and so does every use of `CharacterVsCharacterCollisionSimple`, which is not
// thread-safe (`CharacterVirtual.h`). Vehicle reads through `&self` are joltc getters over
// members of the vehicle constraint, its wheels and controller, which Jolt writes only during
// `step` and the vehicle setters, both behind `&mut self`. Ragdoll reads through `&self` use the
// locking body interface and constraint getters that read constraint members and the bodies'
// rotations, which Jolt writes only in `step` and the ragdoll and body setters, all behind
// `&mut self`. Constraint reads through `&self` are joltc getters over constraint members and the
// bodies' transforms, which Jolt writes only in `step` and the `&mut` constraint and body
// setters. `save_state` and `save_state_of` call `PhysicsSystem::SaveState`, which is const:
// `BodyManager::SaveState` takes every body lock (`LockAllBodies`, in mutex-array order, so
// concurrent saves do not deadlock), `ConstraintManager::SaveState` takes its mutex, and the
// contact cache it reads is written only by `step` and `restore_state`, both behind `&mut self`.
// The characters' `SaveState` is const as well. Soft body reads through `&self` copy the vertices
// under Jolt's body read lock; vertices are written only by `step` and the `&mut` soft body and
// body setters. `active_body_poses` copies the active body lists under Jolt's active-list mutex,
// releases it, and then reads the poses under one multi-body read lock, which takes body mutexes
// shared in mutex-array order. A concurrent `save_state` takes them exclusively in the same order,
// so the two may wait for each other but cannot deadlock. Event callbacks write the listener context only while Jolt steps, activates,
// deactivates or removes bodies, which happens only in `&mut self` methods: no `&self` method
// steps, adds, removes or wakes a body, and `event_settings` only reads the context's immutable
// settings. The character contact listener is `Send + Sync` by its bound and is called only from
// the `&mut self` character updates and refreshes. Vehicle tire max impulse callbacks are
// `Send + Sync` by their bound and run only inside `step`; `&self` methods only read whether a
// vehicle has one.
unsafe impl Sync for PhysicsWorld {}

/// Jolt's `PhysicsSettings` of `system`, copied out by joltc.
///
/// # Safety
/// `system` is a live physics system that no step is updating.
unsafe fn physics_settings(system: *mut JPH_PhysicsSystem) -> JPH_PhysicsSettings {
    // SAFETY: an all-zero `JPH_PhysicsSettings` is valid: integers, floats and `false`.
    let mut settings: JPH_PhysicsSettings = unsafe { std::mem::zeroed() };
    // SAFETY: the system is live and not stepping (contract); joltc writes every field of the
    // live local.
    unsafe { JPH_PhysicsSystem_GetPhysicsSettings(system, &mut settings) };
    settings
}

/// What a step length must satisfy ([`PhysicsWorld::is_valid_delta_time`]).
pub(crate) const DELTA_TIME_RULE: &str =
    "delta time must be finite and between MIN_DELTA_TIME and MAX_DELTA_TIME";

/// What a collision step count must satisfy ([`PhysicsWorld::step_with_collision_steps`]).
const COLLISION_STEPS_RULE: &str = "collision_steps must be between 1 and MAX_COLLISION_STEPS";

impl PhysicsWorld {
    /// Largest time step [`step`](Self::step) accepts, in seconds, inclusive.
    ///
    /// A oxijolt guard against overflow-scale steps, which Jolt runs to NaN positions; not a
    /// Jolt limit and not a stability guarantee. Jolt recommends steps of about 1/60 s, and
    /// larger ones may tunnel or sag depending on the scene.
    pub const MAX_DELTA_TIME: f32 = 1.0;

    /// Smallest time step [`step`](Self::step) accepts, in seconds, inclusive.
    ///
    /// A oxijolt guard, not a Jolt limit. Jolt divides by the step: a kinematic body's
    /// velocity is its move over the step (`MoveKinematic`), a character's velocities are
    /// derived the same way, and a wheel's brake-lock torque is `|ω| · inertia / step`
    /// (`WheeledVehicleController::PostCollide`). A subnormal step makes these infinite. The
    /// bound keeps the divisor away from subnormal values; a huge numerator can still overflow a
    /// quotient, and the bound does not limit every force or velocity a step can produce.
    /// [`step_with_collision_steps`](Self::step_with_collision_steps) also requires each
    /// collision step, `delta_time / collision_steps`, to be at least this long, because Jolt
    /// divides by the collision step.
    pub const MIN_DELTA_TIME: f32 = 1.0e-6;

    /// Largest collision step count [`step_with_collision_steps`](Self::step_with_collision_steps)
    /// accepts, inclusive. A oxijolt bound, not a Jolt limit: with more collision steps one update
    /// can create more jobs than Jolt's fixed job pool holds (`cMaxPhysicsJobs`, 2048), and then
    /// the step asserts or stalls. See [docs/limits.md].
    ///
    /// [docs/limits.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#collision-steps
    pub const MAX_COLLISION_STEPS: u32 = 4;

    /// Whether `delta_time` is finite and within `MIN_DELTA_TIME..=MAX_DELTA_TIME`.
    pub(crate) fn is_valid_delta_time(delta_time: f32) -> bool {
        delta_time.is_finite()
            && (Self::MIN_DELTA_TIME..=Self::MAX_DELTA_TIME).contains(&delta_time)
    }

    /// Creates a world. Nothing is allocated when the settings are invalid.
    ///
    /// With a caller [`JobSystem`], its [`max_concurrency`](crate::JobSystem::max_concurrency) is
    /// read once, before anything is allocated.
    ///
    /// [`JobSystem`]: crate::JobSystem
    pub fn new(settings: WorldSettings) -> Result<Self, WorldError> {
        settings.validate()?;
        let max_concurrency = settings.jobs.max_concurrency()?;
        if !ensure_initialized() {
            return Err(WorldError::InitFailed);
        }

        // SAFETY: Jolt is initialised; the size is positive (`validate`). The handle takes over
        // the returned allocator.
        let temp_allocator =
            unsafe { Owned::from_raw(JPH_TempAllocator_Create(settings.temp_allocator_size)) }
                .ok_or(WorldError::AllocationFailed("temp allocator"))?;

        let job_system = match &settings.jobs {
            JobSystemChoice::ThreadPool(workers) => WorldJobSystem::thread_pool(*workers)?,
            JobSystemChoice::SingleThreaded => WorldJobSystem::single_threaded()?,
            JobSystemChoice::Caller(job_system) => WorldJobSystem::Caller(
                create_caller_job_system(Arc::clone(job_system), max_concurrency)?,
            ),
        };

        // SAFETY: Jolt is initialised and `validate` checked the layers.
        let tables = unsafe { settings.layers.create_tables() }?;
        let (broad_phase, pair_filter, object_vs_broad_phase) = tables.as_raw();
        let system_settings = JPH_PhysicsSystemSettings {
            maxBodies: settings.max_bodies,
            numBodyMutexes: 0,
            maxBodyPairs: settings.max_body_pairs,
            maxContactConstraints: settings.max_contact_constraints,
            _padding: 0,
            broadPhaseLayerInterface: broad_phase,
            objectLayerPairFilter: pair_filter,
            objectVsBroadPhaseLayerFilter: object_vs_broad_phase,
        };
        let system = {
            let _globals = lock_joltc_globals();
            // SAFETY: Jolt is initialised, the globals lock is held, and the three tables are
            // live and consistent. On success the system owns the tables and the handle owns the
            // system.
            unsafe { Owned::from_raw(JPH_PhysicsSystem_Create(&system_settings)) }
        };
        let Some(system) = system else {
            return Err(WorldError::AllocationFailed("physics system"));
        };
        tables.forget();

        let gravity = settings.gravity.to_jph();
        // SAFETY: `system` is live and `gravity` is a live local.
        unsafe { JPH_PhysicsSystem_SetGravity(system.as_ptr(), &gravity) };
        // SAFETY: `system` is live, owned here, and nothing steps or reads it yet.
        let mut physics = unsafe { physics_settings(system.as_ptr()) };
        physics.numVelocitySteps = settings.velocity_steps;
        physics.numPositionSteps = settings.position_steps;
        physics.penetrationSlop = settings.penetration_slop;
        physics.speculativeContactDistance = settings.speculative_contact_distance;
        // SAFETY: as above; `physics` is a live local that joltc copies field by field.
        unsafe { JPH_PhysicsSystem_SetPhysicsSettings(system.as_ptr(), &mut physics) };

        // SAFETY: `system` is live. The interfaces and the narrow-phase and broad-phase queries
        // live inside the Jolt system and stay valid as long as it does; the world stores them
        // next to the system that owns them.
        let (body_interface, body_lock_interface, narrow_phase_query, broad_phase_query) = unsafe {
            (
                JPH_PhysicsSystem_GetBodyInterface(system.as_ptr()),
                JPH_PhysicsSystem_GetBodyLockInterface(system.as_ptr()),
                JPH_PhysicsSystem_GetNarrowPhaseQuery(system.as_ptr()),
                JPH_PhysicsSystem_GetBroadPhaseQuery(system.as_ptr()),
            )
        };
        let body_interface =
            NonNull::new(body_interface).ok_or(WorldError::AllocationFailed("body interface"))?;
        let body_lock_interface = NonNull::new(body_lock_interface.cast_mut())
            .ok_or(WorldError::AllocationFailed("body lock interface"))?;
        let narrow_phase_query = NonNull::new(narrow_phase_query.cast_mut())
            .ok_or(WorldError::AllocationFailed("narrow phase query"))?;
        let broad_phase_query = NonNull::new(broad_phase_query.cast_mut())
            .ok_or(WorldError::AllocationFailed("broad phase query"))?;

        Ok(Self {
            characters: BTreeMap::new(),
            vehicles: BTreeMap::new(),
            ragdolls: BTreeMap::new(),
            constraints: BTreeMap::new(),
            character_collision: None,
            system,
            listeners: Listeners::default(),
            job_system,
            job_system_kind: settings.jobs.kind(),
            max_concurrency,
            temp_allocator,
            body_interface,
            body_lock_interface,
            narrow_phase_query,
            broad_phase_query,
            object_layer_count: settings.layers.object_layer_count(),
            max_body_pairs: settings.max_body_pairs,
            max_contact_constraints: settings.max_contact_constraints,
            temp_allocator_size: settings.temp_allocator_size,
            layers: settings.layers,
            tag: WorldTag::next(),
            inner_bodies: BTreeSet::new(),
            next_character_id: 1,
            vehicle_bodies: BTreeMap::new(),
            next_vehicle_id: 1,
            ragdoll_bodies: BTreeMap::new(),
            next_ragdoll_id: 1,
            constraint_bodies: BTreeMap::new(),
            next_constraint_id: 1,
            structure_epoch: 0,
            pending_cache_invalidations: BTreeSet::new(),
            character_listener: None,
        })
    }

    /// Records a change that a [`WorldState`](crate::WorldState) cannot undo, so states saved
    /// before it are refused. Every API that adds or removes a Jolt object (body, character,
    /// vehicle, ragdoll, constraint) or changes a motion type, a body's shape or its sensor flag
    /// must call it, after its checks and before its first Jolt call that makes the change.
    pub(crate) fn note_structure_change(&mut self) {
        advance_structure_epoch(&mut self.structure_epoch);
    }

    /// Gravity in m/s².
    pub fn gravity(&self) -> Vec3 {
        let mut gravity = Vec3::ZERO.to_jph();
        // SAFETY: the system is live; reading gravity does not change it, and writes need
        // `&mut self`. `gravity` is a live local.
        unsafe { JPH_PhysicsSystem_GetGravity(self.system.as_ptr(), &mut gravity) };
        Vec3::from_jph(gravity)
    }

    /// Sets gravity in m/s². It must be finite and at most [`limits::MAX_ACCELERATION`] long;
    /// zero is allowed. Sleeping bodies stay asleep.
    pub fn set_gravity(&mut self, gravity: Vec3) -> Result<(), WorldError> {
        if !limits::is_acceleration(gravity) {
            return Err(WorldError::InvalidValue(limits::GRAVITY_RULE));
        }
        let gravity = gravity.to_jph();
        // SAFETY: the system is live and borrowed mutably; `gravity` is a live local.
        unsafe { JPH_PhysicsSystem_SetGravity(self.system.as_ptr(), &gravity) };
        Ok(())
    }

    /// The kind of job system the world steps with, as chosen in [`WorldSettings`]. Configuration,
    /// not part of a [`WorldState`](crate::WorldState).
    pub fn job_system_kind(&self) -> JobSystemKind {
        self.job_system_kind
    }

    /// The most jobs Jolt runs at the same time for this world, counting the thread that calls
    /// `step`: the value its job system was created with. Worker threads + 1 for Jolt's thread
    /// pool, 1 for the single-threaded job system, and the caller's
    /// [`JobSystem::max_concurrency`](crate::JobSystem::max_concurrency) as read in
    /// [`new`](Self::new).
    pub fn max_concurrency(&self) -> u32 {
        self.max_concurrency
    }

    /// Velocity iterations per step, as Jolt holds them ([`WorldSettings::velocity_steps`]).
    pub fn velocity_steps(&self) -> u32 {
        // SAFETY: the system is live; settings change only in `new`, before `self` exists.
        unsafe { physics_settings(self.system.as_ptr()) }.numVelocitySteps
    }

    /// Position iterations per step, as Jolt holds them ([`WorldSettings::position_steps`]).
    pub fn position_steps(&self) -> u32 {
        // SAFETY: as in `velocity_steps`.
        unsafe { physics_settings(self.system.as_ptr()) }.numPositionSteps
    }

    /// Penetration slop in metres, as Jolt holds it ([`WorldSettings::penetration_slop`]).
    pub fn penetration_slop(&self) -> f32 {
        // SAFETY: as in `velocity_steps`.
        unsafe { physics_settings(self.system.as_ptr()) }.penetrationSlop
    }

    /// Speculative contact distance in metres, as Jolt holds it
    /// ([`WorldSettings::speculative_contact_distance`]).
    pub fn speculative_contact_distance(&self) -> f32 {
        // SAFETY: as in `velocity_steps`.
        unsafe { physics_settings(self.system.as_ptr()) }.speculativeContactDistance
    }

    /// Maximum number of bodies the world holds, read from Jolt (`PhysicsSystem::GetMaxBodies`)
    /// ([`WorldSettings::max_bodies`]). Configuration, not part of a
    /// [`WorldState`](crate::WorldState).
    pub fn max_bodies(&self) -> u32 {
        // SAFETY: the system is live; the getter reads a constant.
        unsafe { JPH_PhysicsSystem_GetMaxBodies(self.system.as_ptr()) }
    }

    /// Maximum number of body pairs per collision step, as the world was created with
    /// ([`WorldSettings::max_body_pairs`]). Jolt has no getter for it, so the world keeps the
    /// value it passed. Configuration, not part of a [`WorldState`](crate::WorldState).
    pub fn max_body_pairs(&self) -> u32 {
        self.max_body_pairs
    }

    /// Maximum number of contact constraints per collision step, as the world was created with
    /// ([`WorldSettings::max_contact_constraints`]). Jolt has no getter for it, so the world keeps
    /// the value it passed. Configuration, not part of a [`WorldState`](crate::WorldState).
    pub fn max_contact_constraints(&self) -> u32 {
        self.max_contact_constraints
    }

    /// Size in bytes of the world's temp allocator, as the world was created with
    /// ([`WorldSettings::temp_allocator_size`]). Jolt has no getter for it, so the world keeps
    /// the value it passed. Configuration, not part of a [`WorldState`](crate::WorldState).
    pub fn temp_allocator_size(&self) -> u32 {
        self.temp_allocator_size
    }

    /// The collision layer setup the world was created with ([`WorldSettings::layers`]): the
    /// world's copy of the caller's setup, from which it built Jolt's layer tables.
    /// Configuration, not part of a [`WorldState`](crate::WorldState).
    pub fn layers(&self) -> &CollisionLayers {
        &self.layers
    }

    /// Number of bodies in the world.
    pub fn body_count(&self) -> u32 {
        // SAFETY: the system is live; Jolt counts under its own body mutex.
        unsafe { JPH_PhysicsSystem_GetNumBodies(self.system.as_ptr()) }
    }

    /// Whether the world has room for `count` more bodies. Jolt's `BodyManager::AddBody` fails
    /// only when the world holds `GetMaxBodies()` bodies, so creators check this before they
    /// change anything and a full world rejects them cleanly.
    pub(crate) fn has_room_for_bodies(&self, count: usize) -> bool {
        self.max_bodies()
            .checked_sub(self.body_count())
            .is_some_and(|room| count as u64 <= u64::from(room))
    }

    /// Rebuilds the broad phase's trees for fast queries.
    ///
    /// Queries see bodies created, moved and removed through this API immediately, without a
    /// step or a call to this method, and this method never changes which bodies a query finds.
    /// Bodies added one by one (a batch of chunks, say) leave the trees unbalanced, which makes
    /// queries slower until the next steps have rebuilt them; call this after such a batch to
    /// make queries fast at once. It is the explicit refresh point of the world: call it at the
    /// same point of every run, because it is part of the call history that determinism
    /// depends on.
    pub fn optimize_broad_phase(&mut self) {
        // SAFETY: the system is live and borrowed mutably, so no query or body change runs
        // meanwhile, as Jolt requires (`PhysicsSystem::OptimizeBroadPhase`).
        unsafe { JPH_PhysicsSystem_OptimizeBroadPhase(self.system.as_ptr()) };
    }

    /// Advances the world by `delta_time` seconds in one collision step.
    ///
    /// `delta_time` must be finite, at least [`MIN_DELTA_TIME`](Self::MIN_DELTA_TIME) and at
    /// most [`MAX_DELTA_TIME`](Self::MAX_DELTA_TIME), otherwise nothing happens and
    /// [`StepError::InvalidDeltaTime`] is returned. Every other call advances the world and
    /// returns a [`StepReport`]; check [`StepReport::is_complete`] to learn whether Jolt
    /// dropped work because a fixed-size buffer was full.
    /// [`step_with_collision_steps`](Self::step_with_collision_steps) splits a step into several
    /// collision steps.
    ///
    /// # Panics
    /// With a caller [`JobSystem`], a panic in its [`queue_job`](crate::JobSystem::queue_job) does not
    /// stop the step: the world advances, and `step` then resumes the first such panic. Jobs
    /// handed to the caller's job system before the panic may still run on its threads; the jobs
    /// Jolt queues after it are not handed over and run on the stepping thread. The next step
    /// uses the caller's job system again.
    ///
    /// A panic in an event callback (see [`set_event_settings`](Self::set_event_settings))
    /// does not stop the step either: `step` resumes it after the update, unless a `queue_job`
    /// panic came first. It also resumes a callback panic from between steps that
    /// [`take_events`](Self::take_events) has not resumed yet.
    ///
    /// A panic in a vehicle's tire max impulse callback (see
    /// [`VehicleMut::set_tire_max_impulse_callback`](crate::VehicleMut::set_tire_max_impulse_callback))
    /// does not stop the step either: `step` resumes it after the update, unless a `queue_job`
    /// or event callback panic came first.
    ///
    /// [`JobSystem`]: crate::JobSystem
    pub fn step(&mut self, delta_time: f32) -> Result<StepReport, StepError> {
        self.step_with_collision_steps(delta_time, 1)
    }

    /// Advances the world by `delta_time` seconds in `collision_steps` collision steps of
    /// `delta_time / collision_steps` each, inside one Jolt update. Each collision step runs
    /// collision detection, the solver and integration. Jolt's `PhysicsSystem::Update`: "The
    /// world steps for a total of inDeltaTime seconds. This is divided in inCollisionSteps
    /// iterations. Each iteration consists of collision detection followed by an integration
    /// step" ([The Simulation Step]).
    ///
    /// Use it to sub-step a fast scene instead of calling [`step`](Self::step) N times with
    /// `delta_time / N`: Jolt builds one job graph and the caller waits once. Separate calls let
    /// the caller change inputs between sub-steps, and they place force clearing, sleep checks
    /// and event batches at each call, so the two can give different results.
    ///
    /// One [`StepReport`] and one event batch cover the whole call; a report flag is set if any
    /// collision step dropped work.
    ///
    /// Forces and torques added to a rigid body act over all collision steps and are cleared
    /// after the last one. A force added to a soft body acts only in the first collision step,
    /// because Jolt clears it after each one. World gravity acts in every collision step.
    /// Constraint impulses (`total_lambda*`) are those of the last collision step: divide by
    /// `delta_time / collision_steps` for a force ([docs/constraints.md]).
    ///
    /// # Errors
    /// [`StepError::InvalidValue`] when `collision_steps` is 0 or above
    /// [`MAX_COLLISION_STEPS`](Self::MAX_COLLISION_STEPS); [`StepError::InvalidDeltaTime`] as for
    /// [`step`](Self::step), and also when a collision step would be shorter than
    /// [`MIN_DELTA_TIME`](Self::MIN_DELTA_TIME). On `Err` nothing happens.
    ///
    /// # Panics
    /// As [`step`](Self::step).
    ///
    /// # Example
    /// ```
    /// use oxijolt::prelude::math::*;
    /// use oxijolt::prelude::*;
    ///
    /// let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    /// let ball = Shape::new_sphere(0.5).unwrap();
    /// let settings = BodySettings::new_dynamic().position(RVec3::new(0.0, 5.0, 0.0));
    /// world.create_body(&ball, &settings).unwrap();
    /// // 1/30 s in two collision steps of 1/60 s.
    /// assert!(world.step_with_collision_steps(1.0 / 30.0, 2).unwrap().is_complete());
    /// ```
    ///
    /// [The Simulation Step]: https://jrouwe.github.io/JoltPhysicsDocs/5.6.0/index.html#the-simulation-step
    /// [docs/constraints.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/constraints.md#breakable-constraints
    pub fn step_with_collision_steps(
        &mut self,
        delta_time: f32,
        collision_steps: u32,
    ) -> Result<StepReport, StepError> {
        if !(1..=Self::MAX_COLLISION_STEPS).contains(&collision_steps) {
            return Err(StepError::InvalidValue(COLLISION_STEPS_RULE));
        }
        if !Self::is_valid_delta_time(delta_time)
            || delta_time / (collision_steps as f32) < Self::MIN_DELTA_TIME
        {
            return Err(StepError::InvalidDeltaTime);
        }
        self.listeners.begin_step();
        let simulates = self.simulates();
        if simulates {
            self.apply_cache_invalidations();
        }
        // SAFETY: the system, temp allocator and job system are live and owned by this world;
        // `&mut self` guarantees no other call uses them or touches a body during the update.
        let errors = unsafe {
            JPH_PhysicsSystem_Update2(
                self.system.as_ptr(),
                delta_time,
                // Bounded to `1..=MAX_COLLISION_STEPS` above.
                collision_steps as i32,
                self.temp_allocator.as_ptr(),
                self.job_system.as_ptr(),
            )
        };
        if simulates {
            self.pending_cache_invalidations.clear();
        }
        let job_panic = self.job_system.finish_update();
        let listeners = self.listeners.finish_step();
        let panic = first_payload(job_panic, listeners.panic);
        if let Some(payload) = first_payload(panic, self.take_tire_callback_panics()) {
            std::panic::resume_unwind(payload);
        }
        Ok(StepReport {
            manifold_cache_full: errors & JPH_PhysicsUpdateError_ManifoldCacheFull != 0,
            body_pair_cache_full: errors & JPH_PhysicsUpdateError_BodyPairCacheFull != 0,
            contact_constraints_full: errors & JPH_PhysicsUpdateError_ContactConstraintsFull != 0,
            rejected_contact_settings: listeners.rejected_contact_settings,
        })
    }
}

impl PhysicsWorld {
    /// Whether the next update runs Jolt's simulation pass, which consumes contact-cache
    /// invalidations; otherwise it returns early (`PhysicsSystem.cpp:188-207`). It mirrors Jolt's
    /// check: some rigid or soft body awake, or a step listener registered, and the vehicles are
    /// the only step listeners this crate registers.
    fn simulates(&self) -> bool {
        // SAFETY: the system is live and borrowed mutably; the getters count under Jolt's own
        // lock.
        let active = unsafe {
            JPH_PhysicsSystem_GetNumActiveBodies(self.system.as_ptr(), JPH_BodyType_Rigid)
                + JPH_PhysicsSystem_GetNumActiveBodies(self.system.as_ptr(), JPH_BodyType_Soft)
        };
        active > 0 || !self.vehicles.is_empty()
    }

    /// Sets Jolt's invalidation flag on every pending body, in id order.
    fn apply_cache_invalidations(&mut self) {
        for &raw in &self.pending_cache_invalidations {
            // SAFETY: the world is borrowed mutably and this thread holds no body lock. Jolt
            // ignores an id whose body is gone (`BodyLockWrite` fails,
            // `BodyInterface.cpp:1092-1097`).
            unsafe { JPH_BodyInterface_InvalidateContactCache(self.body_interface.as_ptr(), raw) };
        }
    }
}

/// What a [`PhysicsWorld::step`] or [`PhysicsWorld::step_with_collision_steps`] that ran reports.
/// The flags say which fixed-size buffer was full; [`is_complete`](Self::is_complete) says
/// what that means.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use = "a step may have dropped contacts; check `is_complete`"]
#[non_exhaustive]
pub struct StepReport {
    /// The contact manifold cache was full: too many contacts between bodies. Raise
    /// [`WorldSettings::max_contact_constraints`].
    pub manifold_cache_full: bool,
    /// The body pair cache was full: too many bodies touched. Raise
    /// [`WorldSettings::max_body_pairs`].
    pub body_pair_cache_full: bool,
    /// The contact constraint buffer was full. Raise
    /// [`WorldSettings::max_contact_constraints`].
    pub contact_constraints_full: bool,
    /// How many contact settings a [`ContactListener`](crate::ContactListener) returned in this
    /// step that did not fit their contact and were not applied; the world's events name them
    /// ([`WorldEvents::rejected_contact_settings`](crate::WorldEvents::rejected_contact_settings)).
    /// Jolt still resolved those contacts, so they do not make the step incomplete.
    pub rejected_contact_settings: u32,
}

impl StepReport {
    /// Whether the step ran without dropping any work. `false` means Jolt ignored some contacts
    /// because a fixed-size buffer was full in some collision step. The world is still valid and
    /// has advanced; raise the [`WorldSettings`] limit the set flag names. Rejected contact
    /// settings do not count.
    pub fn is_complete(&self) -> bool {
        !(self.manifold_cache_full || self.body_pair_cache_full || self.contact_constraints_full)
    }
}

#[cfg(test)]
mod tests;
