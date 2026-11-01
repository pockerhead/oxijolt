//! The settings a world is created with.

use std::fmt;
use std::sync::Arc;

use crate::limits;
use crate::{CollisionLayers, JobSystem, Vec3, WorldError};

/// Which job system a world runs its jobs on.
#[derive(Clone)]
pub(super) enum JobSystemChoice {
    /// Jolt's thread pool with this many worker threads.
    ThreadPool(u32),
    /// Jolt's single-threaded job system.
    SingleThreaded,
    /// The caller's job system.
    Caller(Arc<dyn JobSystem>),
}

impl fmt::Debug for JobSystemChoice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ThreadPool(workers) => f.debug_tuple("ThreadPool").field(workers).finish(),
            Self::SingleThreaded => f.write_str("SingleThreaded"),
            Self::Caller(_) => f.write_str("Caller(JobSystem)"),
        }
    }
}

/// The kind of job system a world runs its jobs on ([`PhysicsWorld::job_system_kind`],
/// [`WorldSettings::job_system_kind`]), chosen
/// with [`WorldSettings::worker_threads`], [`WorldSettings::single_threaded`] or
/// [`WorldSettings::job_system`].
///
/// [`PhysicsWorld::job_system_kind`]: crate::PhysicsWorld::job_system_kind
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
#[non_exhaustive]
pub enum JobSystemKind {
    /// Jolt's `JobSystemThreadPool` with this many worker threads.
    ThreadPool {
        /// Worker threads, not counting the thread that calls `step`.
        worker_threads: u32,
    },
    /// Jolt's `JobSystemSingleThreaded`: every job runs on the thread that calls `step`.
    SingleThreaded,
    /// The caller's [`JobSystem`].
    Caller,
}

impl JobSystemChoice {
    /// The kind of this choice, without the caller's job system.
    pub(super) fn kind(&self) -> JobSystemKind {
        match self {
            Self::ThreadPool(workers) => JobSystemKind::ThreadPool {
                worker_threads: *workers,
            },
            Self::SingleThreaded => JobSystemKind::SingleThreaded,
            Self::Caller(_) => JobSystemKind::Caller,
        }
    }

    /// Checks the worker count of Jolt's thread pool, which a target without threads refuses.
    pub(super) fn validate(&self, has_threads: bool) -> Result<(), WorldError> {
        let Self::ThreadPool(workers) = self else {
            return Ok(());
        };
        if !has_threads {
            return Err(WorldError::InvalidValue(
                "worker_threads needs threads; use single_threaded on wasm32",
            ));
        }
        if !(1..=WorldSettings::MAX_WORKER_THREADS).contains(workers) {
            return Err(WorldError::InvalidValue(
                "worker_threads must be between 1 and 64",
            ));
        }
        Ok(())
    }

    /// The most jobs Jolt will run at the same time, counting the stepping thread. Reads the
    /// caller's [`JobSystem::max_concurrency`] once and checks it.
    pub(super) fn max_concurrency(&self) -> Result<u32, WorldError> {
        match self {
            Self::ThreadPool(workers) => Ok(workers + 1),
            Self::SingleThreaded => Ok(1),
            Self::Caller(job_system) => {
                let value = job_system.max_concurrency();
                if (1..=WorldSettings::MAX_CONCURRENCY).contains(&value) {
                    Ok(value)
                } else {
                    Err(WorldError::InvalidValue(
                        "job system max_concurrency must be between 1 and 65",
                    ))
                }
            }
        }
    }
}

/// Settings for [`PhysicsWorld::new`]. The defaults are Jolt's and joltc's, except that on
/// wasm32 a world runs its jobs with [`single_threaded`](Self::single_threaded).
///
/// [`PhysicsWorld::new`]: crate::PhysicsWorld::new
#[derive(Clone, Debug)]
pub struct WorldSettings {
    pub(super) max_bodies: u32,
    pub(super) max_body_pairs: u32,
    pub(super) max_contact_constraints: u32,
    pub(super) jobs: JobSystemChoice,
    pub(super) temp_allocator_size: u32,
    pub(super) gravity: Vec3,
    pub(super) layers: CollisionLayers,
    pub(super) velocity_steps: u32,
    pub(super) position_steps: u32,
    pub(super) penetration_slop: f32,
    pub(super) speculative_contact_distance: f32,
}

/// Jolt's thread pool with one worker, or on wasm32, which has no threads, Jolt's single-threaded
/// job system.
const DEFAULT_JOBS: JobSystemChoice = if HAS_THREADS {
    JobSystemChoice::ThreadPool(1)
} else {
    JobSystemChoice::SingleThreaded
};

/// Whether the target can start threads for Jolt's thread pool.
const HAS_THREADS: bool = cfg!(not(target_arch = "wasm32"));

impl Default for WorldSettings {
    fn default() -> Self {
        Self {
            max_bodies: 10240,
            max_body_pairs: 65536,
            max_contact_constraints: 10240,
            jobs: DEFAULT_JOBS,
            temp_allocator_size: 10 * 1024 * 1024,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            layers: CollisionLayers::default(),
            velocity_steps: 10,
            position_steps: 2,
            penetration_slop: 0.02,
            speculative_contact_distance: 0.02,
        }
    }
}

impl WorldSettings {
    /// Largest accepted [`max_bodies`](Self::max_bodies) value, 2²³: Jolt's own limit
    /// (`PhysicsSystem::cMaxBodiesLimit`).
    pub const MAX_BODIES: u32 = 1 << 23;

    /// Largest accepted [`worker_threads`](Self::worker_threads) value. Jolt starts one OS
    /// thread per worker; this bound is chosen by oxijolt to keep thread creation sane and
    /// is not a Jolt limit.
    pub const MAX_WORKER_THREADS: u32 = 64;

    /// Largest accepted [`JobSystem::max_concurrency`] of a [`job_system`](Self::job_system):
    /// the concurrency of Jolt's thread pool at [`MAX_WORKER_THREADS`](Self::MAX_WORKER_THREADS)
    /// (the workers and the stepping thread). A oxijolt bound, not a Jolt limit: Jolt splits
    /// a stage into at most 32 jobs, and 0 or a value above `i32::MAX` would break its `int`
    /// arithmetic.
    pub const MAX_CONCURRENCY: u32 = Self::MAX_WORKER_THREADS + 1;

    /// Largest accepted [`max_contact_constraints`](Self::max_contact_constraints) value: 2²⁰.
    ///
    /// A oxijolt bound below Jolt's own limit (`ContactConstraintManager::
    /// cMaxContactConstraintsLimit`, above which Jolt asserts), which a native compile-time check
    /// pins.
    pub const MAX_CONTACT_CONSTRAINTS: u32 = 1 << 20;

    /// Smallest accepted [`velocity_steps`](Self::velocity_steps) value, 2: Jolt applies friction
    /// with the contact impulse of the previous velocity iteration, so friction needs two
    /// (`PhysicsSettings.h:83`). See [docs/limits.md].
    ///
    /// [docs/limits.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#solver-step-counts
    pub const MIN_VELOCITY_STEPS: u32 = 2;

    /// Largest accepted [`velocity_steps`](Self::velocity_steps) and
    /// [`position_steps`](Self::position_steps) value, 255: Jolt keeps step counts in 8 bits
    /// (`IslandBuilder.cpp:385`, `PhysicsSystem.cpp:1545-1546`, `MotionProperties.h:192,196`).
    /// See [docs/limits.md].
    ///
    /// [docs/limits.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#solver-step-counts
    pub const MAX_SOLVER_STEPS: u32 = 255;

    /// Maximum number of bodies in the world, between 1 and [`MAX_BODIES`](Self::MAX_BODIES).
    /// Default 10240. Raising it does not raise [`max_body_pairs`](Self::max_body_pairs) or
    /// [`max_contact_constraints`](Self::max_contact_constraints): size those separately for the
    /// contacts the scene is expected to have.
    #[must_use]
    pub fn max_bodies(mut self, value: u32) -> Self {
        self.max_bodies = value;
        self
    }

    /// Maximum number of body pairs whose bounding boxes overlap that the broad phase can hand to
    /// the narrow phase per collision step, at least 1. Default 65536. It does not grow with
    /// [`max_bodies`](Self::max_bodies); size it for the expected overlapping pairs. A full buffer
    /// sets [`StepReport::body_pair_cache_full`].
    ///
    /// [`StepReport::body_pair_cache_full`]: crate::StepReport::body_pair_cache_full
    #[must_use]
    pub fn max_body_pairs(mut self, value: u32) -> Self {
        self.max_body_pairs = value;
        self
    }

    /// Maximum number of contact constraints per collision step, at least 1 and at most
    /// [`MAX_CONTACT_CONSTRAINTS`](Self::MAX_CONTACT_CONSTRAINTS). Default 10240. Jolt makes one
    /// per contact manifold, so a body pair touching with several sub-shapes takes several. It
    /// does not grow with [`max_bodies`](Self::max_bodies). A full buffer sets
    /// [`StepReport::contact_constraints_full`] or [`StepReport::manifold_cache_full`].
    ///
    /// [`StepReport::contact_constraints_full`]: crate::StepReport::contact_constraints_full
    /// [`StepReport::manifold_cache_full`]: crate::StepReport::manifold_cache_full
    #[must_use]
    pub fn max_contact_constraints(mut self, value: u32) -> Self {
        self.max_contact_constraints = value;
        self
    }

    /// Runs the world's jobs on Jolt's thread pool with `value` worker threads, in addition to
    /// the thread that calls [`PhysicsWorld::step`], which also runs jobs. At least 1 and at most
    /// [`WorldSettings::MAX_WORKER_THREADS`]. Default 1. The determinism tests get bit-identical
    /// results with 1 and 4 workers on one machine ([docs/determinism.md]). 0 is refused;
    /// [`single_threaded`](Self::single_threaded) runs without worker threads.
    ///
    /// On wasm32 there are no threads: [`PhysicsWorld::new`] refuses any value with
    /// [`WorldError::InvalidValue`], and the default there is
    /// [`single_threaded`](Self::single_threaded).
    ///
    /// Of this, [`single_threaded`](Self::single_threaded) and [`job_system`](Self::job_system),
    /// the call made last decides.
    ///
    /// [docs/determinism.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/determinism.md
    /// [`PhysicsWorld::new`]: crate::PhysicsWorld::new
    /// [`PhysicsWorld::step`]: crate::PhysicsWorld::step
    #[must_use]
    pub fn worker_threads(mut self, value: u32) -> Self {
        self.jobs = JobSystemChoice::ThreadPool(value);
        self
    }

    /// Runs the world's jobs with Jolt's single-threaded job system (`JobSystemSingleThreaded`):
    /// every job runs on the thread that calls [`PhysicsWorld::step`] as soon as it is ready, with
    /// concurrency 1, and Jolt starts no threads for the world. The determinism tests get the same
    /// results as with Jolt's thread pool ([docs/determinism.md]).
    ///
    /// Of this, [`worker_threads`](Self::worker_threads) and [`job_system`](Self::job_system), the
    /// call made last decides.
    ///
    /// [docs/determinism.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/determinism.md
    /// [`PhysicsWorld::step`]: crate::PhysicsWorld::step
    #[must_use]
    pub fn single_threaded(mut self) -> Self {
        self.jobs = JobSystemChoice::SingleThreaded;
        self
    }

    /// Runs the world's jobs on the caller's `value` instead of Jolt's thread pool; Jolt then
    /// starts no threads for the world. The world keeps the `Arc` as long as it lives, and
    /// [`PhysicsWorld::new`] reads [`JobSystem::max_concurrency`] once, which must be within
    /// `1..=`[`WorldSettings::MAX_CONCURRENCY`].
    ///
    /// Of this, [`worker_threads`](Self::worker_threads) and
    /// [`single_threaded`](Self::single_threaded), the call made last decides.
    ///
    /// [`PhysicsWorld::new`]: crate::PhysicsWorld::new
    #[must_use]
    pub fn job_system(mut self, value: Arc<dyn JobSystem>) -> Self {
        self.jobs = JobSystemChoice::Caller(value);
        self
    }

    /// The kind of job system these settings choose: the last of
    /// [`worker_threads`](Self::worker_threads), [`single_threaded`](Self::single_threaded) and
    /// [`job_system`](Self::job_system) called, or Jolt's thread pool with one worker
    /// (`ThreadPool { worker_threads: 1 }`) when none was; on wasm32 the default is
    /// `SingleThreaded`. Settings do not record whether a
    /// choice was made; to tell whether it differs from the default, compare with
    /// `WorldSettings::default().job_system_kind()`. A caller job system reads as
    /// [`JobSystemKind::Caller`] without a call to its [`JobSystem::max_concurrency`].
    pub fn job_system_kind(&self) -> JobSystemKind {
        self.jobs.kind()
    }

    /// Size in bytes of the scratch memory the world's steps allocate from. Default 10 MiB.
    #[must_use]
    pub fn temp_allocator_size(mut self, value: u32) -> Self {
        self.temp_allocator_size = value;
        self
    }

    /// Gravity in m/s², finite and at most [`limits::MAX_ACCELERATION`] long. Default
    /// `(0, -9.81, 0)`, Jolt's default. Zero is allowed.
    #[must_use]
    pub fn gravity(mut self, value: Vec3) -> Self {
        self.gravity = value;
        self
    }

    /// The collision layer setup. Default [`CollisionLayers::default`].
    #[must_use]
    pub fn layers(mut self, value: CollisionLayers) -> Self {
        self.layers = value;
        self
    }

    /// Velocity iterations of the contact and constraint solver per step (Jolt
    /// `PhysicsSettings::mNumVelocitySteps`), between
    /// [`MIN_VELOCITY_STEPS`](Self::MIN_VELOCITY_STEPS) and
    /// [`MAX_SOLVER_STEPS`](Self::MAX_SOLVER_STEPS). Default 10, Jolt's. Fewer steps make a step
    /// cheaper and joints and stacks softer. Set once, when the world is created; a test checks
    /// that a chain at 4 steps gives the same results with 1 and 4 worker threads.
    #[must_use]
    pub fn velocity_steps(mut self, value: u32) -> Self {
        self.velocity_steps = value;
        self
    }

    /// Position iterations of the solver per step, which push apart what still penetrates or
    /// drifted after the velocity iterations (Jolt `PhysicsSettings::mNumPositionSteps`), at most
    /// [`MAX_SOLVER_STEPS`](Self::MAX_SOLVER_STEPS); 0 turns the position pass off. Default 2,
    /// Jolt's. Set once, when the world is created.
    #[must_use]
    pub fn position_steps(mut self, value: u32) -> Self {
        self.position_steps = value;
        self
    }

    /// How far, in metres, two bodies may overlap before Jolt's position iterations push them
    /// apart (Jolt `PhysicsSettings::mPenetrationSlop`). It is a tolerance Jolt leaves
    /// uncorrected, not a depth bodies are pushed to: an overlap within it is left as it is, and
    /// only the part beyond it is corrected, by the [`position_steps`](Self::position_steps) of
    /// each step (not at all with 0 position steps). Smaller values let resting bodies sink less
    /// into each other, at the cost of contacts that break and re-form, which makes stacks
    /// jitter. It also caps how far a
    /// [`MotionQuality::LinearCast`](crate::MotionQuality::LinearCast) body may move into
    /// another before its continuous collision check stops it.
    ///
    /// Finite and between 0 and [`limits::MAX_SHAPE_EXTENT`]; [`PhysicsWorld::new`] refuses
    /// other values with [`WorldError::InvalidValue`]. Default 0.02 m, Jolt's. Set once, when the
    /// world is created. See [docs/limits.md].
    ///
    /// [`PhysicsWorld::new`]: crate::PhysicsWorld::new
    /// [docs/limits.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#contact-distances
    #[must_use]
    pub fn penetration_slop(mut self, value: f32) -> Self {
        self.penetration_slop = value;
        self
    }

    /// The distance, in metres, within which Jolt creates contacts between bodies that do not
    /// touch yet, so the solver can slow them before they hit (Jolt
    /// `PhysicsSettings::mSpeculativeContactDistance`). Such contacts are reported with a
    /// negative [`ContactManifold::penetration_depth`](crate::ContactManifold::penetration_depth).
    /// Larger values catch fast bodies earlier but pair more bodies in the broad phase and can
    /// stop a body at a contact it would have missed (a ghost collision); 0 removes the margin.
    ///
    /// Finite and between 0 and [`limits::MAX_SHAPE_EXTENT`]; [`PhysicsWorld::new`] refuses
    /// other values with [`WorldError::InvalidValue`]. Default 0.02 m, Jolt's. Set once, when the
    /// world is created. See [docs/limits.md].
    ///
    /// [`PhysicsWorld::new`]: crate::PhysicsWorld::new
    /// [docs/limits.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#contact-distances
    #[must_use]
    pub fn speculative_contact_distance(mut self, value: f32) -> Self {
        self.speculative_contact_distance = value;
        self
    }

    pub(crate) fn validate(&self) -> Result<(), WorldError> {
        let invalid = |what| Err(WorldError::InvalidValue(what));
        if !(1..=Self::MAX_BODIES).contains(&self.max_bodies) {
            return invalid("max_bodies must be between 1 and 2^23");
        }
        if self.max_body_pairs == 0 {
            return invalid("max_body_pairs must be at least 1");
        }
        if !(1..=Self::MAX_CONTACT_CONSTRAINTS).contains(&self.max_contact_constraints) {
            return invalid("max_contact_constraints must be between 1 and 2^20");
        }
        self.jobs.validate(HAS_THREADS)?;
        if self.temp_allocator_size == 0 {
            return invalid("temp_allocator_size must be at least 1");
        }
        if !limits::is_acceleration(self.gravity) {
            return invalid(limits::GRAVITY_RULE);
        }
        if !(Self::MIN_VELOCITY_STEPS..=Self::MAX_SOLVER_STEPS).contains(&self.velocity_steps) {
            return invalid("velocity_steps must be between 2 and 255");
        }
        if self.position_steps > Self::MAX_SOLVER_STEPS {
            return invalid("position_steps must be at most 255");
        }
        if !limits::is_local_distance(self.penetration_slop) {
            return invalid("penetration_slop must be between 0 and limits::MAX_SHAPE_EXTENT");
        }
        if !limits::is_local_distance(self.speculative_contact_distance) {
            return invalid(
                "speculative_contact_distance must be between 0 and limits::MAX_SHAPE_EXTENT",
            );
        }
        self.layers.validate()
    }
}
