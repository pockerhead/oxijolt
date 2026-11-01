//! Worlds that run Jolt's jobs on a caller job system or Jolt's single-threaded job system:
//! results equal to Jolt's thread pool, jobs run inside `queue_job`, re-entrant steps, jobs that
//! outlive their world, panics in the caller's `queue_job`, the concurrency bound, Jolt's job
//! pool at the largest collision step count, the choice between the job systems and reading it
//! back from the settings and the world.

mod common;

use std::any::Any;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{mpsc, Arc, Barrier, Mutex};
use std::thread;
use std::time::Duration;

use common::jobs::{InlineJobs, RayonJobs};
use common::ragdoll as humanoid;
use common::*;
use oxijolt::*;

/// Ticks of the stacks scene.
const TICKS: usize = 120;

/// Seconds a test that could deadlock may take before it fails.
const TIMEOUT_SECS: u64 = 300;

/// Runs `f` on its own thread and returns its result; panics if `f` panics or takes longer
/// than `secs` seconds, so that a deadlock fails the test instead of hanging the run.
fn with_timeout<T: Send + 'static>(secs: u64, f: impl FnOnce() -> T + Send + 'static) -> T {
    let (sender, receiver) = mpsc::channel();
    let handle = thread::spawn(move || {
        let _ = sender.send(f());
    });
    match receiver.recv_timeout(Duration::from_secs(secs)) {
        Ok(value) => {
            handle.join().unwrap();
            value
        }
        Err(mpsc::RecvTimeoutError::Timeout) => {
            panic!("no result after {secs} s: the step is probably deadlocked")
        }
        Err(mpsc::RecvTimeoutError::Disconnected) => {
            std::panic::resume_unwind(handle.join().unwrap_err())
        }
    }
}

/// Counts the jobs handed to the job system it wraps.
struct Counting<J> {
    inner: J,
    queued: AtomicUsize,
}

impl<J> Counting<J> {
    fn new(inner: J) -> Self {
        Self {
            inner,
            queued: AtomicUsize::new(0),
        }
    }

    fn queued(&self) -> usize {
        self.queued.load(Ordering::Relaxed)
    }
}

impl<J: JobSystem> JobSystem for Counting<J> {
    fn max_concurrency(&self) -> u32 {
        self.inner.max_concurrency()
    }

    fn queue_job(&self, job: Job) {
        self.queued.fetch_add(1, Ordering::Relaxed);
        self.inner.queue_job(job);
    }
}

/// Keeps every job until the test takes them.
#[derive(Default)]
struct DeferredJobs {
    jobs: Mutex<Vec<Job>>,
}

impl DeferredJobs {
    fn take(&self) -> Vec<Job> {
        std::mem::take(&mut *self.jobs.lock().unwrap())
    }
}

impl JobSystem for DeferredJobs {
    fn max_concurrency(&self) -> u32 {
        4
    }

    fn queue_job(&self, job: Job) {
        self.jobs.lock().unwrap().push(job);
    }
}

/// When a [`PanickingJobs`] panics.
#[derive(Clone, Copy, Debug)]
enum PanicMode {
    /// On the n-th call, while still holding the job, which unwinding then drops.
    BeforeStoring(usize),
    /// On the n-th call, after storing the job for the test to drain.
    AfterStoring(usize),
    /// On every call while armed.
    Always,
}

/// Calls `Job::run` right away, except that it panics as its mode says.
struct PanickingJobs {
    mode: PanicMode,
    armed: AtomicBool,
    calls: AtomicUsize,
    stored: Mutex<Vec<Job>>,
}

impl PanickingJobs {
    fn new(mode: PanicMode) -> Self {
        Self {
            mode,
            armed: AtomicBool::new(true),
            calls: AtomicUsize::new(0),
            stored: Mutex::new(Vec::new()),
        }
    }

    fn calls(&self) -> usize {
        self.calls.load(Ordering::Relaxed)
    }
}

impl JobSystem for PanickingJobs {
    fn max_concurrency(&self) -> u32 {
        3
    }

    fn queue_job(&self, job: Job) {
        let call = self.calls.fetch_add(1, Ordering::Relaxed) + 1;
        match self.mode {
            PanicMode::BeforeStoring(n) if call == n => {
                panic!("injected panic at call {call}")
            }
            PanicMode::AfterStoring(n) if call == n => {
                self.stored.lock().unwrap().push(job);
                panic!("injected panic at call {call}")
            }
            PanicMode::Always if self.armed.load(Ordering::Relaxed) => {
                panic!("injected panic at call {call}")
            }
            _ => job.run(),
        }
    }
}

/// Reads the job system's concurrency from a field and counts the reads; calls `Job::run` right
/// away.
struct FixedConcurrency {
    value: u32,
    reads: AtomicUsize,
}

impl JobSystem for FixedConcurrency {
    fn max_concurrency(&self) -> u32 {
        self.reads.fetch_add(1, Ordering::Relaxed);
        self.value
    }

    fn queue_job(&self, job: Job) {
        job.run();
    }
}

fn panic_message(payload: &(dyn Any + Send)) -> String {
    payload
        .downcast_ref::<String>()
        .cloned()
        .unwrap_or_default()
}

/// The stacks scene in a world with `settings`: per-tick records of every body.
fn stacks_digest(settings: WorldSettings) -> Vec<u8> {
    let mut world = PhysicsWorld::new(settings).unwrap();
    let ids = build_stacks(&mut world);
    run_digest(&mut world, &ids, TICKS)
}

fn native() -> WorldSettings {
    WorldSettings::default().worker_threads(1)
}

#[test]
fn rayon_job_system_matches_the_native_pool() {
    let jobs = Arc::new(Counting::new(RayonJobs::new(3)));
    let caller = stacks_digest(WorldSettings::default().job_system(jobs.clone()));
    assert_eq!(caller, stacks_digest(native()));
    assert!(jobs.queued() > 0);
}

/// Sixteen humanoid ragdolls lying side by side on a floor, touching, for 120 ticks: per-tick
/// records of every part.
fn ragdoll_pile_digest(settings: WorldSettings) -> Vec<u8> {
    let (layers, ids) = humanoid::ragdoll_layers();
    let mut world = PhysicsWorld::new(settings.layers(layers)).unwrap();
    let floor = Shape::new_box(Vec3::new(20.0, 1.0, 20.0)).unwrap();
    world
        .create_body(
            &floor,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(ids.fixed),
        )
        .unwrap();
    let face_up = quat_about(humanoid::X, -std::f32::consts::FRAC_PI_2);
    let ragdoll_settings = humanoid::humanoid_settings(ids.ragdoll);
    let mut parts = Vec::new();
    for key in 0..16 {
        let (i, j) = ((key % 4) as f64 - 1.5, (key / 4) as f64 - 1.5);
        // Row and column spacing let hands and head and feet of neighbours overlap slightly.
        let at = [i * 1.34, 0.63, j * 1.834];
        let pose = humanoid::transformed_pose(&humanoid::bind_pose(), face_up, at);
        let ragdoll = world
            .create_ragdoll(&ragdoll_settings, Some(&pose), Activation::Activate)
            .unwrap();
        parts.extend_from_slice(world.ragdoll(ragdoll).unwrap().body_ids());
    }
    run_digest(&mut world, &parts, TICKS)
}

#[test]
fn inline_job_system_matches_the_native_pool() {
    let jobs = Arc::new(Counting::new(InlineJobs { concurrency: 3 }));
    let caller = stacks_digest(WorldSettings::default().job_system(jobs.clone()));
    assert_eq!(caller, stacks_digest(native()));
    assert!(jobs.queued() > 0);

    // Every job is left to the stepping thread, also in a pile whose large islands Jolt splits
    // into many jobs.
    let pile_jobs = Arc::new(InlineJobs { concurrency: 3 });
    let caller = with_timeout(TIMEOUT_SECS, move || {
        ragdoll_pile_digest(WorldSettings::default().job_system(pile_jobs))
    });
    assert_eq!(caller, ragdoll_pile_digest(native()));
}

#[test]
fn step_from_the_only_thread_of_a_rayon_pool_completes() {
    let caller = with_timeout(TIMEOUT_SECS, || {
        let jobs = Arc::new(RayonJobs::new(1));
        let mut world =
            PhysicsWorld::new(WorldSettings::default().job_system(jobs.clone())).unwrap();
        let ids = build_stacks(&mut world);
        jobs.pool.install(|| run_digest(&mut world, &ids, TICKS))
    });
    assert_eq!(caller, stacks_digest(native()));
}

/// A world on `jobs` with a floor and four awake cubes.
fn cubes_world(jobs: Arc<dyn JobSystem>) -> PhysicsWorld {
    let mut world = PhysicsWorld::new(WorldSettings::default().job_system(jobs)).unwrap();
    add_floor(&mut world);
    for i in 0..4 {
        add_cube(&mut world, RVec3::new(1.5 * i as Real, 2.0, 0.0));
    }
    world
}

#[test]
fn jobs_may_outlive_their_world() {
    let jobs = Arc::new(DeferredJobs::default());
    let mut world = cubes_world(jobs.clone());
    step(&mut world, 2);
    drop(world);

    let mut deferred = jobs.take();
    assert!(deferred.len() > 1);
    let to_run = deferred.split_off(deferred.len() / 2);
    drop(deferred);
    for job in to_run {
        job.run();
    }
}

#[test]
fn a_deferred_job_may_finish_on_another_thread_while_the_world_drops() {
    let jobs = Arc::new(DeferredJobs::default());
    let mut world = cubes_world(jobs.clone());
    step(&mut world, 2);
    let deferred = jobs.take();
    assert!(!deferred.is_empty());

    let start = Arc::new(Barrier::new(2));
    let runner = {
        let start = Arc::clone(&start);
        thread::spawn(move || {
            start.wait();
            for job in deferred {
                job.run();
            }
        })
    };
    start.wait();
    drop(world);
    runner.join().unwrap();
}

#[test]
fn a_job_system_that_keeps_every_job_does_not_run_out_of_jolt_jobs() {
    // Jolt has 2048 job slots per world; jobs the pool keeps past their step must not hold them.
    let queued = with_timeout(TIMEOUT_SECS, || {
        let jobs = Arc::new(Counting::new(DeferredJobs::default()));
        let mut world =
            PhysicsWorld::new(WorldSettings::default().job_system(jobs.clone())).unwrap();
        add_floor(&mut world);
        let shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
        for i in 0..8 {
            let settings = BodySettings::new_dynamic()
                .position(RVec3::new(1.5 * i as Real, 2.0, 0.0))
                .allow_sleeping(false);
            world.create_body(&shape, &settings).unwrap();
        }
        step(&mut world, 600);
        drop(world);
        jobs.inner.take();
        jobs.queued()
    });
    assert!(queued > 3 * 2048, "{queued}");
}

/// The step that panics, the drained jobs and 30 more steps; the result must equal a native
/// world stepped as often.
fn panicking_job_system_case(mode: PanicMode) {
    let jobs = Arc::new(PanickingJobs::new(mode));
    let mut world = PhysicsWorld::new(WorldSettings::default().job_system(jobs.clone())).unwrap();
    let ids = build_stacks(&mut world);

    let payload = catch_unwind(AssertUnwindSafe(|| world.step(DT))).unwrap_err();
    let message = panic_message(&*payload);
    match mode {
        PanicMode::BeforeStoring(n) | PanicMode::AfterStoring(n) => {
            assert_eq!(message, format!("injected panic at call {n}"), "{mode:?}");
        }
        PanicMode::Always => assert!(message.starts_with("injected panic"), "{message}"),
    }
    jobs.armed.store(false, Ordering::Relaxed);
    for job in std::mem::take(&mut *jobs.stored.lock().unwrap()) {
        job.run();
    }

    let calls = jobs.calls();
    step(&mut world, 30);
    assert!(
        jobs.calls() > calls,
        "{mode:?}: the job system was not used again"
    );

    let mut native_world = PhysicsWorld::new(native()).unwrap();
    let native_ids = build_stacks(&mut native_world);
    step(&mut native_world, 31);
    let mut state = Vec::new();
    let mut native_state = Vec::new();
    for (&id, &native_id) in ids.iter().zip(&native_ids) {
        record_body(&world, id, &mut state);
        record_body(&native_world, native_id, &mut native_state);
    }
    assert_eq!(state, native_state, "{mode:?}");
}

#[test]
fn a_panicking_job_system_panics_out_of_step_and_the_world_stays_usable() {
    for mode in [
        PanicMode::BeforeStoring(3),
        PanicMode::AfterStoring(3),
        PanicMode::Always,
    ] {
        panicking_job_system_case(mode);
    }
}

#[test]
fn max_concurrency_is_bounded() {
    for value in [0, WorldSettings::MAX_CONCURRENCY + 1, u32::MAX] {
        let jobs = Arc::new(FixedConcurrency {
            value,
            reads: AtomicUsize::new(0),
        });
        let result = PhysicsWorld::new(WorldSettings::default().job_system(jobs.clone()));
        assert!(
            matches!(result, Err(WorldError::InvalidValue(_))),
            "{value}"
        );
        assert_eq!(jobs.reads.load(Ordering::Relaxed), 1, "{value}");
    }
    for value in [1, WorldSettings::MAX_CONCURRENCY] {
        let jobs = Arc::new(FixedConcurrency {
            value,
            reads: AtomicUsize::new(0),
        });
        let mut world = cubes_world(jobs.clone());
        step(&mut world, 10);
        assert_eq!(jobs.reads.load(Ordering::Relaxed), 1, "{value}");
    }
}

#[test]
fn a_thread_pool_world_reads_back_its_worker_threads_and_concurrency() {
    for workers in [1, 2, 4, WorldSettings::MAX_WORKER_THREADS] {
        let world = PhysicsWorld::new(WorldSettings::default().worker_threads(workers)).unwrap();
        assert_eq!(
            world.job_system_kind(),
            JobSystemKind::ThreadPool {
                worker_threads: workers
            }
        );
        assert_eq!(world.max_concurrency(), workers + 1);
    }
    let world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    assert_eq!(
        world.job_system_kind(),
        JobSystemKind::ThreadPool { worker_threads: 1 }
    );
    assert_eq!(world.max_concurrency(), 2);
}

#[test]
fn a_single_threaded_world_reads_back_its_kind_and_concurrency_one() {
    let world = PhysicsWorld::new(single_threaded()).unwrap();
    assert_eq!(world.job_system_kind(), JobSystemKind::SingleThreaded);
    assert_eq!(world.max_concurrency(), 1);

    let world = PhysicsWorld::new(single_threaded().worker_threads(4)).unwrap();
    assert_eq!(
        world.job_system_kind(),
        JobSystemKind::ThreadPool { worker_threads: 4 }
    );
    assert_eq!(world.max_concurrency(), 5);
}

#[test]
fn a_caller_job_system_world_reads_back_the_concurrency_read_at_creation() {
    let jobs = Arc::new(FixedConcurrency {
        value: 7,
        reads: AtomicUsize::new(0),
    });
    let mut world = cubes_world(jobs.clone());
    step(&mut world, 10);
    assert_eq!(world.job_system_kind(), JobSystemKind::Caller);
    assert_eq!(world.max_concurrency(), 7);
    assert_eq!(jobs.reads.load(Ordering::Relaxed), 1);
}

#[test]
fn default_settings_read_back_a_thread_pool_with_one_worker() {
    assert_eq!(
        WorldSettings::default().job_system_kind(),
        JobSystemKind::ThreadPool { worker_threads: 1 }
    );
}

#[test]
fn settings_read_back_the_thread_pool_and_single_threaded_choices() {
    for workers in [1, 2, WorldSettings::MAX_WORKER_THREADS] {
        assert_eq!(
            WorldSettings::default()
                .worker_threads(workers)
                .job_system_kind(),
            JobSystemKind::ThreadPool {
                worker_threads: workers
            }
        );
    }
    assert_eq!(
        single_threaded().job_system_kind(),
        JobSystemKind::SingleThreaded
    );
}

#[test]
fn settings_read_back_a_caller_job_system_without_calling_it() {
    let jobs = Arc::new(FixedConcurrency {
        value: 7,
        reads: AtomicUsize::new(0),
    });
    let settings = WorldSettings::default().job_system(jobs.clone());
    assert_eq!(settings.job_system_kind(), JobSystemKind::Caller);
    assert_eq!(jobs.reads.load(Ordering::Relaxed), 0);
}

#[test]
fn settings_read_back_the_last_job_system_choice() {
    let jobs: Arc<dyn JobSystem> = Arc::new(InlineJobs { concurrency: 3 });
    let pool = JobSystemKind::ThreadPool { worker_threads: 2 };
    let cases = [
        (single_threaded().worker_threads(2), pool),
        (
            WorldSettings::default()
                .job_system(jobs.clone())
                .worker_threads(2),
            pool,
        ),
        (
            WorldSettings::default().worker_threads(2).single_threaded(),
            JobSystemKind::SingleThreaded,
        ),
        (
            WorldSettings::default()
                .job_system(jobs.clone())
                .single_threaded(),
            JobSystemKind::SingleThreaded,
        ),
        (
            WorldSettings::default()
                .worker_threads(2)
                .job_system(jobs.clone()),
            JobSystemKind::Caller,
        ),
        (single_threaded().job_system(jobs), JobSystemKind::Caller),
    ];
    for (settings, kind) in cases {
        assert_eq!(settings.job_system_kind(), kind, "{settings:?}");
    }
}

#[test]
fn worlds_sharing_one_job_system_step_in_parallel() {
    let digests = with_timeout(TIMEOUT_SECS, || {
        let jobs: Arc<dyn JobSystem> = Arc::new(RayonJobs::new(2));
        thread::scope(|scope| {
            let runs: Vec<_> = (0..2)
                .map(|_| {
                    let jobs = Arc::clone(&jobs);
                    scope.spawn(move || stacks_digest(WorldSettings::default().job_system(jobs)))
                })
                .collect();
            runs.into_iter()
                .map(|run| run.join().unwrap())
                .collect::<Vec<_>>()
        })
    });
    let native = stacks_digest(native());
    for digest in digests {
        assert_eq!(digest, native);
    }
}

#[test]
fn the_last_of_job_system_and_worker_threads_wins() {
    let jobs = Arc::new(Counting::new(InlineJobs { concurrency: 3 }));
    let settings = WorldSettings::default()
        .job_system(jobs.clone())
        .worker_threads(2);
    assert!(
        format!("{settings:?}").contains("ThreadPool(2)"),
        "{settings:?}"
    );
    stacks_digest(settings);
    assert_eq!(jobs.queued(), 0);

    let settings = WorldSettings::default()
        .worker_threads(2)
        .job_system(jobs.clone());
    assert!(
        format!("{settings:?}").contains("Caller(JobSystem)"),
        "{settings:?}"
    );
    stacks_digest(settings);
    assert!(jobs.queued() > 0);
}

fn single_threaded() -> WorldSettings {
    WorldSettings::default().single_threaded()
}

#[test]
fn single_threaded_matches_the_native_pool() {
    assert_eq!(stacks_digest(single_threaded()), stacks_digest(native()));
    assert_eq!(
        ragdoll_pile_digest(single_threaded()),
        ragdoll_pile_digest(native())
    );
}

#[test]
fn the_last_of_three_job_system_choices_wins() {
    let jobs = Arc::new(Counting::new(InlineJobs { concurrency: 3 }));
    let settings = WorldSettings::default()
        .job_system(jobs.clone())
        .single_threaded();
    assert!(
        format!("{settings:?}").contains("SingleThreaded"),
        "{settings:?}"
    );
    stacks_digest(settings);
    assert_eq!(jobs.queued(), 0);

    let settings = single_threaded().job_system(jobs.clone());
    assert!(
        format!("{settings:?}").contains("Caller(JobSystem)"),
        "{settings:?}"
    );
    stacks_digest(settings);
    assert!(jobs.queued() > 0);
}

#[test]
fn a_single_threaded_world_steps_on_another_thread() {
    const MOVED_TICKS: usize = 60;
    let mut world = PhysicsWorld::new(single_threaded()).unwrap();
    let ids = build_stacks(&mut world);
    let moved = thread::spawn(move || run_digest(&mut world, &ids, MOVED_TICKS))
        .join()
        .unwrap();

    let mut world = PhysicsWorld::new(single_threaded()).unwrap();
    let ids = build_stacks(&mut world);
    assert_eq!(moved, run_digest(&mut world, &ids, MOVED_TICKS));
}

#[test]
fn worlds_with_different_native_job_systems_step_in_parallel() {
    let (pool, single) = with_timeout(TIMEOUT_SECS, || {
        thread::scope(|scope| {
            let pool = scope.spawn(|| stacks_digest(native()));
            let single = scope.spawn(|| stacks_digest(single_threaded()));
            (pool.join().unwrap(), single.join().unwrap())
        })
    });
    assert_eq!(pool, stacks_digest(native()));
    assert_eq!(single, stacks_digest(single_threaded()));
}

/// A floor and 2048 awake unit cubes in a 16 x 16 grid of columns of 8, stepped 30 times with
/// [`PhysicsWorld::MAX_COLLISION_STEPS`] collision steps on `jobs`: the largest number of
/// FindCollisions respawns one update queued.
fn largest_respawn_count(jobs: Arc<Counting<impl JobSystem + 'static>>) -> usize {
    let collision_steps = PhysicsWorld::MAX_COLLISION_STEPS as usize;
    let mut world = PhysicsWorld::new(WorldSettings::default().job_system(jobs.clone())).unwrap();
    add_floor(&mut world);
    let shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    for column in 0..256 {
        let (x, z) = ((column % 16) as Real * 1.1, (column / 16) as Real * 1.1);
        for layer in 0..8 {
            let settings = BodySettings::new_dynamic()
                .position(RVec3::new(x, 0.5 + 1.05 * layer as Real, z))
                .motion_quality(MotionQuality::Discrete)
                .allow_sleeping(false);
            world.create_body(&shape, &settings).unwrap();
        }
    }
    // Jobs per collision step of this scene at concurrency 32: 11 single jobs,
    // DetermineActiveConstraints 1, ApplyGravity 30, FindCollisions 32, SetupVelocityConstraints
    // 1, SolveVelocity 32, IntegrateVelocity 32, SolvePosition 32; the last collision step has no
    // StartNextStep. Every job is queued once (docs/limits.md#collision-steps).
    let static_jobs = 171 * collision_steps - 1;
    let mut largest = 0;
    for _ in 0..30 {
        let before = jobs.queued();
        let report = world
            .step_with_collision_steps(DT, PhysicsWorld::MAX_COLLISION_STEPS)
            .unwrap();
        assert!(report.is_complete());
        let queued = jobs.queued() - before;
        assert!(
            queued >= static_jobs,
            "{queued} jobs, {static_jobs} expected at least"
        );
        largest = largest.max(queued - static_jobs);
    }
    largest
}

#[test]
fn a_saturated_update_at_max_collision_steps_fits_jolts_job_pool() {
    // Jolt's job pool and barrier ring hold 2048 jobs. An update of `n` collision steps creates at
    // most `360 * n - 1` jobs besides FindCollisions respawns, and the respawns may take twice
    // their count in the worst case (docs/limits.md#collision-steps).
    let collision_steps = PhysicsWorld::MAX_COLLISION_STEPS as usize;
    let budget = (2048 - (360 * collision_steps - 1)) / 2;
    let inline = with_timeout(TIMEOUT_SECS, || {
        largest_respawn_count(Arc::new(Counting::new(InlineJobs { concurrency: 32 })))
    });
    let rayon = with_timeout(TIMEOUT_SECS, || {
        largest_respawn_count(Arc::new(Counting::new(RayonJobs::new(31))))
    });
    assert!(
        inline <= budget,
        "inline: {inline} respawns, budget {budget}"
    );
    assert!(rayon <= budget, "rayon: {rayon} respawns, budget {budget}");
}

#[test]
fn a_sleeping_world_queues_no_jobs() {
    let jobs = Arc::new(Counting::new(InlineJobs { concurrency: 3 }));
    let mut world = PhysicsWorld::new(WorldSettings::default().job_system(jobs.clone())).unwrap();
    add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    let mut ticks = 0;
    while !world.body(cube).unwrap().is_sleeping() {
        assert!(ticks < 600, "the cube never fell asleep");
        step(&mut world, 1);
        ticks += 1;
    }
    assert!(jobs.queued() > 0);

    // Jolt skips the job stages when no body is awake and no step listener is installed
    // (`PhysicsSystem::Update`).
    let queued = jobs.queued();
    step(&mut world, 10);
    assert_eq!(jobs.queued(), queued);
}
