//! Running Jolt's jobs on the caller's thread pool: the public [`JobSystem`] and [`Job`], and
//! the joltc callback job system behind them.

use std::any::Any;
use std::cell::Cell;
use std::collections::BTreeMap;
use std::ffi::c_void;
use std::fmt;
use std::io::Write;
use std::mem;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock, PoisonError};

use oxijolt_sys::*;

use crate::owned::Owned;
use crate::WorldError;

/// A thread pool that runs Jolt's jobs for a world, set with
/// [`WorldSettings::job_system`](crate::WorldSettings::job_system), for example Rayon or a
/// game's own pool. Without it a world runs its jobs on Jolt's thread pool
/// ([`WorldSettings::worker_threads`](crate::WorldSettings::worker_threads)).
///
/// A step splits its work into jobs, small closures with dependencies. The thread that calls
/// [`PhysicsWorld::step`](crate::PhysicsWorld::step) waits for them at a Jolt barrier, and while
/// it waits it runs the jobs that no other thread has started. A step therefore finishes even
/// when the pool runs a job late or never; a job the stepping thread already ran does nothing
/// when the pool runs it.
///
/// A job that is run or dropped while [`queue_job`](Self::queue_job) is on the same thread's
/// stack is not started there, because Jolt does not let a job start inside another job on one
/// thread; it is left to the stepping thread, and the step hands it back to Jolt after the
/// update.
///
/// Jolt documents its simulation as deterministic for the same binary, the same initial state
/// and the same calls in the same order (Jolt docs, "Deterministic Simulation"); which threads
/// ran the jobs is not part of that state. Side effects of the caller's own code in `queue_job`
/// are the caller's to keep deterministic ([docs/job-system.md#determinism] says what the
/// tests compare).
///
/// Several worlds may share one job system and step on different threads at the same time.
///
/// # Example
/// A Rayon pool. Rust's orphan rule needs the newtype: neither the trait nor
/// `rayon::ThreadPool` belongs to the caller's crate.
///
/// ```
/// use std::sync::Arc;
/// use oxijolt::*;
///
/// struct RayonJobs(rayon::ThreadPool);
///
/// impl JobSystem for RayonJobs {
///     fn max_concurrency(&self) -> u32 {
///         // The pool's threads and the thread that steps the world.
///         u32::try_from(self.0.current_num_threads()).map_or(u32::MAX, |n| n.saturating_add(1))
///     }
///
///     fn queue_job(&self, job: Job) {
///         self.0.spawn(move || job.run());
///     }
/// }
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let pool = rayon::ThreadPoolBuilder::new().num_threads(2).build()?;
/// let settings = WorldSettings::default().job_system(Arc::new(RayonJobs(pool)));
/// let mut world = PhysicsWorld::new(settings)?;
///
/// let floor = Shape::new_box(Vec3::new(100.0, 1.0, 100.0))?;
/// world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
/// let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
/// let falling = world.create_body(
///     &cube,
///     &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
/// )?;
/// for _ in 0..60 {
///     assert!(world.step(1.0 / 60.0)?.is_complete());
/// }
/// assert!(world.body(falling)?.position().y < 2.0);
/// # Ok(())
/// # }
/// ```
///
/// The spawned closure only calls [`Job::run`], which does not panic, so Rayon's default of
/// aborting the process when a spawned task panics does not come into play; an adapter that
/// does more in the spawned closure owns that risk.
///
/// [docs/job-system.md#determinism]: https://github.com/pockerhead/oxijolt/blob/main/docs/job-system.md#determinism
pub trait JobSystem: Send + Sync + 'static {
    /// The most jobs that may run at the same time, counting the thread that calls
    /// [`PhysicsWorld::step`](crate::PhysicsWorld::step).
    ///
    /// [`PhysicsWorld::new`](crate::PhysicsWorld::new) reads it once and rejects the settings
    /// unless it is within `1..=`
    /// [`WorldSettings::MAX_CONCURRENCY`](crate::WorldSettings::MAX_CONCURRENCY). Jolt uses it
    /// to decide how many jobs a stage is split into, at most 32
    /// (`PhysicsUpdateContext::cMaxConcurrency`); it changes how the work is split, not the
    /// results.
    fn max_concurrency(&self) -> u32;

    /// Runs `job` soon, on any thread.
    ///
    /// Jolt calls this from the stepping thread and from threads that run jobs, from inside a
    /// running job. Calling [`Job::run`] here, before returning, is allowed but does not start
    /// the job: it is left to the stepping thread (see above). This must not block waiting for
    /// any job. A job the pool has not run when the step ends is handed back to Jolt by the step
    /// itself; running it later does nothing.
    ///
    /// A panic here does not unwind into Jolt; see
    /// [`PhysicsWorld::step`](crate::PhysicsWorld::step). On wasm32 it traps
    /// ([wasm32](crate#wasm32-unknown-unknown)).
    fn queue_job(&self, job: Job);
}

/// One Jolt job handed to a [`JobSystem`].
///
/// Run it with [`run`](Self::run); dropping it runs it as well. The step that queued it waits
/// until Jolt has run every one of its jobs, so a job run after its step does nothing. A job may
/// be sent to any thread and may outlive its world; until it is dropped it keeps its world's
/// native job system allocated.
pub struct Job {
    /// The key of this job's Jolt reference in `native`'s table of queued jobs.
    id: u64,
    native: Arc<CallbackJobSystem>,
}

impl Job {
    /// Runs the Jolt job unless it already ran, then hands it back to Jolt. While
    /// [`JobSystem::queue_job`] is on this thread's stack it does nothing: the job is left to the
    /// stepping thread.
    pub fn run(self) {
        drop(self);
    }
}

impl Drop for Job {
    fn drop(&mut self) {
        if in_queue_callback() {
            return;
        }
        if let Some(task) = self.native.take_queued(self.id) {
            // SAFETY: `take_queued` removed the job's one reference from the table and handed it
            // to this call, so nothing else releases it. `self.native` is the live native job
            // system the job belongs to; it is a field and drops only after this body returns.
            // No queue callback is on this thread's stack, so no Jolt job runs on this thread.
            unsafe { task.run() }
        }
    }
}

/// joltc's `RunJob` with its argument: a Jolt job holding one reference for its hand-off.
#[derive(Clone, Copy)]
struct JobTask {
    function: unsafe extern "C" fn(*mut c_void),
    arg: NonNull<c_void>,
}

impl JobTask {
    /// Executes the Jolt job unless it already ran, then releases the reference.
    ///
    /// # Safety
    /// The caller owns the task's reference and gives it up here, and the native job system the
    /// job belongs to is alive. Either no Jolt job is running on this thread, or the job has
    /// already run, so that it does not start inside another job.
    unsafe fn run(self) {
        // SAFETY: `function` is joltc's `RunJob`, which executes the job at most once
        // (`Job::Execute` in `JobSystem.h` starts only from zero dependencies) and then releases
        // the caller's reference into the live native job system, whose barrier a running job
        // notifies (contract). Job functions capture only references, pointers and integers, so
        // a job freed after its world is gone touches nothing but the native job system. A job
        // that still has to run belongs to an update in progress: Jolt adds every job of
        // `PhysicsSystem::Update` to the update barrier and returns only after each one ran, so
        // the world is alive while it runs; re-check this when updating Jolt or joltc.
        unsafe { (self.function)(self.arg.as_ptr()) }
    }
}

thread_local! {
    /// How many queue callbacks are on this thread's stack.
    static QUEUE_CALLBACK_DEPTH: Cell<u32> = const { Cell::new(0) };
}

/// Whether a queue callback, and so a running Jolt job or update, is on this thread's stack.
fn in_queue_callback() -> bool {
    QUEUE_CALLBACK_DEPTH
        .try_with(|depth| depth.get() > 0)
        .unwrap_or(false)
}

/// Marks a queue callback on this thread's stack while it lives.
struct InQueueCallback;

impl InQueueCallback {
    fn enter() -> Self {
        QUEUE_CALLBACK_DEPTH.with(|depth| depth.set(depth.get() + 1));
        Self
    }
}

impl Drop for InQueueCallback {
    fn drop(&mut self) {
        QUEUE_CALLBACK_DEPTH.with(|depth| depth.set(depth.get() - 1));
    }
}

impl fmt::Debug for Job {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("Job")
    }
}

/// joltc's callback job system of one world, destroyed by its last owner: the world's
/// [`QueueContext`] or a [`Job`].
pub(crate) struct CallbackJobSystem {
    // Field order: `Drop` releases the queued jobs, then `native` is destroyed.
    queued: Mutex<QueuedJobs>,
    native: Owned<JPH_JobSystem>,
}

/// The Jolt references of the jobs handed to the caller and not taken back yet, by job id. A
/// step releases the ones left once its update returned, so the caller's pool never keeps
/// Jolt's job slots beyond the step.
#[derive(Default)]
struct QueuedJobs {
    next_id: u64,
    tasks: BTreeMap<u64, JobTask>,
}

impl CallbackJobSystem {
    /// Takes over a job reference joltc queued and returns its id.
    fn add_queued(&self, task: JobTask) -> u64 {
        let mut queued = self.queued.lock().unwrap_or_else(PoisonError::into_inner);
        let id = queued.next_id;
        queued.next_id = id.wrapping_add(1);
        queued.tasks.insert(id, task);
        id
    }

    /// Hands over the reference of job `id`, unless the step released it already.
    fn take_queued(&self, id: u64) -> Option<JobTask> {
        let mut queued = self.queued.lock().unwrap_or_else(PoisonError::into_inner);
        queued.tasks.remove(&id)
    }

    /// Releases every reference not taken back. Called when no update of this job system runs:
    /// Jolt's update barrier has run every job, so only the references are released.
    fn release_queued(&self) {
        let tasks = {
            let mut queued = self.queued.lock().unwrap_or_else(PoisonError::into_inner);
            mem::take(&mut queued.tasks)
        };
        for task in tasks.into_values() {
            // SAFETY: the table owned the task's reference and `mem::take` handed it to this
            // loop, which gives it up once. `self` is the live native job system the job belongs
            // to. No update of it runs (caller), so the job has run and does not start here.
            unsafe { task.run() };
        }
    }
}

impl Drop for CallbackJobSystem {
    fn drop(&mut self) {
        // The last owner holds no update in progress: a world that steps with this job system
        // owns a reference to it.
        self.release_queued();
    }
}

// SAFETY: Jolt calls `QueueJob`, `FreeJob` and the barrier methods of one job system from many
// threads (lock-free free list, atomic barrier state), and `GetMaxConcurrency` reads a field
// set at creation. The queued tasks are Jolt jobs, which may be run and released on any thread
// ("If you want to implement your own job system", `JobSystem.h`), behind a mutex. The object
// is destroyed only by the last `Arc`, so no call overlaps the destruction, and destruction has
// no thread affinity.
unsafe impl Send for CallbackJobSystem {}
// SAFETY: as for `Send`; shared references only hand the pointer to Jolt or lock `queued`.
unsafe impl Sync for CallbackJobSystem {}

/// What joltc's queue callbacks of one world get as their `context`: the caller's job system,
/// the native object the jobs belong to, and the first panic of the caller's `queue_job`.
///
/// Jobs do not hold the context, so a job that runs late on one of the caller's threads never
/// drops the caller's job system there.
pub(crate) struct QueueContext {
    job_system: Arc<dyn JobSystem>,
    /// Set once, right after the native object was created and before any step.
    native: OnceLock<Arc<CallbackJobSystem>>,
    /// Set by the first panic, whose payload `panic` then holds; from then on jobs are left to
    /// the stepping thread.
    panicked: AtomicBool,
    panic: Mutex<Option<Box<dyn Any + Send>>>,
}

impl QueueContext {
    fn new(job_system: Arc<dyn JobSystem>) -> Self {
        Self {
            job_system,
            native: OnceLock::new(),
            panicked: AtomicBool::new(false),
            panic: Mutex::new(None),
        }
    }

    /// The native job system to step with.
    pub(crate) fn as_ptr(&self) -> *mut JPH_JobSystem {
        let native = self
            .native
            .get()
            .expect("caller job system is created with its context");
        native.native.as_ptr()
    }

    /// Wraps a job joltc queued; aborts on input joltc never passes.
    fn job(&self, function: JPH_JobFunction, arg: *mut c_void) -> Job {
        match (function, NonNull::new(arg), self.native.get()) {
            (Some(function), Some(arg), Some(native)) => Job {
                id: native.add_queued(JobTask { function, arg }),
                native: Arc::clone(native),
            },
            _ => abort_in_callback("joltc queued a job without a function or argument"),
        }
    }

    /// Hands `job` to the caller's job system, or once that panicked, leaves it to the stepping
    /// thread.
    fn queue(&self, job: Job) {
        if self.panicked.load(Ordering::Acquire) {
            drop(job);
            return;
        }
        let queued = catch_unwind(AssertUnwindSafe(|| self.job_system.queue_job(job)));
        if let Err(payload) = queued {
            self.record_panic(payload);
        }
    }

    /// Keeps the payload of the panic that set `panicked` first and drops later ones without
    /// letting their drop unwind.
    fn record_panic(&self, payload: Box<dyn Any + Send>) {
        let first = self
            .panicked
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_ok();
        if first {
            *self.panic.lock().unwrap_or_else(PoisonError::into_inner) = Some(payload);
        } else if let Err(panic_in_drop) = catch_unwind(AssertUnwindSafe(|| drop(payload))) {
            mem::forget(panic_in_drop);
        }
    }

    /// Finishes an update: releases the jobs the caller's job system has not taken back and
    /// returns the first panic of its `queue_job` since the last call, after which the caller's
    /// job system is used again. Called by `step` after the update returned, when no queue
    /// callback of this world runs.
    pub(crate) fn finish_update(&self) -> Option<Box<dyn Any + Send>> {
        if let Some(native) = self.native.get() {
            native.release_queued();
        }
        let payload = self
            .panic
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take();
        self.panicked.store(false, Ordering::Release);
        payload
    }
}

/// Reports a broken joltc contract and aborts; never unwinds.
fn abort_in_callback(what: &str) -> ! {
    let _ = writeln!(std::io::stderr(), "oxijolt: {what}; aborting");
    std::process::abort()
}

/// joltc's single-job queue callback.
///
/// # Safety
/// joltc calls it with the `context` of a live [`QueueContext`] during that world's
/// `PhysicsSystem::Update`, with `RunJob` and a Jolt job holding one reference for this call.
unsafe extern "C" fn queue_job(context: *mut c_void, function: JPH_JobFunction, arg: *mut c_void) {
    // SAFETY: `context` is the world's live `QueueContext` (contract), which the world keeps in
    // an `Arc` and only reads through shared references.
    let context = unsafe { &*context.cast::<QueueContext>() };
    let _in_callback = InQueueCallback::enter();
    context.queue(context.job(function, arg));
}

/// joltc's batch queue callback.
///
/// # Safety
/// As [`queue_job`], and `args` points to `count` jobs that each hold one reference for this
/// call. The array itself lives only during the call.
unsafe extern "C" fn queue_jobs(
    context: *mut c_void,
    function: JPH_JobFunction,
    args: *mut *mut c_void,
    count: u32,
) {
    if count == 0 {
        return;
    }
    if args.is_null() {
        abort_in_callback("joltc queued a batch without jobs");
    }
    // SAFETY: as in `queue_job`.
    let context = unsafe { &*context.cast::<QueueContext>() };
    let _in_callback = InQueueCallback::enter();
    for i in 0..count as usize {
        // SAFETY: `args` points to `count` job pointers that live during this call (contract);
        // each is read once, here, before the call returns.
        let arg = unsafe { *args.add(i) };
        context.queue(context.job(function, arg));
    }
}

/// Creates the native callback job system of a world that runs its jobs on `job_system`, and
/// returns the context that owns both.
pub(crate) fn create_caller_job_system(
    job_system: Arc<dyn JobSystem>,
    max_concurrency: u32,
) -> Result<Arc<QueueContext>, WorldError> {
    let context = Arc::new(QueueContext::new(job_system));
    let config = JPH_JobSystemConfig {
        context: Arc::as_ptr(&context).cast_mut().cast(),
        queueJob: Some(queue_job),
        queueJobs: Some(queue_jobs),
        maxConcurrency: max_concurrency,
        // Jolt's `cMaxPhysicsBarriers`.
        maxBarriers: 0,
    };
    // SAFETY: Jolt is initialised (the caller creates worlds only after `ensure_initialized`)
    // and `config` is a live local that joltc copies. The native object keeps `context` after
    // the world is gone, but joltc reads it only in `QueueJob(s)`, which Jolt calls only from
    // `CreateJob` and dependency removal inside `PhysicsSystem::Update`; a late `RunJob` of a
    // finished job queues nothing. The handle takes over the returned object.
    let native = unsafe { Owned::from_raw(JPH_JobSystemCallback_Create(&config)) }
        .ok_or(WorldError::AllocationFailed("job system"))?;
    // A fresh `OnceLock` is empty, so this always stores.
    let _ = context.native.set(Arc::new(CallbackJobSystem {
        queued: Mutex::default(),
        native,
    }));
    Ok(context)
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::AtomicUsize;

    use super::*;
    use crate::world::ensure_initialized;

    /// A caller job system that panics on every call.
    struct AlwaysPanics {
        calls: AtomicUsize,
    }

    impl JobSystem for AlwaysPanics {
        fn max_concurrency(&self) -> u32 {
            1
        }

        fn queue_job(&self, job: Job) {
            let call = self.calls.fetch_add(1, Ordering::Relaxed) + 1;
            drop(job);
            panic!("queue_job call {call}");
        }
    }

    /// Counts how often a test job ran; its argument is a `RUNS` counter.
    unsafe extern "C" fn count_run(arg: *mut c_void) {
        // SAFETY: the tests pass a pointer to a live `AtomicUsize`.
        unsafe { &*arg.cast::<AtomicUsize>() }.fetch_add(1, Ordering::Relaxed);
    }

    /// A context with a real native object whose queue callbacks are never called: the tests
    /// queue their own jobs through `QueueContext::queue`.
    fn context(job_system: Arc<dyn JobSystem>) -> Arc<QueueContext> {
        assert!(ensure_initialized());
        create_caller_job_system(job_system, 1).unwrap()
    }

    fn counting_job(context: &QueueContext, runs: &AtomicUsize) -> Job {
        let native = context.native.get().unwrap();
        let task = JobTask {
            function: count_run,
            arg: NonNull::from(runs).cast(),
        };
        Job {
            id: native.add_queued(task),
            native: Arc::clone(native),
        }
    }

    fn message(payload: &(dyn Any + Send)) -> String {
        payload
            .downcast_ref::<String>()
            .cloned()
            .unwrap_or_default()
    }

    fn count(runs: &AtomicUsize) -> usize {
        runs.load(Ordering::Relaxed)
    }

    #[test]
    fn run_and_drop_each_run_the_job_once() {
        let context = context(Arc::new(AlwaysPanics {
            calls: AtomicUsize::new(0),
        }));
        let runs = AtomicUsize::new(0);
        counting_job(&context, &runs).run();
        assert_eq!(count(&runs), 1);
        drop(counting_job(&context, &runs));
        assert_eq!(count(&runs), 2);
        assert!(context.finish_update().is_none());
        assert_eq!(count(&runs), 2);
    }

    #[test]
    fn jobs_run_inside_a_queue_callback_wait_for_the_end_of_the_update() {
        let context = context(Arc::new(AlwaysPanics {
            calls: AtomicUsize::new(0),
        }));
        let runs = AtomicUsize::new(0);
        {
            let _in_callback = InQueueCallback::enter();
            counting_job(&context, &runs).run();
            drop(counting_job(&context, &runs));
            assert_eq!(count(&runs), 0);
        }
        assert!(context.finish_update().is_none());
        assert_eq!(count(&runs), 2);
    }

    #[test]
    fn a_job_released_by_its_update_does_nothing_when_run_later() {
        let context = context(Arc::new(AlwaysPanics {
            calls: AtomicUsize::new(0),
        }));
        let runs = AtomicUsize::new(0);
        let late = counting_job(&context, &runs);
        assert!(context.finish_update().is_none());
        assert_eq!(count(&runs), 1);
        late.run();
        assert_eq!(count(&runs), 1);
    }

    #[test]
    fn the_first_panic_is_kept_later_jobs_skip_the_job_system_and_finish_update_resets() {
        let job_system = Arc::new(AlwaysPanics {
            calls: AtomicUsize::new(0),
        });
        let context = context(job_system.clone());
        let runs = AtomicUsize::new(0);
        let calls = || job_system.calls.load(Ordering::Relaxed);

        context.queue(counting_job(&context, &runs));
        context.queue(counting_job(&context, &runs));
        assert_eq!(count(&runs), 2);
        assert_eq!(calls(), 1);

        context.record_panic(Box::new(String::from("second")));
        let payload = context.finish_update().unwrap();
        assert_eq!(message(&*payload), "queue_job call 1");
        assert!(context.finish_update().is_none());

        context.queue(counting_job(&context, &runs));
        assert_eq!(count(&runs), 3);
        assert_eq!(calls(), 2);
        let payload = context.finish_update().unwrap();
        assert_eq!(message(&*payload), "queue_job call 2");
    }

    #[test]
    fn a_discarded_payload_whose_drop_panics_does_not_unwind() {
        struct PanicsOnDrop;

        impl Drop for PanicsOnDrop {
            fn drop(&mut self) {
                panic!("payload drop");
            }
        }

        let context = context(Arc::new(AlwaysPanics {
            calls: AtomicUsize::new(0),
        }));
        context.record_panic(Box::new(String::from("first")));
        context.record_panic(Box::new(PanicsOnDrop));
        assert_eq!(message(&*context.finish_update().unwrap()), "first");
    }

    #[test]
    #[cfg(not(target_arch = "wasm32"))]
    fn concurrent_panics_keep_one_payload_and_drop_the_others() {
        const THREADS: usize = 8;

        /// Counts its drops in `DROPS`.
        struct Payload;

        static DROPS: AtomicUsize = AtomicUsize::new(0);

        impl Drop for Payload {
            fn drop(&mut self) {
                DROPS.fetch_add(1, Ordering::Relaxed);
            }
        }

        let context = context(Arc::new(AlwaysPanics {
            calls: AtomicUsize::new(0),
        }));
        let start = std::sync::Barrier::new(THREADS);
        std::thread::scope(|scope| {
            for _ in 0..THREADS {
                scope.spawn(|| {
                    start.wait();
                    context.record_panic(Box::new(Payload));
                });
            }
        });
        assert_eq!(DROPS.load(Ordering::Relaxed), THREADS - 1);
        let kept = context.finish_update().unwrap();
        assert!(kept.is::<Payload>());
        drop(kept);
        assert_eq!(DROPS.load(Ordering::Relaxed), THREADS);
        assert!(context.finish_update().is_none());
    }

    #[test]
    #[cfg(not(target_arch = "wasm32"))]
    fn a_job_run_while_its_update_finishes_runs_once() {
        const JOBS: usize = 64;

        let context = context(Arc::new(AlwaysPanics {
            calls: AtomicUsize::new(0),
        }));
        for _ in 0..100 {
            let runs = AtomicUsize::new(0);
            let jobs: Vec<Job> = (0..JOBS).map(|_| counting_job(&context, &runs)).collect();
            let start = std::sync::Barrier::new(2);
            std::thread::scope(|scope| {
                scope.spawn(|| {
                    start.wait();
                    for job in jobs {
                        job.run();
                    }
                });
                start.wait();
                assert!(context.finish_update().is_none());
            });
            assert_eq!(count(&runs), JOBS);
        }
    }
}
