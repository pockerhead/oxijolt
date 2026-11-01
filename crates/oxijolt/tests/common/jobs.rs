//! Caller job systems for the tests: a Rayon pool and an inline job system, both counting the
//! jobs they were handed in one process-wide counter, and the process-wide choice of job system
//! that a determinism child reads from [`JOBS_ENV`].

use std::cell::Cell;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use oxijolt::*;

/// Jobs handed to any [`RayonJobs`] or [`InlineJobs`] of this process.
static QUEUED: AtomicUsize = AtomicUsize::new(0);

/// Jobs handed to any [`RayonJobs`] of this process from inside a job its pool ran.
static QUEUED_FROM_POOL_JOBS: AtomicUsize = AtomicUsize::new(0);

thread_local! {
    /// Whether this thread is inside a [`Job::run`] that a [`RayonJobs`] pool started.
    static IN_POOL_JOB: Cell<bool> = const { Cell::new(false) };
}

/// How many jobs the caller job systems of this process were handed so far.
pub fn queued() -> usize {
    QUEUED.load(Ordering::Relaxed)
}

/// How many jobs the [`RayonJobs`] of this process were handed from inside a job one of their
/// pools ran; see [`RayonJobs::queued_from_pool_jobs`].
pub fn queued_from_pool_jobs() -> usize {
    QUEUED_FROM_POOL_JOBS.load(Ordering::Relaxed)
}

/// Jolt's jobs on a Rayon pool of its own.
pub struct RayonJobs {
    pub pool: rayon::ThreadPool,
    queued_from_pool_jobs: AtomicUsize,
}

impl RayonJobs {
    /// A pool of `threads` threads.
    pub fn new(threads: usize) -> Self {
        Self {
            pool: rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap(),
            queued_from_pool_jobs: AtomicUsize::new(0),
        }
    }

    /// How many jobs this job system was handed from inside a job its pool ran.
    ///
    /// Jolt queues a job from a pool thread only while a Jolt job function runs there: a job
    /// releases the jobs that depend on it as it executes. More than zero therefore proves that
    /// the pool executed Jolt work, which counting the handed jobs or the pool's calls of
    /// [`Job::run`] cannot: Jolt's update barrier runs every job the pool drops or runs late.
    pub fn queued_from_pool_jobs(&self) -> usize {
        self.queued_from_pool_jobs.load(Ordering::Relaxed)
    }
}

impl JobSystem for RayonJobs {
    fn max_concurrency(&self) -> u32 {
        u32::try_from(self.pool.current_num_threads()).map_or(u32::MAX, |n| n.saturating_add(1))
    }

    fn queue_job(&self, job: Job) {
        QUEUED.fetch_add(1, Ordering::Relaxed);
        if IN_POOL_JOB.get() {
            self.queued_from_pool_jobs.fetch_add(1, Ordering::Relaxed);
            QUEUED_FROM_POOL_JOBS.fetch_add(1, Ordering::Relaxed);
        }
        self.pool.spawn(move || {
            IN_POOL_JOB.set(true);
            job.run();
            IN_POOL_JOB.set(false);
        });
    }
}

/// Calls [`Job::run`] right away inside `queue_job`, which leaves every job to the stepping
/// thread.
pub struct InlineJobs {
    pub concurrency: u32,
}

impl JobSystem for InlineJobs {
    fn max_concurrency(&self) -> u32 {
        self.concurrency
    }

    fn queue_job(&self, job: Job) {
        QUEUED.fetch_add(1, Ordering::Relaxed);
        job.run();
    }
}

/// Tells a determinism child which job system its worlds use: `rayon`, `inline` or `single` (Jolt's
/// single-threaded job system); unset for Jolt's thread pool.
pub const JOBS_ENV: &str = "OXIJOLT_DIGEST_JOBS";

/// The job system of the worlds of this process.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JobChoice {
    /// Jolt's thread pool with the requested worker threads.
    Native,
    /// A Rayon pool with the requested threads, concurrency one more.
    Rayon,
    /// [`InlineJobs`] with concurrency 3, whatever thread count is requested.
    Inline,
    /// Jolt's single-threaded job system, whatever thread count is requested.
    SingleThreaded,
}

impl JobChoice {
    /// The value of [`JOBS_ENV`] that selects this choice; `None` for unset.
    pub fn as_env(self) -> Option<&'static str> {
        match self {
            Self::Native => None,
            Self::Rayon => Some("rayon"),
            Self::Inline => Some("inline"),
            Self::SingleThreaded => Some("single"),
        }
    }

    /// The choice [`JOBS_ENV`] selects; panics on a value it does not know.
    pub fn from_env() -> Self {
        match std::env::var(JOBS_ENV) {
            Err(std::env::VarError::NotPresent) => Self::Native,
            Ok(value) if value == "rayon" => Self::Rayon,
            Ok(value) if value == "inline" => Self::Inline,
            Ok(value) if value == "single" => Self::SingleThreaded,
            value => panic!("{JOBS_ENV}={value:?} is not a job system"),
        }
    }
}

/// `settings` with the job system [`JobChoice::from_env`] selects, given `worker_threads`. On
/// wasm32, which has no threads, Jolt's thread pool is replaced by its single-threaded job system.
pub fn with_threads(settings: WorldSettings, worker_threads: u32) -> WorldSettings {
    match JobChoice::from_env() {
        JobChoice::Native if cfg!(target_arch = "wasm32") => settings.single_threaded(),
        JobChoice::Native => settings.worker_threads(worker_threads),
        JobChoice::Rayon => settings.job_system(Arc::new(RayonJobs::new(worker_threads as usize))),
        JobChoice::Inline => settings.job_system(Arc::new(InlineJobs { concurrency: 3 })),
        JobChoice::SingleThreaded => settings.single_threaded(),
    }
}
