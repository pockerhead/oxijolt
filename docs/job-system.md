# Running Jolt's jobs on your own thread pool

A world runs the jobs of a step on one of three job systems, chosen in `WorldSettings`; the
setting called last decides:

- Jolt's own thread pool, the default (`WorldSettings::worker_threads`, one worker unless set).
- Jolt's single-threaded job system (`WorldSettings::single_threaded`): every job runs on the
  thread that calls `step`, with concurrency 1, and Jolt starts no threads.
- The caller's pool, such as Rayon or a game's own task system, through the `JobSystem` trait
  (`WorldSettings::job_system`); Jolt then starts no threads for that world either.

wasm32 has no threads: there the default is the single-threaded job system, and
`PhysicsWorld::new` refuses `WorldSettings::worker_threads` with `WorldError::InvalidValue`. A
caller job system is accepted; the thread that calls `step` runs the jobs it leaves.

`PhysicsWorld::job_system_kind` tells which of the three a world got (`JobSystemKind`, with the
worker count of Jolt's pool), and `PhysicsWorld::max_concurrency` the concurrency its job system
was created with: workers + 1 for Jolt's pool, 1 single-threaded, the caller's `max_concurrency`
as read in `PhysicsWorld::new`. `WorldSettings::job_system_kind` reads the choice before the world
exists, without calling a caller job system; there is no unset state, so a plugin that layers its own
choice on a user's settings compares with `WorldSettings::default().job_system_kind()` to see whether
the user changed it.

## An adapter

```rust
use std::sync::Arc;
use oxijolt::*;

/// Jolt's jobs on a Rayon pool. The newtype satisfies Rust's orphan rule.
struct RayonJobs(rayon::ThreadPool);

impl JobSystem for RayonJobs {
    fn max_concurrency(&self) -> u32 {
        // The pool's threads and the thread that steps the world.
        u32::try_from(self.0.current_num_threads()).map_or(u32::MAX, |n| n.saturating_add(1))
    }

    fn queue_job(&self, job: Job) {
        self.0.spawn(move || job.run());
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let pool = rayon::ThreadPoolBuilder::new().num_threads(2).build()?;
    let jobs: Arc<dyn JobSystem> = Arc::new(RayonJobs(pool));
    // Several worlds may share one job system and step on different threads at the same time.
    let mut world = PhysicsWorld::new(WorldSettings::default().job_system(jobs.clone()))?;

    let floor = Shape::new_box(Vec3::new(50.0, 1.0, 50.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    let falling =
        world.create_body(&cube, &BodySettings::new_dynamic().position(RVec3::new(0.0, 3.0, 0.0)))?;
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    assert!(world.body(falling)?.position().y < 3.0);
    Ok(())
}
```

`Job::run` does not panic, so Rayon's default of aborting the process when a spawned task panics
does not come into play. An adapter that does more in the spawned closure owns that risk.

## The contract

- **Concurrency.** `max_concurrency` counts the thread that calls `step`. `PhysicsWorld::new` reads
  it once and refuses the settings unless it is within `1..=WorldSettings::MAX_CONCURRENCY` (65, the
  concurrency of Jolt's own pool at its 64-worker bound). Jolt uses it to decide how many jobs a
  stage is split into, at most 32 (`PhysicsUpdateContext::cMaxConcurrency`).
- **Queueing.** `queue_job` is called from the stepping thread and from threads that run jobs, from
  inside a running job. It must not block waiting for any job.
- **The barrier.** The stepping thread waits for the step's jobs at a Jolt barrier and, while it
  waits, runs the jobs no other thread has started. A step therefore finishes even when the pool
  runs a job late or never. Jobs the pool has not run when the step ends are handed back to Jolt by
  the step; running or dropping one later does nothing.
- **No job starts inside `queue_job`.** A job run or dropped while `queue_job` is on the same
  thread's stack is not started there; it is left to the stepping thread. Jolt does not let a job
  start inside another job on one thread: some jobs release their dependents while they still hold
  body access rights, which Jolt's assertions track per thread (`BodyAccess::Grant`). A job system
  that calls `job.run()` inside `queue_job` is valid and simply leaves every job to the stepping
  thread.
- **Lifetimes.** A `Job` may be sent to any thread and may outlive its world; until it is dropped it
  keeps its world's native job system allocated. The world keeps the `Arc<dyn JobSystem>` as long as
  it lives.
- **Idle worlds.** Jolt queues no job for a step in which no body is active and no step listener (a
  vehicle) exists.

## Panics

A panic in `queue_job` does not unwind into Jolt. The step goes on and the world advances; `step`
then resumes the first such panic. Jobs handed to the job system before the panic may still run on
its threads; the jobs Jolt queues after it run on the stepping thread. The next step uses the
caller's job system again.

## Determinism

Which threads ran the jobs is not part of Jolt's state, so results do not depend on the pool. The
tests check this: the determinism scenes ([determinism.md](determinism.md)) run with a Rayon pool of
4 threads, with an inline job system (one that calls `job.run()` inside `queue_job`) and with the
single-threaded job system, and must record what Jolt's pool with 1 worker records. `tests/job_system.rs` also compares a Rayon pool of
3 threads with Jolt's pool on stacks of cubes, and the inline job system on those stacks and on a
pile of ragdolls. Side effects of the caller's own code in `queue_job` are the caller's to keep
deterministic.
