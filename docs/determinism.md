# Determinism

What oxijolt asks of the simulation, what the tests check, and what a caller must do to keep its own
part deterministic.

## The requirement

On one machine, the same binary given the same initial state and the same calls in the same order
produces bit-identical body ids, poses, velocities and sleep flags, whichever job system runs the
step: Jolt's thread pool with any `WorldSettings::worker_threads` (n workers plus the thread that
calls `step`) or a caller `JobSystem`. This rests on Jolt's own conditions (Jolt docs,
["Deterministic
Simulation"](https://github.com/jrouwe/JoltPhysics/blob/v5.6.0/Docs/Architecture.md#deterministic-simulation)).

The tests check it with 1 and 4 workers, with a Rayon pool of 4 threads and with an inline job
system, on the scenes listed below. They do not check every thread count or every call sequence.

## The call history is state

The order of body creation and removal decides each `BodyId`'s index and sequence number, so it is
part of the state. So are rebases, `optimize_broad_phase`, forces and velocity writes, and the
collision step count of each call (`step_with_collision_steps(dt, 4)` is not four `step(dt / 4)`).

- **Characters.** Each world numbers its characters from 1 in creation order. Jolt's default
  character id comes from a process-wide counter and orders contacts between characters, so the
  world passes its own. A character's contacts come in a deterministic order while
  `max_hits_exceeded` is false.
- **Removal and waking.** `remove_body` wakes the bodies whose exact bounds come within the
  speculative contact distance of the removed one, in id order, so it adds no hidden state;
  `remove_ragdoll` does the same for each part, `BodyMut::set_shape` for the box enclosing the
  body's old and new bounds, and `activate_bodies_in_box` for a box, without the margin.
  The broad phase only proposes candidates; the exact bounds decide, in the caller's precision.
- **Vehicles** run as Jolt step listeners. A fleet of 40 is spread over a different number of
  listener jobs with 1 and 4 workers.
- **Ragdolls.** A pile of 16 ragdolls forms islands of 128 or more joints and contacts, and Jolt
  splits islands of 128 or more for parallel solving (`LargeIslandSplitter.cpp:206,301`).
- **Vehicles, ragdolls and constraints** are numbered from 1 per world in creation order and never
  reuse an id.
- **Events.** The world sorts each step's events into a canonical order before it queues them
  ([events.md](events.md#canonical-order)).

## What the caller keeps deterministic

- Code that drives the world must not feed hash-map iteration order, time or thread identity into
  its calls.
- Narrow-phase query hits (`collide_shape`) come in an unspecified order; sort them by body id and
  sub-shape id when order matters.
- A `ContactListener` runs on Jolt's workers in no fixed order. Its decision must depend only on its
  arguments and on data fixed for the step.
- A caller `JobSystem` decides only which thread runs a job. Side effects of the caller's own code
  in `queue_job` are the caller's to keep deterministic ([job-system.md](job-system.md)).

## Across machines

Equal results across compilers, compiler flags, operating systems or CPU architectures are not
guaranteed by default. The `cross-platform-deterministic` feature (off by default, slower) builds
Jolt with `CROSS_PLATFORM_DETERMINISTIC`. Jolt then
[claims](https://github.com/jrouwe/JoltPhysics/blob/v5.6.0/Docs/Architecture.md#deterministic-simulation)
equal results across compilers, configurations, operating systems and architectures, as long as the
same source is built with the same defines: never compare a `double-precision` build with a
single-precision one, and the FPU rounding and denormal (DAZ/FTZ) modes must match. The gates below
run on one machine at a time and compare thread counts and job systems in each configuration; they
do not compare machines. The one gate across platforms is wasm against native, below.

## How it is checked

`cargo test -p oxijolt --test determinism` runs each scene in child processes, one with 1 worker and
one with 4, and requires their per-tick records to match byte for byte. Each scene also runs with a
Rayon pool of 4 threads, with an inline job system and with Jolt's single-threaded job system
(`WorldSettings::single_threaded`), which must record what Jolt's pool with 1 worker records.

- **Stacks**: cubes topple on a floor for 120 ticks, also across a rebase, and stepped with
  `PhysicsWorld::MAX_COLLISION_STEPS` collision steps per call (1 and 4 workers, and
  single-threaded).
- **Chunk**: a static compound chunk on a heightfield terrain and three items the caller pulls
  towards a planet centre, created, coming to rest and removed under the game's item rules, with a
  tilted rebase in the middle; ticks 0 to 1000. The same bodies created in another order must fail
  the gate.
- **Walker**: the game's reference controller (`tests/common/walker.rs`) for 600 ticks, with items
  dropped along its path.
- **Vehicle**: a car driving the acceptance route over terrain.
- **Fleet**: 40 vehicles, whose digest must also change when one input changes; also stepped with
  `MAX_COLLISION_STEPS` collision steps per call.
- **Ragdoll pile**: 16 humanoid ragdolls dropped into a pit, touching from the first tick.
- **Constraints**: a hinge chain, a motor-driven slider, a gear pair, a pulley and a path; one
  constraint is removed and re-created halfway.
- **Soft bodies**: a cloth pinned at two corners draped over a sphere and a pressurised ball dropped
  on a box; one corner is unpinned and the other moved halfway.

Other gates of the same kind:
- `tests/event_determinism.rs` compares the serialized events of every step for 1 and 4 workers,
  for caller job systems and for the single-threaded job system, also with `MAX_COLLISION_STEPS`
  collision steps per call, and checks that a scene stepped with every event on and a no-op contact
  listener simulates bit for bit like the same scene with nothing installed.
- `tests/state.rs` replays a rollback in two processes with 1 and 4 workers
  (`rollback_replay_is_identical_across_processes`).
- `tests/body_controls_determinism.rs` runs a scene that uses every body control at fixed ticks
  (impulses, kinematic moves, sensors, deactivation and activation, box activation, a shape and a
  motion type change, a `PLANE_2D` body) with 1 and 4 workers, in one process and in two.
- `tests/vehicle_kinds_determinism.rs` runs 12 tanks, 12 motorcycles and a car over flat ground
  and a ramp with scripted inputs, removing a tank on the way, with 1 and 4 workers, in one process
  and in two; a changed input must change the digest. `tests/vehicle_kinds_state.rs` replays a tank
  and a motorcycle after a rollback with 1 and 4 workers and in two processes.
- `tests/body_poses.rs` compares `PhysicsWorld::active_body_poses`, which returns the awake bodies
  in ascending `BodyId` order, for 1 and 4 workers and for caller job systems while a grid of
  cubes wakes and falls asleep again.
- `tests/skeleton_mapper_determinism.rs` drops a humanoid onto a floor under the caller's gravity
  while its motors follow the reverse-mapped pose of an animated skeleton every tick, and records
  the bodies and both mapped poses, with 1 and 4 workers, in one process and in two; a changed
  animation joint must change the digest from its tick on, and the bodies, recorded apart from
  the mapped poses, in that tick.
- `crates/oxijolt-sys/tests/determinism.rs` checks the raw layer with 1 and 4 workers.

CI runs all of them in each of its configurations (default, `cross-platform-deterministic`,
`double-precision`, `debug-renderer`, `asserts`) on Windows and on Linux.

`bash scripts/check_wasm_determinism.sh` compares platforms: a pile of 24 bodies falling onto a
floor (`tests/wasm-safe/src/scene.rs`) is stepped 300 ticks in a wasm32-unknown-unknown module, in
Node in single and double precision and in headless Chrome in single precision, and its per-tick
records (body id, position, rotation, velocities as exact bits, sleep flag) must equal those of a
native run of the same scene byte for byte, both sides built with `cross-platform-deterministic`.
The script also requires a native run without the feature and a record with one flipped bit to
differ. CI's `wasm` job runs it on Linux (GCC) with wasi-sdk 34.

## Replays

`PhysicsWorld::save_state` and `restore_state` roll a world back for replays; after a restore the
same calls give the same results bit for bit, except for the configuration Jolt does not save
([state.md](state.md)). `CharacterRef::save_state` and `CharacterMut::restore_state` continue a
single character in a world rebuilt the same way.
