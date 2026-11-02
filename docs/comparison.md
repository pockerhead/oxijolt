# Comparison with Rapier and Avian

A comparison of oxijolt with [Rapier](https://rapier.rs) (`rapier3d`) and
[Avian](https://github.com/avianphysics/avian) (`avian3d`) for people choosing a Rust physics
engine: speed on Rapier's own stress scenes, the quality each engine kept while running them, what
each engine offers, and where oxijolt loses. Every number in the results comes from runs whose raw
files are committed under
[`benches/comparison/results/2026-10-07-linux-5950x/`](../benches/comparison/results/2026-10-07-linux-5950x/);
a test (`benches/comparison/tests/doc_tables.rs`) checks that each results row here is in that
directory's `summary.md` and that the `summary.md` is what `comparison summarize` makes of the raw
files.

## Summary

Ten of Rapier's stress scenes on a 16-core Linux machine, matched settings, warm ticks, at 1, 4,
8 and 16 threads:

- **Scenes of many contacts: Rapier is faster.** Rapier's step is 1.9 to 2.2 times faster than
  oxijolt's on `boxes` at every thread count, 1.3 to 1.85 times on `pyramid`, 1.2 to 1.75 times
  on `capsules`, and 1.4 times on `balls` up to 8 threads (level at 16). On `keva` Rapier is 1.6
  times faster at one thread and 1.2 times at four; oxijolt is 1.1 to 1.2 times faster at 8 and
  16. On `many_pyramids` they are within 8 % up to four threads; oxijolt is 1.2 times faster at 8,
  Rapier 1.9 times at 16.
- **Joints from four threads: oxijolt is faster**, 1.1 to 2.1 times on `joint_fixed` (at every
  thread count), 1.1 to 1.7 times on `joint_prismatic` and 1.3 to 1.7 times on `joint_revolute`
  at 8 and 16 threads. At one thread Rapier is 1.2 times faster on the last two. `joint_ball` is
  the exception: Rapier is 1.1 to 1.2 times faster at one and four threads and 3.4 to 4.3 times
  at 8 and 16, where oxijolt's step is slower than its own four-thread step; we have not found
  why.
- **Avian** (its whole Bevy update) is the slowest of the three on every scene and thread count
  but two: on `keva` it is faster than oxijolt at one thread and level with Rapier at 8 and 16,
  and on `joint_fixed` at one thread it is level with oxijolt and ahead of Rapier.
- **Quality**: oxijolt and Rapier each kept 7 of the 10 scenes within the bounds (Avian 4); with
  each engine's defaults Rapier kept 8, oxijolt 7 and Avian 5. oxijolt lets 12 of 3008 capsules
  through the ground, its `keva` towers collapse (Avian's too), and on `joint_revolute`, which no
  engine keeps within the bounds, its joint error is 4.5 times Rapier's. On `pyramid` oxijolt is
  the only engine within the bounds: Rapier's and Avian's deepest boxes end 47 and 33 cm in the
  ground. Avian's ball, prismatic and revolute joints come apart.
- **Fewer solver steps**: with 4 velocity steps instead of Jolt's 10 (`jolt-4`), oxijolt is
  within 8 % of Rapier on `pyramid` from four threads and closer on `boxes` (Rapier 1.2 to 1.65
  times faster), but leaves the bounds on five scenes where 10 steps stay within them.
- **Memory**: oxijolt's peak resident set is the smallest, 36 to 54 % of Rapier's at four threads.
- **Determinism**: every engine gave the same results at 1, 4, 8 and 16 threads, tick by tick,
  on every scene in both profiles.
- **Defaults** (each engine as shipped, sleeping on): Rapier is 5 times faster than oxijolt on
  `balls` and 8 to 9 times on `boxes`, most likely because its bodies fall asleep sooner (the
  runs count awake bodies at tick 600 only: on `balls` none of Rapier's and 3743 of oxijolt's).

## Versions

| | |
|---|---|
| oxijolt | 1.0.0 with per-world solver step counts (`WorldSettings::velocity_steps`), Jolt 5.6.0, joltc `886e088` |
| Rapier | rapier3d 0.36.0 (parry3d 0.31.1) |
| Avian | avian3d 0.7.0 (parry3d 0.27.0), Bevy 0.19.1 |
| Rust | rustc 1.99.0 (b940084d7 2026-09-28), release profile, `RUSTFLAGS=-C target-cpu=x86-64-v3` |
| C++ | GCC 13.3.0; Jolt and joltc built from source by CMake in Release, default features |

Exact versions of every dependency are in `Cargo.lock` at the commit named in the results'
`build-info.txt`.

## Machine and build

An AMD Ryzen 9 5950X: 16 cores and 32 hardware threads in two complexes of 8 cores, each with
its own 32 MiB L3 (CPUs 0-7 and 8-15; their SMT siblings are 16-31), boost on, AVX2 and FMA;
31 GB of memory. A rented machine: Ubuntu 24.04 in a Docker container, kernel 6.8.0-138, the
`schedutil` frequency governor, which the container cannot change. The binaries were built on
it with `build_variants.py`; the compiler flags of Jolt and joltc (GCC 13.3, `-O3 -mavx2 -mfma`
and the rest) are in the results' `build-info.txt`, the topology in `lscpu.txt` and
`lscpu-e.txt`, every dependency with its features in `features.txt`. The results'
[README](../benches/comparison/results/2026-10-07-linux-5950x/README.md) says which binaries made
which runs.

Earlier partial runs on a Windows desktop, one repeat with other work on the same machine, served
to build the harness and are not published.

Build time and size of each engine's smallest program (`hello-<engine>` in
`benches/comparison/src/bin/`), release, from an empty target directory after `cargo fetch`, 16
jobs, from the results' `build.txt`:

| Build | Wall time | Executable | Stripped |
|---|---|---|---|
| oxijolt, Jolt and joltc compiled from source | 18.4 s | 2.54 MB | 1.94 MB |
| oxijolt, prebuilt native library (`JOLTC_LIB_DIR`) | 3.8 s | 2.54 MB | 1.94 MB |
| Rapier | 19.6 s | 3.43 MB | 2.80 MB |
| Avian, with the Bevy it needs | 42.4 s | 37.06 MB | 20.90 MB |

MB in this table is 10^6 bytes; the peak memory columns of the results are in MiB (2^20 bytes).

## Method

### Scenes

The ten scenes of Rapier's `examples3d/stress_tests` that every engine can build, at
[v0.36.0](https://github.com/dimforge/rapier/tree/v0.36.0/examples3d/stress_tests), ported once to
engine-neutral data (`benches/comparison/src/scenes/`) and built from that data by each engine's
adapter. A test steps Rapier's own scene code and the Rapier adapter's world side by side and
requires the same digests tick by tick (`tests/rapier_oracle.rs`); per-engine tests check
positions, creation order, fixed or dynamic, mass (`density * volume`, within 1e-4), the principal
moments of inertia where the engine exposes them, and shape sizes by ray casts.

| Scene | Bodies | Fixed | Joints | What |
|---|---|---|---|---|
| balls | 8000 | 400 | 0 | 20 x 20 x 20 balls, the bottom layer fixed, no ground |
| boxes | 1001 | 1 | 0 | ten layers of 10 x 10 cubes, each layer shifted |
| capsules | 3009 | 1 | 0 | 47 layers of 8 x 8 capsules dropped 3 m into a heap |
| pyramid | 42926 | 1 | 0 | 50 brick-laid layers of (50 - i)² boxes of water density, 0.55 m apart |
| many_pyramids | 8401 | 1 | 0 | 40 pyramids of 20 rows |
| keva | 38271 | 1 | 0 | five stacked Keva plank towers |
| joint_ball | 10000 | 26 | 19800 | a 100 x 100 net of balls on ball joints |
| joint_fixed | 12500 | 1000 | 20000 | 500 small nets on fixed joints |
| joint_prismatic | 19200 | 3200 | 16000 | 3200 chains of five cubes on sliders with limits |
| joint_revolute | 8200 | 200 | 8000 | 200 chains of hinged squares |

Rapier's other stress scenes (heightfields, trimeshes, convex decompositions, CCD and others) are
not ported.

### Settings

Every run: time step 1/60 s, gravity (0, -9.81, 0), the scene's geometry, densities and creation
order, sharp boxes in every engine (Jolt's boxes without convex radius), 600 ticks.

**Matched profile** (the headline): friction 0.5, restitution 0, no damping, no sleeping on any
body, no continuous collision (Rapier `max_ccd_substeps = 0`, Jolt's discrete motion, no Avian
`SweptCcd`), a 2 cm speculative contact distance in all three (Avian's
`NarrowPhaseConfig::default_speculative_margin`), bodies joined by a joint still collide (each
engine's default), no events. Jolt's mass is set to density times volume; Jolt scales its shape's
inertia to it.

**Solver iterations**, each a row of its own:

| Variant | Engine | Solver | Build |
|---|---|---|---|
| `jolt` | oxijolt | 10 velocity and 2 position steps (Jolt's default) | |
| `jolt-4` | oxijolt | 4 velocity and 2 position steps | |
| `rapier-par` | Rapier | 4 iterations (default), 4-lane SIMD | `parallel` |
| `rapier-serial` | Rapier | the same | without `parallel` |
| `rapier-simd8` | Rapier | the same, 8-lane SIMD (tuned) | `parallel`, `simd8` |
| `avian-par` | Avian | 4 substeps (as Avian's own benchmarks), joints with XPBD | `parallel` |
| `avian-serial` | Avian | the same | without `parallel` |

The shared knob is iterations per step: Rapier's solver iterations, Avian's substeps, Jolt's
velocity steps. Jolt keeps its position pass at the default because the other two have no such
pass. A TGS substep also integrates positions, so equal counts are equal pass counts, not equal
work.

**Defaults profile**: each engine as shipped except the time step, gravity and the scene: sleeping
on with each engine's thresholds, Rapier's CCD settings, Jolt's friction 0.2 and damping 0.05,
Avian's 6 substeps and unbounded speculative margin. Bodies awake at the end are reported.

**Not matched, and stated**: the solvers themselves (Jolt: sequential impulses with a position
pass; Rapier and Avian: soft-step TGS; Avian: XPBD joints); Jolt's two broad-phase layers and its
`optimize_broad_phase()` after building (the others build their trees in the first tick);
velocity clamps (Jolt 500 m/s and 0.25 pi * 60 rad/s per body, Rapier 400 m/s); Jolt's
`deterministicSimulation` (on); Jolt's preallocated buffers (below); Jolt shares one shape object
between bodies of the same shape, Rapier and Avian create one per collider, as their scene code
does; Rapier's contact recycling and clustering; friction combine rules (equal here, every body has
0.5).

**Jolt's buffers**: `max_bodies` is the body count; `max_body_pairs` and `max_contact_constraints`
the smallest power of two from 2^14 at which 600 matched ticks at one thread drop no contact,
doubled once; the temp allocator 1 KiB per contact constraint plus 64 MiB, doubled while 120 ticks
ran over 3 % slower than with twice the size (`comparison size-jolt`; the table is in
`benches/comparison/src/engines/jolt.rs`). A step that drops a contact fails the run.

### Threads and pinning

Thread counts 1, 4, 8 and 16, counting the thread that steps. One thread is sequential: Jolt with a
job system of concurrency 1 that leaves every job to the stepping thread; Rapier and Avian from
builds without `parallel`. The parallel builds at one thread are rows of their own. With N threads:
Jolt's thread pool with N - 1 workers plus the stepping thread; Rapier's own pool of N
(`configure_thread_pool`, which the step installs itself) and a global Rayon pool of N; Avian's
Bevy compute pool and a global Rayon pool of N, set before the app exists. A run with N threads is
pinned with `taskset` to N logical CPUs on N different physical cores (CPUs 0 to N - 1, which
`lscpu -e` lists as cores 0 to N - 1; their SMT siblings are 16 to 31 and stay unused). Every row
reports busy cores, the process's CPU time over the wall time of the timed loop; spinning workers
count as busy.

### What is timed

- Jolt: `PhysicsWorld::step`. Rapier: `PhysicsWorld::step`. Avian: `App::update`, which also runs
  Bevy's schedule, time and transform propagation and Avian's systems outside its physics
  schedule; every update must advance Avian's physics clock by exactly one step, or the run fails.
  The split runs (below) time Avian's physics schedule inside the update.
- Build: Jolt's world creation, body and constraint creation and `optimize_broad_phase`; Rapier's
  insertion; Avian's app with its plugins finished and one update with the physics clock paused
  (spawning and collider setup). Tick 1 is the first simulated tick in all three.
- Windows of ticks: tick 1, build plus tick 1, cold (ticks 1-60: the drop and first impacts) and
  warm (ticks 61-600). Means, nearest-rank percentiles and maxima are over the pooled samples of
  the three repeats of a row; the range of the per-run warm means shows the spread between runs.
- Each run is a process of its own, from a binary built with one engine only, so feature
  unification between engines cannot change another engine's code and peak memory belongs to one
  engine. The order of the variants rotates between repeats.
- Memory: the process's peak resident set (`VmHWM`), read right after the last tick. It covers the
  process image, thread pools, Bevy, Jolt's reservations and native allocations.
- Load: before a timed run the campaign waits until other processes use under 10 % of the machine
  in two 5 s windows (from `/proc/stat`, minus the comparison's own processes), samples that load
  about once a minute during the run and once after it, and runs the case again (up to five
  attempts) when a sample passed 10 %. Every attempt is in the results' `load.tsv`.
- The CPU frequency governor is `schedutil` and boost cannot be turned off on the rented machine,
  which is why every timed row has three repeats and shows the spread of their means.

### Quality and determinism

Separate validation runs read every body each tick (timing runs read nothing) and check bounds
that separate a broken simulation from a soft one: a stack keeps at least 95 % of the height it
had at tick 300 (after the drops and the pyramid's settling) and its 99th percentile speed at tick 600 is at most 0.1 m/s; no
body ends more than 5 cm into the ground or more than 1 m below it; balls overlap by at most 5 cm
at tick 600; joints separate by at most 10 cm, twist by at most 0.1 rad and pass a slider limit by
at most 10 cm, as the per-tick 99th percentile over the joints averaged over ticks 61-600. The
capsule scene is a heap, not a stack, so it has no height or final-speed bound. Peaks during impacts
(deepest penetration and overlap, largest joint error) are reported next to the bounded values.
A row outside a bound is shown as "below bound" next to its time, never dropped.

Each validation run also records a digest of every body's position, rotation, velocities and sleep
flag per tick. The digests of 1, 4, 8 and 16 threads are compared tick by tick per engine and
scene; engines are not compared with each other. A scene where no body moved faster than 0.1 m/s
proves nothing and would be reported as "not moving".

## Results

Linux on the 5950X above, release builds, 600 ticks per run, three repeats. The tables are copied
from [`summary.md`](../benches/comparison/results/2026-10-07-linux-5950x/summary.md), which also has
every quality and determinism row. Columns: warm mean, nearest-rank percentiles and maximum over
the pooled ticks 61-600 of all repeats; run means is the range of the per-run warm means; cold is
ticks 1-60; tick 1 and build plus tick 1 are medians over the repeats; busy cores is CPU time over
wall time; peak memory is the peak resident set; quality is the validation run's verdict for the
same variant, scene, profile and thread count. Times are milliseconds per tick.

### Matched profile

<!-- generated:timing-matched -->

**balls**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 9.139 | 9.414 | 9.490 | 9.736 | 10.145 | 9.130–9.153 | 4.259 | 9.610 | 72.864 | 1.00 | 40 | within bounds |
| avian-par | 4 | 3 | 4.278 | 4.293 | 4.524 | 5.801 | 6.657 | 4.270–4.290 | 2.531 | 7.803 | 71.344 | 2.53 | 43 | within bounds |
| avian-par | 8 | 3 | 4.058 | 4.100 | 4.279 | 4.421 | 4.624 | 4.035–4.079 | 2.421 | 7.859 | 72.208 | 3.10 | 46 | within bounds |
| avian-par | 16 | 3 | 6.705 | 6.466 | 8.262 | 8.555 | 9.094 | 6.481–6.998 | 3.253 | 8.725 | 75.010 | 3.62 | 52 | within bounds |
| avian-serial | 1 | 3 | 8.099 | 8.330 | 8.424 | 8.620 | 8.970 | 8.095–8.101 | 3.422 | 8.811 | 70.440 | 1.00 | 39 | not validated |
| jolt | 1 | 3 | 5.135 | 5.338 | 5.699 | 6.094 | 6.216 | 5.107–5.166 | 1.660 | 1.064 | 6.062 | 1.00 | 15 | within bounds |
| jolt | 4 | 3 | 1.933 | 1.996 | 2.083 | 2.160 | 2.221 | 1.923–1.945 | 0.912 | 0.817 | 6.095 | 3.85 | 16 | within bounds |
| jolt | 8 | 3 | 1.370 | 1.401 | 1.449 | 1.471 | 1.530 | 1.365–1.375 | 0.956 | 0.606 | 5.929 | 7.44 | 17 | within bounds |
| jolt | 16 | 3 | 1.846 | 1.857 | 1.888 | 1.917 | 2.060 | 1.841–1.850 | 1.696 | 0.970 | 6.726 | 14.09 | 18 | within bounds |
| jolt-4 | 1 | 3 | 4.541 | 4.581 | 5.222 | 5.491 | 5.594 | 4.513–4.578 | 1.626 | 1.038 | 6.026 | 1.00 | 15 | within bounds |
| jolt-4 | 4 | 3 | 1.776 | 1.800 | 1.938 | 2.024 | 2.214 | 1.771–1.782 | 0.904 | 0.786 | 6.061 | 3.84 | 16 | within bounds |
| jolt-4 | 8 | 3 | 1.280 | 1.298 | 1.353 | 1.382 | 1.427 | 1.277–1.283 | 0.962 | 0.745 | 5.978 | 7.39 | 17 | within bounds |
| jolt-4 | 16 | 3 | 1.822 | 1.830 | 1.858 | 1.874 | 2.026 | 1.819–1.825 | 1.666 | 0.858 | 6.487 | 14.07 | 18 | within bounds |
| rapier-par | 1 | 3 | 3.653 | 3.679 | 4.006 | 4.385 | 5.542 | 3.644–3.662 | 2.200 | 8.321 | 12.691 | 1.00 | 37 | within bounds |
| rapier-par | 4 | 3 | 1.360 | 1.336 | 1.546 | 2.195 | 2.569 | 1.351–1.373 | 1.500 | 8.050 | 12.855 | 3.44 | 38 | within bounds |
| rapier-par | 8 | 3 | 0.968 | 0.945 | 1.143 | 1.618 | 2.065 | 0.950–0.978 | 1.425 | 8.191 | 13.159 | 6.10 | 38 | within bounds |
| rapier-par | 16 | 3 | 1.905 | 1.843 | 2.313 | 2.855 | 3.485 | 1.899–1.912 | 2.169 | 9.637 | 15.712 | 11.98 | 39 | within bounds |
| rapier-serial | 1 | 3 | 3.566 | 3.606 | 3.879 | 4.205 | 4.873 | 3.564–3.568 | 2.096 | 7.538 | 11.611 | 1.00 | 35 | not validated |
| rapier-simd8 | 1 | 3 | 3.287 | 3.279 | 3.651 | 4.048 | 7.733 | 3.260–3.312 | 2.150 | 8.355 | 12.648 | 1.00 | 38 | not validated |
| rapier-simd8 | 4 | 3 | 1.240 | 1.195 | 1.422 | 2.020 | 5.176 | 1.215–1.272 | 1.471 | 8.013 | 12.675 | 3.45 | 38 | not validated |

**boxes**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 11.396 | 11.308 | 12.700 | 12.786 | 12.894 | 11.380–11.423 | 13.245 | 10.523 | 18.476 | 1.00 | 30 | below bound: final_speed_p99 |
| avian-par | 4 | 3 | 6.409 | 6.421 | 7.001 | 7.493 | 7.977 | 6.387–6.442 | 7.009 | 7.348 | 15.400 | 2.51 | 32 | below bound: final_speed_p99 |
| avian-par | 8 | 3 | 5.524 | 5.494 | 6.031 | 6.128 | 6.462 | 5.466–5.575 | 6.052 | 5.864 | 14.539 | 3.34 | 33 | below bound: final_speed_p99 |
| avian-par | 16 | 3 | 10.016 | 9.933 | 12.362 | 12.659 | 12.932 | 9.414–10.388 | 9.389 | 6.000 | 15.387 | 3.67 | 37 | below bound: final_speed_p99 |
| avian-serial | 1 | 3 | 10.852 | 10.804 | 11.986 | 12.062 | 12.164 | 10.837–10.863 | 12.602 | 9.947 | 17.102 | 1.00 | 29 | not validated |
| jolt | 1 | 3 | 6.442 | 6.392 | 6.659 | 7.452 | 8.558 | 6.405–6.516 | 8.058 | 42.661 | 43.787 | 1.00 | 13 | within bounds |
| jolt | 4 | 3 | 2.537 | 2.535 | 2.583 | 2.608 | 2.635 | 2.517–2.558 | 2.970 | 12.234 | 13.494 | 3.92 | 13 | within bounds |
| jolt | 8 | 3 | 1.806 | 1.805 | 1.832 | 1.846 | 1.865 | 1.796–1.816 | 2.016 | 7.192 | 8.550 | 7.70 | 14 | within bounds |
| jolt | 16 | 3 | 3.092 | 3.088 | 3.149 | 3.217 | 3.355 | 3.087–3.099 | 3.411 | 8.068 | 9.609 | 14.88 | 15 | within bounds |
| jolt-4 | 1 | 3 | 4.127 | 4.109 | 4.278 | 4.558 | 4.629 | 4.108–4.157 | 6.926 | 39.801 | 40.931 | 1.00 | 13 | within bounds |
| jolt-4 | 4 | 3 | 1.801 | 1.798 | 1.841 | 1.889 | 1.981 | 1.793–1.816 | 2.519 | 11.353 | 12.620 | 3.85 | 14 | within bounds |
| jolt-4 | 8 | 3 | 1.349 | 1.348 | 1.367 | 1.380 | 1.419 | 1.346–1.352 | 1.694 | 6.531 | 7.950 | 7.50 | 14 | within bounds |
| jolt-4 | 16 | 3 | 2.348 | 2.363 | 2.495 | 2.512 | 2.526 | 2.223–2.453 | 2.602 | 6.959 | 8.505 | 14.59 | 15 | within bounds |
| rapier-par | 1 | 3 | 3.326 | 3.310 | 3.413 | 3.459 | 3.592 | 3.303–3.362 | 3.693 | 23.352 | 24.243 | 1.00 | 25 | within bounds |
| rapier-par | 4 | 3 | 1.152 | 1.147 | 1.186 | 1.203 | 1.505 | 1.139–1.169 | 1.346 | 12.392 | 13.681 | 3.90 | 25 | within bounds |
| rapier-par | 8 | 3 | 0.816 | 0.802 | 0.961 | 1.022 | 1.199 | 0.757–0.891 | 1.141 | 12.393 | 13.873 | 7.58 | 26 | within bounds |
| rapier-par | 16 | 3 | 1.606 | 1.544 | 1.854 | 1.900 | 2.439 | 1.502–1.797 | 1.900 | 15.692 | 18.039 | 14.99 | 26 | within bounds |
| rapier-serial | 1 | 3 | 3.271 | 3.277 | 3.328 | 3.402 | 3.542 | 3.230–3.299 | 3.583 | 19.319 | 19.997 | 1.00 | 24 | not validated |
| rapier-simd8 | 1 | 3 | 2.616 | 2.560 | 2.755 | 2.874 | 3.470 | 2.550–2.739 | 3.031 | 23.081 | 23.913 | 0.99 | 25 | not validated |
| rapier-simd8 | 4 | 3 | 0.926 | 0.924 | 0.947 | 0.953 | 0.965 | 0.914–0.938 | 1.127 | 12.295 | 13.401 | 3.92 | 25 | not validated |

**capsules**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 10.119 | 11.281 | 13.436 | 13.538 | 13.678 | 10.072–10.176 | 3.079 | 7.167 | 28.648 | 1.00 | 35 | within bounds |
| avian-par | 4 | 3 | 5.507 | 5.787 | 7.334 | 9.079 | 10.140 | 5.496–5.517 | 1.660 | 5.644 | 27.217 | 2.39 | 37 | within bounds |
| avian-par | 8 | 3 | 4.571 | 5.024 | 5.947 | 6.091 | 6.370 | 4.552–4.594 | 1.480 | 5.350 | 27.503 | 3.24 | 39 | within bounds |
| avian-par | 16 | 3 | 7.420 | 7.731 | 11.374 | 11.677 | 11.944 | 7.018–7.679 | 1.969 | 5.877 | 29.740 | 3.83 | 44 | within bounds |
| avian-serial | 1 | 3 | 9.303 | 10.410 | 12.669 | 12.894 | 16.762 | 9.286–9.318 | 2.497 | 6.393 | 27.231 | 1.00 | 34 | not validated |
| jolt | 1 | 3 | 8.107 | 10.251 | 11.150 | 11.417 | 11.586 | 8.042–8.161 | 1.014 | 1.067 | 3.347 | 1.00 | 13 | below bound: ground_penetration, fallen |
| jolt | 4 | 3 | 2.915 | 3.624 | 3.965 | 4.053 | 4.080 | 2.910–2.919 | 0.479 | 0.585 | 3.142 | 3.74 | 14 | below bound: ground_penetration, fallen |
| jolt | 8 | 3 | 2.006 | 2.460 | 2.706 | 2.752 | 2.845 | 2.002–2.012 | 0.366 | 0.426 | 2.899 | 7.07 | 14 | below bound: ground_penetration, fallen |
| jolt | 16 | 3 | 2.928 | 3.416 | 4.001 | 4.104 | 4.210 | 2.878–2.999 | 0.668 | 0.587 | 3.517 | 13.66 | 16 | below bound: ground_penetration, fallen |
| jolt-4 | 1 | 3 | 8.306 | 10.495 | 12.139 | 12.394 | 12.540 | 8.274–8.358 | 0.967 | 1.034 | 3.311 | 1.00 | 15 | below bound: ground_penetration, fallen |
| jolt-4 | 4 | 3 | 3.037 | 3.847 | 4.328 | 4.425 | 4.472 | 3.029–3.043 | 0.468 | 0.602 | 3.108 | 3.74 | 15 | below bound: ground_penetration, fallen |
| jolt-4 | 8 | 3 | 2.108 | 2.644 | 2.995 | 3.034 | 3.083 | 2.104–2.114 | 0.360 | 0.441 | 3.046 | 7.10 | 16 | below bound: ground_penetration, fallen |
| jolt-4 | 16 | 3 | 2.901 | 3.514 | 3.963 | 4.130 | 4.242 | 2.891–2.909 | 0.639 | 0.564 | 3.473 | 13.56 | 17 | below bound: ground_penetration, fallen |
| rapier-par | 1 | 3 | 4.630 | 5.141 | 6.063 | 6.167 | 6.460 | 4.619–4.645 | 1.127 | 6.290 | 8.481 | 1.00 | 37 | within bounds |
| rapier-par | 4 | 3 | 1.775 | 1.929 | 2.311 | 2.386 | 2.792 | 1.770–1.784 | 0.801 | 5.033 | 7.704 | 3.59 | 39 | within bounds |
| rapier-par | 8 | 3 | 1.312 | 1.392 | 1.704 | 1.756 | 2.066 | 1.305–1.319 | 0.772 | 4.911 | 7.827 | 6.52 | 40 | within bounds |
| rapier-par | 16 | 3 | 2.431 | 2.588 | 3.082 | 3.182 | 3.392 | 2.391–2.478 | 1.182 | 5.870 | 9.664 | 11.72 | 40 | within bounds |
| rapier-serial | 1 | 3 | 4.524 | 5.016 | 5.935 | 6.140 | 9.541 | 4.478–4.586 | 1.041 | 5.314 | 7.250 | 1.00 | 36 | not validated |
| rapier-simd8 | 1 | 3 | 4.392 | 4.872 | 5.820 | 5.965 | 7.286 | 4.379–4.410 | 1.110 | 6.274 | 8.435 | 1.00 | 38 | not validated |
| rapier-simd8 | 4 | 3 | 1.713 | 1.837 | 2.246 | 2.332 | 3.388 | 1.703–1.731 | 0.814 | 4.970 | 7.602 | 3.58 | 39 | not validated |

**pyramid**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 589.059 | 600.149 | 615.133 | 621.292 | 683.607 | 588.179–589.521 | 103.394 | 47.844 | 889.391 | 1.00 | 285 | below bound: ground_penetration, final_speed_p99 |
| avian-par | 4 | 3 | 231.822 | 235.022 | 252.520 | 257.242 | 308.585 | 230.385–232.831 | 43.800 | 36.086 | 888.149 | 2.78 | 329 | below bound: ground_penetration, final_speed_p99 |
| avian-par | 8 | 3 | 165.067 | 168.955 | 179.960 | 183.382 | 209.460 | 164.714–165.466 | 33.418 | 35.029 | 893.298 | 4.39 | 314 | below bound: ground_penetration, final_speed_p99 |
| avian-par | 16 | 3 | 161.380 | 164.042 | 176.319 | 180.898 | 239.327 | 160.664–162.119 | 38.698 | 37.276 | 892.577 | 6.03 | 349 | below bound: ground_penetration, final_speed_p99 |
| avian-serial | 1 | 3 | 582.178 | 592.939 | 607.578 | 621.007 | 679.104 | 581.492–582.690 | 99.945 | 43.803 | 875.095 | 1.00 | 283 | not validated |
| jolt | 1 | 3 | 473.674 | 473.264 | 535.147 | 542.644 | 632.148 | 473.259–474.348 | 73.989 | 8.825 | 39.153 | 1.00 | 183 | within bounds |
| jolt | 4 | 3 | 173.360 | 174.863 | 189.249 | 191.437 | 238.574 | 172.722–173.841 | 24.860 | 5.132 | 36.348 | 3.90 | 188 | within bounds |
| jolt | 8 | 3 | 122.948 | 125.456 | 130.338 | 132.714 | 188.668 | 122.814–123.056 | 16.975 | 3.555 | 34.915 | 7.66 | 192 | within bounds |
| jolt | 16 | 3 | 119.346 | 122.987 | 125.867 | 133.791 | 171.015 | 118.135–120.037 | 18.782 | 4.606 | 36.512 | 15.07 | 202 | within bounds |
| jolt-4 | 1 | 3 | 311.008 | 298.841 | 402.265 | 427.769 | 520.825 | 310.945–311.128 | 69.585 | 8.796 | 38.892 | 1.00 | 185 | below bound: final_speed_p99 |
| jolt-4 | 4 | 3 | 119.712 | 117.962 | 141.763 | 147.872 | 210.531 | 119.424–120.053 | 23.011 | 5.222 | 36.367 | 3.85 | 191 | below bound: final_speed_p99 |
| jolt-4 | 8 | 3 | 87.224 | 87.602 | 97.595 | 100.940 | 154.940 | 87.012–87.450 | 15.850 | 3.615 | 33.669 | 7.51 | 194 | below bound: final_speed_p99 |
| jolt-4 | 16 | 3 | 86.186 | 87.869 | 92.118 | 98.741 | 132.972 | 86.008–86.349 | 16.984 | 4.271 | 35.631 | 14.71 | 203 | below bound: final_speed_p99 |
| rapier-par | 1 | 3 | 256.213 | 261.979 | 265.904 | 268.976 | 345.855 | 255.866–256.562 | 49.858 | 57.007 | 85.878 | 1.00 | 482 | below bound: ground_penetration |
| rapier-par | 4 | 3 | 113.148 | 115.684 | 117.408 | 119.672 | 168.635 | 113.075–113.203 | 24.705 | 54.474 | 83.352 | 3.80 | 484 | below bound: ground_penetration |
| rapier-par | 8 | 3 | 94.378 | 96.614 | 97.730 | 98.964 | 99.831 | 94.155–94.540 | 23.579 | 55.231 | 84.522 | 7.41 | 483 | below bound: ground_penetration |
| rapier-par | 16 | 3 | 81.097 | 82.637 | 86.900 | 93.468 | 134.393 | 80.740–81.407 | 22.186 | 59.737 | 90.634 | 14.37 | 485 | below bound: ground_penetration |
| rapier-serial | 1 | 3 | 251.521 | 256.666 | 262.454 | 274.853 | 338.982 | 250.481–253.352 | 48.457 | 53.319 | 81.907 | 1.00 | 474 | not validated |
| rapier-simd8 | 1 | 3 | 245.006 | 250.153 | 254.612 | 257.748 | 315.688 | 244.460–245.941 | 49.222 | 56.781 | 85.524 | 1.00 | 482 | not validated |
| rapier-simd8 | 4 | 3 | 102.509 | 104.389 | 106.900 | 111.925 | 186.501 | 102.240–102.735 | 24.553 | 54.106 | 82.575 | 3.77 | 483 | not validated |

**many_pyramids**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 69.058 | 68.825 | 70.214 | 74.065 | 87.059 | 68.665–69.551 | 65.710 | 35.330 | 104.644 | 1.00 | 65 | within bounds |
| avian-par | 4 | 3 | 27.086 | 26.928 | 30.439 | 30.742 | 33.326 | 26.801–27.424 | 23.709 | 21.504 | 92.458 | 2.90 | 70 | within bounds |
| avian-par | 8 | 3 | 17.669 | 17.632 | 18.324 | 18.855 | 20.741 | 17.429–17.848 | 17.229 | 17.718 | 88.717 | 4.74 | 70 | within bounds |
| avian-par | 16 | 3 | 27.035 | 25.382 | 31.490 | 31.726 | 32.044 | 24.789–28.637 | 25.992 | 16.890 | 89.406 | 5.50 | 82 | within bounds |
| avian-serial | 1 | 3 | 66.815 | 66.645 | 67.504 | 71.652 | 75.901 | 66.718–66.869 | 63.501 | 33.552 | 101.680 | 1.00 | 63 | not validated |
| jolt | 1 | 3 | 25.145 | 25.069 | 25.358 | 27.236 | 43.788 | 24.950–25.357 | 27.524 | 61.454 | 66.981 | 1.00 | 33 | within bounds |
| jolt | 4 | 3 | 8.451 | 8.450 | 8.506 | 8.527 | 9.118 | 8.430–8.476 | 9.027 | 19.953 | 25.673 | 3.95 | 34 | within bounds |
| jolt | 8 | 3 | 5.245 | 5.243 | 5.293 | 5.310 | 5.537 | 5.235–5.253 | 5.454 | 12.553 | 18.480 | 7.82 | 35 | within bounds |
| jolt | 16 | 3 | 9.419 | 9.417 | 10.252 | 10.770 | 11.237 | 9.193–9.628 | 9.590 | 14.366 | 20.474 | 15.56 | 37 | within bounds |
| jolt-4 | 1 | 3 | 17.562 | 17.477 | 18.135 | 20.153 | 22.761 | 17.373–17.663 | 25.650 | 51.303 | 56.940 | 1.00 | 33 | below bound: final_speed_p99 |
| jolt-4 | 4 | 3 | 6.076 | 6.067 | 6.240 | 6.783 | 7.429 | 6.026–6.115 | 8.025 | 17.015 | 22.897 | 3.93 | 34 | below bound: final_speed_p99 |
| jolt-4 | 8 | 3 | 3.916 | 3.902 | 4.158 | 4.319 | 4.635 | 3.862–3.989 | 4.799 | 11.176 | 17.729 | 7.75 | 35 | below bound: final_speed_p99 |
| jolt-4 | 16 | 3 | 7.163 | 7.372 | 7.516 | 7.551 | 7.604 | 6.731–7.395 | 7.438 | 12.534 | 18.702 | 15.38 | 37 | below bound: final_speed_p99 |
| rapier-par | 1 | 3 | 23.231 | 23.150 | 24.329 | 24.973 | 25.786 | 23.129–23.338 | 24.777 | 97.384 | 104.555 | 1.00 | 86 | within bounds |
| rapier-par | 4 | 3 | 8.418 | 8.413 | 8.577 | 8.717 | 8.939 | 8.372–8.464 | 9.145 | 53.811 | 61.338 | 3.73 | 86 | within bounds |
| rapier-par | 8 | 3 | 6.272 | 6.267 | 6.364 | 6.457 | 8.647 | 6.262–6.277 | 7.019 | 51.355 | 58.845 | 7.11 | 87 | within bounds |
| rapier-par | 16 | 3 | 4.917 | 4.915 | 5.262 | 5.941 | 6.457 | 4.732–5.077 | 5.823 | 56.076 | 64.621 | 13.33 | 87 | within bounds |
| rapier-serial | 1 | 3 | 23.624 | 23.633 | 24.885 | 25.292 | 25.931 | 23.515–23.759 | 24.482 | 82.688 | 89.279 | 1.00 | 83 | not validated |
| rapier-simd8 | 1 | 3 | 21.880 | 21.756 | 23.604 | 24.539 | 25.060 | 21.654–22.112 | 23.089 | 96.725 | 103.666 | 1.00 | 86 | not validated |
| rapier-simd8 | 4 | 3 | 7.804 | 7.790 | 7.988 | 8.128 | 8.395 | 7.791–7.819 | 8.514 | 53.476 | 60.628 | 3.70 | 86 | not validated |

**keva**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 506.118 | 587.790 | 676.791 | 686.943 | 742.031 | 504.161–509.328 | 643.068 | 314.315 | 1029.065 | 1.00 | 434 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| avian-par | 4 | 3 | 195.277 | 219.053 | 246.462 | 250.315 | 277.198 | 194.847–195.670 | 248.468 | 152.132 | 856.561 | 2.64 | 503 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| avian-par | 8 | 3 | 132.520 | 145.520 | 166.926 | 171.477 | 221.550 | 132.153–132.918 | 170.470 | 118.354 | 835.955 | 4.18 | 478 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| avian-par | 16 | 3 | 121.566 | 131.401 | 153.826 | 161.352 | 216.728 | 121.132–122.048 | 167.164 | 131.848 | 856.240 | 5.96 | 488 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| avian-serial | 1 | 3 | 496.560 | 576.022 | 663.304 | 676.732 | 780.875 | 494.907–497.501 | 634.465 | 306.636 | 1011.842 | 1.00 | 432 | not validated |
| jolt | 1 | 3 | 577.070 | 592.871 | 834.559 | 850.231 | 954.532 | 576.249–578.468 | 814.740 | 975.060 | 1003.044 | 1.00 | 240 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| jolt | 4 | 3 | 191.701 | 199.664 | 276.785 | 281.186 | 367.747 | 191.618–191.862 | 283.819 | 348.684 | 377.902 | 3.93 | 245 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| jolt | 8 | 3 | 122.124 | 127.704 | 177.464 | 186.316 | 222.705 | 121.915–122.305 | 195.172 | 244.070 | 272.519 | 7.74 | 248 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| jolt | 16 | 3 | 102.088 | 107.713 | 151.332 | 161.700 | 192.268 | 101.633–102.760 | 182.778 | 276.332 | 305.328 | 15.18 | 254 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| jolt-4 | 1 | 3 | 745.867 | 767.674 | 1177.652 | 1225.775 | 1290.947 | 745.069–746.837 | 589.750 | 743.277 | 771.035 | 1.00 | 303 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| jolt-4 | 4 | 3 | 241.447 | 247.111 | 377.131 | 395.977 | 401.437 | 240.756–241.877 | 206.025 | 271.120 | 299.506 | 3.94 | 307 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| jolt-4 | 8 | 3 | 153.205 | 154.975 | 240.089 | 252.914 | 257.238 | 152.958–153.472 | 142.536 | 189.660 | 218.392 | 7.76 | 309 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| jolt-4 | 16 | 3 | 125.429 | 125.735 | 196.074 | 207.295 | 219.445 | 125.055–125.854 | 132.565 | 219.005 | 248.089 | 15.23 | 316 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| rapier-par | 1 | 3 | 352.861 | 352.424 | 360.251 | 368.786 | 451.404 | 351.938–354.373 | 371.938 | 1097.665 | 1125.667 | 1.00 | 678 | below bound: final_speed_p99 |
| rapier-par | 4 | 3 | 164.183 | 163.870 | 166.611 | 173.786 | 249.092 | 163.861–164.405 | 173.160 | 584.109 | 612.309 | 3.83 | 681 | below bound: final_speed_p99 |
| rapier-par | 8 | 3 | 137.334 | 137.155 | 138.977 | 141.168 | 208.801 | 137.068–137.481 | 145.428 | 537.131 | 566.002 | 7.51 | 687 | below bound: final_speed_p99 |
| rapier-par | 16 | 3 | 122.105 | 121.886 | 124.546 | 129.483 | 197.895 | 121.761–122.489 | 130.964 | 572.200 | 601.847 | 14.73 | 692 | below bound: final_speed_p99 |
| rapier-serial | 1 | 3 | 356.382 | 355.403 | 365.194 | 374.492 | 454.193 | 353.032–360.010 | 370.593 | 925.590 | 953.111 | 1.00 | 663 | not validated |
| rapier-simd8 | 1 | 3 | 343.351 | 343.572 | 350.024 | 354.437 | 444.261 | 342.408–344.662 | 363.003 | 1060.282 | 1088.076 | 1.00 | 678 | not validated |
| rapier-simd8 | 4 | 3 | 146.720 | 146.602 | 149.043 | 150.124 | 157.087 | 146.540–147.043 | 155.831 | 569.968 | 597.554 | 3.81 | 680 | not validated |

**joint_ball**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 21.046 | 13.354 | 50.248 | 64.374 | 69.131 | 20.952–21.102 | 11.470 | 17.915 | 125.846 | 1.00 | 80 | below bound: anchor_p99 |
| avian-par | 4 | 3 | 15.209 | 10.691 | 32.674 | 41.187 | 47.278 | 15.126–15.285 | 9.330 | 14.633 | 123.471 | 1.62 | 87 | below bound: anchor_p99 |
| avian-par | 8 | 3 | 13.859 | 10.180 | 28.195 | 35.756 | 39.126 | 13.848–13.882 | 9.021 | 13.769 | 122.879 | 1.90 | 89 | below bound: anchor_p99 |
| avian-par | 16 | 3 | 16.895 | 11.775 | 35.926 | 44.367 | 48.076 | 16.334–17.204 | 10.446 | 15.912 | 126.986 | 2.43 | 92 | below bound: anchor_p99 |
| avian-serial | 1 | 3 | 20.060 | 12.374 | 48.676 | 62.580 | 64.663 | 19.844–20.285 | 10.516 | 16.508 | 121.298 | 1.00 | 79 | not validated |
| jolt | 1 | 3 | 11.364 | 11.233 | 11.868 | 13.132 | 25.938 | 11.237–11.592 | 10.322 | 7.826 | 225.895 | 1.00 | 26 | within bounds |
| jolt | 4 | 3 | 4.509 | 4.511 | 4.642 | 4.695 | 4.850 | 4.490–4.543 | 4.252 | 4.617 | 225.456 | 3.42 | 27 | within bounds |
| jolt | 8 | 3 | 10.677 | 10.217 | 17.188 | 18.393 | 19.074 | 8.325–12.128 | 3.776 | 4.116 | 217.150 | 6.23 | 28 | within bounds |
| jolt | 16 | 3 | 15.395 | 15.585 | 18.088 | 18.782 | 20.527 | 15.172–15.606 | 11.962 | 6.100 | 227.830 | 11.26 | 30 | within bounds |
| jolt-4 | 1 | 3 | 9.416 | 9.366 | 9.951 | 10.063 | 10.717 | 9.336–9.505 | 8.212 | 6.534 | 225.375 | 1.00 | 26 | below bound: anchor_p99 |
| jolt-4 | 4 | 3 | 3.771 | 3.763 | 3.948 | 4.015 | 4.367 | 3.741–3.811 | 3.477 | 3.654 | 224.446 | 3.32 | 27 | below bound: anchor_p99 |
| jolt-4 | 8 | 3 | 12.676 | 14.209 | 16.249 | 16.763 | 17.580 | 12.221–13.018 | 2.920 | 3.285 | 216.099 | 5.80 | 28 | below bound: anchor_p99 |
| jolt-4 | 16 | 3 | 8.277 | 5.355 | 15.987 | 16.768 | 18.202 | 5.145–14.520 | 7.271 | 5.133 | 229.973 | 10.81 | 30 | below bound: anchor_p99 |
| rapier-par | 1 | 3 | 10.508 | 10.364 | 11.518 | 11.769 | 12.971 | 10.441–10.562 | 10.388 | 34.672 | 48.544 | 1.00 | 61 | within bounds |
| rapier-par | 4 | 3 | 3.624 | 3.575 | 3.969 | 4.052 | 7.998 | 3.585–3.680 | 3.910 | 29.204 | 42.923 | 3.50 | 62 | within bounds |
| rapier-par | 8 | 3 | 2.492 | 2.467 | 2.722 | 2.782 | 2.964 | 2.482–2.497 | 3.176 | 27.946 | 42.356 | 6.23 | 63 | within bounds |
| rapier-par | 16 | 3 | 4.515 | 4.510 | 4.770 | 4.867 | 5.111 | 4.463–4.545 | 5.102 | 32.027 | 47.509 | 12.92 | 64 | within bounds |
| rapier-serial | 1 | 3 | 10.053 | 9.941 | 11.002 | 11.211 | 11.862 | 10.041–10.060 | 10.019 | 34.081 | 47.430 | 1.00 | 59 | not validated |
| rapier-simd8 | 1 | 3 | 8.808 | 8.691 | 9.816 | 10.024 | 12.413 | 8.728–8.891 | 8.619 | 33.180 | 46.895 | 1.00 | 61 | not validated |
| rapier-simd8 | 4 | 3 | 3.150 | 3.120 | 3.485 | 3.581 | 3.803 | 3.146–3.156 | 3.404 | 28.925 | 42.549 | 3.45 | 62 | not validated |

**joint_fixed**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 15.449 | 15.399 | 16.069 | 16.299 | 16.732 | 15.440–15.457 | 15.673 | 23.057 | 144.579 | 1.00 | 53 | within bounds |
| avian-par | 4 | 3 | 13.843 | 13.842 | 14.247 | 14.589 | 14.952 | 13.819–13.883 | 13.842 | 20.212 | 145.355 | 1.23 | 54 | within bounds |
| avian-par | 8 | 3 | 13.588 | 13.612 | 13.934 | 14.040 | 14.408 | 13.574–13.598 | 13.665 | 20.317 | 142.458 | 1.29 | 55 | within bounds |
| avian-par | 16 | 3 | 15.218 | 15.151 | 15.951 | 16.105 | 16.272 | 15.038–15.418 | 15.135 | 23.126 | 149.500 | 1.47 | 58 | within bounds |
| avian-serial | 1 | 3 | 14.550 | 14.523 | 15.140 | 15.397 | 16.463 | 14.538–14.560 | 14.774 | 21.597 | 141.292 | 1.00 | 51 | not validated |
| jolt | 1 | 3 | 15.436 | 15.433 | 15.542 | 16.091 | 17.693 | 15.354–15.484 | 15.351 | 14.864 | 242.356 | 1.00 | 31 | within bounds |
| jolt | 4 | 3 | 4.361 | 4.359 | 4.391 | 4.412 | 5.096 | 4.350–4.370 | 4.385 | 4.877 | 234.738 | 3.90 | 32 | within bounds |
| jolt | 8 | 3 | 2.361 | 2.362 | 2.392 | 2.404 | 2.520 | 2.352–2.369 | 2.404 | 2.863 | 232.830 | 7.62 | 32 | within bounds |
| jolt | 16 | 3 | 2.743 | 2.691 | 3.024 | 3.093 | 3.328 | 2.735–2.752 | 2.776 | 3.205 | 232.830 | 11.64 | 34 | within bounds |
| jolt-4 | 1 | 3 | 12.939 | 12.922 | 13.056 | 13.467 | 14.679 | 12.890–12.968 | 12.933 | 12.204 | 240.076 | 1.00 | 31 | below bound: angle_p99 |
| jolt-4 | 4 | 3 | 3.710 | 3.710 | 3.747 | 3.763 | 4.312 | 3.694–3.719 | 3.756 | 4.062 | 230.776 | 3.89 | 32 | below bound: angle_p99 |
| jolt-4 | 8 | 3 | 2.005 | 2.004 | 2.021 | 2.041 | 2.194 | 2.004–2.006 | 2.062 | 2.426 | 225.052 | 7.56 | 33 | below bound: angle_p99 |
| jolt-4 | 16 | 3 | 2.762 | 2.719 | 3.063 | 3.163 | 3.264 | 2.729–2.827 | 2.814 | 3.291 | 235.475 | 10.73 | 36 | below bound: angle_p99 |
| rapier-par | 1 | 3 | 17.074 | 17.035 | 17.486 | 17.946 | 19.709 | 17.040–17.121 | 17.732 | 50.355 | 64.763 | 1.00 | 62 | within bounds |
| rapier-par | 4 | 3 | 5.799 | 5.764 | 6.009 | 6.499 | 7.010 | 5.789–5.811 | 6.518 | 39.625 | 54.209 | 3.64 | 63 | within bounds |
| rapier-par | 8 | 3 | 3.887 | 3.856 | 4.099 | 4.624 | 4.854 | 3.831–3.949 | 4.720 | 37.929 | 53.429 | 6.72 | 63 | within bounds |
| rapier-par | 16 | 3 | 5.785 | 5.763 | 6.227 | 6.609 | 7.205 | 5.661–5.922 | 6.664 | 42.704 | 58.805 | 14.08 | 64 | within bounds |
| rapier-serial | 1 | 3 | 16.817 | 16.785 | 17.178 | 17.551 | 19.692 | 16.774–16.865 | 17.640 | 49.248 | 63.277 | 1.00 | 61 | not validated |
| rapier-simd8 | 1 | 3 | 13.350 | 13.265 | 14.003 | 14.651 | 16.433 | 13.207–13.554 | 14.428 | 47.219 | 61.514 | 1.00 | 62 | not validated |
| rapier-simd8 | 4 | 3 | 4.855 | 4.818 | 5.076 | 5.727 | 7.567 | 4.836–4.885 | 5.779 | 38.914 | 53.491 | 3.57 | 63 | not validated |

**joint_prismatic**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 50.235 | 54.261 | 55.854 | 56.442 | 59.967 | 50.167–50.339 | 18.233 | 29.162 | 241.615 | 1.00 | 80 | below bound: anchor_p99, angle_p99, limit_p99 |
| avian-par | 4 | 3 | 31.798 | 33.486 | 35.384 | 35.949 | 36.661 | 31.707–31.924 | 14.568 | 25.149 | 242.423 | 1.82 | 91 | below bound: anchor_p99, angle_p99, limit_p99 |
| avian-par | 8 | 3 | 27.835 | 29.139 | 30.672 | 31.459 | 33.772 | 27.639–28.224 | 14.097 | 24.882 | 241.588 | 2.26 | 91 | below bound: anchor_p99, angle_p99, limit_p99 |
| avian-par | 16 | 3 | 33.957 | 34.716 | 40.341 | 41.648 | 42.254 | 33.704–34.457 | 16.526 | 30.084 | 250.517 | 2.76 | 103 | below bound: anchor_p99, angle_p99, limit_p99 |
| avian-serial | 1 | 3 | 48.336 | 52.193 | 53.846 | 54.839 | 57.212 | 48.145–48.485 | 16.874 | 27.294 | 240.148 | 1.00 | 79 | not validated |
| jolt | 1 | 3 | 18.779 | 18.959 | 19.250 | 19.918 | 24.181 | 18.675–18.942 | 15.898 | 14.803 | 149.780 | 1.00 | 36 | within bounds |
| jolt | 4 | 3 | 5.709 | 5.690 | 6.187 | 7.874 | 10.264 | 5.611–5.884 | 4.902 | 5.170 | 142.325 | 3.91 | 37 | within bounds |
| jolt | 8 | 3 | 3.085 | 3.101 | 3.207 | 3.240 | 3.462 | 3.063–3.110 | 2.680 | 3.162 | 140.518 | 7.68 | 39 | within bounds |
| jolt | 16 | 3 | 3.909 | 3.917 | 4.103 | 4.177 | 4.263 | 3.879–3.938 | 3.643 | 4.516 | 143.922 | 11.77 | 42 | within bounds |
| jolt-4 | 1 | 3 | 15.990 | 16.130 | 16.348 | 16.896 | 20.901 | 15.851–16.075 | 14.147 | 12.340 | 146.338 | 1.00 | 37 | below bound: anchor_p99 |
| jolt-4 | 4 | 3 | 4.890 | 4.929 | 5.000 | 5.032 | 5.304 | 4.877–4.909 | 4.445 | 4.463 | 137.167 | 3.90 | 38 | below bound: anchor_p99 |
| jolt-4 | 8 | 3 | 2.714 | 2.736 | 2.798 | 2.824 | 2.883 | 2.705–2.720 | 2.473 | 2.937 | 138.833 | 7.63 | 39 | below bound: anchor_p99 |
| jolt-4 | 16 | 3 | 3.837 | 3.841 | 4.082 | 4.178 | 4.392 | 3.817–3.852 | 3.673 | 3.762 | 141.769 | 11.18 | 42 | below bound: anchor_p99 |
| rapier-par | 1 | 3 | 15.997 | 15.811 | 16.766 | 17.440 | 20.605 | 15.630–16.424 | 17.253 | 50.420 | 70.393 | 1.00 | 76 | within bounds |
| rapier-par | 4 | 3 | 6.112 | 6.103 | 6.516 | 7.074 | 8.789 | 6.077–6.150 | 7.143 | 41.072 | 61.563 | 3.49 | 77 | within bounds |
| rapier-par | 8 | 3 | 5.319 | 5.156 | 6.450 | 6.992 | 8.504 | 5.249–5.399 | 7.415 | 38.872 | 59.803 | 6.36 | 78 | within bounds |
| rapier-par | 16 | 3 | 6.291 | 6.241 | 7.050 | 7.558 | 8.754 | 6.150–6.466 | 8.483 | 42.935 | 65.083 | 12.57 | 79 | within bounds |
| rapier-serial | 1 | 3 | 15.686 | 15.853 | 16.338 | 16.904 | 20.686 | 15.383–15.842 | 17.115 | 48.920 | 68.465 | 1.00 | 74 | not validated |
| rapier-simd8 | 1 | 3 | 13.799 | 13.896 | 14.419 | 15.076 | 19.580 | 13.710–13.882 | 14.952 | 47.609 | 67.373 | 1.00 | 76 | not validated |
| rapier-simd8 | 4 | 3 | 5.426 | 5.415 | 5.823 | 6.476 | 8.131 | 5.387–5.457 | 6.442 | 40.297 | 60.837 | 3.43 | 77 | not validated |

**joint_revolute**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 13.470 | 13.338 | 14.704 | 15.343 | 15.655 | 13.424–13.540 | 9.191 | 12.415 | 81.736 | 1.00 | 43 | below bound: anchor_p99, angle_p99 |
| avian-par | 4 | 3 | 9.844 | 9.775 | 10.595 | 10.893 | 11.087 | 9.823–9.861 | 7.275 | 10.401 | 80.674 | 1.58 | 45 | below bound: anchor_p99, angle_p99 |
| avian-par | 8 | 3 | 9.177 | 9.135 | 9.775 | 10.026 | 10.311 | 9.144–9.195 | 7.129 | 10.198 | 80.011 | 1.78 | 46 | below bound: anchor_p99, angle_p99 |
| avian-par | 16 | 3 | 10.874 | 10.699 | 11.902 | 12.749 | 13.794 | 10.577–11.386 | 8.253 | 12.253 | 82.176 | 2.10 | 50 | below bound: anchor_p99, angle_p99 |
| avian-serial | 1 | 3 | 12.483 | 12.276 | 13.916 | 14.355 | 21.908 | 12.388–12.617 | 8.293 | 11.539 | 79.990 | 1.00 | 41 | not validated |
| jolt | 1 | 3 | 7.952 | 7.936 | 8.493 | 8.812 | 15.837 | 7.852–8.112 | 6.787 | 4.231 | 28.969 | 1.00 | 22 | below bound: anchor_p99, angle_p99 |
| jolt | 4 | 3 | 2.275 | 2.269 | 2.459 | 2.792 | 2.858 | 2.272–2.279 | 2.007 | 1.564 | 26.883 | 3.90 | 22 | below bound: anchor_p99, angle_p99 |
| jolt | 8 | 3 | 1.254 | 1.251 | 1.348 | 1.605 | 1.659 | 1.252–1.256 | 1.106 | 1.091 | 26.236 | 7.55 | 23 | below bound: anchor_p99, angle_p99 |
| jolt | 16 | 3 | 1.962 | 1.961 | 2.113 | 2.497 | 2.679 | 1.957–1.965 | 1.727 | 1.165 | 26.985 | 10.47 | 25 | below bound: anchor_p99, angle_p99 |
| jolt-4 | 1 | 3 | 6.539 | 6.546 | 7.199 | 7.426 | 7.500 | 6.528–6.546 | 5.291 | 3.706 | 28.264 | 1.00 | 22 | below bound: anchor_p99, angle_p99 |
| jolt-4 | 4 | 3 | 1.907 | 1.888 | 2.210 | 2.326 | 2.377 | 1.901–1.911 | 1.581 | 1.421 | 26.184 | 3.89 | 22 | below bound: anchor_p99, angle_p99 |
| jolt-4 | 8 | 3 | 1.115 | 1.116 | 1.224 | 1.307 | 1.337 | 1.109–1.120 | 0.944 | 1.027 | 26.191 | 7.13 | 23 | below bound: anchor_p99, angle_p99 |
| jolt-4 | 16 | 3 | 1.977 | 1.966 | 2.219 | 2.493 | 2.676 | 1.951–2.024 | 1.826 | 1.195 | 27.141 | 9.88 | 25 | below bound: anchor_p99, angle_p99 |
| rapier-par | 1 | 3 | 6.634 | 6.432 | 7.718 | 7.837 | 7.944 | 6.619–6.655 | 7.513 | 20.101 | 30.050 | 1.00 | 42 | below bound: anchor_p99, angle_p99 |
| rapier-par | 4 | 3 | 2.329 | 2.255 | 3.138 | 3.218 | 3.285 | 2.327–2.333 | 2.688 | 16.956 | 27.065 | 3.54 | 42 | below bound: anchor_p99, angle_p99 |
| rapier-par | 8 | 3 | 1.609 | 1.526 | 2.460 | 2.542 | 2.602 | 1.604–1.615 | 2.102 | 16.262 | 26.891 | 6.37 | 43 | below bound: anchor_p99, angle_p99 |
| rapier-par | 16 | 3 | 3.270 | 3.208 | 4.311 | 4.504 | 4.828 | 3.209–3.333 | 3.224 | 19.105 | 30.753 | 12.62 | 44 | below bound: anchor_p99, angle_p99 |
| rapier-serial | 1 | 3 | 6.555 | 6.326 | 7.609 | 7.811 | 8.299 | 6.513–6.623 | 7.865 | 19.351 | 28.913 | 1.00 | 40 | not validated |
| rapier-simd8 | 1 | 3 | 5.463 | 5.289 | 6.650 | 6.808 | 11.361 | 5.449–5.475 | 6.075 | 19.081 | 28.908 | 1.00 | 46 | not validated |
| rapier-simd8 | 4 | 3 | 1.993 | 1.908 | 2.848 | 2.920 | 7.260 | 1.987–1.999 | 2.269 | 16.644 | 26.800 | 3.50 | 47 | not validated |

<!-- /generated:timing-matched -->

### Defaults profile

<!-- generated:timing-defaults -->

**balls**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 6.140 | 6.527 | 11.754 | 11.875 | 13.205 | 6.129–6.155 | 4.831 | 9.945 | 69.931 | 1.00 | 47 | within bounds |
| avian-par | 4 | 3 | 3.560 | 3.962 | 5.666 | 6.418 | 7.920 | 3.548–3.578 | 3.023 | 8.051 | 69.226 | 2.31 | 49 | within bounds |
| jolt | 1 | 3 | 4.667 | 5.354 | 5.713 | 6.100 | 6.320 | 4.637–4.689 | 1.727 | 1.115 | 6.109 | 1.00 | 15 | within bounds |
| jolt | 4 | 3 | 1.749 | 2.011 | 2.093 | 2.188 | 2.235 | 1.745–1.751 | 0.965 | 0.892 | 6.127 | 3.85 | 16 | within bounds |
| rapier-par | 1 | 3 | 0.900 | 0.011 | 4.271 | 4.605 | 5.465 | 0.894–0.906 | 2.472 | 8.512 | 12.783 | 0.99 | 37 | within bounds |
| rapier-par | 4 | 3 | 0.355 | 0.008 | 1.606 | 2.163 | 2.657 | 0.354–0.356 | 1.633 | 8.352 | 13.133 | 3.02 | 38 | within bounds |

**boxes**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 14.904 | 15.079 | 15.853 | 16.815 | 17.958 | 14.862–14.951 | 16.229 | 10.684 | 18.103 | 1.00 | 30 | within bounds |
| avian-par | 4 | 3 | 8.291 | 8.319 | 8.941 | 9.989 | 10.857 | 8.272–8.305 | 8.905 | 7.468 | 15.271 | 2.54 | 33 | within bounds |
| jolt | 1 | 3 | 2.483 | 0.080 | 6.645 | 6.915 | 7.025 | 2.478–2.487 | 8.376 | 42.632 | 43.757 | 1.00 | 13 | within bounds |
| jolt | 4 | 3 | 0.982 | 0.054 | 2.586 | 2.636 | 2.669 | 0.980–0.983 | 3.024 | 12.211 | 13.634 | 3.85 | 13 | within bounds |
| rapier-par | 1 | 3 | 0.303 | 0.010 | 3.297 | 3.312 | 3.376 | 0.303–0.304 | 3.710 | 23.359 | 24.184 | 0.98 | 25 | within bounds |
| rapier-par | 4 | 3 | 0.107 | 0.005 | 1.136 | 1.159 | 1.170 | 0.105–0.108 | 1.350 | 12.429 | 14.074 | 3.75 | 25 | within bounds |

**capsules**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 13.127 | 15.456 | 16.983 | 17.306 | 17.719 | 13.114–13.133 | 3.416 | 7.223 | 27.508 | 1.00 | 34 | within bounds |
| avian-par | 4 | 3 | 7.395 | 8.038 | 9.594 | 10.357 | 11.841 | 7.364–7.430 | 1.952 | 5.795 | 26.208 | 2.42 | 37 | within bounds |
| jolt | 1 | 3 | 7.699 | 9.268 | 10.913 | 11.107 | 11.163 | 7.683–7.717 | 1.015 | 1.056 | 3.373 | 1.00 | 13 | below bound: ground_penetration, fallen |
| jolt | 4 | 3 | 2.799 | 3.380 | 3.960 | 4.031 | 4.080 | 2.795–2.805 | 0.487 | 0.584 | 3.012 | 3.72 | 14 | below bound: ground_penetration, fallen |
| rapier-par | 1 | 3 | 4.878 | 5.380 | 6.340 | 6.499 | 7.157 | 4.834–4.930 | 1.261 | 6.430 | 8.573 | 1.00 | 38 | within bounds |
| rapier-par | 4 | 3 | 1.919 | 2.054 | 2.439 | 4.188 | 5.947 | 1.853–2.042 | 0.893 | 5.054 | 7.520 | 3.57 | 39 | within bounds |

**pyramid**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 751.793 | 766.411 | 801.873 | 818.228 | 859.866 | 750.839–753.386 | 127.599 | 50.647 | 867.627 | 1.00 | 282 | below bound: final_speed_p99 |
| avian-par | 4 | 3 | 291.128 | 290.706 | 328.329 | 333.217 | 394.826 | 290.490–292.379 | 53.253 | 38.680 | 861.652 | 2.80 | 330 | below bound: final_speed_p99 |
| jolt | 1 | 3 | 462.700 | 459.215 | 528.140 | 558.399 | 625.164 | 460.443–464.080 | 75.043 | 9.419 | 40.258 | 1.00 | 183 | within bounds |
| jolt | 4 | 3 | 170.530 | 171.391 | 188.142 | 194.661 | 260.032 | 169.982–171.297 | 24.887 | 5.164 | 35.184 | 3.89 | 189 | within bounds |
| rapier-par | 1 | 3 | 256.655 | 262.348 | 266.850 | 274.423 | 352.225 | 255.559–257.335 | 50.431 | 57.920 | 86.917 | 1.00 | 483 | within bounds |
| rapier-par | 4 | 3 | 114.092 | 116.463 | 118.871 | 123.763 | 204.279 | 113.527–114.683 | 24.750 | 56.251 | 85.535 | 3.79 | 485 | within bounds |

**many_pyramids**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 76.845 | 76.138 | 82.302 | 87.081 | 91.297 | 76.710–76.954 | 81.588 | 35.556 | 100.902 | 1.00 | 65 | within bounds |
| avian-par | 4 | 3 | 28.669 | 28.114 | 32.232 | 32.938 | 35.030 | 28.556–28.845 | 29.509 | 20.549 | 86.778 | 3.10 | 70 | within bounds |
| jolt | 1 | 3 | 0.001 | 0.001 | 0.001 | 0.001 | 0.004 | 0.001–0.001 | 18.566 | 61.933 | 67.532 | 1.00 | 33 | within bounds |
| jolt | 4 | 3 | 0.001 | 0.001 | 0.001 | 0.001 | 0.005 | 0.001–0.001 | 6.085 | 20.020 | 25.825 | 3.95 | 34 | within bounds |
| rapier-par | 1 | 3 | 0.011 | 0.011 | 0.011 | 0.013 | 0.016 | 0.011–0.011 | 21.911 | 98.084 | 104.907 | 1.00 | 86 | within bounds |
| rapier-par | 4 | 3 | 0.007 | 0.006 | 0.013 | 0.015 | 0.026 | 0.006–0.008 | 8.178 | 52.779 | 60.424 | 3.57 | 86 | within bounds |

**keva**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 718.809 | 784.663 | 919.021 | 941.957 | 1019.130 | 718.075–719.940 | 842.786 | 322.174 | 1015.143 | 1.00 | 428 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| avian-par | 4 | 3 | 272.581 | 282.627 | 371.613 | 386.609 | 460.744 | 271.662–273.474 | 318.026 | 154.663 | 863.631 | 2.68 | 496 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| jolt | 1 | 3 | 568.743 | 574.171 | 805.897 | 834.828 | 940.049 | 566.907–570.180 | 818.127 | 970.693 | 998.586 | 1.00 | 240 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| jolt | 4 | 3 | 184.082 | 184.548 | 260.093 | 270.477 | 282.408 | 183.772–184.451 | 283.830 | 348.749 | 376.627 | 3.92 | 245 | below bound: height_ratio, ground_penetration, fallen, final_speed_p99 |
| rapier-par | 1 | 3 | 352.573 | 352.222 | 359.616 | 365.454 | 463.484 | 350.384–354.233 | 371.626 | 1093.099 | 1121.023 | 1.00 | 680 | below bound: final_speed_p99 |
| rapier-par | 4 | 3 | 163.581 | 163.352 | 166.679 | 168.331 | 175.419 | 162.759–164.911 | 173.842 | 587.928 | 616.348 | 3.83 | 682 | below bound: final_speed_p99 |

**joint_ball**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 17.941 | 18.071 | 19.618 | 19.729 | 20.672 | 17.901–17.967 | 14.684 | 19.657 | 123.598 | 1.00 | 55 | below bound: anchor_p99 |
| avian-par | 4 | 3 | 14.174 | 14.144 | 15.109 | 15.326 | 16.079 | 14.153–14.191 | 12.272 | 15.525 | 117.706 | 1.42 | 55 | below bound: anchor_p99 |
| jolt | 1 | 3 | 11.442 | 11.465 | 11.805 | 12.097 | 12.736 | 11.433–11.460 | 10.412 | 7.994 | 226.044 | 1.00 | 26 | within bounds |
| jolt | 4 | 3 | 4.615 | 4.619 | 4.762 | 4.826 | 4.894 | 4.608–4.620 | 4.394 | 4.490 | 224.171 | 3.39 | 27 | within bounds |
| rapier-par | 1 | 3 | 11.457 | 11.449 | 12.613 | 12.822 | 13.229 | 11.420–11.491 | 10.835 | 37.624 | 51.220 | 1.00 | 62 | within bounds |
| rapier-par | 4 | 3 | 3.920 | 3.911 | 4.301 | 4.393 | 4.715 | 3.895–3.952 | 4.065 | 32.062 | 46.233 | 3.51 | 63 | within bounds |

**joint_fixed**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 20.269 | 20.246 | 20.785 | 21.108 | 21.730 | 20.221–20.315 | 20.520 | 27.577 | 145.599 | 1.00 | 51 | within bounds |
| avian-par | 4 | 3 | 18.759 | 18.726 | 19.457 | 20.445 | 25.634 | 18.657–18.817 | 18.710 | 24.167 | 140.753 | 1.19 | 52 | within bounds |
| jolt | 1 | 3 | 15.522 | 15.515 | 15.616 | 15.989 | 17.427 | 15.470–15.560 | 15.469 | 14.977 | 242.026 | 1.00 | 31 | within bounds |
| jolt | 4 | 3 | 4.399 | 4.399 | 4.428 | 4.440 | 4.462 | 4.389–4.409 | 4.448 | 4.868 | 236.841 | 3.90 | 32 | within bounds |
| rapier-par | 1 | 3 | 8.913 | 17.019 | 17.972 | 18.459 | 20.593 | 8.846–8.950 | 18.281 | 53.171 | 67.447 | 1.00 | 62 | within bounds |
| rapier-par | 4 | 3 | 3.018 | 5.748 | 6.040 | 6.476 | 6.849 | 2.970–3.046 | 6.632 | 42.346 | 56.859 | 3.61 | 63 | within bounds |

**joint_prismatic**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 58.168 | 58.903 | 73.049 | 74.370 | 77.913 | 58.146–58.193 | 23.836 | 33.593 | 239.173 | 1.00 | 77 | below bound: anchor_p99, angle_p99, limit_p99 |
| avian-par | 4 | 3 | 37.092 | 37.461 | 44.039 | 45.113 | 46.438 | 37.074–37.106 | 19.467 | 29.715 | 239.951 | 1.83 | 88 | below bound: anchor_p99, angle_p99, limit_p99 |
| jolt | 1 | 3 | 18.899 | 19.101 | 19.294 | 19.786 | 22.564 | 18.850–18.959 | 15.986 | 15.049 | 150.176 | 1.00 | 36 | within bounds |
| jolt | 4 | 3 | 5.668 | 5.725 | 5.794 | 5.815 | 6.183 | 5.653–5.680 | 4.930 | 5.147 | 142.504 | 3.92 | 37 | within bounds |
| rapier-par | 1 | 3 | 16.800 | 16.929 | 17.427 | 18.142 | 22.425 | 16.462–16.983 | 18.044 | 52.286 | 72.042 | 1.00 | 77 | within bounds |
| rapier-par | 4 | 3 | 6.287 | 6.272 | 6.711 | 7.299 | 8.956 | 6.234–6.352 | 7.305 | 43.244 | 63.261 | 3.49 | 77 | within bounds |

**joint_revolute**

| variant | threads | runs | warm mean ms | warm p50 | warm p95 | warm p99 | warm max | run means | cold mean | tick 1 | build + tick 1 | busy cores | peak memory MiB | quality |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| avian-par | 1 | 3 | 65.498 | 64.936 | 70.929 | 78.138 | 80.724 | 65.414–65.568 | 19.538 | 13.566 | 80.010 | 1.00 | 63 | below bound: anchor_p99, angle_p99 |
| avian-par | 4 | 3 | 38.553 | 38.204 | 42.187 | 45.392 | 65.563 | 38.349–38.701 | 14.112 | 11.906 | 79.154 | 1.98 | 73 | below bound: anchor_p99, angle_p99 |
| jolt | 1 | 3 | 7.890 | 7.924 | 8.291 | 8.495 | 8.676 | 7.874–7.916 | 6.883 | 4.328 | 28.906 | 1.00 | 22 | below bound: anchor_p99, angle_p99 |
| jolt | 4 | 3 | 2.280 | 2.274 | 2.469 | 2.715 | 2.831 | 2.276–2.282 | 2.023 | 1.565 | 26.810 | 3.91 | 22 | below bound: anchor_p99, angle_p99 |
| rapier-par | 1 | 3 | 8.050 | 7.957 | 9.652 | 9.842 | 9.925 | 8.016–8.075 | 8.019 | 21.434 | 31.160 | 1.00 | 42 | below bound: anchor_p99, angle_p99 |
| rapier-par | 4 | 3 | 2.769 | 2.712 | 3.491 | 3.648 | 3.744 | 2.748–2.780 | 2.828 | 17.645 | 27.806 | 3.58 | 43 | below bound: anchor_p99, angle_p99 |

<!-- /generated:timing-defaults -->

### Solver sweep

Four threads, the same iteration count in every engine (Jolt: velocity steps with 2 position
steps; Rapier: solver iterations; Avian: substeps). oxijolt has no row at 2 velocity steps on
`joint_revolute`: the run failed (below). The validation of `many_pyramids` here measured the
height against tick 60, before the reference moved to tick 300 (results'
[README](../benches/comparison/results/2026-10-07-linux-5950x/README.md)).

<!-- generated:sweep -->

**many_pyramids**

| variant | threads | iterations | warm mean ms | warm p99 | anchor p99 m | angle p99 rad | height ratio | quality |
|---|---|---|---|---|---|---|---|---|
| avian-par | 4 | 2 | 21.239 | 24.741 | n/a | n/a | 1.001619 | below bound: final_speed_p99 |
| avian-par | 4 | 4 | 26.817 | 30.705 | n/a | n/a | 1.000369 | within bounds |
| avian-par | 4 | 8 | 38.792 | 43.791 | n/a | n/a | 1.000104 | within bounds |
| jolt | 4 | 2 | 7.282 | 14.078 | n/a | n/a | 1.000731 | below bound: ground_penetration, final_speed_p99 |
| jolt | 4 | 4 | 6.017 | 6.561 | n/a | n/a | 0.999710 | below bound: final_speed_p99 |
| jolt | 4 | 8 | 7.617 | 7.722 | n/a | n/a | 0.999942 | within bounds |
| rapier-par | 4 | 2 | 6.971 | 7.480 | n/a | n/a | 1.000737 | below bound: final_speed_p99 |
| rapier-par | 4 | 4 | 8.386 | 9.109 | n/a | n/a | 0.999990 | within bounds |
| rapier-par | 4 | 8 | 10.768 | 11.240 | n/a | n/a | 0.999989 | within bounds |

**joint_revolute**

| variant | threads | iterations | warm mean ms | warm p99 | anchor p99 m | angle p99 rad | height ratio | quality |
|---|---|---|---|---|---|---|---|---|
| avian-par | 4 | 2 | 6.537 | 7.126 | 17.074408 | 2.864168 | n/a | below bound: anchor_p99, angle_p99 |
| avian-par | 4 | 4 | 9.834 | 10.811 | 21.198238 | 3.007380 | n/a | below bound: anchor_p99, angle_p99 |
| avian-par | 4 | 8 | 16.063 | 17.121 | 22.885859 | 3.010580 | n/a | below bound: anchor_p99, angle_p99 |
| jolt | 4 | 4 | 1.910 | 2.331 | 1.626429 | 0.493550 | n/a | below bound: anchor_p99, angle_p99 |
| jolt | 4 | 8 | 2.143 | 2.634 | 1.065376 | 0.375391 | n/a | below bound: anchor_p99, angle_p99 |
| rapier-par | 4 | 2 | 1.624 | 2.570 | 0.691739 | 0.311043 | n/a | below bound: anchor_p99, angle_p99 |
| rapier-par | 4 | 4 | 2.340 | 3.234 | 0.203237 | 0.109502 | n/a | below bound: anchor_p99, angle_p99 |
| rapier-par | 4 | 8 | 3.718 | 4.529 | 0.068752 | 0.040086 | n/a | within bounds |

<!-- /generated:sweep -->

### Avian's physics schedule inside the update

Split runs with Avian's total step timer, their three repeats pooled like the rows above: the
update as in the tables above, the time of Avian's physics schedule inside it, and the rest
(Bevy's schedule, time and transform propagation, Avian's systems outside its physics schedule).
The last column is the plain timing run's update. The first column minus the last is the timer's
own cost plus the variation between runs; it is negative in four rows, down to -1.6 ms on `keva`.

<!-- generated:split -->

| scene | profile | threads | update mean ms | physics schedule mean ms | rest of the update mean ms | timing-run update mean ms |
|---|---|---|---|---|---|---|
| balls | matched | 1 | 9.186 | 8.109 | 1.076 | 9.139 |
| balls | matched | 4 | 4.287 | 3.665 | 0.622 | 4.278 |
| boxes | matched | 1 | 11.416 | 11.052 | 0.364 | 11.396 |
| boxes | matched | 4 | 6.436 | 6.196 | 0.240 | 6.409 |
| capsules | matched | 1 | 10.101 | 9.389 | 0.712 | 10.119 |
| capsules | matched | 4 | 5.528 | 5.155 | 0.373 | 5.507 |
| joint_ball | matched | 1 | 21.160 | 19.537 | 1.623 | 21.046 |
| joint_ball | matched | 4 | 15.436 | 14.203 | 1.233 | 15.209 |
| joint_fixed | matched | 1 | 15.541 | 13.937 | 1.603 | 15.449 |
| joint_fixed | matched | 4 | 13.869 | 12.657 | 1.212 | 13.843 |
| joint_prismatic | matched | 1 | 50.676 | 48.123 | 2.552 | 50.235 |
| joint_prismatic | matched | 4 | 31.889 | 29.930 | 1.960 | 31.798 |
| joint_revolute | matched | 1 | 13.485 | 12.208 | 1.276 | 13.470 |
| joint_revolute | matched | 4 | 9.843 | 8.969 | 0.874 | 9.844 |
| keva | matched | 1 | 504.754 | 500.444 | 4.311 | 506.118 |
| keva | matched | 4 | 193.663 | 189.483 | 4.180 | 195.277 |
| many_pyramids | matched | 1 | 69.164 | 67.959 | 1.205 | 69.058 |
| many_pyramids | matched | 4 | 27.245 | 26.392 | 0.854 | 27.086 |
| pyramid | matched | 1 | 589.255 | 584.637 | 4.617 | 589.059 |
| pyramid | matched | 4 | 232.916 | 229.199 | 3.717 | 231.822 |

<!-- /generated:split -->

### Quality at four threads

The validation runs at 1, 4, 8 and 16 threads gave the same digests, so the same measures, for
every engine and scene (next section); the four-thread rows stand for all four.

<!-- generated:quality -->

| variant | scene | profile | threads | iterations | height_ratio | ground_penetration | ground_penetration_max | fallen | final_speed_p99 | ball_overlap | ball_overlap_max | anchor_p99 | angle_p99 | limit_p99 | awake_at_end | violations |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| jolt | balls | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | 0.029945 | 0.262690 | n/a | n/a | n/a | 7600 | none |
| jolt-4 | balls | matched | 4 | 4 | n/a | n/a | n/a | n/a | n/a | 0.030387 | 0.340679 | n/a | n/a | n/a | 7600 | none |
| rapier-par | balls | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | 0.009950 | 0.301052 | n/a | n/a | n/a | 7600 | none |
| avian-par | balls | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | 0.004417 | 0.287813 | n/a | n/a | n/a | 7600 | none |
| jolt | balls | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | 0.041929 | 0.295475 | n/a | n/a | n/a | 3743 | none |
| jolt-4 | balls | defaults | 4 | 4 | n/a | n/a | n/a | n/a | n/a | 0.043679 | 0.334797 | n/a | n/a | n/a | 0 | none |
| rapier-par | balls | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | 0.009990 | 0.301052 | n/a | n/a | n/a | 0 | none |
| avian-par | balls | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | 0.004442 | 0.088102 | n/a | n/a | n/a | 0 | none |
| jolt | capsules | matched | 4 | default | n/a | 172.175339 | 172.175339 | 12 | 8.562654 | n/a | n/a | n/a | n/a | n/a | 3008 | ground_penetration,fallen |
| jolt-4 | capsules | matched | 4 | 4 | n/a | 217.529099 | 217.529099 | 240 | 57.326023 | n/a | n/a | n/a | n/a | n/a | 3008 | ground_penetration,fallen |
| rapier-par | capsules | matched | 4 | default | n/a | 0.030036 | 0.210192 | 0 | 7.100881 | n/a | n/a | n/a | n/a | n/a | 3008 | none |
| avian-par | capsules | matched | 4 | default | n/a | 0.035773 | 0.266310 | 0 | 7.593885 | n/a | n/a | n/a | n/a | n/a | 3008 | none |
| jolt | capsules | defaults | 4 | default | n/a | 166.532562 | 166.532562 | 24 | 10.718449 | n/a | n/a | n/a | n/a | n/a | 3008 | ground_penetration,fallen |
| jolt-4 | capsules | defaults | 4 | 4 | n/a | 244.233658 | 244.233658 | 198 | 49.629009 | n/a | n/a | n/a | n/a | n/a | 3008 | ground_penetration,fallen |
| rapier-par | capsules | defaults | 4 | default | n/a | 0.015109 | 0.210192 | 0 | 8.287679 | n/a | n/a | n/a | n/a | n/a | 3008 | none |
| avian-par | capsules | defaults | 4 | default | n/a | 0.012632 | 0.143123 | 0 | 7.562224 | n/a | n/a | n/a | n/a | n/a | 3004 | none |
| jolt | joint_ball | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.075314 | n/a | n/a | 9974 | none |
| jolt-4 | joint_ball | matched | 4 | 4 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.141170 | n/a | n/a | 9974 | anchor_p99 |
| rapier-par | joint_ball | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.058051 | n/a | n/a | 9974 | none |
| avian-par | joint_ball | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 1.141398 | n/a | n/a | 9974 | anchor_p99 |
| jolt | joint_ball | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.069071 | n/a | n/a | 9974 | none |
| jolt-4 | joint_ball | defaults | 4 | 4 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.132719 | n/a | n/a | 9974 | anchor_p99 |
| rapier-par | joint_ball | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.058051 | n/a | n/a | 9974 | none |
| avian-par | joint_ball | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.140782 | n/a | n/a | 9974 | anchor_p99 |
| jolt | joint_fixed | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.008434 | 0.044251 | n/a | 11500 | none |
| jolt-4 | joint_fixed | matched | 4 | 4 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.024531 | 0.108449 | n/a | 11500 | angle_p99 |
| rapier-par | joint_fixed | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.013977 | 0.032331 | n/a | 11500 | none |
| avian-par | joint_fixed | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.035137 | 0.058193 | n/a | 11500 | none |
| jolt | joint_fixed | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.008299 | 0.044173 | n/a | 11500 | none |
| jolt-4 | joint_fixed | defaults | 4 | 4 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.023685 | 0.108335 | n/a | 11500 | angle_p99 |
| rapier-par | joint_fixed | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.014179 | 0.032753 | n/a | 0 | none |
| avian-par | joint_fixed | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.015305 | 0.025398 | n/a | 11500 | none |
| jolt | joint_prismatic | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.057239 | 0.044255 | 0.011519 | 16000 | none |
| jolt-4 | joint_prismatic | matched | 4 | 4 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.107055 | 0.092380 | 0.021988 | 16000 | anchor_p99 |
| rapier-par | joint_prismatic | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.039868 | 0.039926 | 0.002505 | 16000 | none |
| avian-par | joint_prismatic | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 2.770000 | 2.999875 | 1.551825 | 16000 | anchor_p99,angle_p99,limit_p99 |
| jolt | joint_prismatic | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.051587 | 0.039619 | 0.011875 | 16000 | none |
| jolt-4 | joint_prismatic | defaults | 4 | 4 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.101996 | 0.092511 | 0.019823 | 16000 | anchor_p99 |
| rapier-par | joint_prismatic | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.039868 | 0.039926 | 0.002505 | 16000 | none |
| avian-par | joint_prismatic | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 1.167864 | 2.699181 | 0.499149 | 16000 | anchor_p99,angle_p99,limit_p99 |
| jolt | joint_revolute | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.919359 | 0.346165 | n/a | 8000 | anchor_p99,angle_p99 |
| jolt-4 | joint_revolute | matched | 4 | 4 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 1.626429 | 0.493550 | n/a | 8000 | anchor_p99,angle_p99 |
| rapier-par | joint_revolute | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.203237 | 0.109502 | n/a | 8000 | anchor_p99,angle_p99 |
| avian-par | joint_revolute | matched | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 21.198238 | 3.007380 | n/a | 8000 | anchor_p99,angle_p99 |
| jolt | joint_revolute | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.906718 | 0.339715 | n/a | 8000 | anchor_p99,angle_p99 |
| jolt-4 | joint_revolute | defaults | 4 | 4 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 1.737172 | 0.493844 | n/a | 8000 | anchor_p99,angle_p99 |
| rapier-par | joint_revolute | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.203237 | 0.109502 | n/a | 8000 | anchor_p99,angle_p99 |
| avian-par | joint_revolute | defaults | 4 | default | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 12.288465 | 3.006289 | n/a | 8000 | anchor_p99,angle_p99 |
| jolt | many_pyramids | matched | 4 | 2 | 1.000731 | 0.343223 | 0.394575 | 0 | 9.010623 | n/a | n/a | n/a | n/a | n/a | 8400 | ground_penetration,final_speed_p99 |
| jolt | many_pyramids | matched | 4 | 4 | 0.999710 | 0.020040 | 0.021105 | 0 | 0.861535 | n/a | n/a | n/a | n/a | n/a | 8400 | final_speed_p99 |
| jolt | many_pyramids | matched | 4 | 8 | 0.999942 | 0.007690 | 0.010547 | 0 | 0.010480 | n/a | n/a | n/a | n/a | n/a | 8400 | none |
| rapier-par | many_pyramids | matched | 4 | 2 | 1.000737 | 0.001469 | 0.002442 | 0 | 0.627694 | n/a | n/a | n/a | n/a | n/a | 8400 | final_speed_p99 |
| rapier-par | many_pyramids | matched | 4 | 4 | 0.999990 | 0.001008 | 0.001216 | 0 | 0.000145 | n/a | n/a | n/a | n/a | n/a | 8400 | none |
| rapier-par | many_pyramids | matched | 4 | 8 | 0.999989 | 0.001286 | 0.001295 | 0 | 0.000414 | n/a | n/a | n/a | n/a | n/a | 8400 | none |
| avian-par | many_pyramids | matched | 4 | 2 | 1.001619 | 0.030292 | 0.047729 | 0 | 2.436389 | n/a | n/a | n/a | n/a | n/a | 8400 | final_speed_p99 |
| avian-par | many_pyramids | matched | 4 | 4 | 1.000369 | 0.008015 | 0.010451 | 0 | 0.096379 | n/a | n/a | n/a | n/a | n/a | 8400 | none |
| avian-par | many_pyramids | matched | 4 | 8 | 1.000104 | 0.002184 | 0.002583 | 0 | 0.028137 | n/a | n/a | n/a | n/a | n/a | 8400 | none |
| jolt | joint_revolute | matched | 4 | 4 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 1.626429 | 0.493550 | n/a | 8000 | anchor_p99,angle_p99 |
| jolt | joint_revolute | matched | 4 | 8 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 1.065376 | 0.375391 | n/a | 8000 | anchor_p99,angle_p99 |
| rapier-par | joint_revolute | matched | 4 | 2 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.691739 | 0.311043 | n/a | 8000 | anchor_p99,angle_p99 |
| rapier-par | joint_revolute | matched | 4 | 4 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.203237 | 0.109502 | n/a | 8000 | anchor_p99,angle_p99 |
| rapier-par | joint_revolute | matched | 4 | 8 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 0.068752 | 0.040086 | n/a | 8000 | none |
| avian-par | joint_revolute | matched | 4 | 2 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 17.074408 | 2.864168 | n/a | 8000 | anchor_p99,angle_p99 |
| avian-par | joint_revolute | matched | 4 | 4 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 21.198238 | 3.007380 | n/a | 8000 | anchor_p99,angle_p99 |
| avian-par | joint_revolute | matched | 4 | 8 | n/a | n/a | n/a | n/a | n/a | n/a | n/a | 22.885859 | 3.010580 | n/a | 8000 | anchor_p99,angle_p99 |
| jolt | boxes | matched | 4 | default | 1.000000 | 0.008708 | 0.009564 | 0 | 0.000221 | n/a | n/a | n/a | n/a | n/a | 1000 | none |
| jolt-4 | boxes | matched | 4 | 4 | 1.000001 | 0.020343 | 0.024173 | 0 | 0.001542 | n/a | n/a | n/a | n/a | n/a | 1000 | none |
| rapier-par | boxes | matched | 4 | default | 1.000000 | 0.003215 | 0.003467 | 0 | 0.000173 | n/a | n/a | n/a | n/a | n/a | 1000 | none |
| avian-par | boxes | matched | 4 | default | 0.999981 | 0.047923 | 0.270944 | 0 | 10.806533 | n/a | n/a | n/a | n/a | n/a | 1000 | final_speed_p99 |
| jolt | boxes | defaults | 4 | default | 1.000000 | 0.020000 | 0.259614 | 0 | 0.000000 | n/a | n/a | n/a | n/a | n/a | 0 | none |
| jolt-4 | boxes | defaults | 4 | 4 | 1.000000 | 0.020001 | 0.258268 | 0 | 0.000000 | n/a | n/a | n/a | n/a | n/a | 0 | none |
| rapier-par | boxes | defaults | 4 | default | 1.000000 | 0.003210 | 0.003467 | 0 | 0.000000 | n/a | n/a | n/a | n/a | n/a | 0 | none |
| avian-par | boxes | defaults | 4 | default | 1.000398 | 0.007493 | 0.015598 | 0 | 0.057631 | n/a | n/a | n/a | n/a | n/a | 1000 | none |
| jolt | pyramid | matched | 4 | default | 1.000075 | 0.028015 | 0.034396 | 0 | 0.041453 | n/a | n/a | n/a | n/a | n/a | 42925 | none |
| jolt-4 | pyramid | matched | 4 | 4 | 0.999857 | 0.020174 | 0.105019 | 0 | 0.181281 | n/a | n/a | n/a | n/a | n/a | 42925 | final_speed_p99 |
| rapier-par | pyramid | matched | 4 | default | 0.994331 | 0.474314 | 0.601858 | 0 | 0.010267 | n/a | n/a | n/a | n/a | n/a | 42925 | ground_penetration |
| avian-par | pyramid | matched | 4 | default | 0.978940 | 0.333171 | 0.443739 | 0 | 0.188523 | n/a | n/a | n/a | n/a | n/a | 42925 | ground_penetration,final_speed_p99 |
| jolt | pyramid | defaults | 4 | default | 0.999956 | 0.023687 | 0.043731 | 0 | 0.015515 | n/a | n/a | n/a | n/a | n/a | 42925 | none |
| jolt-4 | pyramid | defaults | 4 | 4 | 1.000260 | 0.020040 | 0.079906 | 0 | 0.090711 | n/a | n/a | n/a | n/a | n/a | 42925 | none |
| rapier-par | pyramid | defaults | 4 | default | 0.994331 | 0.003792 | 0.043466 | 0 | 0.010267 | n/a | n/a | n/a | n/a | n/a | 42924 | none |
| avian-par | pyramid | defaults | 4 | default | 1.000186 | 0.020856 | 0.033686 | 0 | 0.107847 | n/a | n/a | n/a | n/a | n/a | 42925 | final_speed_p99 |
| jolt | many_pyramids | matched | 4 | default | 1.000000 | 0.005366 | 0.008088 | 0 | 0.026427 | n/a | n/a | n/a | n/a | n/a | 8400 | none |
| jolt-4 | many_pyramids | matched | 4 | 4 | 0.999941 | 0.020040 | 0.021105 | 0 | 0.861535 | n/a | n/a | n/a | n/a | n/a | 8400 | final_speed_p99 |
| rapier-par | many_pyramids | matched | 4 | default | 1.000000 | 0.001008 | 0.001216 | 0 | 0.000145 | n/a | n/a | n/a | n/a | n/a | 8400 | none |
| avian-par | many_pyramids | matched | 4 | default | 1.000166 | 0.008015 | 0.010451 | 0 | 0.096379 | n/a | n/a | n/a | n/a | n/a | 8400 | none |
| jolt | many_pyramids | defaults | 4 | default | 1.000000 | 0.005852 | 0.008047 | 0 | 0.000000 | n/a | n/a | n/a | n/a | n/a | 0 | none |
| jolt-4 | many_pyramids | defaults | 4 | 4 | 1.000000 | 0.020098 | 0.021347 | 0 | 1.563359 | n/a | n/a | n/a | n/a | n/a | 6300 | final_speed_p99 |
| rapier-par | many_pyramids | defaults | 4 | default | 1.000000 | 0.001001 | 0.001216 | 0 | 0.000000 | n/a | n/a | n/a | n/a | n/a | 0 | none |
| avian-par | many_pyramids | defaults | 4 | default | 1.000009 | 0.003246 | 0.004294 | 0 | 0.047213 | n/a | n/a | n/a | n/a | n/a | 7140 | none |
| jolt | keva | matched | 4 | default | 0.246323 | 68.152679 | 68.152679 | 13 | 13.694255 | n/a | n/a | n/a | n/a | n/a | 38270 | height_ratio,ground_penetration,fallen,final_speed_p99 |
| jolt-4 | keva | matched | 4 | 4 | 0.180675 | 433.153961 | 433.153961 | 2269 | 54.213150 | n/a | n/a | n/a | n/a | n/a | 38270 | height_ratio,ground_penetration,fallen,final_speed_p99 |
| rapier-par | keva | matched | 4 | default | 1.000506 | 0.010382 | 0.017400 | 0 | 0.269966 | n/a | n/a | n/a | n/a | n/a | 38270 | final_speed_p99 |
| avian-par | keva | matched | 4 | default | 0.228516 | 361.583618 | 361.583618 | 779 | 30.800726 | n/a | n/a | n/a | n/a | n/a | 38270 | height_ratio,ground_penetration,fallen,final_speed_p99 |
| jolt | keva | defaults | 4 | default | 0.241268 | 109.771332 | 109.771332 | 3793 | 27.464169 | n/a | n/a | n/a | n/a | n/a | 38267 | height_ratio,ground_penetration,fallen,final_speed_p99 |
| jolt-4 | keva | defaults | 4 | 4 | 0.149925 | 380.599731 | 380.599731 | 4602 | 44.016476 | n/a | n/a | n/a | n/a | n/a | 38270 | height_ratio,ground_penetration,fallen,final_speed_p99 |
| rapier-par | keva | defaults | 4 | default | 1.000506 | 0.010382 | 0.017400 | 0 | 0.269966 | n/a | n/a | n/a | n/a | n/a | 38270 | final_speed_p99 |
| avian-par | keva | defaults | 4 | default | 0.234649 | 17.720209 | 17.720209 | 12 | 13.007080 | n/a | n/a | n/a | n/a | n/a | 38263 | height_ratio,ground_penetration,fallen,final_speed_p99 |

<!-- /generated:quality -->

What the bounds caught, at both profiles unless a profile is named:

- `capsules`: oxijolt's heap pushes capsules through the 20 cm ground box, 12 of 3008 at
  matched and 24 at defaults (240 and 198 with 4 velocity steps). Rapier and Avian lose none.
  The scene runs without continuous collision in oxijolt at both profiles (Jolt's default motion
  quality is discrete).
- `keva`: oxijolt's and Avian's towers collapse (a quarter of the tick-300 height is left, and
  bodies fall off the ground); Rapier's stand, still moving at 0.27 m/s (99th percentile) at
  tick 600.
- `pyramid`: oxijolt keeps the pyramid and its lowest boxes 3 cm in the ground. At matched,
  Rapier's deepest box ends 47 cm in the ground and Avian's 33 cm; at defaults (with each
  engine's own contact settings) they stay under 5 cm.
- `boxes`: Avian's stack at matched still moves at 10.8 m/s (99th percentile) at tick 600.
- `joint_ball`, `joint_prismatic`, `joint_revolute`: Avian's joints come apart (anchor errors at
  matched of 1.1 m, 2.8 m and 21 m); oxijolt and Rapier stay within 10 cm on the first two.
- `joint_revolute` is outside the bounds in every engine: anchor error 0.20 m for Rapier, 0.92 m
  for oxijolt. Rapier needs 8 iterations to stay within them (solver sweep).
- `balls`: every engine ends within the 5 cm overlap; during the impacts at matched the deepest
  overlap is 26 to 30 cm in every engine.
- With 4 velocity steps, oxijolt leaves the bounds on `joint_ball`, `joint_fixed` and
  `joint_prismatic` (joint error) and on `pyramid` at matched and `many_pyramids` (final speed),
  where 10 steps stay within them.

### Determinism across thread counts

All 80 groups (ten scenes, two profiles, `jolt`, `jolt-4`, `rapier-par` and `avian-par`) gave
the same digest at 1, 4, 8 and 16 threads on all 600 ticks, and every group moved, so the check
is not empty. The table is in [`summary.md`](../benches/comparison/results/2026-10-07-linux-5950x/summary.md#determinism-across-thread-counts).
This is one binary per engine on one machine; it says nothing about other machines or
compilers.

## Features

Checked on 2026-10-07 against oxijolt at this commit, rapier3d 0.36.0 (tag `v0.36.0`) and avian3d
0.7.0 (tag `v0.7.0`), in each crate's source and on crates.io. "None found" names where we looked.

| | oxijolt | Rapier 0.36.0 | Avian 0.7.0 |
|---|---|---|---|
| Character controller | Jolt's `CharacterVirtual`, stairs and floor sticking ([features](features.md)) | `KinematicCharacterController` ([source](https://github.com/dimforge/rapier/blob/v0.36.0/src/control/character_controller.rs#L173)) | `MoveAndSlide` utilities for writing one, not a controller ([source](https://github.com/avianphysics/avian/blob/v0.7.0/src/character_controller/move_and_slide.rs#L69)) |
| Vehicles | wheeled, tracked and motorcycle, with engine, gearbox and differentials ([guide](vehicles.md)) | ray-cast vehicle `DynamicRayCastVehicleController` ([source](https://github.com/dimforge/rapier/blob/v0.36.0/src/control/ray_cast_vehicle_controller.rs#L17)) | none found in `src/` ([tree](https://github.com/avianphysics/avian/tree/v0.7.0/src)) |
| Ragdolls | a ragdoll type from a skeleton, with motors and a skeleton mapper ([guide](guide.md#ragdolls)) | none built in; built from joints ([joints](https://github.com/dimforge/rapier/blob/v0.36.0/src/dynamics/joint/mod.rs)) | none built in; built from joints ([joints](https://github.com/avianphysics/avian/blob/v0.7.0/src/dynamics/joints/mod.rs)) |
| Soft bodies | cloth, pressure and volume bodies ([guide](soft-bodies.md)) | yes, new in 0.36.0 ([changelog](https://github.com/dimforge/rapier/blob/v0.36.0/CHANGELOG.md#L5), [`SoftBodySet`](https://github.com/dimforge/rapier/blob/v0.36.0/src/dynamics/soft_body/soft_body_set/soft_body_set.rs#L26)) | not included ("Elasticity (soft bodies)" is listed as not covered, [source](https://github.com/avianphysics/avian/blob/v0.7.0/src/dynamics/mod.rs#L16-L19)) |
| Joint kinds | 12: fixed, point, distance, hinge, slider, cone, swing-twist, six-DOF, gear, rack and pinion, pulley, path ([guide](constraints.md)) | 8 impulse-joint builders (fixed, spherical, revolute, prismatic, rope, spring, pin-slot, generic) and multibody joints ([source](https://github.com/dimforge/rapier/tree/v0.36.0/src/dynamics/joint)) | 5: fixed, distance, prismatic, revolute, spherical ([source](https://github.com/avianphysics/avian/tree/v0.7.0/src/dynamics/joints)) |
| Contact events | added, persisted and removed contacts, a contact listener ([guide](events.md)) | `EventHandler`, collision and contact-force events ([source](https://github.com/dimforge/rapier/blob/v0.36.0/src/pipeline/event_handler.rs#L97)) | `CollisionStart` and `CollisionEnd` events ([source](https://github.com/avianphysics/avian/blob/v0.7.0/src/collision/collision_events.rs#L171)) |
| Same results for any thread count | tested with 1 and 4 workers ([determinism](determinism.md)); measured here at 1, 4, 8 and 16 | promised for any Rayon pool size and tested ([test](https://github.com/dimforge/rapier/blob/v0.36.0/crates/rapier3d/tests/thread_count_determinism.rs#L1-L4)); measured here | no promise found in the docs; the narrow phase keeps a serial loop "to preserve determinism" ([source](https://github.com/avianphysics/avian/blob/v0.7.0/src/collision/narrow_phase/mod.rs#L13)); measured here |
| Same results across operating systems | opt-in feature `cross-platform-deterministic`; CI compares a wasm build with native Linux for one scene ([determinism](determinism.md#how-it-is-checked)) | `enhanced-determinism` feature, with a snapshot test run on native and `wasm32-wasip1` ([test](https://github.com/dimforge/rapier/blob/v0.36.0/crates/rapier3d/tests/snapshot_portability.rs#L1-L13)) | `enhanced-determinism` feature ([source](https://github.com/avianphysics/avian/blob/v0.7.0/src/lib.rs#L66)); its cross-platform test is 2D ([test](https://github.com/avianphysics/avian/blob/v0.7.0/src/tests/determinism_2d.rs#L1)) |
| Save and restore | the same world's state, for rollback and replay ([guide](state.md)) | serde snapshots of the whole world (`serde-serialize`, [manifest](https://github.com/dimforge/rapier/blob/v0.36.0/crates/rapier3d/Cargo.toml#L69)) | none built in; `serialize` derives serde for components ([manifest](https://github.com/avianphysics/avian/blob/v0.7.0/crates/avian3d/Cargo.toml#L53)) |
| Double precision | `f64` world positions (`double-precision`, [features](features.md)) | the `rapier3d-f64` crate ([manifest](https://github.com/dimforge/rapier/blob/v0.36.0/crates/rapier3d-f64/Cargo.toml)) | `f64` feature ([manifest](https://github.com/avianphysics/avian/blob/v0.7.0/crates/avian3d/Cargo.toml#L28)) |
| WASM | `oxijolt` builds for `wasm32-unknown-unknown` from a release archive or from source, single-threaded, tested in CI in Node and headless Chrome ([building](building.md#webassembly)); Jolt itself builds for WASM ([JoltPhysics.js](https://github.com/jrouwe/JoltPhysics.js)) | yes, `wasm32-wasip1` in its CI test ([test](https://github.com/dimforge/rapier/blob/v0.36.0/crates/rapier3d/tests/snapshot_portability.rs#L11-L13)) and JavaScript bindings ([bindings](https://github.com/dimforge/rapier/tree/v0.36.0/bindings/typescript)) | through Bevy, which supports WASM ([Bevy](https://github.com/bevyengine/bevy/tree/v0.19.0/examples#wasm)); no WASM job in Avian's CI ([ci.yml](https://github.com/avianphysics/avian/blob/v0.7.0/.github/workflows/ci.yml)) |
| Language | Rust over C++ (Jolt and joltc) ([lineage](../LINEAGE.md)) | pure Rust ([manifest](https://github.com/dimforge/rapier/blob/v0.36.0/crates/rapier3d/Cargo.toml)) | pure Rust ([manifest](https://github.com/avianphysics/avian/blob/v0.7.0/crates/avian3d/Cargo.toml)) |
| Build requirements | on x86_64 Windows and Linux and on Apple silicon a verified prebuilt download; elsewhere a C++ toolchain and CMake 3.20 ([building](building.md)) | Rust only ([manifest](https://github.com/dimforge/rapier/blob/v0.36.0/crates/rapier3d/Cargo.toml)) | Rust only ([manifest](https://github.com/avianphysics/avian/blob/v0.7.0/crates/avian3d/Cargo.toml)) |
| Needs Bevy | no ([manifest](../crates/oxijolt/Cargo.toml)) | no; `bevy_rapier` is separate ([bindings](https://github.com/dimforge/rapier/tree/v0.36.0/bindings/bevy_rapier)) | yes ([manifest](https://github.com/avianphysics/avian/blob/v0.7.0/crates/avian3d/Cargo.toml#L80-L85)) |
| Minimum Rust | 1.88 ([manifest](../crates/oxijolt/Cargo.toml)) | 1.86 ([manifest](https://github.com/dimforge/rapier/blob/v0.36.0/Cargo.toml#L67)) | not declared; its Bevy 0.19.1 (the version this comparison's lockfile resolves) needs 1.95 ([Bevy manifest](https://github.com/bevyengine/bevy/blob/v0.19.1/Cargo.toml#L13)) |
| Licence | MIT or Apache-2.0; Jolt and joltc MIT ([lineage](../LINEAGE.md)) | Apache-2.0 ([manifest](https://github.com/dimforge/rapier/blob/v0.36.0/Cargo.toml#L66)) | MIT or Apache-2.0 ([manifest](https://github.com/avianphysics/avian/blob/v0.7.0/crates/avian3d/Cargo.toml#L5)) |

## Where oxijolt loses

- **A C++ toolchain and CMake** outside x86_64 Windows and Linux, Apple silicon and
  `wasm32-unknown-unknown`, and for any
  build no release archive fits: oxijolt then compiles Jolt and joltc with MSVC, GCC or Clang
  through CMake ([building](building.md)). Rapier and Avian are pure Rust.
- **WASM is single-threaded.** `oxijolt` builds for `wasm32-unknown-unknown` without threads;
  CI runs it in Node and headless Chrome ([building](building.md#webassembly)). Rapier runs a WASM
  test in CI and has JavaScript bindings.
- **macOS** comes prebuilt and is tested in CI on Apple silicon only; Intel Macs build from
  source. This comparison runs on Windows and Linux only: its harness reads CPU time and memory
  from those two systems.
- **Same results across operating systems** are an opt-in feature; CI checks wasm in Node and
  Chrome against Linux native, but not native operating systems against each other yet. Rapier
  tests a snapshot on native and WASM targets.
- **Save and restore** covers the same world with an unchanged structure only ([state](state.md)).
- **Speed on scenes of many contacts.** At matched settings Rapier's step is 1.9 to 2.2 times
  faster on `boxes`, 1.3 to 1.85 times on `pyramid`, 1.2 to 1.75 times on `capsules`, 1.4 times
  on `balls` up to 8 threads, and 1.6 times on `keva` at one thread. Jolt's default of 10
  velocity steps is part of it; with 4 Rapier is still 1.2 to 1.65 times faster on `boxes`.
- **`joint_ball` at 8 and 16 threads.** oxijolt's warm step is 10.7 ms at 8 threads against 4.5
  ms at 4, with per-run means from 8.3 to 12.1 ms, while its first 60 ticks keep scaling (3.8
  ms). Rapier is 3.4 to 4.3 times faster there. We have not found the cause.
- **Quality on three scenes.** Capsules through the ground, collapsing `keva` towers, and on
  `joint_revolute` a joint error 4.5 times Rapier's (above).
- **Building worlds with many joints.** World creation plus the first tick at four threads takes
  oxijolt 225 ms on `joint_ball`, 235 ms on `joint_fixed` and 142 ms on `joint_prismatic`, against
  Rapier's 43, 54 and 62 ms; oxijolt's first tick alone is under 6 ms there.
- **Sleeping at defaults.** With each engine's own thresholds Rapier is 5 times faster on `balls`
  and 8 to 9 times on `boxes`.
- **Two velocity steps.** The solver sweep's `joint_revolute` at 2 velocity steps fails in
  oxijolt: at tick 339 the step drops contacts because its body-pair buffer, sized at 10 steps,
  is full. The run is in `failures.tsv` and has no row.

## In a real game

The owner's numbers from the game VOIDRUN, which replaced Rapier's kinematic character controller
with oxijolt's `CharacterVirtual` ([the table as given](../benches/comparison/results/owner-voidrun-2026-10-04.md)).
The game rows are a release build on a weak 4-core machine. They are the game's own observations,
not runs of this repository, and the two sides did not always time the same work:

| What | Rapier | Jolt | What each side timed |
|---|---|---|---|
| Whole walking system, 60 NPCs in the near band, mean per tick | 3.43 ms | 0.83 ms | the game's walking system with each engine; a system-level observation, not an engine speed |
| One controller move in the game | 86-156 us | 16-19 us p50 / 28-43 us p99 | Rapier: in the game, statistic and window not recorded; Jolt: in the game, with the game's wrapper. No ratio. |
| One controller move, the binding alone | n/a | 2.7 us p50 / 6 us p99 | `update_character` with terrain in the binding's own benchmark: 30 characters on a flat 3 x 3 chunk scene, on an Intel Core i9-11900K under Windows 11, not the game's machine and not the machine of the runs above ([benchmarks](benchmarks.md)); no Rapier counterpart |
| Physics world step p50/p99 | 110 / 268 us | 45 / 111 us | the world step; the setups differ in damping and sleeping, and Rapier's item gravity was applied before its timer, Jolt's inside it |
| Refresh before queries when the world changed, p99 | 3.8 ms | 164 us | Rapier `detect_collisions` (broad and narrow phase) against Jolt `optimize_broad_phase`: different work, no ratio |

The game ran rapier3d 0.36.0: its manifest required `0.36` with no patch, and 0.36.0 is the only
0.36 release. Rapier 0.36 already serves queries from its broad phase's tree, so the refresh row is
not a query-structure rebuild; the game's refresh ran collision detection. The owner's caveats:
the gains for the whole game are smaller than the bare controller's, because the rest of the frame
is unchanged, and the game's own wrapper around the controller dominates its cost in the game.

A head-to-head of the character controllers on one scene is not part of this comparison.

## Reproduce

On Linux with a C++ toolchain, CMake 3.20 or newer, Python 3 and Rust 1.95 or newer (Bevy 0.19.1,
which Avian pulls in, needs it; oxijolt alone needs 1.88), from the repository root, one cargo
process at a time on an otherwise idle machine:

```bash
python3 benches/comparison/scripts/build_variants.py          # six binaries in target/comparison-bins
R=benches/comparison/results/my-run
python3 benches/comparison/scripts/campaign.py $R validate
python3 benches/comparison/scripts/campaign.py $R sweep-validate
for rep in 1 2 3; do
  for plan in main extras split sweep; do
    python3 benches/comparison/scripts/campaign.py $R $plan --repeat $rep
  done
done
python3 benches/comparison/scripts/build_times.py $R
python3 benches/comparison/scripts/make_doc.py $R              # copies the tables into this page
```

The same scripts run on Windows (load from `load.ps1`, no pinning). `comparison help` lists the
options of the harness itself.
