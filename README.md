<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/assets/oxijolt-logo-dark.png">
    <img alt="oxijolt" src="docs/assets/oxijolt-logo-light.png" width="420">
  </picture>
</p>

# oxijolt — Rust bindings for Jolt Physics
[![crates.io](https://img.shields.io/crates/v/oxijolt.svg)](https://crates.io/crates/oxijolt)
[![docs.rs](https://img.shields.io/docsrs/oxijolt)](https://docs.rs/oxijolt)
[![CI](https://github.com/pockerhead/oxijolt/actions/workflows/ci.yml/badge.svg)](https://github.com/pockerhead/oxijolt/actions/workflows/ci.yml)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)
[![Jolt Physics 5.6.0](https://img.shields.io/badge/Jolt%20Physics-5.6.0-orange.svg)](https://github.com/jrouwe/JoltPhysics/releases/tag/v5.6.0)

Rust bindings for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6: a safe API with
rigid bodies, a character controller, wheeled and tracked vehicles and motorcycles, ragdolls, soft
bodies, constraints and contact events, built for deterministic simulation. Underneath are raw
bindings to the [joltc] C wrapper. Everything runs headless and is tested without a window.

## Playground

Thirteen small scenes in a window show what the binding does: a character, vehicles, a pile of
bodies, ragdolls, constraints, soft bodies, buoyancy, destruction, contact control, queries, real
meshes and any model file. In a clone of the repository with Rust, CMake and a C++ toolchain:

```sh
git clone --recursive https://github.com/pockerhead/oxijolt
cd oxijolt
cargo run -p playground --release
```

[docs/playground.md](docs/playground.md) has a clip of every scene, the keys, the Linux packages,
a faster first build with a prebuilt Jolt, and the headless and record modes.

## Example

```rust
use oxijolt::prelude::math::*;
use oxijolt::prelude::*;

fn main() -> oxijolt::error::Result<()> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;

    // A static floor whose top is at y = 0, and a ball dropped onto it.
    let floor = Shape::new_box(Vec3::new(50.0, 1.0, 50.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    let ball_shape = Shape::new_sphere(0.5)?;
    let ball = world.create_body(
        &ball_shape,
        &BodySettings::new_dynamic().position(RVec3::new(0.0, 4.0, 0.0)),
    )?;

    for _ in 0..120 {
        // A step that had to drop contacts says so; raise a `WorldSettings` limit then.
        let report = world.step(1.0 / 60.0)?;
        assert!(report.is_complete(), "{report:?}");
    }
    println!("the ball rests at {:?}", world.body(ball)?.position());

    // A ray cast down from above hits the ball first.
    let ray = RayCast::new(RVec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -20.0, 0.0));
    let hit = world.cast_ray(&ray, &QueryFilter::new())?.expect("the ray hits");
    assert_eq!(hit.body, ball);
    Ok(())
}
```

With Bevy, `use oxijolt::prelude::*` next to `bevy::prelude::*`; see `oxijolt::prelude`.

```toml
[dependencies]
oxijolt = "1"
```

## Roadmap

Everything planned for 1.0 is done; what works today, with links to the guides, is in
[docs/features.md](docs/features.md). Next:

- [x] Comparison with Rapier and Avian
- [x] Solver step counts per world
- [x] Contact tuning per world (penetration slop, speculative contact distance)
- [ ] Bevy plugin, in a separate repository
- [x] macOS (Apple silicon) in CI
- [x] macOS (Apple silicon) in releases
- [ ] Same results across operating systems, checked in CI
- [x] Prebuilt native libraries downloaded by the build script
- [x] Constraint force readout and breakable constraints
- [x] Playground: debris that breaks again when it hits the ground
- [x] Collision sub-steps and a single-threaded job system
- [x] Vehicle tuning: tire impulse callback, live drivetrain, mass and inertia override
- [x] World-space inverse inertia readout
- [x] glam 0.33 conversions for Bevy 0.20
- [x] Soft body skinning (cloth on an animated skeleton)

## Status

- Version 1.4.0 on [crates.io](https://crates.io/crates/oxijolt). The API follows semantic
  versioning from 1.0: a breaking change waits for the next major version.
- CI builds and tests Windows (MSVC) and Linux (GCC) on x86_64 and macOS (Apple clang) on Apple
  silicon, each in five configurations: default, `cross-platform-deterministic`,
  `double-precision`, `debug-renderer` and `asserts`. The bindings are committed for 64-bit
  Windows, Linux, macOS and Android targets; only the three CI targets are tested.
- Rust 1.88 or newer (checked in CI). The public API follows the rules in
  [docs/api-guidelines.md](docs/api-guidelines.md).
- Jolt and joltc come prebuilt on x86_64 Windows (MSVC), x86_64 Linux (GNU) and macOS on Apple
  silicon: the build script downloads the release archive and checks it against a sha256 in the
  crate. Elsewhere, or when no archive fits, they are built from pinned submodules with a C++
  toolchain and CMake. No build needs LLVM ([building](docs/building.md)).

## Guarantees and limits

- **Errors.** A call that returns `Err` was refused and changed nothing, unless its documentation
  says otherwise. A step that ran but dropped contacts returns `Ok` and says so in its
  `StepReport`.
- **Magnitudes.** Values Jolt checks only with debug assertions (non-finite poses, non-unit
  quaternions, zero dimensions, ids of another world) and magnitudes outside `oxijolt::limits`
  are refused with a typed error before they reach Jolt. Not every accepted value is proven safe:
  [limits](docs/limits.md) shows how the bounds follow from Jolt's arithmetic, and
  [coverage](docs/coverage.md) says which values only tests check and which no bound covers. CI
  also runs every test with Jolt's assertions on.
- **Precision.** Positions are `f32` by default, and the API accepts them within
  `limits::MAX_POSITION`, 5 km of the origin. The `double-precision` feature raises that to
  10 000 km. Or keep the scene near the origin with `PhysicsWorld::rebase`
  ([limits](docs/limits.md#frame-and-extent), [floating origin](docs/guide.md#floating-origin)).
- **Determinism.** On one machine the tests get bit-identical results with 1 and 4 worker threads
  and with the single-threaded and caller job systems, under Jolt's conditions (same binary, same calls in the same
  order). Across platforms and compilers results agree only with `cross-platform-deterministic`
  ([determinism](docs/determinism.md)).
- **Threads.** Changing a world takes `&mut`, reading it takes `&`; worlds are `Send` and `Sync`.
- **Global state.** oxijolt initialises Jolt once per process and owns joltc's process-global
  filter, listener, debug renderer and assertion procs; code that also uses `oxijolt-sys` directly
  must leave them alone.

## Compared with rolt

[rolt](https://crates.io/crates/rolt) and joltc-sys, from Second Half Games, are the Rust
bindings listed in Jolt's README. On crates.io they are at 0.3.1 with Jolt 5.0.0 (May 2024), and
they have no character controller, vehicles, ragdolls or heightfields. oxijolt binds Jolt 5.6
through [joltc] and has a safe API for all of those. It also has constraints, soft bodies, contact
events, saving and restoring a world, and stepping on your own job system. Its tests check that
results match bit for bit with 1 and 4 worker threads and on a caller job system. Out-of-range
values get a typed error before they reach Jolt. It builds without LLVM. Its shape and rigid-body
API is not complete yet; the [roadmap](#roadmap) says what is missing.

## Compared with Rapier and Avian

On ten of Rapier's own stress scenes, on a 16-core Linux machine at 1 to 16 threads with matched
settings ([comparison](docs/comparison.md), raw results and scripts in the repository):
Rapier's step is faster on scenes of many contacts, 1.9 to 2.2 times on a box stack and 1.3 to
1.85 times on a 43 000-box pyramid. oxijolt is faster on joints from four threads, up to 2.1
times, except a ball-joint net where its step gets slower at 8 and 16 threads. Avian is the slowest on
nearly every scene. oxijolt uses the least memory, under 55 % of Rapier's. All three gave the
same results at every thread count. oxijolt and Rapier each kept 7 of 10 scenes within the
quality bounds (with each engine's defaults Rapier 8, oxijolt 7); oxijolt lets some capsules
through the ground and its plank towers collapse. oxijolt brings
Jolt's character controller, wheeled and tracked vehicles and ragdolls; outside x86_64 Windows
and Linux and Apple silicon it needs a C++ toolchain, and it has no WASM target.

## Documentation

- [Guide](docs/guide.md): a game scene with terrain, queries, a floating origin, a character, a
  car and a ragdoll, run as doctests.
- Topic guides: [constraints](docs/constraints.md), [soft bodies](docs/soft-bodies.md),
  [events](docs/events.md), [save and restore](docs/state.md), [job systems](docs/job-system.md),
  [determinism](docs/determinism.md), [shape cooking](docs/shape-cooking.md),
  [building](docs/building.md).
- [Comparison with Rapier and Avian](docs/comparison.md): speed, quality and features on Rapier's
  stress scenes.
- [Limits](docs/limits.md) and [coverage](docs/coverage.md); [benchmarks](docs/benchmarks.md)
  against a game's budgets; [real meshes](docs/real-meshes.md) from open sources.
- [Character study](docs/character-study.md): which character laws CharacterVirtual's settings
  carry, and at what cost.
- [Playground](docs/playground.md): thirteen scenes with a window, headless runs and recorded clips.
- API docs: `cargo doc -p oxijolt --open`. Example: `cargo run -p oxijolt --example hello_world`.
- [CHANGELOG](CHANGELOG.md).

## Credits

The project began as a fork of SecondHalfGames/jolt-rust; [LINEAGE.md](LINEAGE.md) credits the
projects it builds on.

## License

Licensed under either of

* Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option. Jolt Physics and joltc are MIT licensed.

### Contribution

See [CONTRIBUTING.md](CONTRIBUTING.md) for how to build, test and send a change. Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

[joltc]: https://github.com/amerkoleci/joltc
