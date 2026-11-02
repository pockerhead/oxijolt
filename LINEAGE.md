# Lineage

What this repository is built on, who wrote it, and under which licence. Credit stays with the
original authors.

| Layer | Project | Authors | Licence | Where it lives here |
|---|---|---|---|---|
| Physics engine | [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6 | Jorrit Rouwe and contributors | MIT | `crates/oxijolt-sys/vendor/JoltPhysics` (submodule, tag v5.6.0), built from source, unmodified |
| C wrapper | [joltc](https://github.com/amerkoleci/joltc) | Amer Koleci and contributors (the C layer of JoltPhysicsSharp, also used by LÖVR) | MIT | `crates/oxijolt-sys/vendor/joltc` (submodule, pinned commit), unmodified; input of `crates/oxijolt-sys`. Functions this repository adds to joltc, in joltc's naming, live in `crates/oxijolt-sys/native/joltc_ext/` and are compiled into the same archive |
| wasm runtime | [wasi-sdk](https://github.com/WebAssembly/wasi-sdk) 34.0: libc++, libc++abi and compiler-rt builtins from LLVM, and [wasi-libc](https://github.com/WebAssembly/wasi-libc) (with parts of musl, cloudlibc and musl-fts) | the LLVM project and the wasi-sdk and wasi-libc contributors | Apache-2.0 WITH LLVM-exception; wasi-libc also Apache-2.0 or MIT, its third-party parts MIT, BSD-2-Clause and BSD-3-Clause | not in the repository: copied from the SDK into wasm prefixes and bundled in the wasm release archives with their licence texts (`licenses/`) |
| Benchmark scenes | [Rapier](https://github.com/dimforge/rapier) `examples3d/stress_tests` at v0.36.0 | Dimforge and contributors | Apache-2.0 | `benches/comparison/src/scenes/` and `benches/comparison/tests/`, ported to engine-neutral data; each file names its source and the change |

The project began as a fork of
[SecondHalfGames/jolt-rust](https://github.com/SecondHalfGames/jolt-rust). No code from it remains:
the raw and safe layers and the build script were written anew on joltc.

## What this repository adds
- `oxijolt-sys`: bindings over joltc's `joltc.h` and the extension's `joltc_ext.h`, committed per
  ABI family so a build needs no LLVM; layout assertions for the FFI types in use on the C++ side
  (`native/layout_checks.cpp`) and the Rust side (`src/layout.rs`); a native build through CMake or
  a validated prebuilt prefix (`JOLTC_LIB_DIR`).
- The joltc extension (`native/joltc_ext/`): state recording, character updates with explicit
  filters and temp allocator, ragdoll parts and joints that keep every setting, constraint motor
  access, path, pulley and rack-and-pinion constraints, soft body functions, materials with user
  data and a soft body contact listener.
- `oxijolt`: a safe, idiomatic Rust API with headless tests. The README lists its features, and
  `docs/coverage.md` lists each one with its tests and what is not bound yet.
- A magnitude policy (`oxijolt::limits`) that refuses values out of bounds before they reach Jolt.
- Determinism gates over thread counts and job systems, and leak gates.
- CI on Windows and Linux in five configurations, one of them with Jolt's assertions on.

## Licence
Our additions are dual-licensed MIT OR Apache-2.0 (`LICENSE-MIT`, `LICENSE-APACHE`). Jolt Physics
and joltc remain MIT under their own `LICENSE` files.
