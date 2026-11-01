# Building

`oxijolt-sys` links joltc and Jolt Physics statically. On x86_64 Windows (MSVC), x86_64 Linux
(GNU) and macOS on Apple silicon its build script downloads them prebuilt from the GitHub release
of the crate's version and checks them against a sha256 packaged in the crate. Everywhere else, and whenever an archive does
not fit the build, it compiles them from the pinned submodules with CMake, always in Release. The
Rust bindings are committed, so no build needs LLVM.

## Requirements

- Rust 1.88 or newer (the `rust-version` of both crates, checked in CI). Raising it is a
  minor release, noted in the changelog.
- The platform linker Rust needs anyway: the MSVC Build Tools on Windows, `cc` and `libstdc++` on
  Linux, the Xcode Command Line Tools on macOS.
- For the download: `curl`. Windows 10 (1803 and later) ships it in `System32`, which is found
  without the `PATH`; macOS ships it; on Linux it has to be on the `PATH`, as it is on most
  systems.
- For the source build only: a C++ toolchain (MSVC on Windows, GCC or Clang elsewhere) and CMake
  3.20 or newer.

LLVM/libclang is needed only to regenerate the bindings (`cargo xtask bindings`) or with the
`bindgen` feature.

## Getting the crates

From crates.io; the `oxijolt-sys` package carries the archive list and the joltc and Jolt sources:

```toml
[dependencies]
oxijolt = "1"
```

Or from the repository, for changes not released yet; Cargo checks out the submodules with it:

```toml
[dependencies]
oxijolt = { git = "https://github.com/pockerhead/oxijolt" }
```

In a clone of the repository, fetch the submodules once:

```bash
git submodule update --init
cargo build                         # takes the release archive when the sources match a release, otherwise builds joltc + Jolt through CMake
cargo test --workspace              # everything, also the playground window and a debug-renderer native build
cargo run -p oxijolt --example hello_world
cargo run -p playground --release   # the playground window
```

`cargo build` and `cargo test` leave the playground (`examples/playground`) out. `--workspace`
includes it, and its default features turn on the window and `debug-renderer` for the whole build:
a second native build, and a `JOLTC_LIB_DIR` prefix without the debug renderer is refused. Use
`--workspace --exclude playground` for the library alone, as CI does, and
`cargo test -p playground --no-default-features` for the playground's scenes without a window.

Checkouts made before the crate rename run `git submodule sync && git submodule update --init` once.

The first source build compiles joltc and Jolt. Later builds of the same target and profile reuse
them; a change to the native sources, the pinned commits, a feature or `build.rs` runs CMake again.
The C++ build is heavy: run one cargo process at a time on a shared machine. On Windows a very long
`CARGO_TARGET_DIR` can make the CMake configure step fail (path length limit).

## Prebuilt native libraries

The default feature `prebuilt` of both crates lets the build script take a release archive instead
of compiling Jolt. Versions 1.0.1 and older have no archive list and always build from source.

### When an archive is used

The crate packages `prebuilt.txt`, written by the release workflow: the release URL, a fingerprint
of the native sources the archives were built from, and one line per archive with its target, C
runtime, features, toolchain baseline and sha256. The build script takes an archive when all of
these hold:

- the list is for this crate version (build metadata such as `+jolt-5.6.0` aside);
- it has an archive for the target, the C runtime and the crate features. Releases have one for
  `x86_64-pc-windows-msvc`, `x86_64-unknown-linux-gnu` and `aarch64-apple-darwin` with each of
  the eight subsets of `double-precision`, `cross-platform-deterministic` and `debug-renderer`;
  `asserts` and other targets build from source;
- the crate's native sources (`native/`, `build.rs`, `build/cmake_options.rs` and the joltc and
  Jolt sources CMake reads) give the list's fingerprint, so a build with changed native sources
  does not link an archive built from other code;
- on Windows: the dynamic C runtime (`/MD`, so not `crt-static`), and the MSVC linker the build
  uses (found as the `cc` crate finds it, or `RUSTC_LINKER` when that is a `link.exe`) is of the
  archive's toolset or newer;
- on Linux: a native build (host equals target), no linker or sysroot override, and the host's
  glibc (`getconf GNU_LIBC_VERSION`) is the archive's or newer;
- on macOS: the deployment target rustc links for (`rustc --print deployment-target`, which
  honours `MACOSX_DEPLOYMENT_TARGET`; 11.0 by default on Apple silicon) is the archive's or newer.

The archive is then fetched with `curl` from `<release URL>/<name>.tar.gz`, checked against the
list's sha256, unpacked into `OUT_DIR/prebuilt/<name>` and validated like a `JOLTC_LIB_DIR`
prefix. A successful build prints `using the prebuilt archive <name>` (visible with `cargo build
-vv`) and runs neither CMake nor a C++ compiler.

A git dependency or a clone has the list of the last release merged into its branch. It downloads
only while its native sources equal that release's: `main` downloads after the maintainer merges
`release/v<version>` (see [Releases](#releases)) and stops at the next change to the native code.

### Verification

The sha256 of every archive is part of the published crate, which Cargo checks against its own
registry checksum (and `Cargo.lock`). Nothing next to the archive is fetched to verify it. A
mirror, proxy or changed release asset can therefore only make the download fail, and the build
then falls back to the source build; it cannot make the build link other bytes. What it links was
built by this repository's Release workflow from the tagged commit. To compile everything
yourself, set `JOLTC_PREBUILT=off` or turn the feature off.

The archive is also checked before it is written to disk: only the 14 expected files and
directories, no links, no paths outside its directory, at most 64 MiB downloaded and 256 MiB
unpacked.

### Order of precedence

1. Under docs.rs (`DOCS_RS`) only the headers are copied; nothing is downloaded, built or linked.
2. `JOLTC_LIB_DIR` links that prefix (see [below](#your-own-prefix-joltc_lib_dir)).
3. `JOLTC_PREBUILT=off` builds from source.
4. With the `prebuilt` feature, a matching archive; otherwise one warning
   `building joltc from source: <reason>` and the source build. Without the feature, the source
   build, silently.

Cargo shows build-script warnings only for path and workspace crates; a registry dependency falls
back without a visible warning (`cargo build -vv` shows it).

### Switches

| Setting | Effect |
|---|---|
| `JOLTC_PREBUILT=auto` (or unset) | an archive when one fits, otherwise the source build with a warning |
| `JOLTC_PREBUILT=off` (also `0`, `false`) | never look for an archive |
| `JOLTC_PREBUILT=require` | an archive or a build error that names the reason; also an error with the feature off |
| `JOLTC_PREBUILT_URL=<base>` | fetch `<base>/<name>.tar.gz` instead of the release URL, for a mirror. It may be `http`, and holds no `@`, `?`, `#` or spaces; the sha256 still comes from the crate |
| `default-features = false` on `oxijolt` (or `oxijolt-sys`) | turns the `prebuilt` feature off: no download, the source build |

`oxijolt` depends on `oxijolt-sys` without its default features and forwards `prebuilt`, so
`default-features = false` on `oxijolt` alone is enough.

The release URL and its redirects must be `https`. One download may take 10 s to connect and
120 s in all. `curl` takes proxies (`https_proxy`, `ALL_PROXY`, `NO_PROXY`) and certificates from
the system, so a corporate proxy or CA works as it does for `curl` itself.

### Offline, vendored and Nix builds

Cargo does not pass `--offline` to build scripts. These environments skip the download and build
from source without trying the network:

- `CARGO_NET_OFFLINE=true` (or `1`) in the environment. With `cargo build --offline` alone, the
  script cannot tell; set `CARGO_NET_OFFLINE=true` or `JOLTC_PREBUILT=off` too;
- vendored sources (`cargo vendor`, recognised by Cargo's `.cargo-checksum.json`);
- a Nix build (`NIX_BUILD_TOP`).

For an offline build that should still link an archive, download it beforehand, unpack it and
point `JOLTC_LIB_DIR` at it.

### Fallback reasons

The warning (or the `require` error) ends with one of these reasons:

| Reason | What to do |
|---|---|
| offline build, vendored sources, inside a Nix build | nothing: the source build is intended; use `JOLTC_LIB_DIR` for a prebuilt prefix |
| the archive list has no archives | a git checkout from before the first release with archives: build from source or set `JOLTC_PREBUILT=off` |
| the archive list is malformed | the packaged `prebuilt.txt` was edited by hand: restore it from the release |
| the archive list is for version X, the crate is Y | the version was bumped after the list was recorded: as above |
| the native sources differ from the released ones | a checkout with native changes: as above |
| native sources missing or not regular files | `git submodule update --init` |
| no archive for this target, CRT and features | another target or `asserts`: the source build is the only way |
| crt-static is enabled; the archives use /MD | use the dynamic runtime, or build from source |
| a linker or sysroot override is set; a cross build (Windows and Linux) | link with the default linker on the target itself, or build from source |
| MSVC toolset X is older than the archive's Y, or unknown | update Visual Studio (Build Tools) 2022 to its latest release, or build from source |
| glibc X is older than the archive's Y, or unknown | a newer distribution, or build from source |
| macOS deployment target X is older than the archive's Y, or unknown | unset or raise `MACOSX_DEPLOYMENT_TARGET`, or build from source |
| curl was not found | install `curl`, or build from source |
| download failed, checksum mismatch, cannot unpack the archive | check the network, proxy or `JOLTC_PREBUILT_URL`; a checksum mismatch means the served file is not the released one |
| the archive was refused | the unpacked prefix failed the manifest check (report it), or `vendor/joltc` is missing (`git submodule update --init`) |

### Cache

The unpacked archive stays in `OUT_DIR/prebuilt/<name>` with a list of its files' sha256. Later
builds of the same target directory check those hashes and use it without a request; a changed or
missing file fetches the archive again. `cargo clean` removes it.

## Targets

The bindings are committed under `crates/oxijolt-sys/src/bindings/`, one file per ABI family and
configuration. The supported targets are those with committed bindings (`build/targets.rs`):

| Family | Targets |
|---|---|
| `msvc` | `x86_64-pc-windows-msvc`, `aarch64-pc-windows-msvc` |
| `gnu` | `x86_64-unknown-linux-gnu`, `aarch64-unknown-linux-gnu`, `x86_64-apple-darwin`, `aarch64-apple-darwin`, `x86_64-linux-android`, `aarch64-linux-android`, `x86_64-pc-windows-gnu` |
| `wasm32` | `wasm32-unknown-unknown`, `wasm32-unknown-emscripten`, `wasm32-wasip1` |

Other 32-bit targets and unregistered ones are refused when the build starts.
`wasm32-unknown-unknown` builds from source with [wasi-sdk](https://github.com/WebAssembly/wasi-sdk):
`WASI_SDK_PATH` names its install directory, Ninja is on `PATH` and Rust is 1.91 or newer; wasi-sdk
34.0 is tested. A `JOLTC_LIB_DIR` prefix for it records the SDK and needs the same `WASI_SDK_PATH`
to link. No archive is published for wasm, so the default `prebuilt` feature falls back to the
source build. `wasm32-unknown-emscripten` and `wasm32-wasip1` have committed bindings only and are
refused, also under `DOCS_RS`. The safe crate `oxijolt` builds for wasm32-unknown-unknown as
well, single-threaded; its crate documentation lists what changes on that target.
`bash scripts/check_wasm_raw.sh` builds a raw wasm32-unknown-unknown module in both precisions
and runs it in Node (a world kept across export calls, no WASI imports); it needs `WASI_SDK_PATH`, Ninja and Node. CI builds and tests
`x86_64-pc-windows-msvc`, `x86_64-unknown-linux-gnu` and `aarch64-apple-darwin`; the other
targets have bindings that the generator produced byte-identical to their family's, but CI does
not build or test them. Android cross
builds go through the NDK's CMake toolchain (`ANDROID_NDK_HOME`).

On Windows the native libraries use the same C runtime as the Rust target: the dynamic one (`/MD`)
by default, the static one (`/MT`) with `-C target-feature=+crt-static`.

## Features

| Feature | Effect |
|---|---|
| `prebuilt` (default) | takes a verified release archive instead of the source build when one fits ([above](#prebuilt-native-libraries)) |
| `double-precision` | world positions in `f64` (`Real`, `RVec3`) |
| `cross-platform-deterministic` | builds Jolt with `CROSS_PLATFORM_DETERMINISTIC` ([determinism.md](determinism.md#across-machines)); slower |
| `debug-renderer` | compiles Jolt's debug renderer; enables `PhysicsWorld::debug_lines` and `debug_lines_into` |
| `asserts` | compiles Jolt with its debug assertions; with `oxijolt`, a failed assertion prints its message and aborts the process, except the physics-update-error assertion, which `step` reports in its `StepReport` |
| `bindgen` | generates the bindings with libclang at build time instead of using the committed ones; it adds no target |

`oxijolt` forwards each of them to `oxijolt-sys`. A feature that changes the native build
(everything except `prebuilt` and `bindgen`) is part of the native library's identity, see below.

## Your own prefix: `JOLTC_LIB_DIR`

Set `JOLTC_LIB_DIR` to skip both the download and CMake and link native libraries built earlier.
It points at an install prefix:

```text
<prefix>/lib/          joltc (or joltc_double) and Jolt static libraries
<prefix>/include/joltc.h
<prefix>/include/joltc_ext.h
<prefix>/oxijolt-sys-manifest.txt
```

A source build leaves such a prefix in `target/<profile>/build/oxijolt-sys-*/out/joltc`, and an
unpacked release archive is one. A prefix is tied to the target, the C runtime, the crate features
and the pinned joltc and Jolt commits. The build script checks all of them against the manifest
(refusing a manifest with a repeated key) and refuses a prefix built for another configuration. CI
builds the native library once per configuration and links every later build through such a
prefix.

## Release archives

Each tagged release from the first one with an archive list carries prebuilt prefixes on its
[GitHub release page](https://github.com/pockerhead/oxijolt/releases), one archive per target and
feature subset (`oxijolt-sys-<version>-<target>-<subset>.tar.gz`, the subset `default` or the
features joined by `+`), with a `.sha256` file each:

- Windows archives are built on `windows-2022` with Visual Studio 2022 (MSVC toolset 14.44) and
  the dynamic runtime (`/MD`). MSVC links objects only from its own toolset or an older one, so
  link them with toolset 14.44 or newer (Visual Studio 2022 17.14 or 2026), and not with
  `crt-static`;
- Linux archives are built with GCC 12 in an `ubuntu:22.04` container and need glibc 2.35 or newer
  and a compatible libstdc++ at run time;
- macOS archives are built for Apple silicon (arm64 only) on the `macos-15` runner with its Xcode
  (the versions are in `PROVENANCE.txt`) for macOS 11.0 or newer; the release checks that every
  object in them has that minimum. Intel Macs build from source;
- on x86_64 the CPU needs AVX2, BMI1, LZCNT, POPCNT and F16C; builds without
  `cross-platform-deterministic` also use FMA. Apple silicon archives use no extra CPU features;
- Jolt's assertions are off.

Each archive holds `lib/`, `include/`, the manifest, `PROVENANCE.txt` (compiler, options and
source commit; on Linux the glibc, on macOS the Xcode, SDK and minimum macOS), and the licences of oxijolt, joltc and Jolt. The build script uses them on its own;
to link one by hand, unpack it and point `JOLTC_LIB_DIR` at its directory.

## Releases

A maintainer bumps the version of both crates, commits, and pushes the tag `v<version>`. The
`Release` workflow then:

1. builds the 24 archives from the tagged commit, each with its smoke test;
2. writes their list into `crates/oxijolt-sys/prebuilt.txt` as one commit on top of the tag, and
   checks on Linux and macOS that both packaged crates build from the archives (served locally)
   without CMake;
3. attaches the archives to the tag's GitHub release;
4. pushes that commit to the branch `release/v<version>`;
5. publishes both crates to crates.io from it (`Publish to crates.io`), after checking every public
   asset against the list and building the packages from the real download on Linux and macOS.

CI never writes `main`: the maintainer merges `release/v<version>` into `main` by hand, after which
git dependencies on `main` download too.

When a step fails:

- "Re-run failed jobs" on the Release run resumes it;
- or run `Publish to crates.io` by hand with `ref=release/v<version>`. It publishes the crates that
  are not on crates.io yet and skips the others; if the published `oxijolt-sys` carries another
  archive list, it fails, and the fix is a new version;
- pushing the tag again works only while the version is not on crates.io: it rebuilds the archives,
  replaces the release assets and rewrites `release/v<version>`. Once the version is published, a
  tag run stops at its first job.

[CONTRIBUTING.md](../CONTRIBUTING.md) ("Releases", "Publishing") has the repository side of this.

## Regenerating the bindings

A change to `joltc.h`, `joltc_ext.h`, `build/bindgen_options.rs` or `build/targets.rs` regenerates
the bindings in the same commit:

```bash
cargo xtask bindings            # regenerates src/bindings/ and src/bindings/inputs.txt
cargo xtask bindings --check    # fails if the committed bindings differ from the generator
```

The build script refuses bindings whose recorded inputs differ from the linked headers. The
canonical bytes are those of CI's `Committed bindings` job (LLVM 18 on Ubuntu 24.04).
