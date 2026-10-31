//! Builds joltc and Jolt (or validates a prebuilt copy), links them and
//! generates the raw bindings over `joltc.h` and the fork's `joltc_ext.h`.
//!
//! Four ways to get the native libraries, in this order:
//! - under docs.rs (`DOCS_RS`), only the headers are copied and nothing is linked;
//! - with `JOLTC_LIB_DIR` set, an already built install prefix (`lib/`,
//!   `include/joltc.h`, `include/joltc_ext.h` and a manifest describing everything
//!   that affects the ABI) is used after its manifest, archives and headers are
//!   validated, and CMake is not run at all;
//! - with the `prebuilt` feature, the release archive of this version listed in
//!   `prebuilt.txt` for the target, CRT and features, when the environment, the
//!   native sources and the toolchain allow it (`build/prebuilt.rs`);
//! - otherwise CMake builds `native/` from the `vendor/` submodules into
//!   `OUT_DIR/joltc`, a prefix of the same shape. A reason not to use an archive
//!   is reported as one warning; `JOLTC_PREBUILT=require` makes it an error and
//!   `JOLTC_PREBUILT=off` skips the archives.
//!
//! The bindings are committed under `src/bindings/`, one file per ABI family
//! (see `build/targets.rs`) and configuration; the script checks that they were
//! generated from the linked headers and picks one. With the `bindgen` feature
//! it generates them into `OUT_DIR` with libclang instead.

use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{bail, Context};

#[cfg(feature = "bindgen")]
#[path = "build/bindgen_options.rs"]
mod bindgen_options;
#[path = "build/cmake_options.rs"]
mod cmake_options;
// Shared with the xtask list writer, which uses the parts build.rs does not.
#[allow(dead_code)]
#[path = "build/prebuilt.rs"]
mod prebuilt;
#[cfg(feature = "prebuilt")]
#[allow(dead_code)]
#[path = "build/prebuilt_archive.rs"]
mod prebuilt_archive;
#[cfg(feature = "prebuilt")]
#[allow(dead_code)]
#[path = "build/prebuilt_curl.rs"]
mod prebuilt_curl;
// Shared with the xtask regenerator; each bindings mode uses a part of it.
#[allow(dead_code)]
#[path = "build/targets.rs"]
mod targets;

use cmake_options::{AndroidNdk, FeatureSwitches, ANDROID_GENERATOR};
use prebuilt::{Mode, Unavailable};
use targets::Family;

/// joltc commit of the `vendor/joltc` submodule. Must match the gitlink; CI checks this.
const JOLTC_COMMIT: &str = "886e088675bae3a086f8318c7803f8ee962c2f2c";
/// Jolt Physics commit of the `vendor/JoltPhysics` submodule. Must match the gitlink; CI checks this.
const JOLT_COMMIT: &str = "e77f175595e64cb44218cc9d9d56fc365ad0e36a";
/// Jolt Physics release of [`JOLT_COMMIT`].
const JOLT_VERSION: &str = "5.6.0";
/// Revision of the fork's joltc additions in `native/joltc_ext/`. Bump it whenever
/// anything there changes, so prebuilt prefixes built before the change are refused.
const JOLTC_EXT_REVISION: &str = "27";
/// Name of the manifest file in an install prefix.
const MANIFEST_FILE: &str = "oxijolt-sys-manifest.txt";

/// Everything about the target and the crate features that shapes the native build.
struct NativeConfig {
    double_precision: bool,
    cross_platform_deterministic: bool,
    asserts: bool,
    /// Jolt's debug renderer and joltc's drawing functions are compiled in.
    debug_renderer: bool,
    target: String,
    target_os: String,
    target_env: String,
    target_arch: String,
    /// The target's family of committed bindings.
    #[cfg_attr(feature = "bindgen", allow(dead_code))]
    family: &'static Family,
    /// MSVC runtime library in CMake's spelling, or `"none"` off MSVC.
    crt: &'static str,
}

impl NativeConfig {
    /// Reads the target from Cargo's environment; `cfg!` would describe the build host.
    ///
    /// Fails for a target without committed bindings, before any native work.
    fn from_env() -> anyhow::Result<Self> {
        let feature = |name: &str| env::var_os(format!("CARGO_FEATURE_{name}")).is_some();
        let var = |name: &str| env::var(name).unwrap_or_default();

        let target_env = var("CARGO_CFG_TARGET_ENV");
        let crt_static = var("CARGO_CFG_TARGET_FEATURE")
            .split(',')
            .any(|f| f == "crt-static");
        let crt = match (target_env.as_str(), crt_static) {
            ("msvc", true) => "MultiThreaded",
            ("msvc", false) => "MultiThreadedDLL",
            _ => "none",
        };

        let target = var("TARGET");
        let family = targets::check_target(&target, &var("CARGO_CFG_TARGET_POINTER_WIDTH"))?;

        Ok(NativeConfig {
            double_precision: feature("DOUBLE_PRECISION"),
            cross_platform_deterministic: feature("CROSS_PLATFORM_DETERMINISTIC"),
            asserts: feature("ASSERTS"),
            debug_renderer: feature("DEBUG_RENDERER"),
            target,
            target_os: var("CARGO_CFG_TARGET_OS"),
            target_env,
            target_arch: var("CARGO_CFG_TARGET_ARCH"),
            family,
            crt,
        })
    }

    /// Name of the joltc library target, which joltc derives from the precision.
    fn joltc_lib(&self) -> &'static str {
        if self.double_precision {
            "joltc_double"
        } else {
            "joltc"
        }
    }

    /// File name of a static library as the target's toolchain spells it.
    fn archive_name(&self, lib: &str) -> String {
        if self.target_env == "msvc" {
            format!("{lib}.lib")
        } else {
            format!("lib{lib}.a")
        }
    }
}

/// `ON` or `OFF`, as CMake options expect.
fn on_off(value: bool) -> &'static str {
    if value {
        "ON"
    } else {
        "OFF"
    }
}

/// Every input that changes the ABI or the archive contents, in manifest order.
///
/// Both the CMake path (writing) and the prebuilt path (validating) use this
/// one list, so they cannot drift apart.
fn manifest_entries(cfg: &NativeConfig) -> Vec<(&'static str, String)> {
    vec![
        ("format", "1".to_owned()),
        ("joltc_commit", JOLTC_COMMIT.to_owned()),
        ("joltc_ext", JOLTC_EXT_REVISION.to_owned()),
        ("jolt_commit", JOLT_COMMIT.to_owned()),
        ("jolt_version", JOLT_VERSION.to_owned()),
        ("target", cfg.target.clone()),
        ("crt", cfg.crt.to_owned()),
        ("double_precision", on_off(cfg.double_precision).to_owned()),
        (
            "cross_platform_deterministic",
            on_off(cfg.cross_platform_deterministic).to_owned(),
        ),
        ("asserts", on_off(cfg.asserts).to_owned()),
        ("object_layer_bits", "32".to_owned()),
        ("debug_renderer", on_off(cfg.debug_renderer).to_owned()),
        ("floating_point_exceptions", "OFF".to_owned()),
        ("profiler", "OFF".to_owned()),
        ("joltc_lib", cfg.joltc_lib().to_owned()),
    ]
}

/// The manifest file contents: one `key=value` per line.
fn manifest_text(cfg: &NativeConfig) -> String {
    manifest_entries(cfg)
        .iter()
        .map(|(key, value)| format!("{key}={value}\n"))
        .collect()
}

/// Directory of this crate.
fn manifest_dir() -> anyhow::Result<PathBuf> {
    env::var_os("CARGO_MANIFEST_DIR")
        .map(PathBuf::from)
        .context("CARGO_MANIFEST_DIR is not set")
}

/// Path with forward slashes, which CMake accepts on every platform.
fn cmake_path(path: &Path) -> String {
    path.display().to_string().replace('\\', "/")
}

fn main() -> anyhow::Result<()> {
    println!("cargo:rerun-if-env-changed=JOLTC_LIB_DIR");
    println!("cargo:rerun-if-env-changed=DOCS_RS");
    for var in [
        "JOLTC_PREBUILT",
        "JOLTC_PREBUILT_URL",
        "CARGO_NET_OFFLINE",
        "NIX_BUILD_TOP",
        "RUSTC_LINKER",
    ] {
        println!("cargo:rerun-if-env-changed={var}");
    }
    println!("cargo:rerun-if-changed=build.rs");

    let cfg = NativeConfig::from_env()?;
    let prefix = if env::var_os("DOCS_RS").is_some() {
        headers_only_prefix()?
    } else {
        let prefix = match env::var_os("JOLTC_LIB_DIR") {
            Some(dir) => use_prebuilt(PathBuf::from(dir), &cfg)?,
            None => native_prefix(&cfg)?,
        };
        link(&prefix, &cfg);
        prefix
    };

    let bindings = select_bindings(&prefix, &cfg)?;
    println!("cargo:rustc-env=JOLTC_BINDINGS={}", bindings.display());
    println!("cargo:rustc-env=OXIJOLT_SYS_JOLT_COMMIT={JOLT_COMMIT}");
    println!("cargo:rustc-env=OXIJOLT_SYS_JOLTC_COMMIT={JOLTC_COMMIT}");
    println!("cargo:rustc-env=OXIJOLT_SYS_JOLTC_EXT_REVISION={JOLTC_EXT_REVISION}");
    Ok(())
}

/// A prefix with only `include/joltc.h` and `include/joltc_ext.h`, copied from the package.
///
/// docs.rs runs rustdoc offline under a time limit, and rustdoc links nothing, so the
/// bindings are all it needs: Jolt is not built there.
fn headers_only_prefix() -> anyhow::Result<PathBuf> {
    let crate_dir = manifest_dir()?;
    let prefix =
        PathBuf::from(env::var_os("OUT_DIR").context("OUT_DIR is not set")?).join("headers");
    let include = prefix.join("include");
    fs::create_dir_all(&include).with_context(|| format!("cannot create {}", include.display()))?;
    for (source, name) in [
        ("vendor/joltc/include/joltc.h", "joltc.h"),
        ("native/joltc_ext/joltc_ext.h", "joltc_ext.h"),
    ] {
        fs::copy(crate_dir.join(source), include.join(name))
            .with_context(|| format!("cannot copy {source}"))?;
    }
    Ok(prefix)
}

/// The native prefix outside docs.rs without `JOLTC_LIB_DIR`: a release archive when one can
/// be used, otherwise the CMake build, as `JOLTC_PREBUILT` says.
fn native_prefix(cfg: &NativeConfig) -> anyhow::Result<PathBuf> {
    let mode =
        Mode::parse(env::var("JOLTC_PREBUILT").ok().as_deref()).map_err(anyhow::Error::msg)?;
    if mode == Mode::Off {
        return build_with_cmake(cfg);
    }
    match try_prebuilt(cfg) {
        Ok(prefix) => Ok(prefix),
        Err(reason) if mode == Mode::Require => bail!("JOLTC_PREBUILT=require: {reason}"),
        // Without the feature the user chose the source build.
        Err(Unavailable::FeatureOff) => build_with_cmake(cfg),
        Err(reason) => {
            println!("cargo:warning=building joltc from source: {reason}");
            build_with_cmake(cfg)
        }
    }
}

#[cfg(not(feature = "prebuilt"))]
fn try_prebuilt(_cfg: &NativeConfig) -> Result<PathBuf, Unavailable> {
    Err(Unavailable::FeatureOff)
}

/// The prefix of the release archive for this build, or why there is none.
#[cfg(feature = "prebuilt")]
fn try_prebuilt(cfg: &NativeConfig) -> Result<PathBuf, Unavailable> {
    let crate_dir = manifest_dir().map_err(|e| Unavailable::Refused(format!("{e:#}")))?;
    // Printed before any early return, so that a later change of the list or the sources
    // reruns the decision.
    println!("cargo:rerun-if-changed={}", prebuilt::LIST_FILE);
    for input in prebuilt::SOURCE_INPUTS {
        println!("cargo:rerun-if-changed={input}");
    }

    let var = |name: &str| env::var(name).ok();
    if let Some(reason) = prebuilt::skip_reason(var, &crate_dir) {
        return Err(reason);
    }
    let text = fs::read_to_string(crate_dir.join(prebuilt::LIST_FILE))
        .map_err(|e| Unavailable::List(e.to_string()))?;
    let list = prebuilt::parse(&text).map_err(Unavailable::List)?;

    let crt_static = var("CARGO_CFG_TARGET_FEATURE")
        .unwrap_or_default()
        .split(',')
        .any(|f| f == "crt-static");
    if crt_static {
        return Err(Unavailable::CrtStatic);
    }
    let crate_version = var("CARGO_PKG_VERSION").unwrap_or_default();
    let archive = prebuilt::select(&list, &crate_version, &archive_config(cfg))?;
    if list.sources != Some(prebuilt_archive::sources_fingerprint(&crate_dir)?) {
        return Err(Unavailable::SourcesDiffer);
    }

    // From here the outcome depends on tools found through the environment.
    for name in ["PATH", "VCToolsVersion", "VCINSTALLDIR"] {
        println!("cargo:rerun-if-env-changed={name}");
    }
    prebuilt::check_linker(archive, &linker(cfg)?)?;

    let out_dir = env::var_os("OUT_DIR")
        .map(PathBuf::from)
        .ok_or_else(|| Unavailable::Refused("OUT_DIR is not set".to_owned()))?;
    let prefix = fetch_archive(&list, archive, cfg, &out_dir)?;
    if !crate_dir.join("vendor/joltc/include/joltc.h").is_file() {
        return Err(Unavailable::Refused("vendor/joltc is missing".to_owned()));
    }
    let prefix = use_prebuilt(prefix, cfg).map_err(|e| {
        prebuilt_archive::remove_cached(&out_dir, &archive.name);
        Unavailable::Refused(format!("{e:#}"))
    })?;
    println!("using the prebuilt archive {}", archive.name);
    Ok(prefix)
}

/// The archive configuration this build needs, in the manifest's spelling.
#[cfg(feature = "prebuilt")]
fn archive_config(cfg: &NativeConfig) -> prebuilt::ArchiveConfig {
    prebuilt::ArchiveConfig {
        target: cfg.target.clone(),
        crt: cfg.crt.to_owned(),
        double_precision: cfg.double_precision,
        cross_platform_deterministic: cfg.cross_platform_deterministic,
        debug_renderer: cfg.debug_renderer,
        asserts: cfg.asserts,
    }
}

/// What will link an archive for this target. Only MSVC, native Linux GNU and macOS builds
/// have archives.
#[cfg(feature = "prebuilt")]
fn linker(cfg: &NativeConfig) -> Result<prebuilt::Linker, Unavailable> {
    let var = |name: &str| env::var(name).ok();
    if cfg.target_env == "msvc" {
        let found =
            prebuilt::effective_msvc_linker(var, || prebuilt_archive::find_link_exe(&cfg.target));
        Ok(prebuilt::Linker::Msvc(found))
    } else if cfg.target_os == "linux" && cfg.target_env == "gnu" {
        Ok(prebuilt::Linker::Gnu {
            cross: var("HOST") != var("TARGET"),
            overridden: prebuilt::gnu_linker_overridden(var),
            glibc: prebuilt_archive::host_glibc(),
        })
    } else if cfg.target_os == "macos" {
        println!("cargo:rerun-if-env-changed=MACOSX_DEPLOYMENT_TARGET");
        let rustc = env::var_os("RUSTC").unwrap_or_else(|| "rustc".into());
        Ok(prebuilt::Linker::Apple {
            deployment_target: prebuilt_archive::rustc_deployment_target(&rustc, &cfg.target),
        })
    } else {
        Err(Unavailable::NoArchive)
    }
}

/// The unpacked prefix of `archive`: the copy in `OUT_DIR/prebuilt` while it is intact,
/// otherwise downloaded from the release (or `JOLTC_PREBUILT_URL`) with `curl`, checked against
/// the list's sha256 and unpacked.
#[cfg(feature = "prebuilt")]
fn fetch_archive(
    list: &prebuilt::ArchiveList,
    archive: &prebuilt::Archive,
    cfg: &NativeConfig,
    out_dir: &Path,
) -> Result<PathBuf, Unavailable> {
    let joltc = cfg.archive_name(cfg.joltc_lib());
    let jolt = cfg.archive_name("Jolt");
    let libraries = [joltc.as_str(), jolt.as_str()];
    if let Some(prefix) = prebuilt_archive::cached(out_dir, archive, libraries) {
        return Ok(prefix);
    }
    let list_url = list.url.as_deref().unwrap_or_default();
    let (base, https_only) =
        prebuilt_curl::base_url(list_url, env::var("JOLTC_PREBUILT_URL").ok().as_deref())?;
    let url = prebuilt_curl::archive_url(&base, &archive.name);
    let limits = prebuilt_curl::Limits::DEFAULT;
    let bytes = prebuilt_curl::download("curl".as_ref(), &url, https_only, &limits)?;
    prebuilt_archive::verify_and_unpack(&bytes, archive, libraries, out_dir)
}

/// Builds the native prefix from the submodules with CMake and returns its path.
fn build_with_cmake(cfg: &NativeConfig) -> anyhow::Result<PathBuf> {
    let crate_dir = manifest_dir()?;
    let joltc_dir = crate_dir.join("vendor").join("joltc");
    let jolt_dir = crate_dir.join("vendor").join("JoltPhysics");

    // Printing any rerun rule replaces Cargo's scan of the whole package, so
    // Rust-only edits do not rerun CMake. Jolt's Assets are left out on purpose.
    for input in prebuilt::SOURCE_INPUTS {
        println!("cargo:rerun-if-changed={input}");
    }

    if !joltc_dir.join("CMakeLists.txt").exists() || !jolt_dir.join("Build").exists() {
        bail!(
            "the joltc and Jolt Physics sources are missing under {}: run `git submodule update --init`, \
             or set JOLTC_LIB_DIR to a prebuilt prefix",
            crate_dir.join("vendor").display()
        );
    }

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").context("OUT_DIR is not set")?);
    let mut config = cmake::Config::new(crate_dir.join("native"));

    // Release whatever Cargo's profile: the cmake crate maps an unoptimized Rust
    // build to CMake's Debug, which on MSVC selects the debug CRT that Rust
    // binaries do not link. One fixed profile also keeps Jolt's own
    // configuration switches the same for every Cargo profile.
    config.profile("Release");
    config.out_dir(out_dir.join("joltc"));

    config.define("JOLTC_SOURCE_DIR", cmake_path(&joltc_dir));
    config.define("JOLT_PHYSICS_ROOT", cmake_path(&jolt_dir));
    // Jolt comes from the submodule; never download anything at build time.
    config.define("FETCHCONTENT_FULLY_DISCONNECTED", "ON");

    // Having IPO/LTO turned on breaks lld on Windows, and MSVC `/GL` objects
    // cannot be linked by other toolchains.
    config.define("INTERPROCEDURAL_OPTIMIZATION", "OFF");
    // Warnings when building Jolt or joltc don't matter to users of oxijolt-sys.
    config.define("ENABLE_ALL_WARNINGS", "OFF");
    // build.rs installs only what it links (see native/CMakeLists.txt).
    config.define("ENABLE_INSTALL", "OFF");
    config.define("GENERATE_DEBUG_SYMBOLS", "OFF");

    // Headless: no GPU compute backends. DX12 would also add system libraries
    // to the link that this script does not emit.
    for backend in [
        "JPH_USE_DX12",
        "JPH_USE_VK",
        "JPH_USE_MTL",
        "JPH_USE_CPU_COMPUTE",
    ] {
        config.define(backend, "OFF");
    }

    // Otherwise every Jolt worker thread enables floating point traps, and
    // Rust callbacks would later run under them.
    config.define("FLOATING_POINT_EXCEPTIONS_ENABLED", "OFF");
    // No profiler API is bound; the instrumentation would only cost time.
    config.define("PROFILER_IN_DEBUG_AND_RELEASE", "OFF");
    // Jolt's debug renderer only with the `debug-renderer` feature. Both switches are needed:
    // DEBUG_RENDERER_IN_DEBUG_AND_RELEASE defaults to ON and would enable it for this Release build.
    config.define("DEBUG_RENDERER_IN_DISTRIBUTION", on_off(cfg.debug_renderer));
    config.define(
        "DEBUG_RENDERER_IN_DEBUG_AND_RELEASE",
        on_off(cfg.debug_renderer),
    );

    let features = FeatureSwitches {
        double_precision: cfg.double_precision,
        asserts: cfg.asserts,
        cross_platform_deterministic: cfg.cross_platform_deterministic,
    };
    for (option, enabled) in features.options() {
        config.define(option, on_off(enabled));
    }

    if !matches!(cfg.target_os.as_str(), "macos" | "ios") {
        // joltc caches `CMAKE_OSX_ARCHITECTURES=x86_64;arm64` on every platform, and Jolt reads an
        // `arm64` there as an ARM build: GCC and Clang would then compile Jolt without its x86
        // instruction-set flags (`-mavx2 -mfma ...`) and its `JPH_USE_*` definitions. An empty
        // cache entry keeps joltc's default out.
        config.define("CMAKE_OSX_ARCHITECTURES", "");
    }

    if cfg.target_env == "msvc" {
        // Jolt and joltc strip `/EHsc` from their own scopes while C++
        // exceptions are off; the layout checks in the wrapper scope need it.
        config.cxxflag("/EHsc");

        // The native libraries use the same CRT as the Rust binary: the DLL
        // CRT by default, the static CRT with `crt-static`. Without this, Jolt
        // picks the static CRT while joltc keeps the DLL one.
        config.define("USE_STATIC_MSVC_RUNTIME_LIBRARY", "OFF");
        config.define("CMAKE_MSVC_RUNTIME_LIBRARY", cfg.crt);
    }

    if cfg.target_os == "android" {
        let ndk = AndroidNdk::locate(&cfg.target_arch, |name| env::var(name).ok())?;
        for (name, value) in ndk.options() {
            config.define(name, value);
        }
        config.generator(ANDROID_GENERATOR);
    }

    let prefix = config.build();
    write_manifest(&prefix, cfg)?;
    Ok(prefix)
}

/// Writes the manifest into the prefix, only when its content changes.
fn write_manifest(prefix: &Path, cfg: &NativeConfig) -> anyhow::Result<()> {
    let path = prefix.join(MANIFEST_FILE);
    let text = manifest_text(cfg);
    if fs::read_to_string(&path).ok().as_deref() != Some(text.as_str()) {
        fs::write(&path, text).with_context(|| format!("cannot write {}", path.display()))?;
    }
    Ok(())
}

/// Validates a prebuilt prefix from `JOLTC_LIB_DIR` and returns it. Never runs CMake.
fn use_prebuilt(dir: PathBuf, cfg: &NativeConfig) -> anyhow::Result<PathBuf> {
    let header = dir.join("include").join("joltc.h");
    let ext_header = dir.join("include").join("joltc_ext.h");
    let manifest = dir.join(MANIFEST_FILE);
    let archives = [
        dir.join("lib").join(cfg.archive_name(cfg.joltc_lib())),
        dir.join("lib").join(cfg.archive_name("Jolt")),
    ];

    let inputs: Vec<&PathBuf> = [&header, &ext_header, &manifest]
        .into_iter()
        .chain(&archives)
        .collect();
    // A configuration mismatch explains a missing archive (for example
    // joltc_double without double precision), so it is reported first.
    if manifest.is_file() {
        check_manifest(&manifest, cfg)?;
    }
    let missing: Vec<String> = inputs
        .iter()
        .filter(|path| !path.is_file())
        .map(|path| format!("  {}", path.display()))
        .collect();
    if !missing.is_empty() {
        bail!(
            "JOLTC_LIB_DIR={} is not a usable oxijolt-sys prefix; missing:\n{}",
            dir.display(),
            missing.join("\n")
        );
    }

    check_header(&header)?;
    check_ext_header(&ext_header)?;
    // Only a usable prefix is watched: a refused archive prefix is deleted before the source
    // build, and a missing watched file would rerun the build script on every build.
    for input in &inputs {
        println!("cargo:rerun-if-changed={}", input.display());
    }
    Ok(dir)
}

/// Fails unless the prebuilt manifest equals the one this build would write.
fn check_manifest(path: &Path, cfg: &NativeConfig) -> anyhow::Result<()> {
    let text =
        fs::read_to_string(path).with_context(|| format!("cannot read {}", path.display()))?;
    let mut found = BTreeMap::new();
    for line in text.lines().filter(|line| !line.trim().is_empty()) {
        let (key, value) = line
            .split_once('=')
            .with_context(|| format!("{}: malformed line {line:?}", path.display()))?;
        let key = key.trim();
        if found
            .insert(key.to_owned(), value.trim().to_owned())
            .is_some()
        {
            bail!("{}: duplicate key `{key}`", path.display());
        }
    }

    let expected = manifest_entries(cfg);
    let mut differences = Vec::new();
    for (key, value) in &expected {
        match found.remove(*key) {
            Some(prebuilt) if prebuilt == *value => {}
            Some(prebuilt) => {
                differences.push(format!("  {key}: prebuilt={prebuilt} expected={value}"))
            }
            None => differences.push(format!("  {key}: prebuilt=<missing> expected={value}")),
        }
    }
    for (key, prebuilt) in found {
        differences.push(format!(
            "  {key}: prebuilt={prebuilt} expected=<unknown key>"
        ));
    }

    if !differences.is_empty() {
        bail!(
            "the prebuilt joltc in {} was built for a different configuration:\n{}\n\
             Linking a mismatched native library is undefined behaviour at the FFI boundary. \
             Rebuild the prefix for this configuration or unset JOLTC_LIB_DIR.",
            path.display(),
            differences.join("\n")
        );
    }
    Ok(())
}

/// Fails if the prebuilt header differs from the vendored one, when the submodule is present.
/// Line endings are not compared.
fn check_header(prebuilt: &Path) -> anyhow::Result<()> {
    let vendored = manifest_dir()?
        .join("vendor")
        .join("joltc")
        .join("include")
        .join("joltc.h");
    if !vendored.is_file() {
        println!(
            "cargo:warning=vendor/joltc is not checked out; the prebuilt joltc.h could not be cross-checked"
        );
        return Ok(());
    }
    println!("cargo:rerun-if-changed={}", vendored.display());
    let read =
        |path: &Path| fs::read(path).with_context(|| format!("cannot read {}", path.display()));
    if targets::normalize_crlf(&read(prebuilt)?) != targets::normalize_crlf(&read(&vendored)?) {
        bail!(
            "{} differs from {}: the prebuilt prefix is from another joltc version. \
             Rebuild it or unset JOLTC_LIB_DIR.",
            prebuilt.display(),
            vendored.display()
        );
    }
    Ok(())
}

/// Fails if the prebuilt `joltc_ext.h` differs from the one in `native/joltc_ext/`, which
/// always ships with the crate. Line endings are not compared.
fn check_ext_header(prebuilt: &Path) -> anyhow::Result<()> {
    let ours = manifest_dir()?
        .join("native")
        .join("joltc_ext")
        .join("joltc_ext.h");
    println!("cargo:rerun-if-changed={}", ours.display());
    let read =
        |path: &Path| fs::read(path).with_context(|| format!("cannot read {}", path.display()));
    if targets::normalize_crlf(&read(prebuilt)?) != targets::normalize_crlf(&read(&ours)?) {
        bail!(
            "{} differs from {}: the prebuilt prefix is from another revision of the joltc \
             additions. Rebuild it or unset JOLTC_LIB_DIR.",
            prebuilt.display(),
            ours.display()
        );
    }
    Ok(())
}

/// Links joltc and Jolt statically and exports the prefix to dependants.
fn link(prefix: &Path, cfg: &NativeConfig) {
    println!(
        "cargo:rustc-link-search=native={}",
        prefix.join("lib").display()
    );
    // joltc depends on Jolt, so it comes first for single-pass linkers.
    println!("cargo:rustc-link-lib=static={}", cfg.joltc_lib());
    println!("cargo:rustc-link-lib=static=Jolt");

    match cfg.target_os.as_str() {
        "macos" | "ios" => println!("cargo:rustc-link-lib=dylib=c++"),
        "linux" => println!("cargo:rustc-link-lib=dylib=stdc++"),
        _ => {}
    }

    println!("cargo:include={}", prefix.join("include").display());
    println!("cargo:root={}", prefix.display());
}

/// The bindings `src/generated.rs` includes: generated into `OUT_DIR` with libclang.
#[cfg(feature = "bindgen")]
fn select_bindings(prefix: &Path, cfg: &NativeConfig) -> anyhow::Result<PathBuf> {
    generate_bindings(&prefix.join("include").join("joltc_ext.h"), cfg)
}

/// The bindings `src/generated.rs` includes: the committed file of the target's family and
/// configuration, after checking that it was generated from the linked headers and the
/// current generator sources.
#[cfg(not(feature = "bindgen"))]
fn select_bindings(prefix: &Path, cfg: &NativeConfig) -> anyhow::Result<PathBuf> {
    let crate_dir = manifest_dir()?;
    let read =
        |path: &Path| fs::read(path).with_context(|| format!("cannot read {}", path.display()));

    let include = prefix.join("include");
    let policy = targets::POLICY_SOURCES
        .iter()
        .map(|source| {
            // Without the `bindgen` feature rustc never reads `bindgen_options.rs`, so
            // cargo would not rerun this script when it changes.
            let path = crate_dir.join(source);
            println!("cargo:rerun-if-changed={}", path.display());
            Ok((*source, read(&path)?))
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    let policy: Vec<(&str, &[u8])> = policy
        .iter()
        .map(|(source, bytes)| (*source, bytes.as_slice()))
        .collect();
    let expected = targets::input_fingerprints(
        &read(&include.join("joltc.h"))?,
        &read(&include.join("joltc_ext.h"))?,
        &policy,
    );

    let inputs = crate_dir.join(targets::INPUTS_FILE);
    println!("cargo:rerun-if-changed={}", inputs.display());
    let recorded = String::from_utf8_lossy(&read(&inputs)?).into_owned();
    let stale = targets::differing_keys(&expected, &recorded);
    if !stale.is_empty() {
        bail!(
            "the committed bindings were generated from other inputs ({}); regenerate them with `cargo xtask bindings` or enable the `bindgen` feature",
            stale.join(", ")
        );
    }

    let bindings = crate_dir.join(targets::bindings_file(
        cfg.family.name,
        cfg.double_precision,
        cfg.debug_renderer,
    ));
    if !bindings.is_file() {
        bail!("{} is missing", bindings.display());
    }
    Ok(bindings)
}

/// Generates `OUT_DIR/bindings.rs` from `joltc_ext.h`, which includes `joltc.h`, and returns
/// its path.
#[cfg(feature = "bindgen")]
fn generate_bindings(header: &Path, cfg: &NativeConfig) -> anyhow::Result<PathBuf> {
    let include_dir = header
        .parent()
        .context("joltc_ext.h has no parent directory")?;
    // The explicit rerun rules cover the header's sources; the header the
    // CMake path binds lives in OUT_DIR, whose timestamps we do not control.
    let builder = bindgen_options::builder(
        header,
        include_dir,
        &cfg.target,
        cfg.double_precision,
        cfg.debug_renderer,
    )
    .parse_callbacks(Box::new(
        bindgen::CargoCallbacks::new().rerun_on_header_files(false),
    ));

    let bindings = builder
        .generate()
        .context("failed to generate joltc bindings")?;
    let out_path =
        PathBuf::from(env::var_os("OUT_DIR").context("OUT_DIR is not set")?).join("bindings.rs");
    bindings
        .write_to_file(&out_path)
        .with_context(|| format!("cannot write {}", out_path.display()))?;
    Ok(out_path)
}
