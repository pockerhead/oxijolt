//! The release archive list packaged with the crate (`prebuilt.txt`) and every decision about
//! using one of its archives instead of building Jolt from source.
//!
//! Shared by `build.rs` and the `xtask` list writer through `#[path]`. It uses only `std` and
//! reads the environment and the file system only through its arguments, so each decision is
//! a pure function with unit tests in `xtask`.

use std::fmt;
use std::path::Path;

/// The archive list, relative to the crate.
pub const LIST_FILE: &str = "prebuilt.txt";

/// The inputs of the native build, relative to the crate: every file CMake reads to build the
/// archives, and the build script code that passes its options. Directories count recursively.
///
/// A release records a fingerprint of these files, and a build uses an archive only while its
/// own files give the same fingerprint. A new input of the CMake build is added here; the
/// CMake path prints its rerun rules from this list too.
pub const SOURCE_INPUTS: &[&str] = &[
    "native",
    "build.rs",
    "build/cmake_options.rs",
    "build/wasi_sdk.rs",
    "vendor/joltc/CMakeLists.txt",
    "vendor/joltc/include",
    "vendor/joltc/src",
    "vendor/JoltPhysics/Build/CMakeLists.txt",
    "vendor/JoltPhysics/Jolt",
];

/// How the build script treats the release archives, from `JOLTC_PREBUILT`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Use a matching archive, otherwise build from source with a warning.
    Auto,
    /// Never look for an archive.
    Off,
    /// Use a matching archive or fail.
    Require,
}

impl Mode {
    /// The mode of a `JOLTC_PREBUILT` value: unset, empty or `auto`; `off`, `0` or `false`;
    /// `require`. Letter case does not matter.
    pub fn parse(value: Option<&str>) -> Result<Mode, String> {
        let value = value.unwrap_or("");
        match value.to_ascii_lowercase().as_str() {
            "" | "auto" => Ok(Mode::Auto),
            "off" | "0" | "false" => Ok(Mode::Off),
            "require" => Ok(Mode::Require),
            _ => Err(format!(
                "JOLTC_PREBUILT={value:?} is not one of auto, off, require"
            )),
        }
    }
}

/// Why no release archive is used for this build.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Unavailable {
    Offline,
    Vendored,
    NixBuild,
    FeatureOff,
    NoArchives,
    List(String),
    VersionDiffers { list: String, crate_: String },
    SourcesDiffer,
    SourcesMissing(String),
    NoArchive,
    CrtStatic,
    LinkerOverride,
    MsvcUnknown,
    MsvcTooOld { have: Version2, need: Version2 },
    CrossBuild,
    GlibcUnknown,
    GlibcTooOld { have: Version2, need: Version2 },
    MacosUnknown,
    MacosTooOld { have: Version2, need: Version2 },
    NoCurl,
    Download(String),
    TooLarge,
    Checksum,
    Unpack(String),
    Refused(String),
}

impl fmt::Display for Unavailable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Unavailable::Offline => write!(f, "offline build (CARGO_NET_OFFLINE)"),
            Unavailable::Vendored => write!(f, "vendored sources (.cargo-checksum.json)"),
            Unavailable::NixBuild => write!(f, "inside a Nix build (NIX_BUILD_TOP)"),
            Unavailable::FeatureOff => write!(f, "the prebuilt feature is off"),
            Unavailable::NoArchives => write!(f, "the archive list has no archives"),
            Unavailable::List(error) => write!(f, "the archive list is malformed: {error}"),
            Unavailable::VersionDiffers { list, crate_ } => write!(
                f,
                "the archive list is for version {list}, the crate is {crate_}"
            ),
            Unavailable::SourcesDiffer => {
                write!(f, "the native sources differ from the released ones")
            }
            Unavailable::SourcesMissing(path) => {
                write!(f, "native sources missing or not regular files: {path}")
            }
            Unavailable::NoArchive => write!(f, "no archive for this target, CRT and features"),
            Unavailable::CrtStatic => write!(f, "crt-static is enabled; the archives use /MD"),
            Unavailable::LinkerOverride => write!(f, "a linker or sysroot override is set"),
            Unavailable::MsvcUnknown => write!(f, "the MSVC toolset version is unknown"),
            Unavailable::MsvcTooOld { have, need } => {
                write!(f, "MSVC toolset {have} is older than the archive's {need}")
            }
            Unavailable::CrossBuild => write!(f, "a cross build (host differs from target)"),
            Unavailable::GlibcUnknown => write!(f, "the host glibc version is unknown"),
            Unavailable::GlibcTooOld { have, need } => {
                write!(f, "glibc {have} is older than the archive's {need}")
            }
            Unavailable::MacosUnknown => write!(f, "the macOS deployment target is unknown"),
            Unavailable::MacosTooOld { have, need } => write!(
                f,
                "macOS deployment target {have} is older than the archive's {need}"
            ),
            Unavailable::NoCurl => write!(f, "curl was not found"),
            Unavailable::Download(error) => write!(f, "download failed: {error}"),
            Unavailable::TooLarge => write!(f, "download failed: the archive is too large"),
            Unavailable::Checksum => {
                write!(f, "checksum mismatch: the archive differs from the list")
            }
            Unavailable::Unpack(error) => write!(f, "cannot unpack the archive: {error}"),
            Unavailable::Refused(error) => write!(f, "the archive was refused: {error}"),
        }
    }
}

/// A reason of the build environment not to download anything, before the list is read:
/// `CARGO_NET_OFFLINE` set to `true` or `1`, vendored sources (Cargo's `.cargo-checksum.json`
/// in the crate directory), or a Nix build (`NIX_BUILD_TOP`).
///
/// Cargo does not tell build scripts about `--offline`, so only an exported
/// `CARGO_NET_OFFLINE` is seen.
pub fn skip_reason(env: impl Fn(&str) -> Option<String>, crate_dir: &Path) -> Option<Unavailable> {
    let offline = env("CARGO_NET_OFFLINE")
        .is_some_and(|value| value.eq_ignore_ascii_case("true") || value == "1");
    if offline {
        Some(Unavailable::Offline)
    } else if crate_dir.join(".cargo-checksum.json").exists() {
        Some(Unavailable::Vendored)
    } else if env("NIX_BUILD_TOP").is_some() {
        Some(Unavailable::NixBuild)
    } else {
        None
    }
}

/// A `major.minor` version: an MSVC toolset (`14.44`), a glibc release (`2.35`) or a macOS
/// version (`11.0`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Version2 {
    pub major: u32,
    pub minor: u32,
}

impl Version2 {
    /// The first two dot-separated numeric components; later components may be anything.
    pub fn parse(text: &str) -> Option<Version2> {
        let mut parts = text.split('.');
        let mut number = || -> Option<u32> {
            let part = parts.next()?;
            if part.is_empty() || !part.bytes().all(|b| b.is_ascii_digit()) {
                return None;
            }
            part.parse().ok()
        };
        Some(Version2 {
            major: number()?,
            minor: number()?,
        })
    }
}

impl fmt::Display for Version2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}", self.major, self.minor)
    }
}

/// The MSVC toolset of a compiler version: `cl` 19.x belongs to toolset 14.x. `None` for any
/// other major version.
pub fn toolset_from_compiler(compiler: Version2) -> Option<Version2> {
    (compiler.major == 19).then_some(Version2 {
        major: 14,
        minor: compiler.minor,
    })
}

/// Everything that selects one archive, in the native manifest's spelling.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct ArchiveConfig {
    pub target: String,
    /// MSVC runtime library in CMake's spelling, or `none` off MSVC.
    pub crt: String,
    pub double_precision: bool,
    pub cross_platform_deterministic: bool,
    pub debug_renderer: bool,
    pub asserts: bool,
}

/// One archive of the list.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Archive {
    /// File stem: the archive is `<name>.tar.gz` and its top directory is `<name>/`.
    pub name: String,
    pub sha256: [u8; 32],
    pub config: ArchiveConfig,
    /// Oldest MSVC toolset that links it; present for every MSVC archive.
    pub msvc: Option<Version2>,
    /// Oldest glibc it links against; present for every `-linux-gnu` archive.
    pub glibc: Option<Version2>,
    /// Oldest macOS it runs on (its deployment target); present for every `-apple-darwin`
    /// archive.
    pub macos: Option<Version2>,
}

/// The parsed archive list. The header fields are present whenever there are archives.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ArchiveList {
    /// Crate version of the release.
    pub version: Option<String>,
    /// Base URL of the release assets.
    pub url: Option<String>,
    /// Fingerprint of [`SOURCE_INPUTS`] at the release.
    pub sources: Option<[u8; 32]>,
    pub archives: Vec<Archive>,
}

/// `version` without its `+build` metadata; prerelease identifiers stay.
pub fn release_version(version: &str) -> &str {
    version.split('+').next().unwrap_or(version)
}

/// The archive of `list` for `config`. The list must be for `crate_version`; build metadata is
/// ignored on both sides.
pub fn select<'a>(
    list: &'a ArchiveList,
    crate_version: &str,
    config: &ArchiveConfig,
) -> Result<&'a Archive, Unavailable> {
    if list.archives.is_empty() {
        return Err(Unavailable::NoArchives);
    }
    let listed = list.version.as_deref().unwrap_or_default();
    if release_version(listed) != release_version(crate_version) {
        return Err(Unavailable::VersionDiffers {
            list: listed.to_owned(),
            crate_: crate_version.to_owned(),
        });
    }
    list.archives
        .iter()
        .find(|archive| archive.config == *config)
        .ok_or(Unavailable::NoArchive)
}

/// The text of [`LIST_FILE`]: LF line endings, archives sorted by name. [`parse`] reads it
/// back unchanged.
pub fn write(list: &ArchiveList) -> String {
    let mut text = String::from(
        "# Release archives of oxijolt-sys. The Release workflow writes this file with\n\
         # `cargo xtask prebuilt-list`; a build takes an archive only for the listed version.\n\
         format=1\n",
    );
    if let Some(version) = &list.version {
        text += &format!("version={version}\n");
    }
    if let Some(url) = &list.url {
        text += &format!("url={url}\n");
    }
    if let Some(sources) = &list.sources {
        text += &format!("sources={}\n", hex(sources));
    }
    let mut archives: Vec<&Archive> = list.archives.iter().collect();
    archives.sort_by(|a, b| a.name.cmp(&b.name));
    for archive in archives {
        let config = &archive.config;
        text += &format!(
            "archive={} sha256={} target={} crt={} double_precision={} \
             cross_platform_deterministic={} debug_renderer={} asserts={}",
            archive.name,
            hex(&archive.sha256),
            config.target,
            config.crt,
            on_off(config.double_precision),
            on_off(config.cross_platform_deterministic),
            on_off(config.debug_renderer),
            on_off(config.asserts),
        );
        if let Some(msvc) = archive.msvc {
            text += &format!(" msvc={msvc}");
        }
        if let Some(glibc) = archive.glibc {
            text += &format!(" glibc={glibc}");
        }
        if let Some(macos) = archive.macos {
            text += &format!(" macos={macos}");
        }
        text.push('\n');
    }
    text
}

/// Reads [`LIST_FILE`]. `#` lines and blank lines are skipped; the first other line is
/// `format=1`. Zero archives is a valid list; archives need the `version`, `url` and `sources`
/// header lines.
pub fn parse(text: &str) -> Result<ArchiveList, String> {
    let mut list = ArchiveList::default();
    let mut format_seen = false;
    for (index, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let at = |error: String| format!("line {}: {error}", index + 1);
        let (key, value) = line
            .split_once('=')
            .ok_or_else(|| at(format!("expected key=value in {line:?}")))?;
        if !format_seen {
            if (key, value) != ("format", "1") {
                return Err(at(format!("expected format=1, found {line:?}")));
            }
            format_seen = true;
            continue;
        }
        match key {
            "version" => set_once(&mut list.version, value.to_owned(), key).map_err(at)?,
            "url" => set_once(&mut list.url, value.to_owned(), key).map_err(at)?,
            "sources" => {
                let sources = parse_hex32(value).map_err(at)?;
                set_once(&mut list.sources, sources, key).map_err(at)?
            }
            "archive" => {
                let archive = parse_archive(line).map_err(at)?;
                check_unique(&list.archives, &archive).map_err(at)?;
                list.archives.push(archive);
            }
            _ => return Err(at(format!("unknown key `{key}`"))),
        }
    }
    if !format_seen {
        return Err("missing format=1".to_owned());
    }
    if !list.archives.is_empty() {
        for (present, key) in [
            (list.version.is_some(), "version"),
            (list.url.is_some(), "url"),
            (list.sources.is_some(), "sources"),
        ] {
            if !present {
                return Err(format!("archives without a `{key}` line"));
            }
        }
    }
    Ok(list)
}

fn set_once<T>(slot: &mut Option<T>, value: T, key: &str) -> Result<(), String> {
    if slot.replace(value).is_some() {
        return Err(format!("duplicate `{key}`"));
    }
    Ok(())
}

fn check_unique(archives: &[Archive], archive: &Archive) -> Result<(), String> {
    if archives.iter().any(|a| a.name == archive.name) {
        return Err(format!("duplicate archive {}", archive.name));
    }
    if archives.iter().any(|a| a.config == archive.config) {
        return Err(format!(
            "archive {} repeats the configuration of another",
            archive.name
        ));
    }
    Ok(())
}

/// One `archive=<name> key=value ...` line.
fn parse_archive(line: &str) -> Result<Archive, String> {
    let mut fields = line.split_whitespace();
    let name = fields
        .next()
        .and_then(|first| first.strip_prefix("archive="))
        .ok_or("expected archive=<name> first")?;
    if !is_valid_name(name) {
        return Err(format!("invalid archive name {name:?}"));
    }

    let mut values: Vec<(&str, &str)> = Vec::new();
    for field in fields {
        let (key, value) = field
            .split_once('=')
            .ok_or_else(|| format!("expected key=value in {field:?}"))?;
        if !ARCHIVE_KEYS.contains(&key) {
            return Err(format!("unknown archive key `{key}`"));
        }
        if values.iter().any(|(k, _)| *k == key) {
            return Err(format!("duplicate archive key `{key}`"));
        }
        values.push((key, value));
    }
    let optional = |key: &str| values.iter().find(|(k, _)| *k == key).map(|(_, v)| *v);
    let required = |key: &str| optional(key).ok_or_else(|| format!("{name}: missing `{key}`"));
    let flag = |key: &str| -> Result<bool, String> {
        match required(key)? {
            "ON" => Ok(true),
            "OFF" => Ok(false),
            other => Err(format!("{name}: `{key}` is {other:?}, not ON or OFF")),
        }
    };
    let version = |key: &str| -> Result<Option<Version2>, String> {
        optional(key)
            .map(|text| {
                Version2::parse(text).ok_or_else(|| format!("{name}: bad `{key}` {text:?}"))
            })
            .transpose()
    };

    let archive = Archive {
        name: name.to_owned(),
        sha256: parse_hex32(required("sha256")?)?,
        config: ArchiveConfig {
            target: required("target")?.to_owned(),
            crt: required("crt")?.to_owned(),
            double_precision: flag("double_precision")?,
            cross_platform_deterministic: flag("cross_platform_deterministic")?,
            debug_renderer: flag("debug_renderer")?,
            asserts: flag("asserts")?,
        },
        msvc: version("msvc")?,
        glibc: version("glibc")?,
        macos: version("macos")?,
    };
    if archive.config.crt != "none" && archive.msvc.is_none() {
        return Err(format!("{name}: an MSVC archive needs `msvc`"));
    }
    if archive.config.target.ends_with("-linux-gnu") && archive.glibc.is_none() {
        return Err(format!("{name}: a -linux-gnu archive needs `glibc`"));
    }
    if archive.config.target.ends_with("-apple-darwin") && archive.macos.is_none() {
        return Err(format!("{name}: an -apple-darwin archive needs `macos`"));
    }
    Ok(archive)
}

const ARCHIVE_KEYS: &[&str] = &[
    "sha256",
    "target",
    "crt",
    "double_precision",
    "cross_platform_deterministic",
    "debug_renderer",
    "asserts",
    "msvc",
    "glibc",
    "macos",
];

/// Whether `name` is a non-empty archive stem of `[A-Za-z0-9._+-]`.
pub fn is_valid_name(name: &str) -> bool {
    !name.is_empty()
        && name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"._+-".contains(&b))
}

fn on_off(value: bool) -> &'static str {
    if value {
        "ON"
    } else {
        "OFF"
    }
}

/// Lowercase hex of a sha256.
pub fn hex(bytes: &[u8; 32]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// A sha256 from 64 lowercase hex digits.
pub fn parse_hex32(text: &str) -> Result<[u8; 32], String> {
    let digits = text.as_bytes();
    let valid = digits.len() == 64
        && digits
            .iter()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(b));
    if !valid {
        return Err(format!("expected 64 lowercase hex digits, found {text:?}"));
    }
    let mut bytes = [0; 32];
    for (byte, pair) in bytes.iter_mut().zip(digits.chunks(2)) {
        let pair = std::str::from_utf8(pair).map_err(|e| e.to_string())?;
        *byte = u8::from_str_radix(pair, 16).map_err(|e| e.to_string())?;
    }
    Ok(bytes)
}

/// The arguments of `CARGO_ENCODED_RUSTFLAGS`, which separates them with `0x1f`.
fn encoded_flags(encoded: &str) -> impl Iterator<Item = &str> {
    encoded.split('\x1f').filter(|flag| !flag.is_empty())
}

/// Whether the encoded rustflags choose a linker: `-Clinker=`, `-C linker=` or
/// `--codegen linker=`.
pub fn rustflags_set_linker(encoded: &str) -> bool {
    let mut flags = encoded_flags(encoded);
    while let Some(flag) = flags.next() {
        let codegen = match flag {
            "-C" | "--codegen" => flags.next().unwrap_or_default(),
            _ => flag
                .strip_prefix("-C")
                .or_else(|| flag.strip_prefix("--codegen="))
                .unwrap_or_default(),
        };
        if codegen.starts_with("linker=") {
            return true;
        }
    }
    false
}

/// Whether the encoded rustflags pass a sysroot, to rustc or to the linker.
pub fn rustflags_set_sysroot(encoded: &str) -> bool {
    encoded_flags(encoded).any(|flag| flag.contains("--sysroot"))
}

/// Whether a GNU target's link is overridden: `RUSTC_LINKER`, a linker in the rustflags, or a
/// sysroot. `env` reads the build script's environment.
pub fn gnu_linker_overridden(env: impl Fn(&str) -> Option<String>) -> bool {
    let flags = env("CARGO_ENCODED_RUSTFLAGS").unwrap_or_default();
    env("RUSTC_LINKER").is_some() || rustflags_set_linker(&flags) || rustflags_set_sysroot(&flags)
}

/// The MSVC toolset of a `link.exe` path: the component after `VC\Tools\MSVC\`.
pub fn msvc_toolset_of_linker(path: &str) -> Option<Version2> {
    let path = path.replace('\\', "/");
    let components: Vec<&str> = path.split('/').collect();
    components
        .windows(4)
        .find(|w| {
            w[0].eq_ignore_ascii_case("VC")
                && w[1].eq_ignore_ascii_case("Tools")
                && w[2].eq_ignore_ascii_case("MSVC")
        })
        .and_then(|w| Version2::parse(w[3]))
}

/// The toolset of the MSVC linker this build uses. A linker in the rustflags, or a
/// `RUSTC_LINKER` that is not `link.exe`, is an override. Otherwise the linker is
/// `RUSTC_LINKER`, or what `find_link` discovers (the lookup rustc does). `Ok(None)` when its
/// path names no toolset.
pub fn effective_msvc_linker(
    env: impl Fn(&str) -> Option<String>,
    find_link: impl FnOnce() -> Option<String>,
) -> Result<Option<Version2>, Unavailable> {
    if rustflags_set_linker(&env("CARGO_ENCODED_RUSTFLAGS").unwrap_or_default()) {
        return Err(Unavailable::LinkerOverride);
    }
    let path = match env("RUSTC_LINKER") {
        Some(linker) => {
            let file = linker.rsplit(['/', '\\']).next().unwrap_or_default();
            if !file.eq_ignore_ascii_case("link.exe") {
                return Err(Unavailable::LinkerOverride);
            }
            linker
        }
        None => match find_link() {
            Some(path) => path,
            None => return Ok(None),
        },
    };
    Ok(msvc_toolset_of_linker(&path))
}

/// What links the archive on this machine, as `build.rs` found it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Linker {
    /// The MSVC toolset from [`effective_msvc_linker`].
    Msvc(Result<Option<Version2>, Unavailable>),
    /// A GNU target: whether the host differs from the target, whether the link is overridden
    /// ([`gnu_linker_overridden`]) and the host's glibc.
    Gnu {
        cross: bool,
        overridden: bool,
        glibc: Option<Version2>,
    },
    /// An Apple target: the deployment target rustc links for
    /// ([`deployment_target_from_rustc`]).
    Apple { deployment_target: Option<Version2> },
    /// `wasm32-unknown-unknown`: Rust's own rust-lld links the archive, whose version the build
    /// script checks for every wasm build.
    Wasm,
}

/// Whether `linker` can link `archive`: the toolset, glibc or macOS deployment target is at
/// least the archive's. A wasm archive bundles its runtime and has no such baseline.
pub fn check_linker(archive: &Archive, linker: &Linker) -> Result<(), Unavailable> {
    match linker {
        Linker::Msvc(found) => {
            let have = found.clone()?.ok_or(Unavailable::MsvcUnknown)?;
            let need = archive.msvc.ok_or(Unavailable::MsvcUnknown)?;
            if have < need {
                return Err(Unavailable::MsvcTooOld { have, need });
            }
        }
        Linker::Gnu {
            cross,
            overridden,
            glibc,
        } => {
            if *cross {
                return Err(Unavailable::CrossBuild);
            }
            if *overridden {
                return Err(Unavailable::LinkerOverride);
            }
            let have = glibc.ok_or(Unavailable::GlibcUnknown)?;
            let need = archive.glibc.ok_or(Unavailable::GlibcUnknown)?;
            if have < need {
                return Err(Unavailable::GlibcTooOld { have, need });
            }
        }
        Linker::Apple { deployment_target } => {
            let have = deployment_target.ok_or(Unavailable::MacosUnknown)?;
            let need = archive.macos.ok_or(Unavailable::MacosUnknown)?;
            if have < need {
                return Err(Unavailable::MacosTooOld { have, need });
            }
        }
        Linker::Wasm => {}
    }
    Ok(())
}

/// The deployment target in the output of `rustc --print deployment-target`: one line
/// `MACOSX_DEPLOYMENT_TARGET=<version>` (or `deployment_target=<version>` from older rustc).
/// A patch component is dropped, which can only make the target look older. `None` for any
/// other output.
pub fn deployment_target_from_rustc(output: &str) -> Option<Version2> {
    let mut lines = output
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty());
    let line = lines.next()?;
    if lines.next().is_some() {
        return None;
    }
    let (key, value) = line.split_once('=')?;
    if key != "MACOSX_DEPLOYMENT_TARGET" && key != "deployment_target" {
        return None;
    }
    Version2::parse(value)
}
