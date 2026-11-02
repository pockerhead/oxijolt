//! Fixtures for the archive list tests: temporary directories, a fake `oxijolt-sys` crate
//! directory with every native source input, and in-memory release archives.

use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};

use flate2::write::GzEncoder;
use flate2::Compression;

use crate::prebuilt::{ArchiveConfig, SOURCE_INPUTS};
use crate::prebuilt_archive::PrefixFiles;

/// A directory under the system temp dir, removed on drop.
pub struct TempDir(PathBuf);

impl TempDir {
    /// A new empty directory whose name contains `tag`, the process id and a counter.
    pub fn new(tag: &str) -> TempDir {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let n = NEXT.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("xtask-{tag}-{}-{n}", std::process::id()));
        let _ = fs::remove_dir_all(&path);
        fs::create_dir_all(&path).expect("temp dir");
        TempDir(path)
    }

    pub fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

/// Writes `bytes` to `root/relative`, creating its directories.
pub fn write_file(root: &Path, relative: &str, bytes: &[u8]) {
    let path = root.join(relative);
    fs::create_dir_all(path.parent().expect("parent")).expect("create dirs");
    fs::write(&path, bytes).expect("write");
}

/// Fills `root` with a crate directory at version `1.2.0+jolt-5.6.0` that has every
/// [`SOURCE_INPUTS`] entry (one or two files per directory), with CRLF line endings when `crlf`.
pub fn fake_crate(root: &Path, crlf: bool) {
    let eol = if crlf { "\r\n" } else { "\n" };
    let text = |lines: &[&str]| lines.join(eol).into_bytes();
    write_file(
        root,
        "Cargo.toml",
        &text(&[
            "[package]",
            "name = \"oxijolt-sys\"",
            "version = \"1.2.0+jolt-5.6.0\"",
            "",
            "[dependencies]",
            "version = \"0.0.0\"",
        ]),
    );
    for input in SOURCE_INPUTS {
        let files: &[&str] = match *input {
            "native" => &["native/CMakeLists.txt", "native/joltc_ext/joltc_ext.cpp"],
            "vendor/joltc/include" => &["vendor/joltc/include/joltc.h"],
            "vendor/joltc/src" => &["vendor/joltc/src/joltc.cpp"],
            "vendor/JoltPhysics/Jolt" => &["vendor/JoltPhysics/Jolt/Jolt.cmake"],
            file => &[file][..],
        };
        for file in files {
            write_file(root, file, &text(&[file, "line two", ""]));
        }
    }
}

/// A `.tar.gz` of `entries`: regular files, or directories for paths ending in `/`.
pub fn tar_gz(entries: &[(&str, &[u8])]) -> Vec<u8> {
    let mut builder = tar::Builder::new(GzEncoder::new(Vec::new(), Compression::fast()));
    for (path, bytes) in entries {
        let mut header = tar::Header::new_gnu();
        if path.ends_with('/') {
            header.set_entry_type(tar::EntryType::Directory);
            header.set_mode(0o755);
            header.set_size(0);
        } else {
            header.set_entry_type(tar::EntryType::Regular);
            header.set_mode(0o644);
            header.set_size(bytes.len() as u64);
        }
        header.set_mtime(1_700_000_000);
        builder
            .append_data(&mut header, path, *bytes)
            .expect("append");
    }
    builder.into_inner().expect("tar").finish().expect("gzip")
}

/// The native manifest of a prefix built for `config`; on wasm it names the wasi-sdk.
pub fn manifest_for(config: &ArchiveConfig) -> String {
    let on_off = |value: bool| if value { "ON" } else { "OFF" };
    let wasi_sdk = if config.target == crate::wasi_sdk::WASM_TARGET {
        "wasi_sdk=34.0 wasi-libc:2e6fb9d8ee0c llvm:895aa2c896ad\n"
    } else {
        ""
    };
    let manifest = format!(
        "format=1\njoltc_ext=22\ntarget={}\ncrt={}\ndouble_precision={}\n\
         cross_platform_deterministic={}\nasserts={}\ndebug_renderer={}\njoltc_lib={}\n",
        config.target,
        config.crt,
        on_off(config.double_precision),
        on_off(config.cross_platform_deterministic),
        on_off(config.asserts),
        on_off(config.debug_renderer),
        if config.double_precision {
            "joltc_double"
        } else {
            "joltc"
        },
    );
    manifest + wasi_sdk
}

/// The provenance the Release workflow writes: source commit, compiler, CPU options and, on
/// Linux glibc, on macOS the deployment target.
pub fn provenance_for(config: &ArchiveConfig, commit: &str) -> String {
    let linux = config.target.ends_with("-linux-gnu");
    let macos = config.target.ends_with("-apple-darwin");
    let (compiler, cpu) = if linux {
        ("GNU 12.3.0", "USE_AVX2:BOOL=ON")
    } else if macos {
        (
            "AppleClang 17.0.0.17000013",
            "CMAKE_OSX_ARCHITECTURES:STRING=arm64",
        )
    } else {
        ("MSVC 19.44.35229.0", "USE_AVX2:BOOL=ON")
    };
    let mut text = format!(
        "source: owner/oxijolt@{commit}\nruntime image: test\ncompiler: {compiler} \n\
         cargo target: {}\nCPU options:\n{cpu}\n",
        config.target
    );
    if linux {
        text += "glibc: 2.35\n";
    }
    if macos {
        text += "macos: 11.0\n";
    }
    text
}

/// A release archive named `name` for `config`, built from `commit`, with the prefix layout
/// (libraries in the target's spelling, headers, manifest, provenance, licences; on wasm the
/// bundled runtime and its licences).
pub fn release_archive(name: &str, config: &ArchiveConfig, commit: &str) -> Vec<u8> {
    release_archive_with(
        name,
        &manifest_for(config),
        &provenance_for(config, commit),
        &[],
    )
}

/// A release archive with the given manifest and provenance texts and `extra` entries.
pub fn release_archive_with(
    name: &str,
    manifest: &str,
    provenance: &str,
    extra: &[(&str, &[u8])],
) -> Vec<u8> {
    let path = |relative: &str| format!("{name}/{relative}");
    let value = |key: &str| {
        manifest
            .lines()
            .find_map(|line| line.strip_prefix(key)?.strip_prefix('='))
            .unwrap_or_default()
    };
    let files = PrefixFiles::new(value("target"), value("joltc_lib"));
    let mut owned: Vec<(String, Vec<u8>)> =
        vec![(path(""), Vec::new()), (path("lib/"), Vec::new())];
    for (index, library) in files.libraries.iter().enumerate() {
        // joltc first, then Jolt: the bytes name the library, as the tests read them back.
        let bytes = match index {
            0 => b"joltc".to_vec(),
            1 => b"Jolt".to_vec(),
            _ => library.as_bytes().to_vec(),
        };
        owned.push((path(&format!("lib/{library}")), bytes));
    }
    owned.extend([
        (path("include/"), Vec::new()),
        (path("include/joltc.h"), b"// joltc.h\n".to_vec()),
        (path("include/joltc_ext.h"), b"// joltc_ext.h\n".to_vec()),
        (
            path("oxijolt-sys-manifest.txt"),
            manifest.as_bytes().to_vec(),
        ),
        (path("PROVENANCE.txt"), provenance.as_bytes().to_vec()),
        (
            path("LICENSE-MIT"),
            b"MIT
"
            .to_vec(),
        ),
        (
            path("LICENSE-APACHE"),
            b"Apache
"
            .to_vec(),
        ),
        (path("licenses/"), Vec::new()),
        (
            path("licenses/joltc-LICENSE"),
            b"joltc
"
            .to_vec(),
        ),
        (
            path("licenses/JoltPhysics-LICENSE"),
            b"Jolt
"
            .to_vec(),
        ),
    ]);
    for license in &files.runtime_licenses {
        owned.push((
            path(&format!("licenses/{license}")),
            license.as_bytes().to_vec(),
        ));
    }
    let mut entries: Vec<(&str, &[u8])> = owned
        .iter()
        .map(|(path, bytes)| (path.as_str(), bytes.as_slice()))
        .collect();
    entries.extend_from_slice(extra);
    tar_gz(&entries)
}

/// The configuration of `target` with the three archive features from the bits of `subset`.
pub fn config(target: &str, subset: u8) -> ArchiveConfig {
    let msvc = target.ends_with("-msvc");
    ArchiveConfig {
        target: target.to_owned(),
        crt: if msvc { "MultiThreadedDLL" } else { "none" }.to_owned(),
        double_precision: subset & 1 != 0,
        cross_platform_deterministic: subset & 2 != 0,
        debug_renderer: subset & 4 != 0,
        asserts: false,
    }
}
