//! `cargo xtask prebuilt-list`: the archive list of a release.
//!
//! `prebuilt-list --dist <dir> --url <base> --commit <sha>` reads every `*.tar.gz` in `dir`
//! (the Release workflow's archives of commit `sha`) and writes `crates/oxijolt-sys/prebuilt.txt`
//! (or `--out <file>`): the crate version, the asset URL `base`, the fingerprint of the native
//! sources of the checkout, and each archive's sha256, configuration and oldest toolchain or OS.
//! Without `--allow-partial` the archives must be the full release set. The `.sha256` files next
//! to the archives are ignored: every hash is taken from the archive bytes.
//!
//! `prebuilt-list --check <list> --dist <dir>` fails unless every archive of the list is in
//! `dir` with the listed sha256.

use std::fs;
use std::io::Read;
use std::path::{Path, PathBuf};

use anyhow::{bail, ensure, Context};
use flate2::read::GzDecoder;

use crate::prebuilt::{self, Archive, ArchiveConfig, ArchiveList, Version2};
use crate::prebuilt_archive::{self, PrefixFiles};
use crate::wasi_sdk::WASM_TARGET;

/// The manifest every native prefix carries (`MANIFEST_FILE` in build.rs).
const MANIFEST_FILE: &str = "oxijolt-sys-manifest.txt";
/// The build record the Release workflow adds to each archive.
const PROVENANCE_FILE: &str = "PROVENANCE.txt";

/// Targets of a release with their CRT; each comes in every subset of the three archive features.
const RELEASE_TARGETS: &[(&str, &str)] = &[
    ("x86_64-pc-windows-msvc", "MultiThreadedDLL"),
    ("x86_64-unknown-linux-gnu", "none"),
    ("aarch64-apple-darwin", "none"),
    (WASM_TARGET, "none"),
];

/// The oldest macOS Rust links `target` for; an archive must not need a newer one.
fn rust_macos_minimum(target: &str) -> Option<Version2> {
    match target {
        "aarch64-apple-darwin" => Some(Version2 {
            major: 11,
            minor: 0,
        }),
        _ => None,
    }
}

/// Runs the subcommand with its arguments.
pub fn run(args: &[String], usage: &str) -> anyhow::Result<()> {
    let mut dist = None;
    let mut url = None;
    let mut commit = None;
    let mut out = None;
    let mut check = None;
    let mut allow_partial = false;
    let mut args = args.iter();
    while let Some(arg) = args.next() {
        let mut value = || args.next().cloned().context(usage.to_owned());
        match arg.as_str() {
            "--dist" => dist = Some(PathBuf::from(value()?)),
            "--url" => url = Some(value()?),
            "--commit" => commit = Some(value()?),
            "--out" => out = Some(PathBuf::from(value()?)),
            "--check" => check = Some(PathBuf::from(value()?)),
            "--allow-partial" => allow_partial = true,
            _ => bail!(usage.to_owned()),
        }
    }
    let dist = dist.context(usage.to_owned())?;

    if let Some(list) = check {
        ensure!(
            url.is_none() && commit.is_none() && out.is_none(),
            usage.to_owned()
        );
        check_list(&list, &dist)?;
        println!(
            "{} matches the archives in {}",
            list.display(),
            dist.display()
        );
        return Ok(());
    }

    let (url, commit) = (
        url.context(usage.to_owned())?,
        commit.context(usage.to_owned())?,
    );
    let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("crates")
        .join("oxijolt-sys");
    let list = build_list(&dist, &url, &commit, &crate_dir, allow_partial)?;
    let out = out.unwrap_or_else(|| crate_dir.join(prebuilt::LIST_FILE));
    fs::write(&out, prebuilt::write(&list))
        .with_context(|| format!("cannot write {}", out.display()))?;
    println!(
        "wrote {} archives to {}",
        list.archives.len(),
        out.display()
    );
    Ok(())
}

/// The list of the archives in `dist`, built from `commit`, for the crate in `crate_dir`.
pub fn build_list(
    dist: &Path,
    url: &str,
    commit: &str,
    crate_dir: &Path,
    allow_partial: bool,
) -> anyhow::Result<ArchiveList> {
    ensure!(
        !url.is_empty() && !url.contains(char::is_whitespace),
        "invalid --url {url:?}"
    );
    let archives = archive_paths(dist)?
        .iter()
        .map(|path| read_archive(path, commit))
        .collect::<anyhow::Result<Vec<_>>>()?;
    if !allow_partial {
        check_inventory(&archives)?;
    }
    let sources = prebuilt_archive::sources_fingerprint(crate_dir)
        .map_err(|e| anyhow::anyhow!("{}: {e}", crate_dir.display()))?;
    let list = ArchiveList {
        version: Some(crate_version(crate_dir)?),
        url: Some(url.trim_end_matches('/').to_owned()),
        sources: Some(sources),
        archives,
    };
    // The parser refuses duplicate names and configurations.
    prebuilt::parse(&prebuilt::write(&list)).map_err(anyhow::Error::msg)
}

/// Every `*.tar.gz` in `dist`, sorted by file name.
fn archive_paths(dist: &Path) -> anyhow::Result<Vec<PathBuf>> {
    let mut paths = Vec::new();
    for entry in fs::read_dir(dist).with_context(|| format!("cannot read {}", dist.display()))? {
        let path = entry?.path();
        if path.to_string_lossy().ends_with(".tar.gz") {
            paths.push(path);
        }
    }
    paths.sort();
    Ok(paths)
}

/// The list entry of one archive of `commit`.
pub fn read_archive(path: &Path, commit: &str) -> anyhow::Result<Archive> {
    let file = path
        .file_name()
        .and_then(|name| name.to_str())
        .with_context(|| format!("{}: not a UTF-8 file name", path.display()))?;
    let name = file.strip_suffix(".tar.gz").unwrap_or(file);
    ensure!(
        prebuilt::is_valid_name(name),
        "{file}: invalid archive name"
    );
    let bytes = fs::read(path).with_context(|| format!("cannot read {}", path.display()))?;
    let (manifest, provenance) = metadata_files(&bytes, name).with_context(|| file.to_owned())?;

    let manifest = key_values(&manifest, '=').with_context(|| format!("{file}: manifest"))?;
    let provenance = key_values(&provenance, ':').with_context(|| format!("{file}: provenance"))?;
    let get = |entries: &[(String, String)], key: &str| {
        entries
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.clone())
            .with_context(|| format!("{file}: `{key}` missing"))
    };
    let flag = |key: &str| -> anyhow::Result<bool> {
        match get(&manifest, key)?.as_str() {
            "ON" => Ok(true),
            "OFF" => Ok(false),
            other => bail!("{file}: `{key}` is {other:?}"),
        }
    };

    let files = PrefixFiles::new(&get(&manifest, "target")?, &get(&manifest, "joltc_lib")?);
    let entries = prebuilt_archive::check_entries(&bytes, name, &files)
        .map_err(|e| anyhow::anyhow!("{file}: {e}"))?;
    let missing: Vec<String> = prebuilt_archive::allowed_entries(name, &files)
        .into_iter()
        .filter(|entry| !entry.directory && !entries.contains(entry))
        .map(|entry| entry.path)
        .collect();
    ensure!(missing.is_empty(), "{file}: missing {missing:?}");

    let source = get(&provenance, "source")?;
    ensure!(
        source.ends_with(&format!("@{commit}")),
        "{file}: built from {source}, not from commit {commit}"
    );
    let config = ArchiveConfig {
        target: get(&manifest, "target")?,
        crt: get(&manifest, "crt")?,
        double_precision: flag("double_precision")?,
        cross_platform_deterministic: flag("cross_platform_deterministic")?,
        debug_renderer: flag("debug_renderer")?,
        asserts: flag("asserts")?,
    };
    let msvc = if config.crt == "none" {
        None
    } else {
        let compiler = get(&provenance, "compiler")?;
        Some(msvc_toolset(&compiler).with_context(|| format!("{file}: compiler {compiler:?}"))?)
    };
    if config.target == WASM_TARGET {
        // The wasi-sdk whose runtime the archive bundles.
        ensure!(
            !get(&manifest, "wasi_sdk")?.is_empty(),
            "{file}: `wasi_sdk` is empty"
        );
    }
    let glibc = if config.target.ends_with("-linux-gnu") {
        let glibc = get(&provenance, "glibc")?;
        Some(Version2::parse(&glibc).with_context(|| format!("{file}: glibc {glibc:?}"))?)
    } else {
        None
    };
    let macos = if config.target.ends_with("-apple-darwin") {
        Some(macos_minimum(
            file,
            &config.target,
            &get(&provenance, "macos")?,
        )?)
    } else {
        None
    };
    Ok(Archive {
        name: name.to_owned(),
        sha256: prebuilt_archive::sha256(&bytes),
        config,
        msvc,
        glibc,
        macos,
    })
}

/// The provenance `macos:` value `raw` of the archive `file` for `target`: exactly
/// `major.minor`, and not newer than [`rust_macos_minimum`].
fn macos_minimum(file: &str, target: &str, raw: &str) -> anyhow::Result<Version2> {
    let macos = Version2::parse(raw)
        .filter(|macos| macos.to_string() == raw)
        .with_context(|| format!("{file}: macos {raw:?} is not major.minor"))?;
    let minimum = rust_macos_minimum(target)
        .with_context(|| format!("{file}: no macOS minimum known for {target}"))?;
    ensure!(
        macos <= minimum,
        "{file}: built for macOS {macos}, newer than Rust's minimum {minimum}"
    );
    Ok(macos)
}

/// The manifest and the provenance of a `.tar.gz` whose entries all lie in `<name>/`.
fn metadata_files(bytes: &[u8], name: &str) -> anyhow::Result<(String, String)> {
    let manifest_path = format!("{name}/{MANIFEST_FILE}");
    let provenance_path = format!("{name}/{PROVENANCE_FILE}");
    let (mut manifest, mut provenance) = (None, None);
    let mut archive = tar::Archive::new(GzDecoder::new(bytes));
    for entry in archive.entries().context("not a tar.gz archive")? {
        let mut entry = entry.context("not a tar.gz archive")?;
        let path = std::str::from_utf8(&entry.path_bytes())
            .context("a path is not UTF-8")?
            .to_owned();
        let inside = path
            .strip_prefix(name)
            .is_some_and(|rest| rest.is_empty() || rest.starts_with('/'));
        ensure!(inside, "{path} lies outside {name}/");
        let slot = if path == manifest_path {
            &mut manifest
        } else if path == provenance_path {
            &mut provenance
        } else {
            continue;
        };
        ensure!(slot.is_none(), "{path} appears twice");
        let mut text = String::new();
        entry
            .read_to_string(&mut text)
            .with_context(|| format!("cannot read {path}"))?;
        *slot = Some(text);
    }
    Ok((
        manifest.with_context(|| format!("{manifest_path} missing"))?,
        provenance.with_context(|| format!("{provenance_path} missing"))?,
    ))
}

/// The `key<separator>value` lines of `text`, trimmed. A repeated key is an error; lines
/// without the separator are skipped (the provenance has free-form lines).
fn key_values(text: &str, separator: char) -> anyhow::Result<Vec<(String, String)>> {
    let mut entries: Vec<(String, String)> = Vec::new();
    for line in text.lines() {
        let Some((key, value)) = line.split_once(separator) else {
            continue;
        };
        let key = key.trim().to_owned();
        ensure!(
            entries.iter().all(|(k, _)| *k != key),
            "duplicate key `{key}`"
        );
        entries.push((key, value.trim().to_owned()));
    }
    Ok(entries)
}

/// The MSVC toolset of a provenance `compiler:` value such as `MSVC 19.44.35229.0`.
fn msvc_toolset(compiler: &str) -> Option<Version2> {
    let mut words = compiler.split_whitespace();
    if words.next() != Some("MSVC") {
        return None;
    }
    prebuilt::toolset_from_compiler(Version2::parse(words.next()?)?)
}

/// Fails unless `archives` are exactly the release set: every subset of the three archive
/// features for each of [`RELEASE_TARGETS`], without asserts.
fn check_inventory(archives: &[Archive]) -> anyhow::Result<()> {
    let mut expected = Vec::new();
    for (target, crt) in RELEASE_TARGETS {
        for bits in 0..8 {
            expected.push(ArchiveConfig {
                target: (*target).to_owned(),
                crt: (*crt).to_owned(),
                double_precision: bits & 1 != 0,
                cross_platform_deterministic: bits & 2 != 0,
                debug_renderer: bits & 4 != 0,
                asserts: false,
            });
        }
    }
    let mut found: Vec<ArchiveConfig> = archives.iter().map(|a| a.config.clone()).collect();
    expected.sort();
    found.sort();
    if found != expected {
        let missing = expected.iter().filter(|c| !found.contains(c)).count();
        let unexpected: Vec<&str> = archives
            .iter()
            .filter(|a| !expected.contains(&a.config))
            .map(|a| a.name.as_str())
            .collect();
        bail!(
            "expected the {} release archives: {missing} missing, unexpected {unexpected:?} \
             (use --allow-partial for a test list)",
            expected.len()
        );
    }
    Ok(())
}

/// The `[package]` version of the crate in `crate_dir` without build metadata.
fn crate_version(crate_dir: &Path) -> anyhow::Result<String> {
    let path = crate_dir.join("Cargo.toml");
    let text =
        fs::read_to_string(&path).with_context(|| format!("cannot read {}", path.display()))?;
    let mut in_package = false;
    for line in text.lines().map(str::trim) {
        if line.starts_with('[') {
            in_package = line == "[package]";
        } else if let Some(value) = in_package
            .then(|| line.strip_prefix("version"))
            .flatten()
            .and_then(|rest| rest.trim_start().strip_prefix('='))
        {
            let version = value.trim().trim_matches('"');
            return Ok(prebuilt::release_version(version).to_owned());
        }
    }
    bail!("{}: no [package] version", path.display())
}

/// Fails unless every archive of the list in `list_path` is in `dist` with its listed sha256.
pub fn check_list(list_path: &Path, dist: &Path) -> anyhow::Result<()> {
    let text = fs::read_to_string(list_path)
        .with_context(|| format!("cannot read {}", list_path.display()))?;
    let list =
        prebuilt::parse(&text).map_err(|e| anyhow::anyhow!("{}: {e}", list_path.display()))?;
    let mut problems = Vec::new();
    for archive in &list.archives {
        let path = dist.join(format!("{}.tar.gz", archive.name));
        match fs::read(&path) {
            Ok(bytes) if prebuilt_archive::sha256(&bytes) == archive.sha256 => {}
            Ok(_) => problems.push(format!("{}: sha256 differs", archive.name)),
            Err(_) => problems.push(format!("{}: missing in {}", archive.name, dist.display())),
        }
    }
    ensure!(problems.is_empty(), "{}", problems.join("\n"));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prebuilt_archive::sources_fingerprint;
    use crate::test_support::*;

    const COMMIT: &str = "1bb4c1ac08a8ace47f0e826dc14e090f0fc784c1";
    const WINDOWS: &str = "x86_64-pc-windows-msvc";
    const LINUX: &str = "x86_64-unknown-linux-gnu";
    const MACOS: &str = "aarch64-apple-darwin";
    const WASM: &str = "wasm32-unknown-unknown";
    const URL: &str = "http://127.0.0.1:1/";

    struct Fixture {
        crate_dir: TempDir,
        dist: TempDir,
    }

    impl Fixture {
        fn new() -> Fixture {
            let fixture = Fixture {
                crate_dir: TempDir::new("list-crate"),
                dist: TempDir::new("list-dist"),
            };
            fake_crate(fixture.crate_dir.path(), false);
            fixture
        }

        fn add(&self, name: &str, bytes: &[u8]) {
            write_file(self.dist.path(), &format!("{name}.tar.gz"), bytes);
        }

        fn add_release(&self, target: &str, subset: u8) -> String {
            let name = format!("oxijolt-sys-1.2.0-{target}-{subset}");
            self.add(
                &name,
                &release_archive(&name, &config(target, subset), COMMIT),
            );
            name
        }

        fn list(&self, allow_partial: bool) -> anyhow::Result<ArchiveList> {
            build_list(
                self.dist.path(),
                URL,
                COMMIT,
                self.crate_dir.path(),
                allow_partial,
            )
        }
    }

    #[test]
    fn a_partial_list_records_every_field() {
        let fixture = Fixture::new();
        let windows = fixture.add_release(WINDOWS, 5);
        let linux = fixture.add_release(LINUX, 0);
        let macos = fixture.add_release(MACOS, 2);
        write_file(
            fixture.dist.path(),
            &format!("{linux}.tar.gz.sha256"),
            b"0 x\n",
        );

        let list = fixture.list(true).expect("list");
        assert_eq!(list.version.as_deref(), Some("1.2.0"));
        assert_eq!(list.url.as_deref(), Some("http://127.0.0.1:1"));
        assert_eq!(
            list.sources,
            sources_fingerprint(fixture.crate_dir.path()).ok()
        );
        assert_eq!(list.archives.len(), 3);
        let by_name = |name: &str| {
            list.archives
                .iter()
                .find(|a| a.name == name)
                .expect("listed")
        };

        let archive = by_name(&windows);
        assert_eq!(archive.config, config(WINDOWS, 5));
        assert_eq!(archive.msvc, Version2::parse("14.44"));
        assert_eq!(archive.glibc, None);
        assert_eq!(archive.macos, None);
        let bytes = fs::read(fixture.dist.path().join(format!("{windows}.tar.gz"))).expect("read");
        assert_eq!(archive.sha256, prebuilt_archive::sha256(&bytes));
        let archive = by_name(&linux);
        assert_eq!(archive.glibc, Version2::parse("2.35"));
        assert_eq!(archive.msvc, None);
        assert_eq!(archive.macos, None);
        let archive = by_name(&macos);
        assert_eq!(archive.config, config(MACOS, 2));
        assert_eq!(archive.macos, Version2::parse("11.0"));
        assert_eq!(archive.msvc, None);
        assert_eq!(archive.glibc, None);
        assert_eq!(prebuilt::parse(&prebuilt::write(&list)), Ok(list));
    }

    #[test]
    fn a_full_release_needs_no_flag_and_a_partial_one_does() {
        let fixture = Fixture::new();
        fixture.add_release(WINDOWS, 0);
        fixture.add_release(LINUX, 0);
        let error = fixture.list(false).expect_err("partial").to_string();
        assert!(error.contains("--allow-partial"), "{error}");
        for target in [WINDOWS, LINUX, MACOS, WASM] {
            for subset in 0..8 {
                let first = subset == 0 && (target == WINDOWS || target == LINUX);
                if !first {
                    fixture.add_release(target, subset);
                }
            }
        }
        assert_eq!(fixture.list(false).expect("full").archives.len(), 32);

        let without_apple_default = Fixture::new();
        for target in [WINDOWS, LINUX, MACOS, WASM] {
            for subset in 0..8 {
                if (target, subset) != (MACOS, 0) {
                    without_apple_default.add_release(target, subset);
                }
            }
        }
        let error = without_apple_default
            .list(false)
            .expect_err("missing")
            .to_string();
        assert!(error.contains("1 missing"), "{error}");

        let intel = "oxijolt-sys-1.2.0-x86_64-apple-darwin-0";
        fixture.add(
            intel,
            &release_archive(intel, &config("x86_64-apple-darwin", 0), COMMIT),
        );
        // Intel Macs have no release archives and no macOS minimum.
        let error = format!("{:#}", fixture.list(false).expect_err("intel"));
        assert!(error.contains(intel), "{error}");
        assert!(error.contains("no macOS minimum known"), "{error}");
    }

    #[test]
    fn inconsistent_archives_are_refused() {
        let other_commit = "0000000000000000000000000000000000000000";
        let windows = config(WINDOWS, 0);
        let linux = config(LINUX, 0);
        let gnu_compiler = provenance_for(&windows, COMMIT).replace("MSVC 19.44.35229.0", "GNU 12");
        let no_glibc = provenance_for(&linux, COMMIT).replace("glibc: 2.35\n", "");
        let twice = format!("{}target=x\n", manifest_for(&windows));
        let wasm = config(WASM, 0);
        let no_wasi_sdk: String = manifest_for(&wasm)
            .lines()
            .filter(|line| !line.starts_with("wasi_sdk="))
            .map(|line| format!("{line}\n"))
            .collect();
        let without = |config: &ArchiveConfig, dropped: &str| {
            let name = "a";
            let bytes = release_archive(name, config, COMMIT);
            let mut builder = tar::Builder::new(flate2::write::GzEncoder::new(
                Vec::new(),
                flate2::Compression::fast(),
            ));
            let mut archive = tar::Archive::new(GzDecoder::new(&bytes[..]));
            for entry in archive.entries().expect("entries") {
                let mut entry = entry.expect("entry");
                let path = entry.path().expect("path").to_string_lossy().into_owned();
                if path.trim_end_matches('/') == format!("{name}/{dropped}") {
                    continue;
                }
                let mut header = entry.header().clone();
                let mut data = Vec::new();
                entry.read_to_end(&mut data).expect("read");
                builder
                    .append_data(&mut header, &path, &data[..])
                    .expect("append");
            }
            builder.into_inner().expect("tar").finish().expect("gzip")
        };
        let macos = config(MACOS, 0);
        let apple = |from: &str, to: &str| {
            let provenance = provenance_for(&macos, COMMIT);
            assert!(provenance.contains(from), "{from}");
            release_archive_with(
                "a",
                &manifest_for(&macos),
                &provenance.replace(from, to),
                &[],
            )
        };
        let cases: Vec<(&str, Vec<u8>, &str)> = vec![
            (
                "wrong stem",
                release_archive("other", &windows, COMMIT),
                "outside",
            ),
            (
                "foreign commit",
                release_archive("a", &windows, other_commit),
                "not from commit",
            ),
            (
                "outside the top directory",
                release_archive_with(
                    "a",
                    &manifest_for(&windows),
                    &provenance_for(&windows, COMMIT),
                    &[("b/x", b"x")],
                ),
                "outside",
            ),
            (
                "compiler is not MSVC",
                release_archive_with("a", &manifest_for(&windows), &gnu_compiler, &[]),
                "compiler",
            ),
            (
                "linux without glibc",
                release_archive_with("a", &manifest_for(&linux), &no_glibc, &[]),
                "glibc",
            ),
            (
                "repeated manifest key",
                release_archive_with("a", &twice, &provenance_for(&windows, COMMIT), &[]),
                "duplicate",
            ),
            (
                "a file outside the prefix layout",
                release_archive_with(
                    "a",
                    &manifest_for(&windows),
                    &provenance_for(&windows, COMMIT),
                    &[("a/lib/extra.dll", b"x")],
                ),
                "is not a file of the archive",
            ),
            (
                "a library in another target's spelling",
                release_archive_with(
                    "a",
                    &manifest_for(&windows),
                    &provenance_for(&windows, COMMIT),
                    &[("a/lib/libjoltc.a", b"x")],
                ),
                "is not a file of the archive",
            ),
            (
                "macos without macos",
                apple("macos: 11.0\n", ""),
                "`macos` missing",
            ),
            (
                "macos with a patch",
                apple("macos: 11.0\n", "macos: 11.0.1\n"),
                "major.minor",
            ),
            (
                "macos newer than Rust's minimum",
                apple("macos: 11.0\n", "macos: 12.0\n"),
                "newer than Rust's minimum",
            ),
            (
                "an Apple library in the MSVC spelling",
                release_archive_with(
                    "a",
                    &manifest_for(&macos),
                    &provenance_for(&macos, COMMIT),
                    &[("a/lib/joltc.lib", b"x")],
                ),
                "is not a file of the archive",
            ),
            (
                "wasm without wasi_sdk",
                release_archive_with("a", &no_wasi_sdk, &provenance_for(&wasm, COMMIT), &[]),
                "`wasi_sdk` missing",
            ),
            (
                "wasm without a runtime licence",
                without(&wasm, "licenses/wasi-libc-musl-COPYRIGHT"),
                "missing [\"a/licenses/wasi-libc-musl-COPYRIGHT\"]",
            ),
            (
                "wasm without a runtime library",
                without(&wasm, "lib/libc++abi.a"),
                "missing [\"a/lib/libc++abi.a\"]",
            ),
            (
                "native without a licence",
                without(&windows, "licenses/JoltPhysics-LICENSE"),
                "missing [\"a/licenses/JoltPhysics-LICENSE\"]",
            ),
            (
                "a wasm library in a native archive",
                release_archive_with(
                    "a",
                    &manifest_for(&linux),
                    &provenance_for(&linux, COMMIT),
                    &[("a/lib/libc.a", b"x")],
                ),
                "is not a file of the archive",
            ),
            ("not gzip", b"plain bytes".to_vec(), "tar.gz"),
        ];
        for (case, bytes, expected) in cases {
            let fixture = Fixture::new();
            fixture.add("a", &bytes);
            let error = format!("{:#}", fixture.list(true).expect_err(case));
            assert!(error.contains(expected), "{case}: {error}");
        }
    }

    #[test]
    fn a_repeated_configuration_is_refused() {
        let fixture = Fixture::new();
        fixture.add_release(WINDOWS, 3);
        fixture.add(
            "copy",
            &release_archive("copy", &config(WINDOWS, 3), COMMIT),
        );
        let error = fixture.list(true).expect_err("duplicate").to_string();
        assert!(error.contains("repeats the configuration"), "{error}");
    }

    #[test]
    fn check_finds_changed_and_missing_archives() {
        let fixture = Fixture::new();
        let windows = fixture.add_release(WINDOWS, 0);
        let linux = fixture.add_release(LINUX, 0);
        let list_path = fixture.dist.path().join("prebuilt.txt");
        fs::write(
            &list_path,
            prebuilt::write(&fixture.list(true).expect("list")),
        )
        .expect("write");
        check_list(&list_path, fixture.dist.path()).expect("matches");

        let path = fixture.dist.path().join(format!("{windows}.tar.gz"));
        let mut bytes = fs::read(&path).expect("read");
        let last = bytes.len() - 1;
        bytes[last] ^= 1;
        fs::write(&path, bytes).expect("write");
        fs::remove_file(fixture.dist.path().join(format!("{linux}.tar.gz"))).expect("remove");
        let error = check_list(&list_path, fixture.dist.path())
            .expect_err("changed")
            .to_string();
        assert!(
            error.contains(&format!("{windows}: sha256 differs")),
            "{error}"
        );
        assert!(error.contains(&format!("{linux}: missing")), "{error}");
    }

    #[test]
    fn the_crate_version_comes_from_the_package_section() {
        let fixture = Fixture::new();
        assert_eq!(
            crate_version(fixture.crate_dir.path()).expect("version"),
            "1.2.0"
        );
    }

    #[test]
    fn library_names_follow_the_target_and_precision() {
        let libraries =
            |target: &str, joltc_lib: &str| PrefixFiles::new(target, joltc_lib).libraries;
        assert_eq!(libraries(WINDOWS, "joltc"), ["joltc.lib", "Jolt.lib"]);
        assert_eq!(
            libraries(LINUX, "joltc_double"),
            ["libjoltc_double.a", "libJolt.a"]
        );
        assert_eq!(libraries(MACOS, "joltc"), ["libjoltc.a", "libJolt.a"]);
        assert_eq!(
            libraries(MACOS, "joltc_double"),
            ["libjoltc_double.a", "libJolt.a"]
        );
        assert_eq!(
            libraries(WASM, "joltc_double")[..2],
            ["libjoltc_double.a", "libJolt.a"]
        );
    }

    #[test]
    fn compiler_versions_map_to_toolsets() {
        assert_eq!(
            msvc_toolset("MSVC 19.44.35229.0 "),
            Version2::parse("14.44")
        );
        assert_eq!(msvc_toolset("GNU 13.3.0"), None);
        assert_eq!(msvc_toolset("MSVC"), None);
    }
}
