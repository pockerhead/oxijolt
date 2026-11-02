//! Tests of the archive checks, the unpacking and the cache of `build/prebuilt_archive.rs`.

use std::fs;
use std::path::Path;
use std::time::{Duration, UNIX_EPOCH};

use flate2::write::GzEncoder;
use flate2::Compression;
use tar::{EntryType, Header};

use crate::prebuilt::{Archive, Unavailable};
use crate::prebuilt_archive::*;
use crate::test_support::{config, release_archive, TempDir};

const NAME: &str = "oxijolt-sys-1.2.0-x86_64-pc-windows-msvc-default";
const COMMIT: &str = "1bb4c1ac08a8ace47f0e826dc14e090f0fc784c1";
/// The files of the archive [`NAME`].
fn windows_files() -> PrefixFiles {
    PrefixFiles::new("x86_64-pc-windows-msvc", "joltc")
}

/// A good archive and its list entry.
fn good() -> (Vec<u8>, Archive) {
    let config = config("x86_64-pc-windows-msvc", 0);
    let bytes = release_archive(NAME, &config, COMMIT);
    let archive = Archive {
        name: NAME.to_owned(),
        sha256: sha256(&bytes),
        config,
        msvc: None,
        glibc: None,
        macos: None,
    };
    (bytes, archive)
}

/// A `.tar.gz` whose headers are written byte for byte: `name` goes into the old header's name
/// field unchecked, so the tests can build paths the `tar` builder refuses.
fn raw_tar_gz(entries: &[(&[u8], EntryType, &[u8])]) -> Vec<u8> {
    let mut builder = tar::Builder::new(GzEncoder::new(Vec::new(), Compression::fast()));
    for (name, kind, data) in entries {
        let mut header = Header::new_old();
        header.as_old_mut().name[..name.len()].copy_from_slice(name);
        header.set_entry_type(*kind);
        header.set_mode(0o644);
        header.set_size(data.len() as u64);
        header.set_mtime(1_700_000_000);
        header.set_cksum();
        builder.append(&header, *data).expect("append");
    }
    builder.into_inner().expect("tar").finish().expect("gzip")
}

/// An owned entry for [`raw_tar_gz`]: name, type and data.
type RawEntry = (String, EntryType, Vec<u8>);

/// A good archive's directory and library entries followed by `extra`.
fn with_extra(extra: (&[u8], EntryType, &[u8])) -> Vec<u8> {
    let top = format!("{NAME}/");
    let lib = format!("{NAME}/lib/");
    let joltc = format!("{NAME}/lib/joltc.lib");
    raw_tar_gz(&[
        (top.as_bytes(), EntryType::Directory, b""),
        (lib.as_bytes(), EntryType::Directory, b""),
        (joltc.as_bytes(), EntryType::Regular, b"joltc"),
        extra,
    ])
}

fn unpack_into(bytes: &[u8], out: &Path) -> Result<std::path::PathBuf, Unavailable> {
    let (_, mut archive) = good();
    archive.sha256 = sha256(bytes);
    unpack(bytes, &archive, &windows_files(), out)
}

#[test]
fn a_good_archive_unpacks_with_its_marker_and_times() {
    let out = TempDir::new("unpack-good");
    let (bytes, archive) = good();
    let prefix = verify_and_unpack(&bytes, &archive, &windows_files(), out.path()).expect("unpack");
    assert_eq!(prefix, out.path().join("prebuilt").join(NAME));
    let joltc = prefix.join("lib/joltc.lib");
    assert_eq!(fs::read(&joltc).expect("read"), b"joltc");
    for file in [
        "include/joltc_ext.h",
        "licenses/JoltPhysics-LICENSE",
        "PROVENANCE.txt",
    ] {
        assert!(prefix.join(file).is_file(), "{file}");
    }
    let modified = fs::metadata(&joltc)
        .and_then(|m| m.modified())
        .expect("mtime");
    assert_eq!(modified, UNIX_EPOCH + Duration::from_secs(1_700_000_000));

    let marker =
        fs::read_to_string(out.path().join(format!("prebuilt/{NAME}.files"))).expect("marker");
    let mut lines = marker.lines();
    assert_eq!(
        lines.next(),
        Some(format!("archive {}", crate::prebuilt::hex(&archive.sha256)).as_str())
    );
    let joltc_line = format!("{} lib/joltc.lib", crate::prebuilt::hex(&sha256(b"joltc")));
    assert!(lines.any(|line| line == joltc_line), "{marker}");
    assert!(!out.path().join(format!("prebuilt/.tmp-{NAME}")).exists());

    assert_eq!(cached(out.path(), &archive, &windows_files()), Some(prefix));
}

#[test]
fn an_apple_archive_unpacks_with_unix_library_names() {
    let name = "oxijolt-sys-1.2.0-aarch64-apple-darwin-default";
    let libraries = PrefixFiles::new("aarch64-apple-darwin", "joltc");
    let out = TempDir::new("unpack-apple");
    let config = config("aarch64-apple-darwin", 0);
    let bytes = release_archive(name, &config, COMMIT);
    let archive = Archive {
        name: name.to_owned(),
        sha256: sha256(&bytes),
        config,
        msvc: None,
        glibc: None,
        macos: crate::prebuilt::Version2::parse("11.0"),
    };
    match verify_and_unpack(&bytes, &archive, &windows_files(), out.path()) {
        Err(Unavailable::Unpack(reason)) => {
            assert!(reason.contains("not a file of the archive"), "{reason}")
        }
        other => panic!("MSVC spelling: {other:?}"),
    }
    assert!(!out.path().join("prebuilt").exists());

    let prefix = verify_and_unpack(&bytes, &archive, &libraries, out.path()).expect("unpack");
    assert_eq!(prefix, out.path().join("prebuilt").join(name));
    assert_eq!(
        fs::read(prefix.join("lib/libjoltc.a")).expect("read"),
        b"joltc"
    );
    assert_eq!(
        fs::read(prefix.join("lib/libJolt.a")).expect("read"),
        b"Jolt"
    );
    assert_eq!(cached(out.path(), &archive, &libraries), Some(prefix));
}

#[test]
fn an_unpack_replaces_a_stale_prefix_and_staging_directory() {
    let out = TempDir::new("unpack-stale");
    let (bytes, archive) = good();
    let cache = out.path().join("prebuilt");
    crate::test_support::write_file(&cache, &format!("{NAME}/lib/old.lib"), b"old");
    crate::test_support::write_file(
        &cache,
        &format!(".tmp-{NAME}/{NAME}/lib/joltc.lib"),
        b"half",
    );
    let prefix = unpack(&bytes, &archive, &windows_files(), out.path()).expect("unpack");
    assert!(!prefix.join("lib/old.lib").exists());
    assert_eq!(
        fs::read(prefix.join("lib/joltc.lib")).expect("read"),
        b"joltc"
    );
}

#[test]
fn a_changed_cached_file_is_found_and_the_cache_removed() {
    let out = TempDir::new("cache-flip");
    let (bytes, archive) = good();
    let prefix = unpack(&bytes, &archive, &windows_files(), out.path()).expect("unpack");
    fs::write(prefix.join("lib/Jolt.lib"), b"Jolu").expect("flip");
    assert_eq!(cached(out.path(), &archive, &windows_files()), None);
    assert!(!prefix.exists());
    assert!(!out.path().join(format!("prebuilt/{NAME}.files")).exists());
}

#[test]
fn a_cache_of_another_archive_hash_is_not_used() {
    let out = TempDir::new("cache-other");
    let (bytes, mut archive) = good();
    let prefix = unpack(&bytes, &archive, &windows_files(), out.path()).expect("unpack");
    archive.sha256[0] ^= 1;
    assert_eq!(cached(out.path(), &archive, &windows_files()), None);
    assert!(!prefix.exists());
}

#[test]
fn a_cache_without_its_marker_is_not_used() {
    let out = TempDir::new("cache-marker");
    let (bytes, archive) = good();
    let prefix = unpack(&bytes, &archive, &windows_files(), out.path()).expect("unpack");
    fs::remove_file(out.path().join(format!("prebuilt/{NAME}.files"))).expect("remove");
    assert_eq!(cached(out.path(), &archive, &windows_files()), None);
    assert!(!prefix.exists());
}

/// Unpacks the good archive, rewrites its marker with `edit` and changes `lib/Jolt.lib`: the
/// cache must not be used.
fn assert_marker_edit_is_refused(label: &str, edit: impl Fn(&str) -> String) {
    let out = TempDir::new(label);
    let (bytes, archive) = good();
    let prefix = unpack(&bytes, &archive, &windows_files(), out.path()).expect("unpack");
    let marker = out.path().join(format!("prebuilt/{NAME}.files"));
    let record = fs::read_to_string(&marker).expect("marker");
    fs::write(&marker, edit(&record)).expect("edit marker");
    fs::write(prefix.join("lib/Jolt.lib"), b"Jolu").expect("flip");
    assert_eq!(
        cached(out.path(), &archive, &windows_files()),
        None,
        "{label}"
    );
    assert!(!prefix.exists(), "{label}");
}

/// The marker without the record of `relative`.
fn without(record: &str, relative: &str) -> String {
    record
        .lines()
        .filter(|line| !line.ends_with(&format!(" {relative}")))
        .map(|line| {
            format!(
                "{line}
"
            )
        })
        .collect()
}

#[test]
fn a_marker_that_misses_a_native_file_is_not_used() {
    assert_marker_edit_is_refused("cache-truncated", |record| {
        record
            .lines()
            .take(2)
            .map(|line| {
                format!(
                    "{line}
"
                )
            })
            .collect()
    });
    assert_marker_edit_is_refused("cache-no-jolt", |record| without(record, "lib/Jolt.lib"));
    for relative in [
        "lib/joltc.lib",
        "include/joltc.h",
        "include/joltc_ext.h",
        "oxijolt-sys-manifest.txt",
    ] {
        let out = TempDir::new("cache-missing-record");
        let (bytes, archive) = good();
        let prefix = unpack(&bytes, &archive, &windows_files(), out.path()).expect("unpack");
        let marker = out.path().join(format!("prebuilt/{NAME}.files"));
        let record = fs::read_to_string(&marker).expect("marker");
        fs::write(&marker, without(&record, relative)).expect("edit marker");
        assert_eq!(
            cached(out.path(), &archive, &windows_files()),
            None,
            "{relative}"
        );
        assert!(!prefix.exists(), "{relative}");
    }
}

#[test]
fn a_marker_that_records_a_file_twice_is_not_used() {
    let out = TempDir::new("cache-duplicate-intact");
    let (bytes, archive) = good();
    let prefix = unpack(&bytes, &archive, &windows_files(), out.path()).expect("unpack");
    let marker = out.path().join(format!("prebuilt/{NAME}.files"));
    let record = fs::read_to_string(&marker).expect("marker");
    let first_file = record.lines().nth(1).expect("a file record");
    fs::write(
        &marker,
        format!(
            "{record}{first_file}
"
        ),
    )
    .expect("edit marker");
    assert_eq!(cached(out.path(), &archive, &windows_files()), None);
    assert!(!prefix.exists());
}

#[test]
fn the_marker_is_written_whole() {
    let out = TempDir::new("cache-atomic");
    let (bytes, archive) = good();
    unpack(&bytes, &archive, &windows_files(), out.path()).expect("unpack");
    let cache = out.path().join("prebuilt");
    let mut names: Vec<String> = fs::read_dir(&cache)
        .expect("cache")
        .map(|entry| {
            entry
                .expect("entry")
                .file_name()
                .into_string()
                .expect("utf-8")
        })
        .collect();
    names.sort();
    assert_eq!(names, [NAME.to_owned(), format!("{NAME}.files")]);
}

#[test]
fn a_changed_download_is_never_decompressed() {
    let out = TempDir::new("verify");
    let (mut bytes, archive) = good();
    let last = bytes.len() - 1;
    bytes[last] ^= 1;
    assert_eq!(verify(&bytes, &archive.sha256), Err(Unavailable::Checksum));
    assert_eq!(
        verify_and_unpack(&bytes, &archive, &windows_files(), out.path()),
        Err(Unavailable::Checksum)
    );
    assert!(!out.path().join("prebuilt").exists());
}

#[test]
fn unexpected_entries_refuse_the_whole_archive() {
    let joltc = format!("{NAME}/lib/joltc.lib");
    let file = |name: String| (name, EntryType::Regular, b"x".to_vec());
    let cases: Vec<(&str, RawEntry, &str)> = vec![
        ("parent", file(format!("{NAME}/../x")), "not a file"),
        (
            "absolute",
            file(format!("/{NAME}/lib/Jolt.lib")),
            "not a file",
        ),
        (
            "backslash",
            file(format!("{NAME}\\lib\\Jolt.lib")),
            "not a file",
        ),
        (
            "drive",
            file(format!("C:{NAME}/lib/Jolt.lib")),
            "not a file",
        ),
        ("stream", file(format!("{joltc}:ads")), "not a file"),
        (
            "empty component",
            file(format!("{NAME}//lib/Jolt.lib")),
            "not a file",
        ),
        (
            "other file",
            file(format!("{NAME}/lib/extra.dll")),
            "not a file",
        ),
        (
            "other top",
            file("other/lib/Jolt.lib".to_owned()),
            "not a file",
        ),
        (
            "other spelling",
            file(format!("{NAME}/lib/libJolt.a")),
            "not a file",
        ),
        ("duplicate", file(joltc.clone()), "appears twice"),
        (
            "symlink",
            (
                format!("{NAME}/lib/Jolt.lib"),
                EntryType::Symlink,
                Vec::new(),
            ),
            "Symlink",
        ),
        (
            "hard link",
            (format!("{NAME}/lib/Jolt.lib"), EntryType::Link, Vec::new()),
            "Link",
        ),
        (
            "fifo",
            (format!("{NAME}/lib/Jolt.lib"), EntryType::Fifo, Vec::new()),
            "Fifo",
        ),
        (
            "directory with data",
            (
                format!("{NAME}/include/"),
                EntryType::Directory,
                b"x".to_vec(),
            ),
            "directory with data",
        ),
    ];
    for (case, (name, kind, data), expected) in cases {
        let out = TempDir::new("refuse");
        let bytes = with_extra((name.as_bytes(), kind, &data));
        match unpack_into(&bytes, out.path()) {
            Err(Unavailable::Unpack(reason)) => {
                assert!(reason.contains(expected), "{case}: {reason}")
            }
            other => panic!("{case}: {other:?}"),
        }
        assert!(!out.path().join("prebuilt").exists(), "{case}");
    }
}

#[test]
fn many_entries_are_refused() {
    let out = TempDir::new("refuse-many");
    let path = format!("{NAME}/lib/");
    let entries: Vec<(&[u8], EntryType, &[u8])> = (0..65)
        .map(|_| (path.as_bytes(), EntryType::Directory, &b""[..]))
        .collect();
    assert!(matches!(
        unpack_into(&raw_tar_gz(&entries), out.path()),
        Err(Unavailable::Unpack(_))
    ));
    assert!(!out.path().join("prebuilt").exists());
}

#[test]
fn an_entry_over_the_decoded_limit_is_refused() {
    let out = TempDir::new("refuse-large");
    let mut builder = tar::Builder::new(GzEncoder::new(Vec::new(), Compression::fast()));
    let mut header = Header::new_gnu();
    header
        .set_path(format!("{NAME}/lib/Jolt.lib"))
        .expect("path");
    header.set_entry_type(EntryType::Regular);
    header.set_size(MAX_DECODED_BYTES + 1);
    header.set_cksum();
    // Only the header: the size is refused before any data would be read.
    builder.append(&header, &b""[..]).expect("append");
    let bytes = builder.into_inner().expect("tar").finish().expect("gzip");
    match unpack_into(&bytes, out.path()) {
        Err(Unavailable::Unpack(reason)) => assert!(reason.contains("bytes"), "{reason}"),
        other => panic!("{other:?}"),
    }
    assert!(!out.path().join("prebuilt").exists());
}

#[test]
fn a_truncated_stream_is_refused() {
    let out = TempDir::new("refuse-truncated");
    let (bytes, _) = good();
    let truncated = &bytes[..bytes.len() / 2];
    match unpack_into(truncated, out.path()) {
        Err(Unavailable::Unpack(reason)) => assert!(reason.contains("damaged"), "{reason}"),
        other => panic!("{other:?}"),
    }
    assert!(!out.path().join("prebuilt").exists());
}

#[test]
fn the_whitelist_follows_the_library_names() {
    let allowed = allowed_entries(
        "a",
        &PrefixFiles::new("x86_64-unknown-linux-gnu", "joltc_double"),
    );
    let files: Vec<&str> = allowed
        .iter()
        .filter(|entry| !entry.directory)
        .map(|entry| entry.path.as_str())
        .collect();
    assert!(files.contains(&"a/lib/libjoltc_double.a"));
    assert!(files.contains(&"a/lib/libJolt.a"));
    assert!(!files.iter().any(|file| file.ends_with(".lib")));
    assert!(!files.contains(&"a/lib/libc.a"));
    assert_eq!(allowed.len(), 14);
}

#[test]
fn a_wasm_archive_holds_its_runtime_and_licences() {
    let name = "oxijolt-sys-1.2.0-wasm32-unknown-unknown-default";
    let files = PrefixFiles::new("wasm32-unknown-unknown", "joltc");
    assert_eq!(
        files.libraries,
        [
            "libjoltc.a",
            "libJolt.a",
            "libc++.a",
            "libc++abi.a",
            "libc.a",
            "libclang_rt.builtins.a"
        ]
    );
    assert_eq!(files.runtime_licenses.len(), 11);
    assert_eq!(allowed_entries(name, &files).len(), 14 + 4 + 11);

    let out = TempDir::new("unpack-wasm");
    let config = config("wasm32-unknown-unknown", 0);
    let bytes = release_archive(name, &config, COMMIT);
    let archive = Archive {
        name: name.to_owned(),
        sha256: sha256(&bytes),
        config,
        msvc: None,
        glibc: None,
        macos: None,
    };
    // A native layout does not allow the runtime.
    let native = PrefixFiles::new("x86_64-unknown-linux-gnu", "joltc");
    match verify_and_unpack(&bytes, &archive, &native, out.path()) {
        Err(Unavailable::Unpack(reason)) => {
            assert!(reason.contains("not a file of the archive"), "{reason}")
        }
        other => panic!("native layout: {other:?}"),
    }

    let prefix = verify_and_unpack(&bytes, &archive, &files, out.path()).expect("unpack");
    for file in [
        "lib/libc++abi.a",
        "lib/libclang_rt.builtins.a",
        "licenses/wasi-libc-musl-COPYRIGHT",
    ] {
        assert!(prefix.join(file).is_file(), "{file}");
    }
    assert_eq!(cached(out.path(), &archive, &files), Some(prefix.clone()));

    // The runtime is linked, so a cache without it is fetched again.
    fs::write(prefix.join("lib/libc.a"), b"changed").expect("write");
    assert_eq!(cached(out.path(), &archive, &files), None);
}
