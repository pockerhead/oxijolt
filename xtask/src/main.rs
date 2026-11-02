//! Repository maintenance tasks.
//!
//! `cargo xtask bindings` regenerates the committed raw bindings of `oxijolt-sys` for every
//! ABI family and configuration, and their fingerprint record. `cargo xtask bindings --check`
//! regenerates them in memory and fails when the committed files differ, writing the fresh set
//! to `--out <dir>` (default `target/xtask-bindings`).
//!
//! `cargo xtask prebuilt-list` writes `crates/oxijolt-sys/prebuilt.txt`, the list of release
//! archives the build script downloads, from a directory of built archives;
//! `cargo xtask prebuilt-list --check` checks archives against a list (see [`prebuilt_list`]).

use std::env;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{bail, Context};

#[path = "../../crates/oxijolt-sys/build/bindgen_options.rs"]
mod bindgen_options;
// The target check runs in build.rs; here only the tests call it.
#[allow(dead_code)]
#[path = "../../crates/oxijolt-sys/build/targets.rs"]
mod targets;
// build.rs passes these options to CMake; its tests run here.
#[cfg(test)]
#[path = "../../crates/oxijolt-sys/build/cmake_options.rs"]
mod cmake_options;
// build.rs reads wasi-sdk with these helpers; its tests run here, and the archive checks name
// the runtime files a wasm prefix bundles.
#[allow(dead_code)]
#[path = "../../crates/oxijolt-sys/build/wasi_sdk.rs"]
mod wasi_sdk;
// The build script's archive list and archive checks; the list writer uses a part of them.
#[allow(dead_code)]
#[path = "../../crates/oxijolt-sys/build/prebuilt.rs"]
mod prebuilt;
#[allow(dead_code)]
#[path = "../../crates/oxijolt-sys/build/prebuilt_archive.rs"]
mod prebuilt_archive;
#[cfg(test)]
mod prebuilt_archive_tests;
// The download is the build script's; here only its tests run.
#[cfg(test)]
#[allow(dead_code)]
#[path = "../../crates/oxijolt-sys/build/prebuilt_curl.rs"]
mod prebuilt_curl;
#[cfg(test)]
mod prebuilt_curl_tests;
mod prebuilt_list;
#[cfg(test)]
mod prebuilt_tests;
#[cfg(test)]
mod test_support;

/// The bindgen version in `Cargo.toml`, recorded as provenance.
const BINDGEN_VERSION: &str = "0.73.2";

const USAGE: &str = concat!(
    "usage: cargo xtask bindings [--check] [--out <dir>]\n",
    "       cargo xtask prebuilt-list --dist <dir> --url <base> --commit <sha> [--out <file>] [--allow-partial]\n",
    "       cargo xtask prebuilt-list --check <list> --dist <dir>",
);

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = env::args().skip(1).collect();
    match args.split_first() {
        Some((task, rest)) if task == "bindings" => bindings(rest),
        Some((task, rest)) if task == "prebuilt-list" => prebuilt_list::run(rest, USAGE),
        _ => bail!(USAGE),
    }
}

/// One committed bindings file or the fingerprint record, by crate-relative path.
struct Output {
    path: String,
    text: Vec<u8>,
}

fn bindings(args: &[String]) -> anyhow::Result<()> {
    let mut check = false;
    let mut out = None;
    let mut args = args.iter();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--check" => check = true,
            "--out" => out = Some(PathBuf::from(args.next().context(USAGE)?)),
            _ => bail!(USAGE),
        }
    }

    let repo = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .context("xtask has no parent directory")?;
    let crate_dir = repo.join("crates").join("oxijolt-sys");
    refuse_ambient_clang_args()?;

    let outputs = generate_all(&crate_dir)?;
    if !check {
        for output in &outputs {
            write(&crate_dir, output)?;
        }
        println!(
            "wrote {} files under {}",
            outputs.len(),
            crate_dir.display()
        );
        return Ok(());
    }

    let stale = stale_outputs(&crate_dir, &outputs)?;
    if stale.is_empty() {
        println!("committed bindings are current");
        return Ok(());
    }
    let out = out.unwrap_or_else(|| repo.join("target").join("xtask-bindings"));
    for output in &outputs {
        write(&out, output)?;
    }
    bail!(
        "committed bindings differ from the generator:\n  {}\nthe fresh set is in {}",
        stale.join("\n  "),
        out.display()
    )
}

/// Bindgen and libclang read these from the environment and would change the output.
fn refuse_ambient_clang_args() -> anyhow::Result<()> {
    let ambient = ambient_clang_vars();
    if !ambient.is_empty() {
        bail!("unset {} first: bindgen reads them", ambient.join(", "));
    }
    for key in ["CLANG_PATH", "LIBCLANG_PATH"] {
        println!("{key}={}", env::var(key).unwrap_or_default());
    }
    println!("libclang {}", bindgen::clang_version().full);
    Ok(())
}

/// Bindgen reads `TARGET`, `BINDGEN_EXTRA_CLANG_ARGS` and, with `TARGET` set,
/// `BINDGEN_EXTRA_CLANG_ARGS_<target>`; libclang adds the include directories in
/// [`CLANG_INCLUDE_VARS`]. Windows looks environment names up regardless of case,
/// so there `target` is `TARGET`.
const CASE_INSENSITIVE_ENV: bool = cfg!(windows);

/// Include path variables libclang honours (clang's "Environment" documentation). A header found
/// through them can redefine the configuration macros.
const CLANG_INCLUDE_VARS: [&str; 5] = [
    "CPATH",
    "C_INCLUDE_PATH",
    "CPLUS_INCLUDE_PATH",
    "OBJC_INCLUDE_PATH",
    "OBJCPLUS_INCLUDE_PATH",
];

/// Whether bindgen or libclang may read the variable `name`.
fn is_bindgen_var(name: &str, case_insensitive: bool) -> bool {
    let name = if case_insensitive {
        name.to_ascii_uppercase()
    } else {
        name.to_owned()
    };
    name == "TARGET"
        || name.starts_with("BINDGEN_EXTRA_CLANG_ARGS")
        || CLANG_INCLUDE_VARS.contains(&name.as_str())
}

/// The variables bindgen would read, spelled as the environment has them.
fn ambient_clang_vars() -> Vec<String> {
    let mut found: Vec<String> = env::vars_os()
        .map(|(name, _)| name.to_string_lossy().into_owned())
        .filter(|name| is_bindgen_var(name, CASE_INSENSITIVE_ENV))
        .collect();
    // The readers' own lookups decide; they also match spellings an ASCII case fold misses.
    for name in ["TARGET", "BINDGEN_EXTRA_CLANG_ARGS"]
        .into_iter()
        .chain(CLANG_INCLUDE_VARS)
    {
        let listed = found.iter().any(|f| f.eq_ignore_ascii_case(name));
        if env::var_os(name).is_some() && !listed {
            found.push(name.to_owned());
        }
    }
    found
}

/// The bindings files (four per family) and the fingerprint record, generated in memory.
///
/// Every target of a family must give the same bindings; the first difference fails.
fn generate_all(crate_dir: &Path) -> anyhow::Result<Vec<Output>> {
    let include_dir = crate_dir.join("vendor").join("joltc").join("include");
    let joltc_h = include_dir.join("joltc.h");
    if !joltc_h.is_file() {
        bail!(
            "{} is missing: run `git submodule update --init`",
            joltc_h.display()
        );
    }
    let header = crate_dir
        .join("native")
        .join("joltc_ext")
        .join("joltc_ext.h");

    let mut outputs = Vec::new();
    for family in targets::FAMILIES {
        for double_precision in [false, true] {
            for debug_renderer in [false, true] {
                let path = targets::bindings_file(family.name, double_precision, debug_renderer);
                let mut first: Option<(&str, Vec<u8>)> = None;
                for target in family.targets {
                    let text = generate(
                        &header,
                        &include_dir,
                        target,
                        double_precision,
                        debug_renderer,
                    )?;
                    match &first {
                        None => first = Some((target, text)),
                        Some((first_target, first_text)) if *first_text != text => {
                            bail!("{first_target} and {target} give different bindings for {path}")
                        }
                        Some(_) => {}
                    }
                }
                let (_, text) = first.context("a family without targets")?;
                outputs.push(Output { path, text });
            }
        }
    }

    let read =
        |path: &Path| fs::read(path).with_context(|| format!("cannot read {}", path.display()));
    let policy: Vec<(&str, Vec<u8>)> = targets::POLICY_SOURCES
        .iter()
        .map(|source| Ok((*source, read(&crate_dir.join(source))?)))
        .collect::<anyhow::Result<_>>()?;
    let policy: Vec<(&str, &[u8])> = policy
        .iter()
        .map(|(path, bytes)| (*path, bytes.as_slice()))
        .collect();
    let mut inputs = targets::input_fingerprints(&read(&joltc_h)?, &read(&header)?, &policy);
    inputs += &format!("bindgen={BINDGEN_VERSION}\n");
    inputs += &format!("libclang={}\n", bindgen::clang_version().full);
    outputs.push(Output {
        path: targets::INPUTS_FILE.to_owned(),
        text: inputs.into_bytes(),
    });
    Ok(outputs)
}

/// Bindings of one target and configuration, with LF line endings.
fn generate(
    header: &Path,
    include_dir: &Path,
    target: &str,
    double_precision: bool,
    debug_renderer: bool,
) -> anyhow::Result<Vec<u8>> {
    let bindings = bindgen_options::builder(
        header,
        include_dir,
        target,
        double_precision,
        debug_renderer,
    )
    .generate()
    .with_context(|| format!("bindgen failed for {target}"))?;
    Ok(targets::normalize_crlf(bindings.to_string().as_bytes()))
}

/// Paths of committed outputs that differ from `outputs`. The fingerprint record is compared
/// without its provenance lines, which only report a note.
fn stale_outputs(crate_dir: &Path, outputs: &[Output]) -> anyhow::Result<Vec<String>> {
    let mut stale = Vec::new();
    for output in outputs {
        let committed =
            fs::read(crate_dir.join(&output.path)).map(|bytes| targets::normalize_crlf(&bytes));
        let Ok(committed) = committed else {
            stale.push(format!("{} (missing)", output.path));
            continue;
        };
        if output.path != targets::INPUTS_FILE {
            if committed != output.text {
                stale.push(output.path.clone());
            }
            continue;
        }
        let fresh = String::from_utf8_lossy(&output.text);
        let recorded = String::from_utf8_lossy(&committed);
        let keys = targets::differing_keys(&fresh, &recorded);
        if !keys.is_empty() {
            stale.push(format!("{} ({})", output.path, keys.join(", ")));
        }
        if provenance(&fresh) != provenance(&recorded) {
            println!(
                "note: committed bindings record {:?}, this run {:?}",
                provenance(&recorded),
                provenance(&fresh)
            );
        }
    }
    Ok(stale)
}

/// The provenance lines of a fingerprint record.
fn provenance(text: &str) -> Vec<&str> {
    text.lines()
        .filter(|line| {
            line.split_once('=')
                .is_some_and(|(key, _)| targets::PROVENANCE_KEYS.contains(&key))
        })
        .collect()
}

fn write(root: &Path, output: &Output) -> anyhow::Result<()> {
    let path = root.join(&output.path);
    if let Some(dir) = path.parent() {
        fs::create_dir_all(dir).with_context(|| format!("cannot create {}", dir.display()))?;
    }
    fs::write(&path, &output.text).with_context(|| format!("cannot write {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::is_bindgen_var;
    use super::targets::*;

    #[test]
    fn bindgen_vars_are_matched_without_case_where_the_os_does() {
        for name in [
            "TARGET",
            "target",
            "Target",
            "BINDGEN_EXTRA_CLANG_ARGS",
            "bindgen_extra_clang_args",
            "Bindgen_Extra_Clang_Args_x86_64-pc-windows-msvc",
            "bindgen_extra_clang_args_x86_64_unknown_linux_gnu",
            "CPATH",
            "cpath",
            "C_INCLUDE_PATH",
            "cplus_include_path",
            "OBJC_INCLUDE_PATH",
            "ObjCPlus_Include_Path",
        ] {
            assert!(is_bindgen_var(name, true), "{name}");
        }
        assert!(is_bindgen_var(
            "BINDGEN_EXTRA_CLANG_ARGS_x86_64_pc_windows_msvc",
            false
        ));
        assert!(!is_bindgen_var("target", false));
        assert!(!is_bindgen_var("bindgen_extra_clang_args", false));
        assert!(!is_bindgen_var("cpath", false));
        for name in [
            "TARGET_DIR",
            "CARGO_TARGET_DIR",
            "BINDGEN",
            "LIBCLANG_PATH",
            "INCLUDE",
            "CPATH_X",
        ] {
            assert!(!is_bindgen_var(name, true), "{name}");
        }
    }

    #[test]
    fn every_registered_target_is_accepted() {
        for family in FAMILIES {
            for target in family.targets {
                let found = check_target(target, family.pointer_width).expect("registered");
                assert_eq!(found.name, family.name);
            }
        }
        let wasm = check_target("wasm32-unknown-unknown", "32").expect("registered");
        assert_eq!(wasm.name, "wasm32");
    }

    #[test]
    fn narrow_and_unregistered_targets_are_refused() {
        for target in [
            "i686-pc-windows-msvc",
            "i686-unknown-linux-gnu",
            "armv7-linux-androideabi",
        ] {
            assert_eq!(
                check_target(target, "32").err(),
                Some(TargetError::PointerWidth(target.to_owned()))
            );
        }
        assert_eq!(
            check_target("x86_64-unknown-linux-gnu", "32").err(),
            Some(TargetError::PointerWidth(
                "x86_64-unknown-linux-gnu".to_owned()
            ))
        );
        assert_eq!(
            check_target("wasm32-unknown-unknown", "64").err(),
            Some(TargetError::PointerWidth(
                "wasm32-unknown-unknown".to_owned()
            ))
        );
        assert_eq!(
            check_target("wasm64-unknown-unknown", "64").err(),
            Some(TargetError::Unregistered(
                "wasm64-unknown-unknown".to_owned()
            ))
        );
        assert_eq!(
            check_target("x86_64-unknown-freebsd", "64").err(),
            Some(TargetError::Unregistered(
                "x86_64-unknown-freebsd".to_owned()
            ))
        );
    }

    #[test]
    fn only_crlf_pairs_are_normalized() {
        assert_eq!(normalize_crlf(b"a\r\nb"), b"a\nb");
        assert_ne!(normalize_crlf(b"a\rb"), b"ab");
        assert_ne!(normalize_crlf(b"a\rb"), b"a\nb");
        assert_eq!(fingerprint(b"a\r\nb"), fingerprint(b"a\nb"));
    }

    #[test]
    fn fingerprint_changes_with_one_byte() {
        assert_ne!(fingerprint(b"JPH_Body"), fingerprint(b"JPH_Bodz"));
    }

    #[test]
    fn input_fingerprints_keep_their_format() {
        let text = input_fingerprints(b"a", b"b", &[("build/x.rs", b"c"), ("build/y.rs", b"")]);
        assert_eq!(
            text,
            "format=1\n\
             joltc.h=af63dc4c8601ec8c\n\
             joltc_ext.h=af63df4c8601f1a5\n\
             build/x.rs=af63de4c8601eff2\n\
             build/y.rs=cbf29ce484222325\n"
        );
    }

    #[test]
    fn provenance_lines_do_not_make_bindings_stale() {
        let expected = "format=1\njoltc.h=01\n";
        assert!(differing_keys(expected, "format=1\r\njoltc.h=01\r\nlibclang=12\r\n").is_empty());
        assert_eq!(
            differing_keys(expected, "format=1\njoltc.h=02\n"),
            ["joltc.h"]
        );
        assert_eq!(
            differing_keys(expected, "format=1\njoltc.h=01\nbuild/z.rs=03\n"),
            ["build/z.rs"]
        );
    }

    #[test]
    fn bindings_files_are_distinct() {
        let mut paths: Vec<String> = FAMILIES
            .iter()
            .flat_map(|family| {
                [(false, false), (false, true), (true, false), (true, true)]
                    .map(|(dp, dr)| bindings_file(family.name, dp, dr))
            })
            .collect();
        assert_eq!(paths[0], "src/bindings/msvc/single_precision.rs");
        assert_eq!(
            paths[3],
            "src/bindings/msvc/double_precision_debug_renderer.rs"
        );
        paths.sort();
        paths.dedup();
        assert_eq!(paths.len(), 4 * FAMILIES.len());
    }

    #[test]
    fn every_selectable_bindings_file_is_committed() {
        let crate_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("crates")
            .join("oxijolt-sys");
        for family in FAMILIES {
            for (dp, dr) in [(false, false), (false, true), (true, false), (true, true)] {
                let path = crate_dir.join(bindings_file(family.name, dp, dr));
                assert!(path.is_file(), "{} is missing", path.display());
            }
        }
        assert!(crate_dir.join(INPUTS_FILE).is_file());
    }
}
