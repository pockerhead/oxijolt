//! Targets with committed bindings, and the fingerprint record that ties those bindings to the
//! headers and generator settings they were produced from.
//!
//! Shared by `build.rs` and the `xtask` regenerator through `#[path]`; it has no dependencies.

use std::fmt;

/// Targets whose bindgen output is byte-identical, so one committed file per configuration
/// serves all of them.
///
/// The 64-bit families differ because C compilers disagree on the type of a C enum: MSVC types
/// every enum as `int`, Clang and GCC as `unsigned int` when every value is non-negative.
/// `wasm32` types enums like `gnu` but has 32-bit pointers, which changes the layout of every
/// struct that holds a pointer.
///
/// Membership means the regenerator produced identical bindings for the target; it does not
/// mean the native build of that target is tested.
pub struct Family {
    /// Directory of the family under `src/bindings/`.
    pub name: &'static str,
    /// Rust target triples of the family.
    pub targets: &'static [&'static str],
    /// Cargo's `target_pointer_width` of every target of the family.
    pub pointer_width: &'static str,
}

/// Every registered target, by family.
pub const FAMILIES: &[Family] = &[
    Family {
        name: "msvc",
        targets: &["x86_64-pc-windows-msvc", "aarch64-pc-windows-msvc"],
        pointer_width: "64",
    },
    Family {
        name: "gnu",
        targets: &[
            "x86_64-unknown-linux-gnu",
            "aarch64-unknown-linux-gnu",
            "x86_64-apple-darwin",
            "aarch64-apple-darwin",
            "x86_64-linux-android",
            "aarch64-linux-android",
            "x86_64-pc-windows-gnu",
        ],
        pointer_width: "64",
    },
    Family {
        name: "wasm32",
        targets: &[
            "wasm32-unknown-unknown",
            "wasm32-unknown-emscripten",
            "wasm32-wasip1",
        ],
        pointer_width: "32",
    },
];

/// The family of a registered target.
pub fn family_of(target: &str) -> Option<&'static Family> {
    FAMILIES
        .iter()
        .find(|family| family.targets.contains(&target))
}

/// Why a target has no committed bindings.
#[derive(Debug, PartialEq, Eq)]
pub enum TargetError {
    /// The target's pointers are not 64 bits wide and it is not in [`FAMILIES`], or its
    /// pointer width is not its family's.
    PointerWidth(String),
    /// The target is 64-bit but not in [`FAMILIES`].
    Unregistered(String),
}

impl fmt::Display for TargetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TargetError::PointerWidth(target) => {
                write!(
                    f,
                    "{target} is not supported: only 64-bit and wasm32 targets are"
                )
            }
            TargetError::Unregistered(target) => write!(
                f,
                "{target} is not supported yet: a target is added by verifying its bindings \
                 in the registry in build/targets.rs"
            ),
        }
    }
}

impl std::error::Error for TargetError {}

/// The family of `target`, given Cargo's `target_pointer_width` for it.
pub fn check_target(target: &str, pointer_width: &str) -> Result<&'static Family, TargetError> {
    match family_of(target) {
        Some(family) if family.pointer_width == pointer_width => Ok(family),
        None if pointer_width == "64" => Err(TargetError::Unregistered(target.to_owned())),
        _ => Err(TargetError::PointerWidth(target.to_owned())),
    }
}

/// Path of a committed bindings file, relative to the crate.
pub fn bindings_file(family: &str, double_precision: bool, debug_renderer: bool) -> String {
    let precision = if double_precision { "double" } else { "single" };
    let renderer = if debug_renderer {
        "_debug_renderer"
    } else {
        ""
    };
    format!("src/bindings/{family}/{precision}_precision{renderer}.rs")
}

/// Fingerprint record of the committed bindings, relative to the crate.
pub const INPUTS_FILE: &str = "src/bindings/inputs.txt";

/// Generator sources whose content shapes the bindings, relative to the crate.
pub const POLICY_SOURCES: &[&str] = &["build/bindgen_options.rs", "build/targets.rs"];

/// `bytes` with every `\r\n` replaced by `\n`; a lone `\r` stays.
pub fn normalize_crlf(bytes: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(bytes.len());
    let mut iter = bytes.iter().copied().peekable();
    while let Some(byte) = iter.next() {
        if byte == b'\r' && iter.peek() == Some(&b'\n') {
            continue;
        }
        out.push(byte);
    }
    out
}

/// FNV-1a 64 of `bytes` with CRLF line endings normalized.
///
/// A checksum that notices a stale bindings file, not an integrity check.
pub fn fingerprint(bytes: &[u8]) -> u64 {
    normalize_crlf(bytes)
        .iter()
        .fold(0xcbf2_9ce4_8422_2325, |hash, &byte| {
            (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3)
        })
}

/// The fingerprint lines of [`INPUTS_FILE`]: the format, both headers, then each entry of
/// `policy` (path and content, in [`POLICY_SOURCES`] order). LF line endings.
pub fn input_fingerprints(joltc_h: &[u8], joltc_ext_h: &[u8], policy: &[(&str, &[u8])]) -> String {
    let mut text = String::from("format=1\n");
    text += &format!("joltc.h={:016x}\n", fingerprint(joltc_h));
    text += &format!("joltc_ext.h={:016x}\n", fingerprint(joltc_ext_h));
    for (path, bytes) in policy {
        text += &format!("{path}={:016x}\n", fingerprint(bytes));
    }
    text
}

/// Keys of [`INPUTS_FILE`] lines that record the generator's versions. They are informational:
/// a regeneration with another libclang may change them without making the bindings stale.
pub const PROVENANCE_KEYS: &[&str] = &["bindgen", "libclang"];

/// Keys whose fingerprint differs between `expected` and the recorded [`INPUTS_FILE`] text,
/// ignoring provenance lines and line endings. Empty when the bindings are current.
pub fn differing_keys(expected: &str, recorded: &str) -> Vec<String> {
    let entries = |text: &str| -> Vec<(String, String)> {
        text.lines()
            .filter_map(|line| line.split_once('='))
            .filter(|(key, _)| !PROVENANCE_KEYS.contains(key))
            .map(|(key, value)| (key.to_owned(), value.to_owned()))
            .collect()
    };
    let expected = entries(expected);
    let recorded = entries(recorded);
    let value_in = |entries: &[(String, String)], key: &str| {
        entries
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.clone())
    };

    let mut keys: Vec<String> = Vec::new();
    for (key, _) in expected.iter().chain(&recorded) {
        if !keys.contains(key) && value_in(&expected, key) != value_in(&recorded, key) {
            keys.push(key.clone());
        }
    }
    keys
}
