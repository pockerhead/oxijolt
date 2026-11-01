//! The wasi-sdk toolchain that builds Jolt and joltc for `wasm32-unknown-unknown`: where its
//! CMake toolchain file and runtime libraries are, and the SDK identity the manifest records.
//!
//! Pure functions, so the `xtask` tests can check them without an SDK. Shared with those tests
//! through `#[path]`; it has no dependencies.

use std::fmt;

/// Environment variable that names the wasi-sdk install directory.
pub const WASI_SDK_PATH_VAR: &str = "WASI_SDK_PATH";

/// CMake generator for wasm: the SDK toolchain file needs a single-config generator, which the
/// Visual Studio generator a Windows host would pick is not.
pub const WASI_GENERATOR: &str = "Ninja";

/// The wasi-sdk release oxijolt-sys is tested with; others build with a warning.
pub const TESTED_SDK_VERSION: &str = "34.0";

/// Oldest Rust release that builds for `wasm32-unknown-unknown`: the first with the standard C
/// ABI on that target, which the C++ objects use.
pub const MIN_RUSTC: (u32, u32) = (1, 89);

/// The SDK libraries Jolt's C++ code needs at link time, in link order: libc++ and libc++abi of
/// the `noeh` variant (Jolt builds without exceptions), wasi-libc and clang's builtins.
pub const RUNTIME_LIBRARIES: [&str; 4] = ["c++", "c++abi", "c", "clang_rt.builtins"];

/// `(major, minor)` of `rustc --version` output such as `rustc 1.89.0 (29483883e 2025-08-04)`.
pub fn rustc_version(output: &str) -> Option<(u32, u32)> {
    let mut words = output.split_whitespace();
    if words.next()? != "rustc" {
        return None;
    }
    let release = words.next()?.split('-').next()?;
    let mut parts = release.split('.');
    let major = parts.next()?.parse().ok()?;
    let minor = parts.next()?.parse().ok()?;
    Some((major, minor))
}

/// Why the wasi-sdk could not be used.
#[derive(Debug, PartialEq)]
pub enum WasiSdkError {
    /// [`WASI_SDK_PATH_VAR`] is unset or empty.
    NotSet,
    /// The SDK's `VERSION` file does not read as a wasi-sdk's.
    NotAnSdk(String),
    /// A file the build needs is not in the SDK.
    Missing(String),
}

impl fmt::Display for WasiSdkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            WasiSdkError::NotSet => write!(
                f,
                "building for wasm32-unknown-unknown needs wasi-sdk: set {WASI_SDK_PATH_VAR} to its install directory"
            ),
            WasiSdkError::NotAnSdk(reason) => {
                write!(f, "{WASI_SDK_PATH_VAR} is not a wasi-sdk install: {reason}")
            }
            WasiSdkError::Missing(path) => write!(f, "wasi-sdk file missing: {path}"),
        }
    }
}

impl std::error::Error for WasiSdkError {}

/// What a wasi-sdk's `VERSION` file says about it: the release, then `key: value` lines naming
/// the wasi-libc, LLVM and configuration it was built from.
#[derive(Debug, PartialEq)]
pub struct SdkIdentity {
    version: String,
    llvm_major: String,
    manifest_value: String,
}

impl SdkIdentity {
    /// Reads the contents of the SDK's `VERSION` file. `wasi-libc`, `llvm` and `llvm-version`
    /// lines are required; keys may not repeat.
    pub fn parse(version_file: &str) -> Result<Self, WasiSdkError> {
        let not_an_sdk = |reason: &str| WasiSdkError::NotAnSdk(reason.to_owned());
        let mut lines = version_file
            .lines()
            .map(str::trim)
            .filter(|line| !line.is_empty());
        let version = lines.next().ok_or_else(|| not_an_sdk("empty VERSION"))?;
        if version.contains(':') {
            return Err(not_an_sdk("VERSION does not start with the release"));
        }

        let mut fields: Vec<(&str, &str)> = Vec::new();
        for line in lines {
            let (key, value) = line
                .split_once(':')
                .map(|(key, value)| (key.trim(), value.trim()))
                .filter(|(key, value)| !key.is_empty() && !value.is_empty())
                .ok_or_else(|| WasiSdkError::NotAnSdk(format!("malformed line {line:?}")))?;
            if fields.iter().any(|(seen, _)| *seen == key) {
                return Err(WasiSdkError::NotAnSdk(format!("duplicate key {key:?}")));
            }
            fields.push((key, value));
        }

        let field = |key: &str| {
            fields
                .iter()
                .find(|(seen, _)| *seen == key)
                .map(|(_, value)| *value)
                .ok_or_else(|| WasiSdkError::NotAnSdk(format!("no {key} line")))
        };
        field("wasi-libc")?;
        field("llvm")?;
        let llvm_major = field("llvm-version")?.split('.').next().unwrap_or_default();
        if llvm_major.is_empty() || !llvm_major.bytes().all(|b| b.is_ascii_digit()) {
            return Err(not_an_sdk("llvm-version does not start with a number"));
        }

        let manifest_value = std::iter::once(version.to_owned())
            .chain(fields.iter().map(|(key, value)| format!("{key}:{value}")))
            .collect::<Vec<_>>()
            .join(" ");
        Ok(SdkIdentity {
            version: version.to_owned(),
            llvm_major: llvm_major.to_owned(),
            manifest_value,
        })
    }

    /// The SDK release, such as `34.0`.
    pub fn version(&self) -> &str {
        &self.version
    }

    /// Major version of the SDK's LLVM, which names its clang resource directory.
    pub fn llvm_major(&self) -> &str {
        &self.llvm_major
    }

    /// Every `VERSION` line on one line, as the manifest's `wasi_sdk` value.
    pub fn manifest_value(&self) -> &str {
        &self.manifest_value
    }
}

/// A wasi-sdk install directory, with forward slashes.
#[derive(Debug, PartialEq)]
pub struct WasiSdk {
    root: String,
}

impl WasiSdk {
    /// Reads the install directory from [`WASI_SDK_PATH_VAR`] through `env_var`.
    pub fn locate(env_var: impl Fn(&str) -> Option<String>) -> Result<Self, WasiSdkError> {
        let value = env_var(WASI_SDK_PATH_VAR).unwrap_or_default();
        let root = value.trim().replace('\\', "/");
        let root = root.trim_end_matches('/');
        if root.is_empty() {
            return Err(WasiSdkError::NotSet);
        }
        Ok(WasiSdk {
            root: root.to_owned(),
        })
    }

    /// The SDK's `VERSION` file.
    pub fn version_file(&self) -> String {
        format!("{}/VERSION", self.root)
    }

    /// The SDK's CMake toolchain file for WASI preview 1, which Jolt and joltc build against.
    pub fn toolchain_file(&self) -> String {
        format!("{}/share/cmake/wasi-sdk-p1.cmake", self.root)
    }

    /// The cache entries that route the build through the SDK, in configure order.
    pub fn cmake_options(&self) -> [(&'static str, String); 2] {
        [
            ("CMAKE_TOOLCHAIN_FILE", self.toolchain_file()),
            ("WASI_SDK_PREFIX", self.root.clone()),
        ]
    }

    /// Directories of [`RUNTIME_LIBRARIES`], the `noeh` one first.
    pub fn link_search_dirs(&self, id: &SdkIdentity) -> [String; 3] {
        let lib = format!("{}/share/wasi-sysroot/lib/wasm32-wasip1", self.root);
        [format!("{lib}/noeh"), lib, self.builtins_dir(id)]
    }

    /// Every SDK file the build reads or links, as full paths.
    pub fn required_files(&self, id: &SdkIdentity) -> [String; 5] {
        let [noeh, lib, builtins] = self.link_search_dirs(id);
        [
            self.toolchain_file(),
            format!("{noeh}/libc++.a"),
            format!("{noeh}/libc++abi.a"),
            format!("{lib}/libc.a"),
            format!("{builtins}/libclang_rt.builtins.a"),
        ]
    }

    fn builtins_dir(&self, id: &SdkIdentity) -> String {
        format!(
            "{}/lib/clang/{}/lib/wasm32-unknown-wasip1",
            self.root,
            id.llvm_major()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SDK_34_VERSION: &str = "34.0\nwasi-libc: 2e6fb9d8ee0c\nllvm: 895aa2c896ad\nllvm-version: 23.1.0\nconfig: f992bcc08219\n";

    fn sdk(value: &'static str) -> Result<WasiSdk, WasiSdkError> {
        WasiSdk::locate(move |name| (name == "WASI_SDK_PATH").then(|| value.to_owned()))
    }

    #[test]
    fn locate_needs_a_non_empty_path() {
        assert_eq!(WasiSdk::locate(|_| None), Err(WasiSdkError::NotSet));
        assert_eq!(sdk(""), Err(WasiSdkError::NotSet));
        assert_eq!(sdk("  "), Err(WasiSdkError::NotSet));
    }

    #[test]
    fn locate_uses_forward_slashes_without_a_trailing_one() {
        assert_eq!(sdk("C:\\sdk\\").unwrap().root, "C:/sdk");
        assert_eq!(sdk("/opt/wasi-sdk/").unwrap().root, "/opt/wasi-sdk");
        let spaced = sdk("C:\\Program Files\\wasi sdk").unwrap();
        assert_eq!(spaced.root, "C:/Program Files/wasi sdk");
        assert_eq!(
            spaced.toolchain_file(),
            "C:/Program Files/wasi sdk/share/cmake/wasi-sdk-p1.cmake"
        );
    }

    #[test]
    fn sdk_34_paths() {
        let sdk = sdk("/opt/wasi-sdk").unwrap();
        let id = SdkIdentity::parse(SDK_34_VERSION).unwrap();
        assert_eq!(sdk.version_file(), "/opt/wasi-sdk/VERSION");
        assert_eq!(
            sdk.cmake_options(),
            [
                (
                    "CMAKE_TOOLCHAIN_FILE",
                    "/opt/wasi-sdk/share/cmake/wasi-sdk-p1.cmake".to_owned()
                ),
                ("WASI_SDK_PREFIX", "/opt/wasi-sdk".to_owned()),
            ]
        );
        assert_eq!(
            sdk.link_search_dirs(&id),
            [
                "/opt/wasi-sdk/share/wasi-sysroot/lib/wasm32-wasip1/noeh".to_owned(),
                "/opt/wasi-sdk/share/wasi-sysroot/lib/wasm32-wasip1".to_owned(),
                "/opt/wasi-sdk/lib/clang/23/lib/wasm32-unknown-wasip1".to_owned(),
            ]
        );
        assert_eq!(
            sdk.required_files(&id),
            [
                "/opt/wasi-sdk/share/cmake/wasi-sdk-p1.cmake".to_owned(),
                "/opt/wasi-sdk/share/wasi-sysroot/lib/wasm32-wasip1/noeh/libc++.a".to_owned(),
                "/opt/wasi-sdk/share/wasi-sysroot/lib/wasm32-wasip1/noeh/libc++abi.a".to_owned(),
                "/opt/wasi-sdk/share/wasi-sysroot/lib/wasm32-wasip1/libc.a".to_owned(),
                "/opt/wasi-sdk/lib/clang/23/lib/wasm32-unknown-wasip1/libclang_rt.builtins.a"
                    .to_owned(),
            ]
        );
        assert_eq!(
            RUNTIME_LIBRARIES,
            ["c++", "c++abi", "c", "clang_rt.builtins"]
        );
        assert_eq!(WASI_GENERATOR, "Ninja");
    }

    #[test]
    fn sdk_34_version_file() {
        let id = SdkIdentity::parse(SDK_34_VERSION).unwrap();
        assert_eq!(id.version(), "34.0");
        assert_eq!(id.version(), TESTED_SDK_VERSION);
        assert_eq!(id.llvm_major(), "23");
        assert_eq!(
            id.manifest_value(),
            "34.0 wasi-libc:2e6fb9d8ee0c llvm:895aa2c896ad llvm-version:23.1.0 config:f992bcc08219"
        );
        let crlf = SDK_34_VERSION.replace('\n', "\r\n");
        assert_eq!(SdkIdentity::parse(&crlf), Ok(id));
    }

    #[test]
    fn version_files_that_are_not_an_sdks_are_refused() {
        let refused =
            |text: &str| matches!(SdkIdentity::parse(text), Err(WasiSdkError::NotAnSdk(_)));
        assert!(refused(""));
        assert!(refused("34.0\nwasi-libc: a\nllvm: b\n"));
        assert!(refused(
            "34.0\nwasi-libc: a\nllvm: b\nllvm-version: 23.1.0\nllvm: c\n"
        ));
        assert!(refused(
            "34.0\nwasi-libc: a\nllvm b\nllvm-version: 23.1.0\n"
        ));
        assert!(refused("wasi-libc: a\nllvm: b\nllvm-version: 23.1.0\n"));
        assert!(refused("34.0\nwasi-libc: a\nllvm: b\nllvm-version: x.1\n"));
        assert_eq!(
            SdkIdentity::parse("34.0\nwasi-libc: a\nllvm: b\n"),
            Err(WasiSdkError::NotAnSdk("no llvm-version line".to_owned()))
        );
    }

    #[test]
    fn rustc_versions() {
        assert_eq!(
            rustc_version("rustc 1.88.0 (6b00bc388 2025-06-23)"),
            Some((1, 88))
        );
        assert!(rustc_version("rustc 1.88.0 (6b00bc388 2025-06-23)").unwrap() < MIN_RUSTC);
        assert_eq!(
            rustc_version("rustc 1.89.0 (29483883e 2025-08-04)"),
            Some(MIN_RUSTC)
        );
        assert_eq!(
            rustc_version("rustc 1.90.0-nightly (abc 2025-07-01)"),
            Some((1, 90))
        );
        assert_eq!(
            rustc_version("rustc 1.95.0 (59807616e 2026-04-14)\n"),
            Some((1, 95))
        );
        assert_eq!(rustc_version("garbage"), None);
        assert_eq!(rustc_version(""), None);
    }
}
