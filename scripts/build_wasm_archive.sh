#!/usr/bin/env bash
# Builds one wasm32-unknown-unknown release archive of oxijolt-sys, as the Release workflow does.
#
# Usage: bash scripts/build_wasm_archive.sh <features> <dist>
#
# <features> is empty or a comma-separated subset of double-precision,
# cross-platform-deterministic and debug-renderer. The script builds the native prefix with
# wasi-sdk (which copies the SDK's runtime libraries into it), adds the licences of oxijolt,
# joltc and Jolt and those of the bundled runtime (downloaded at the commits wasi-sdk 34.0 was
# built from and checked against the sha256 below), writes PROVENANCE.txt, and packs
# <dist>/oxijolt-sys-<version>-wasm32-unknown-unknown-<subset>.tar.gz with its .sha256 file.
# It then unpacks the archive and builds the raw wasm fixture against it the way a user would:
# without WASI_SDK_PATH and with CMake and the C++ compiler missing, and runs it in Node.
#
# Needs WASI_SDK_PATH (wasi-sdk 34.0), Ninja, Node, curl, tar and cargo with the
# wasm32-unknown-unknown target. The crate is $CRATE (default oxijolt-sys); work files go under
# $RUNNER_TEMP (default: target/).
set -euo pipefail

features=${1?usage: build_wasm_archive.sh <features> <dist>}
dist=${2:?usage: build_wasm_archive.sh <features> <dist>}
repo=$(cd "$(dirname "$0")/.." && pwd)
: "${WASI_SDK_PATH:?set WASI_SDK_PATH to a wasi-sdk 34.0 install}"
for tool in ninja node cargo curl tar; do
  command -v "$tool" > /dev/null || { echo "build_wasm_archive: $tool not found" >&2; exit 1; }
done

crate=${CRATE:-oxijolt-sys}
target=wasm32-unknown-unknown
manifest_file=oxijolt-sys-manifest.txt
base=${RUNNER_TEMP:-$repo/target}
mkdir -p "$base" "$dist"
work=$(cd "$base" && pwd)/wasm-archive
dist=$(cd "$dist" && pwd)

fail() {
  echo "build_wasm_archive: $*" >&2
  exit 1
}

# A path for native programs (cargo, node), which do not read MSYS paths on Windows.
native() {
  if command -v cygpath > /dev/null; then cygpath -m "$1"; else printf '%s\n' "$1"; fi
}

sha256_of() {
  if command -v sha256sum > /dev/null; then sha256sum "$1"; else shasum -a 256 "$1"; fi | cut -d ' ' -f 1
}

# The licence texts of the bundled runtime: file name under licenses/, source, sha256. The
# sources are the wasi-sdk-34 tag and the wasi-libc and LLVM commits its VERSION file names.
wasi_sdk_commit=5a0bf653a1a06e1c18867567c5937006d3394a69
wasi_libc_commit=2e6fb9d8ee0cdf9e431fbcabe8af3115de000a13
llvm_commit=895aa2c896ada719451be2e3673c83da8ddf1141
raw=https://raw.githubusercontent.com
sdk_src="$raw/WebAssembly/wasi-sdk/$wasi_sdk_commit"
libc_src="$raw/WebAssembly/wasi-libc/$wasi_libc_commit"
llvm_src="$raw/llvm/llvm-project/$llvm_commit"
licenses="
wasi-sdk-LICENSE $sdk_src/LICENSE 268872b9816f90fd8e85db5a28d33f8150ebb8dd016653fb39ef1f94f2686bc5
libcxx-LICENSE.TXT $llvm_src/libcxx/LICENSE.TXT 539dd7aed86e8a4f12cbdd0e6c50c189c7d74847e4fecc64ce2c6ee3a01da38b
libcxxabi-LICENSE.TXT $llvm_src/libcxxabi/LICENSE.TXT e2b35be49f7284a45b7baca8fc7b3ab7440e7902392b2528a457816b5bb2a15c
compiler-rt-LICENSE.TXT $llvm_src/compiler-rt/LICENSE.TXT 1a8f1058753f1ba890de984e48f0242a3a5c29a6a8f2ed9fd813f36985387e8d
wasi-libc-LICENSE $libc_src/LICENSE 2711a8b5a5cdfef0e639f96c1aca12ae23d7d64a02d0507f1bdf14d2b27bbc3a
wasi-libc-LICENSE-APACHE $libc_src/LICENSE-APACHE a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2
wasi-libc-LICENSE-APACHE-LLVM $libc_src/LICENSE-APACHE-LLVM 268872b9816f90fd8e85db5a28d33f8150ebb8dd016653fb39ef1f94f2686bc5
wasi-libc-LICENSE-MIT $libc_src/LICENSE-MIT 23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3
wasi-libc-musl-COPYRIGHT $libc_src/libc-top-half/musl/COPYRIGHT b870108ec5e7790e9f9919064f1b9421d62d5f9b0e6c230c6adf7ea2da62e97b
wasi-libc-cloudlibc-LICENSE $libc_src/libc-bottom-half/cloudlibc/LICENSE c8b789cf5a746611e6300a0cc7750dbf92b61912a709d04e639245f7290656d0
wasi-libc-musl-fts-COPYING $libc_src/fts/musl-fts/COPYING 55af87e4017668f54467a3380e7ebbac5e672d8c763bfe95e6fc882a6fdc4046
"

# The licences above belong to this SDK only.
sdk=$WASI_SDK_PATH
[ "$(head -n 1 "$sdk/VERSION" | tr -d '\r')" = 34.0 ] || fail "$sdk is not wasi-sdk 34.0"
grep -q "^wasi-libc: ${wasi_libc_commit:0:12}" "$sdk/VERSION" || fail "$sdk has another wasi-libc"
grep -q "^llvm: ${llvm_commit:0:12}" "$sdk/VERSION" || fail "$sdk has another LLVM"

version=$(sed -n 's/^version = "\([^"+]*\).*/\1/p' "$repo/crates/$crate/Cargo.toml" | head -n 1)
subset=${features:-default}
name="$crate-$version-$target-${subset//,/+}"
rm -rf "$work/stage" "$work/u" "$work/m" "$work/t"
mkdir -p "$work/stage/$name/licenses" "$work/licenses" "$work/u"

# A fresh target directory, so exactly one native prefix exists.
(cd "$repo" && env -u JOLTC_LIB_DIR JOLTC_PREBUILT=off CARGO_TARGET_DIR="$(native "$work/t")" \
  cargo build --locked -p "$crate" --release --target "$target" --features "$features")
shopt -s nullglob
prefixes=("$work/t/$target/release/build/$crate"-*/out/joltc)
shopt -u nullglob
[ ${#prefixes[@]} -eq 1 ] || fail "expected exactly one native prefix, found ${#prefixes[@]}"
prefix=${prefixes[0]}

stage="$work/stage/$name"
cp -R "$prefix/lib" "$prefix/include" "$prefix/$manifest_file" "$stage/"
for library in libc++.a libc++abi.a libc.a libclang_rt.builtins.a; do
  [ -f "$stage/lib/$library" ] || fail "the prefix does not bundle $library"
done
cp "$repo/LICENSE-MIT" "$repo/LICENSE-APACHE" "$stage/"
cp "$repo/crates/$crate/vendor/joltc/LICENSE" "$stage/licenses/joltc-LICENSE"
cp "$repo/crates/$crate/vendor/JoltPhysics/LICENSE" "$stage/licenses/JoltPhysics-LICENSE"
while read -r file url sha256; do
  [ -n "$file" ] || continue
  cached="$work/licenses/$file"
  if [ ! -f "$cached" ] || [ "$(sha256_of "$cached")" != "$sha256" ]; then
    curl -fsSL --retry 3 --proto '=https' -A "oxijolt-release (github.com/pockerhead/oxijolt)" \
      -o "$cached" "$url"
  fi
  [ "$(sha256_of "$cached")" = "$sha256" ] || fail "$url does not have the pinned sha256"
  cp "$cached" "$stage/licenses/$file"
done <<< "$licenses"

compiler=$(sed -nE 's/^set\(CMAKE_CXX_COMPILER_(ID|VERSION) "(.*)"\)$/\2/p' \
  "$prefix"/build/CMakeFiles/*/CMakeCXXCompiler.cmake | tr '\n' ' ')
{
  echo "source: ${GITHUB_REPOSITORY:-local}@$(git -C "$repo" rev-parse HEAD)"
  if [ -n "${ImageOS:-}" ]; then
    echo "runner image: $ImageOS ${ImageVersion:-}"
  else
    echo "runner image: local $(uname -s)"
  fi
  echo "compiler: $compiler"
  echo "cargo target: $target"
  echo "features: ${features:-none}"
  grep -E '^(joltc_commit|jolt_commit|jolt_version|joltc_ext|wasi_sdk)=' "$prefix/$manifest_file"
  echo "bundled runtime: libc++ and libc++abi (noeh), wasi-libc and clang_rt.builtins of wasi-sdk 34.0"
  echo "Jolt asserts: OFF"
} > "$stage/PROVENANCE.txt"
cat "$stage/PROVENANCE.txt"

# Relative paths: GNU tar would read a Windows drive letter as a remote host.
(cd "$work/stage" && tar -czf "$name.tar.gz" "$name")
mv "$work/stage/$name.tar.gz" "$dist/"
(cd "$dist" && echo "$(sha256_of "$name.tar.gz")  $name.tar.gz" > "$name.tar.gz.sha256" && cat "$name.tar.gz.sha256")

# The archive as a user gets it: no SDK, no CMake, no C++ compiler.
(cd "$work/u" && tar -xzf "$dist/$name.tar.gz")
consumer() {
  (cd "$repo" && env -u WASI_SDK_PATH -u JOLTC_PREBUILT CMAKE=oxijolt-no-cmake CXX=oxijolt-no-cxx \
    CARGO_TERM_COLOR=never CARGO_TARGET_DIR="$(native "$work/s")" "$@")
}
log="$work/smoke.log"
fixture_features=$(echo "$features" | sed "s/[^,][^,]*/$crate\/&/g")
consumer env JOLTC_LIB_DIR="$(native "$work/u/$name")" cargo rustc --locked -p oxijolt-sys-wasm-raw \
  --target "$target" --features "$fixture_features" -vv -- -C link-arg=--fatal-warnings > "$log" 2>&1 ||
  { tail -50 "$log"; fail "the fixture did not build against the archive"; }
cmake_runs=$(grep -c "Build files have been written" "$log" || true)
[ "$cmake_runs" -eq 0 ] || fail "the smoke build ran CMake $cmake_runs times"
node "$repo/tests/wasm-raw/run.mjs" "$(native "$work/s/$target/debug/oxijolt_sys_wasm_raw.wasm")"

# A wasm prefix must be refused for a configuration it was not built for.
if [ -z "$features" ]; then
  cp -R "$work/u/$name" "$work/m"
  sed 's/^double_precision=OFF$/double_precision=ON/' "$work/m/$manifest_file" > "$work/m/$manifest_file.tmp"
  mv "$work/m/$manifest_file.tmp" "$work/m/$manifest_file"
  grep -q '^double_precision=ON$' "$work/m/$manifest_file"
  if consumer env JOLTC_LIB_DIR="$(native "$work/m")" cargo check --locked -p "$crate" --target "$target" \
    > "$work/mismatch.log" 2>&1; then
    fail "the mismatched prefix was accepted"
  fi
  grep "built for a different configuration" "$work/mismatch.log"
fi
echo "build_wasm_archive: $dist/$name.tar.gz"
