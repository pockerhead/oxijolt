#!/usr/bin/env bash
# Checks how the oxijolt-sys build script uses a release archive, against a local server.
#
# Usage: bash scripts/check_prebuilt_download.sh <prefix>
#
# Hosts: Windows (MSVC), Linux (GNU) and macOS on Apple silicon.
#
# <prefix> is a native prefix for the host target with the default features (what
# JOLTC_LIB_DIR takes). The script packs it as a release archive, writes an archive list for
# it into crates/oxijolt-sys/prebuilt.txt (restored on exit), serves the archive with
# `python -m http.server` on 127.0.0.1 and runs cargo for every way the build script can
# take or refuse the archive: each fallback in auto mode (a warning, then the source build)
# and in require mode (an error), the precedence of docs.rs and JOLTC_LIB_DIR, the download
# itself, the cache in OUT_DIR and its integrity check. The server log counts the requests.
# Every list the cases use is written here, so the committed list may hold any release's.
#
# Every cargo run has CMAKE and CXX pointing at programs that do not exist, so a source build
# fails as soon as it starts CMake, and a success proves that the download path needs neither.
# Requires Bash 3.2 or newer, cargo, curl, tar, python (3) and a checkout with its submodules.
set -euo pipefail

prefix=${1:?usage: check_prebuilt_download.sh <prefix>}
repo=$(cd "$(dirname "$0")/.." && pwd)
base=${RUNNER_TEMP:-$repo/target}
mkdir -p "$base"
work=$(cd "$base" && pwd)/prebuilt-check
list="$repo/crates/oxijolt-sys/prebuilt.txt"
checksum_file="$repo/crates/oxijolt-sys/.cargo-checksum.json"

# A path for native programs (python, cargo), which do not read MSYS paths on Windows.
native() {
  if command -v cygpath > /dev/null; then cygpath -m "$1"; else printf '%s\n' "$1"; fi
}

# Rewrites a file with a sed script (BSD sed has no portable in-place edit).
edit() {
  sed "$1" "$2" > "$2.tmp" && mv "$2.tmp" "$2"
}

sha256_of() {
  if command -v sha256sum > /dev/null; then sha256sum "$1"; else shasum -a 256 "$1"; fi | cut -d ' ' -f 1
}

# A gzipped tar without macOS metadata (AppleDouble `._` entries), which the build script refuses.
pack() {
  if [ "$os" = macos ]; then
    COPYFILE_DISABLE=1 tar --no-mac-metadata --no-xattrs -czf "$@"
  else
    tar -czf "$@"
  fi
}

python=$(command -v python3 || command -v python)
server_pid=
server_log=

stop_server() {
  if [ -n "$server_pid" ]; then
    kill "$server_pid" 2> /dev/null || true
    wait "$server_pid" 2> /dev/null || true
    server_pid=
  fi
}

cleanup() {
  stop_server
  rm -f "$checksum_file"
  if [ -f "$work/prebuilt.txt.orig" ]; then
    cp "$work/prebuilt.txt.orig" "$list"
    touch "$list"
  fi
}

rm -rf "$work/www" "$work/stage" "$work"/*.log
mkdir -p "$work/www/good" "$work/www/bad" "$work/www/refused" "$work/stage" "$work/empty-bin"
cp "$list" "$work/prebuilt.txt.orig"
trap cleanup EXIT

# Starts the server on a free port; sets `url` to its address and starts a new request log.
start_server() {
  server_log="$work/server-$RANDOM.log"
  : > "$server_log"
  "$python" -u -m http.server 0 --bind 127.0.0.1 --directory "$(native "$work/www")" \
    > "$server_log" 2>&1 &
  server_pid=$!
  local port=
  # Up to a minute: the server prints its port only after a reverse lookup of its address
  # (`socket.getfqdn`), which can take seconds on macOS.
  for _ in $(seq 600); do
    port=$(sed -n 's/.*port \([0-9]*\).*/\1/p' "$server_log" | head -n 1)
    [ -n "$port" ] && break
    sleep 0.1
  done
  [ -n "$port" ] || { echo "the server did not start"; cat "$server_log"; exit 1; }
  url="http://127.0.0.1:$port"
}

requests() {
  if [ -n "$server_log" ]; then grep -c '"GET ' "$server_log" || true; else echo 0; fi
}

# The fixture: the prefix packed as a release archive of this version for the host.
version=$(sed -n 's/^version = "\([^"+]*\).*/\1/p' "$repo/crates/oxijolt-sys/Cargo.toml" | head -n 1)
host=$(rustc -vV | sed -n 's/^host: //p')
head=$(git -C "$repo" rev-parse HEAD)
name="oxijolt-sys-$version-$host-default"
stage="$work/stage/$name"
mkdir -p "$stage/licenses"
cp -R "$prefix/lib" "$prefix/include" "$prefix/oxijolt-sys-manifest.txt" "$stage/"
cp "$repo/LICENSE-MIT" "$repo/LICENSE-APACHE" "$stage/"
cp "$repo/crates/oxijolt-sys/vendor/joltc/LICENSE" "$stage/licenses/joltc-LICENSE"
cp "$repo/crates/oxijolt-sys/vendor/JoltPhysics/LICENSE" "$stage/licenses/JoltPhysics-LICENSE"
case "$host" in
  *-windows-msvc) os=windows; provenance="compiler: MSVC 19.0.0" ;;
  *-linux-gnu) os=linux; provenance=$'compiler: GNU 12\nglibc: 2.17' ;;
  *-apple-darwin) os=macos; provenance=$'compiler: AppleClang 17.0.0\nmacos: 11.0' ;;
  *) echo "no archives for $host"; exit 1 ;;
esac
printf 'source: test@%s\n%s\n' "$head" "$provenance" > "$stage/PROVENANCE.txt"
(cd "$work/stage" && pack "../www/good/$name.tar.gz" "$name")
cp "$work/www/good/$name.tar.gz" "$work/www/bad/$name.tar.gz"
"$python" -c 'import sys; p = sys.argv[1]; b = bytearray(open(p, "rb").read()); b[-1] ^= 1; open(p, "wb").write(b)' \
  "$(native "$work/www/bad/$name.tar.gz")"
joltc_lib=$(ls "$stage/lib" | grep -i joltc)
# The same archive with a manifest of another configuration: it passes the checksum of its own
# list (below) and is refused as a prefix.
refused_stage="$work/stage/refused/$name"
rm -rf "$work/stage/refused"
mkdir -p "$work/stage/refused"
cp -R "$stage" "$refused_stage"
edit 's/^double_precision=OFF/double_precision=ON/' "$refused_stage/oxijolt-sys-manifest.txt"
(cd "$work/stage/refused" && pack "../../www/refused/$name.tar.gz" "$name")

# Every cargo child starts from the same environment: nothing of the caller's archive
# settings leaks in, CMake and the C++ compiler are missing, the local server is not proxied,
# and the output has no colour codes for the checks to trip over. Arrays that may be empty
# expand as ${a[@]+"${a[@]}"}: before 4.4, Bash calls an empty "${a[@]}" unbound under `set -u`.
case_env=()
cargo_in() {
  local dir=$1
  shift
  env -u JOLTC_LIB_DIR -u JOLTC_PREBUILT -u JOLTC_PREBUILT_URL -u CARGO_NET_OFFLINE \
    -u NIX_BUILD_TOP -u RUSTC_LINKER -u DOCS_RS -u RUSTFLAGS -u CARGO_TARGET_DIR \
    CMAKE=oxijolt-no-cmake CXX=oxijolt-no-cxx NO_PROXY=127.0.0.1 no_proxy=127.0.0.1 \
    CARGO_TERM_COLOR=never CARGO_TARGET_DIR="$(native "$work/$dir")" ${case_env[@]+"${case_env[@]}"} cargo "$@"
}

(cd "$repo" && env -u CARGO_TARGET_DIR cargo fetch --locked > /dev/null)
(cd "$repo" && env -u CARGO_TARGET_DIR CARGO_TARGET_DIR="$(native "$work/t_xtask")" \
  cargo run -q --locked -p xtask -- prebuilt-list --allow-partial --dist "$(native "$work/www/good")" \
  --url http://127.0.0.1:1 --commit "$head")
cp "$list" "$work/good.txt"
printf 'format=1\n' > "$work/placeholder.txt"
refused_sha=$(sha256_of "$work/www/refused/$name.tar.gz")
sed "s/^\(archive=[^ ]* sha256=\)[0-9a-f]*/\1$refused_sha/" "$work/good.txt" > "$work/refused.txt"

use_list() {
  cp "$1" "$list"
  touch "$list"
}

# check NAME DIR EXPECT(ok|fail) REQUESTS(n|+) [has:TEXT | lacks:TEXT | unwatched:TEXT | reran | fresh]...
#   -- CARGO ARGS
# `unwatched:TEXT`: no rerun-if-changed line of the build script (needs -vv) contains TEXT.
# REQUESTS is the exact number of new requests, `+` for at least one, `*` for any.
failures=0
check() {
  local case_name=$1 dir=$2 expect=$3 want_requests=$4
  shift 4
  local checks=()
  while [ "$1" != "--" ]; do checks+=("$1"); shift; done
  shift
  local log="$work/$case_name.log" before status=0 problems=()
  before=$(requests)
  (cd "$repo" && cargo_in "$dir" "$@") > "$log" 2>&1 || status=$?
  local made=$(( $(requests) - before ))
  if [ "$expect" = ok ] && [ "$status" -ne 0 ]; then problems+=("exit $status"); fi
  if [ "$expect" = fail ] && [ "$status" -eq 0 ]; then problems+=("succeeded"); fi
  if [ "$want_requests" = "*" ]; then
    :
  elif [ "$want_requests" = + ]; then
    [ "$made" -ge 1 ] || problems+=("no request")
  elif [ "$made" -ne "$want_requests" ]; then
    problems+=("$made requests, expected $want_requests")
  fi
  local rerun_pattern='Running `[^`]*oxijolt-sys[-/][0-9a-f]+[/\\]([^`]*[/\\])?build-script-build'
  for c in ${checks[@]+"${checks[@]}"}; do
    case "$c" in
      has:*) grep -qF -- "${c#has:}" "$log" || problems+=("missing '${c#has:}'") ;;
      lacks:*) ! grep -qF -- "${c#lacks:}" "$log" || problems+=("unexpected '${c#lacks:}'") ;;
      unwatched:*) ! grep -F 'rerun-if-changed=' "$log" | grep -qF -- "${c#unwatched:}" ||
        problems+=("a rerun rule names '${c#unwatched:}'") ;;
      reran) grep -qE "$rerun_pattern" "$log" || problems+=("the build script did not run") ;;
      fresh) ! grep -qE "$rerun_pattern" "$log" || problems+=("the build script ran again") ;;
    esac
  done
  if [ ${#problems[@]} -eq 0 ]; then
    echo "ok    $case_name"
  else
    echo "FAIL  $case_name: ${problems[*]}"
    tail -n 25 "$log" | sed 's/^/      /'
    failures=$((failures + 1))
  fi
}

source_build="building joltc from source"
cmake_started="oxijolt-no-cmake"

# A fallback in both modes: auto warns and starts the source build, require fails before it.
fallback() {
  local row=$1 reason=$2 want_requests=$3 dir=$4
  shift 4
  check "$row-auto" "$dir" fail "$want_requests" "has:$source_build: $reason" "has:$cmake_started" -- \
    check --locked -p oxijolt-sys "$@"
  local saved=(${case_env[@]+"${case_env[@]}"})
  case_env+=(JOLTC_PREBUILT=require)
  check "$row-require" "$dir" fail "$want_requests" "has:JOLTC_PREBUILT=require: $reason" \
    "lacks:$cmake_started" -- check --locked -p oxijolt-sys "$@"
  case_env=(${saved[@]+"${saved[@]}"})
}

start_server
good_url="$url/good"
closed_port=$("$python" -c 'import socket; s = socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1]); s.close()')

# Refusals and fallbacks, in a target dir that never gets an archive.
use_list "$work/good.txt"
case_env=(JOLTC_PREBUILT_URL="$good_url" CARGO_NET_OFFLINE=true)
fallback offline "offline build (CARGO_NET_OFFLINE)" 0 t_no
case_env=(JOLTC_PREBUILT_URL="$good_url")
echo '{"files":{},"package":null}' > "$checksum_file"
fallback vendored "vendored sources (.cargo-checksum.json)" 0 t_no
rm -f "$checksum_file"
case_env=(JOLTC_PREBUILT_URL="$good_url" NIX_BUILD_TOP=/build)
fallback nix "inside a Nix build (NIX_BUILD_TOP)" 0 t_no
case_env=(JOLTC_PREBUILT_URL="$good_url")
use_list "$work/placeholder.txt"
fallback placeholder "the archive list has no archives" 0 t_no
printf 'format=2\n' > "$work/malformed.txt"
use_list "$work/malformed.txt"
fallback malformed "the archive list is malformed" 0 t_no
sed 's/^version=.*/version=0.0.1/' "$work/good.txt" > "$work/version.txt"
use_list "$work/version.txt"
fallback version "the archive list is for version 0.0.1" 0 t_no
sed 's/^sources=.*/sources=0000000000000000000000000000000000000000000000000000000000000000/' \
  "$work/good.txt" > "$work/sources.txt"
use_list "$work/sources.txt"
fallback sources "the native sources differ from the released ones" 0 t_no
use_list "$work/good.txt"
fallback no-archive "no archive for this target, CRT and features" 0 t_no --features asserts
case "$os" in
  windows)
    case_env=(JOLTC_PREBUILT_URL="$good_url" RUSTFLAGS=-Ctarget-feature=+crt-static)
    fallback crt-static "crt-static is enabled" 0 t_crt
    case_env=(JOLTC_PREBUILT_URL="$good_url" RUSTC_LINKER=lld-link)
    fallback linker-override "a linker or sysroot override is set" 0 t_no
    sed 's/msvc=[0-9.]*/msvc=14.99/' "$work/good.txt" > "$work/toolchain.txt"
    too_old="MSVC toolset"
    ;;
  linux)
    case_env=(JOLTC_PREBUILT_URL="$good_url" RUSTC_LINKER=clang)
    fallback linker-override "a linker or sysroot override is set" 0 t_no
    sed 's/glibc=[0-9.]*/glibc=9.99/' "$work/good.txt" > "$work/toolchain.txt"
    too_old="glibc"
    ;;
  macos)
    # Apple targets have no linker-override refusal; the archive's minimum macOS is compared
    # with the deployment target rustc links for.
    sed 's/macos=[0-9.]*/macos=99.0/' "$work/good.txt" > "$work/toolchain.txt"
    too_old="macOS deployment target"
    ;;
esac
case_env=(JOLTC_PREBUILT_URL="$good_url")
use_list "$work/toolchain.txt"
fallback toolchain "$too_old" 0 t_no
use_list "$work/good.txt"
case_env=(JOLTC_PREBUILT_URL="http://127.0.0.1:$closed_port")
fallback unreachable "download failed: curl exit" 0 t_no
case_env=(JOLTC_PREBUILT_URL="$url/bad")
fallback checksum "checksum mismatch: the archive differs from the list" 1 t_no
case_env=(JOLTC_PREBUILT_URL="$url/missing")
fallback http-404 "download failed: curl exit 22" 1 t_no
case_env=(JOLTC_PREBUILT_URL="ftp://x")
fallback bad-url "download failed: invalid JOLTC_PREBUILT_URL" 0 t_no
# A refused prefix is deleted, and the source build that follows must not watch its files: a
# missing watched file would rerun the build script, and the download, on every build.
use_list "$work/refused.txt"
case_env=(JOLTC_PREBUILT_URL="$url/refused")
fallback prefix-refused "the archive was refused" 1 t_refused -vv
check prefix-refused-unwatched t_refused fail 1 "has:$source_build: the archive was refused" \
  "has:$cmake_started" "unwatched:$name" -- check --locked -p oxijolt-sys -vv
use_list "$work/good.txt"

# No archive is looked at without the feature (at either crate), with `off`, under docs.rs or
# with JOLTC_LIB_DIR; the successes come last so that they cannot hide a refusal above.
case_env=(JOLTC_PREBUILT_URL="$good_url")
check feature-off-sys t_no fail 0 "has:$cmake_started" "lacks:$source_build" -- \
  check --locked -p oxijolt-sys --no-default-features
check feature-off-safe t_no fail 0 "has:$cmake_started" "lacks:$source_build" -- \
  check --locked -p oxijolt --no-default-features
case_env=(JOLTC_PREBUILT_URL="$good_url" JOLTC_PREBUILT=require)
check feature-off-require t_no fail 0 "has:JOLTC_PREBUILT=require: the prebuilt feature is off" \
  "lacks:$cmake_started" -- check --locked -p oxijolt-sys --no-default-features
case_env=(JOLTC_PREBUILT_URL="$good_url" JOLTC_PREBUILT=off)
check off t_no fail 0 "has:$cmake_started" "lacks:$source_build" -- check --locked -p oxijolt-sys
mismatched="$work/stage/mismatched"
rm -rf "$mismatched"
cp -R "$prefix" "$mismatched"
edit 's/^double_precision=OFF/double_precision=ON/' "$mismatched/oxijolt-sys-manifest.txt"
case_env=(JOLTC_PREBUILT_URL="$good_url" JOLTC_PREBUILT=require JOLTC_LIB_DIR="$(native "$mismatched")")
check lib-dir-mismatch t_no fail 0 "has:different configuration" -- check --locked -p oxijolt-sys
case_env=(JOLTC_PREBUILT_URL="$good_url" JOLTC_PREBUILT=require JOLTC_LIB_DIR="$(native "$prefix")")
check lib-dir-wins t_no ok 0 "lacks:$source_build" -- check --locked -p oxijolt-sys
case_env=(JOLTC_PREBUILT_URL="$good_url" JOLTC_PREBUILT=require DOCS_RS=1)
check docs-rs t_no ok 0 "lacks:$source_build" -- check --locked -p oxijolt-sys

# The download, in its own target dir.
smoke=(test --locked -p oxijolt-sys --test smoke_test -vv)
success="using the prebuilt archive $name"
case_env=(JOLTC_PREBUILT_URL="$good_url")
check download t_ok ok + "has:$success" "lacks:$source_build" "lacks:$cmake_started" -- "${smoke[@]}"
check safe-crate t_ok ok "*" "lacks:$source_build" "lacks:$cmake_started" -- \
  test --locked -p oxijolt --test group_table_first_call -vv
check no-rerun t_ok ok 0 fresh -- "${smoke[@]}"
(cd "$repo" && cargo_in t_ok clean -p oxijolt-sys) > "$work/clean.log" 2>&1
case_env=(JOLTC_PREBUILT_URL="$good_url" JOLTC_PREBUILT=require)
check download-require t_ok ok + "has:$success" "lacks:$cmake_started" -- "${smoke[@]}"

# The unpacked archive in OUT_DIR is used while it is intact, also after a change of PATH
# (which reruns the build script), and fetched again once a library in it changed.
stop_server
touch "$list"
check cached t_ok ok 0 reran "has:$success" -- "${smoke[@]}"
if [ "$os" = macos ]; then
  # Another deployment target reruns the build script, which keeps the cached archive.
  case_env=(JOLTC_PREBUILT_URL="$good_url" JOLTC_PREBUILT=require MACOSX_DEPLOYMENT_TARGET=12.0)
  check deployment-target-rerun t_ok ok 0 reran "has:$success" -- "${smoke[@]}"
fi
case_env=(JOLTC_PREBUILT_URL="$good_url" JOLTC_PREBUILT=require PATH="$work/empty-bin:$PATH")
check cached-new-path t_ok ok 0 reran "has:$success" -- "${smoke[@]}"
flipped=0
for library in "$work"/t_ok/debug/build/oxijolt-sys-*/out/prebuilt/"$name"/lib/"$joltc_lib" \
  "$work"/t_ok/debug/build/oxijolt-sys/*/out/prebuilt/"$name"/lib/"$joltc_lib"; do
  [ -f "$library" ] || continue
  "$python" -c 'import sys; p = sys.argv[1]; b = bytearray(open(p, "rb").read()); b[0] ^= 1; open(p, "wb").write(b)' \
    "$(native "$library")"
  flipped=$((flipped + 1))
done
[ "$flipped" -gt 0 ] || { echo "FAIL  corrupted-cache: no cached $joltc_lib found"; failures=$((failures + 1)); }
touch "$list"
case_env=(JOLTC_PREBUILT_URL="$good_url")
check corrupted-cache-offline t_ok fail 0 "has:$source_build: download failed" "has:$cmake_started" -- \
  "${smoke[@]}"
start_server
case_env=(JOLTC_PREBUILT_URL="$url/good")
check corrupted-cache-refetch t_ok ok + "has:$success" -- "${smoke[@]}"

echo "failures: $failures"
[ "$failures" -eq 0 ]
