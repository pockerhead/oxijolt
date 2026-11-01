#!/usr/bin/env bash
# Builds the raw wasm fixture in single and double precision and runs it in Node.
set -euo pipefail
cd "$(dirname "$0")/.."
: "${WASI_SDK_PATH:?set WASI_SDK_PATH to a wasi-sdk install (34.0 is tested)}"
for tool in ninja node cargo; do
    command -v "$tool" >/dev/null || { echo "check_wasm_raw: $tool not found" >&2; exit 1; }
done
export JOLTC_PREBUILT=off
target_dir=${CARGO_TARGET_DIR:-target}
for features in "" "--features double-precision"; do
    # shellcheck disable=SC2086 # $features is empty or two words on purpose
    cargo rustc --locked -p oxijolt-sys-wasm-raw --target wasm32-unknown-unknown $features \
        -- -C link-arg=--fatal-warnings
    node tests/wasm-raw/run.mjs "$target_dir/wasm32-unknown-unknown/debug/oxijolt_sys_wasm_raw.wasm"
done
