#!/usr/bin/env bash
# Cross-platform determinism gate: the determinism scene's per-tick records (tests/wasm-safe/src/scene.rs)
# from the wasm32-unknown-unknown module, run in Node in single and double precision and in
# headless Chrome in single precision, must equal those of a native run of the same scene, both
# sides built with cross-platform-deterministic. The gate then checks that it turns red for a
# native run without that feature and for a record with one flipped bit.
# Needs WASI_SDK_PATH, Ninja, Node 22 and Chrome (see tests/wasm-safe/scene_records.mjs).
set -euo pipefail
cd "$(dirname "$0")/.."
: "${WASI_SDK_PATH:?set WASI_SDK_PATH to a wasi-sdk install (34.0 is tested)}"
for tool in ninja node cargo cmp diff; do
    command -v "$tool" >/dev/null || { echo "check_wasm_determinism: $tool not found" >&2; exit 1; }
done
export JOLTC_PREBUILT=off
target_dir=${CARGO_TARGET_DIR:-target}
module="$target_dir/wasm32-unknown-unknown/debug/oxijolt_wasm_safe.wasm"
records="$target_dir/wasm-determinism"
mkdir -p "$records"

fail() {
    echo "check_wasm_determinism: $*" >&2
    exit 1
}

# Builds the wasm module with the features in $1.
build_wasm() {
    cargo rustc --locked -p oxijolt-wasm-safe --lib --target wasm32-unknown-unknown --features "$1" \
        -- -C link-arg=--fatal-warnings
}

# Runs the module last built in $1 (node or chrome) and writes its records to $2.
wasm_records() {
    node tests/wasm-safe/scene_records.mjs "$1" "$module" "$2"
}

# Runs the native example with the features in $1 and writes its records to $2.
native_records() {
    cargo run --locked -p oxijolt-wasm-safe --example scene_records --features "$1" >"$2"
    [ -s "$2" ] || fail "no native records in $2"
}

# Fails unless the records in $1 and $2 are byte-identical.
require_same() {
    if ! cmp -s "$1" "$2"; then
        diff "$1" "$2" | head -n 4 >&2 || true
        fail "$1 and $2 differ"
    fi
    echo "check_wasm_determinism: $1 equals $2"
}

# Fails when the records in $1 and $2 are byte-identical: a sabotage the gate did not see.
require_different() {
    cmp -s "$1" "$2" && fail "the gate did not tell $1 from $2"
    echo "check_wasm_determinism: $1 differs from $2, as it must"
}

# Writes $1 to $2 with the lowest bit of one position component flipped, halfway through.
flip_one_bit() {
    node -e '
        const fs = require("node:fs");
        const [from, to] = process.argv.slice(1);
        const lines = fs.readFileSync(from, "utf8").split("\n");
        const line = Math.floor(lines.length / 2);
        const fields = lines[line].split(" ");
        fields[2] = (BigInt("0x" + fields[2]) ^ 1n).toString(16);
        lines[line] = fields.join(" ");
        fs.writeFileSync(to, lines.join("\n"));
    ' "$1" "$2"
}

for precision in single double; do
    features=cross-platform-deterministic
    [ "$precision" = double ] && features="$features,double-precision"
    build_wasm "$features"
    wasm_records node "$records/node-$precision.txt"
    if [ "$precision" = single ]; then
        wasm_records chrome "$records/chrome-$precision.txt"
    fi
    native_records "$features" "$records/native-$precision.txt"
    require_same "$records/native-$precision.txt" "$records/node-$precision.txt"
    if [ "$precision" = single ]; then
        require_same "$records/native-$precision.txt" "$records/chrome-$precision.txt"
    fi
done

native_records "" "$records/native-default-single.txt"
require_different "$records/native-default-single.txt" "$records/node-single.txt"
flip_one_bit "$records/node-single.txt" "$records/flipped-single.txt"
require_different "$records/native-single.txt" "$records/flipped-single.txt"
