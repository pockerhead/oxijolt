// Runs the raw wasm32-unknown-unknown module built from this crate in Node, across export calls.
// Usage: node tests/wasm-raw/run.mjs <path to oxijolt_sys_wasm_raw.wasm>
import { readFileSync } from "node:fs";

const WASI = "__imported_wasi_snapshot_preview1_";
const ERRNO_BADF = 8;
const ERRNO_INVAL = 28;
const ERRNO_SPIPE = 70;

// Offsets in the scratch buffer.
const IOVS = 0;
const CANARY_BEFORE = 28;
const OUTPUT = 32;
const CANARY_AFTER = 36;
const STAT = 64;
const STAT_BYTES = 24;
const SEEK = 96;
const SEEK_BYTES = 8;
const CANARY = 0xaaaaaaaa;
const PRESET = 0x55555555;
const FILL = 0x77;

function fail(step, detail) {
    throw new Error(`${step}: ${detail}`);
}

function expect(step, actual, expected) {
    if (actual !== expected) {
        fail(step, `returned ${actual}, expected ${expected}`);
    }
}

function instantiate(module) {
    return new WebAssembly.Instance(module, {}).exports;
}

// A fresh view each time: growing memory detaches the previous buffer.
function view(exports) {
    return new DataView(exports.memory.buffer);
}

function checkImports(module) {
    const imports = WebAssembly.Module.imports(module);
    if (imports.length !== 0) {
        const names = imports.map((i) => `${i.module}.${i.name}`).join(", ");
        fail("imports", `the module imports ${names}`);
    }
}

// Writes the iovecs, presets the output and the canaries, calls fd_write and checks the result.
function checkFdWrite(exports, step, fd, lengths, expectedErrno, expectedOutput) {
    const base = exports.scratch_buffer();
    let memory = view(exports);
    lengths.forEach((length, i) => {
        memory.setUint32(base + IOVS + 8 * i, base + 128, true);
        memory.setUint32(base + IOVS + 8 * i + 4, length, true);
    });
    memory.setUint32(base + CANARY_BEFORE, CANARY, true);
    memory.setUint32(base + OUTPUT, PRESET, true);
    memory.setUint32(base + CANARY_AFTER, CANARY, true);
    const iovs = lengths.length === 0 ? 0 : base + IOVS;
    expect(step, exports[`${WASI}fd_write`](fd, iovs, lengths.length, base + OUTPUT), expectedErrno);
    memory = view(exports);
    const output = memory.getUint32(base + OUTPUT, true);
    if (output !== expectedOutput) {
        fail(step, `wrote ${output} bytes, expected ${expectedOutput}`);
    }
    for (const offset of [CANARY_BEFORE, CANARY_AFTER]) {
        if (memory.getUint32(base + offset, true) !== CANARY) {
            fail(step, `canary at ${offset} overwritten`);
        }
    }
}

// Fills `length` bytes at `offset`, runs `call` and checks the bytes are unchanged.
function checkUntouched(exports, step, offset, length, call, expectedErrno) {
    const base = exports.scratch_buffer();
    let memory = view(exports);
    for (let i = 0; i < length; i++) {
        memory.setUint8(base + offset + i, FILL);
    }
    expect(step, call(base + offset), expectedErrno);
    memory = view(exports);
    for (let i = 0; i < length; i++) {
        if (memory.getUint8(base + offset + i) !== FILL) {
            fail(step, `byte ${i} of the output was written`);
        }
    }
}

function checkTraps(module, step, call) {
    const exports = instantiate(module);
    try {
        call(exports, exports.scratch_buffer());
    } catch (error) {
        if (error instanceof WebAssembly.RuntimeError) {
            return;
        }
        throw error;
    }
    fail(step, "returned instead of trapping");
}

function checkWasi(module, exports) {
    for (const fd of [1, 2]) {
        checkFdWrite(exports, `fd_write(${fd}, 5 + 7)`, fd, [5, 7], 0, 12);
        checkFdWrite(exports, `fd_write(${fd}, no vectors)`, fd, [], 0, 0);
        checkFdWrite(exports, `fd_write(${fd}, one empty vector)`, fd, [0], 0, 0);
        checkFdWrite(exports, `fd_write(${fd}, overflow)`, fd, [0xffffffff, 1], ERRNO_INVAL, PRESET);
    }
    checkFdWrite(exports, "fd_write(3)", 3, [5, 7], ERRNO_BADF, PRESET);
    expect("fd_close(1)", exports[`${WASI}fd_close`](1), ERRNO_BADF);
    checkUntouched(exports, "fd_fdstat_get(1)", STAT, STAT_BYTES,
        (p) => exports[`${WASI}fd_fdstat_get`](1, p), ERRNO_BADF);
    checkUntouched(exports, "fd_seek(1)", SEEK, SEEK_BYTES,
        (p) => exports[`${WASI}fd_seek`](1, 0n, 0, p), ERRNO_SPIPE);
    expect("sched_yield", exports[`${WASI}sched_yield`](), 0);
    checkTraps(module, "clock_time_get", (e, p) => e[`${WASI}clock_time_get`](0, 0n, p));
    checkTraps(module, "poll_oneoff", (e, p) => e[`${WASI}poll_oneoff`](p, p, 1, p));
}

function checkGrowth(exports, name) {
    const before = exports.memory.buffer.byteLength;
    expect(name, exports[name](), 0);
    const after = exports.memory.buffer.byteLength;
    if (after <= before) {
        fail(name, `memory stayed at ${before} bytes`);
    }
}

function stepTimes(exports, count) {
    for (let i = 0; i < count; i++) {
        expect(`step_world #${i + 1}`, exports.step_world(), 0);
    }
}

function checkWorld(exports) {
    expect("create_world", exports.create_world(), 0);
    expect("create_world with a world", exports.create_world(), 1);
    expect("init_again", exports.init_again(), 0);
    stepTimes(exports, 5);
    expect("init_again after steps", exports.init_again(), 0);
    stepTimes(exports, 5);
    expect("listener_calls", exports.listener_calls(), 10);
    checkGrowth(exports, "grow_rust_heap");
    checkGrowth(exports, "grow_native_heap");
    checkGrowth(exports, "grow_rust_heap");
    checkGrowth(exports, "grow_native_heap");
    expect("check_heaps", exports.check_heaps(), 0);
    expect("free_heaps", exports.free_heaps(), 0);
    expect("destroy_world", exports.destroy_world(), 0);
    expect("destroy_world without a world", exports.destroy_world(), 1);
    expect("step_world without a world", exports.step_world(), 1);
    expect("create_world again", exports.create_world(), 0);
    stepTimes(exports, 10);
    expect("destroy_world again", exports.destroy_world(), 0);
}

function main(path) {
    if (!path) {
        fail("usage", "node run.mjs <module.wasm>");
    }
    const module = new WebAssembly.Module(readFileSync(path));
    checkImports(module);
    const exports = instantiate(module);
    checkWasi(module, exports);
    expect("libc_stdout", exports.libc_stdout(), 0);
    checkWorld(exports);
    const count = WebAssembly.Module.exports(module).length;
    console.log(`wasm-raw: ok (${count} exports, no imports)`);
}

try {
    main(process.argv[2]);
} catch (error) {
    console.error(`wasm-raw: ${error.message}`);
    process.exit(1);
}
