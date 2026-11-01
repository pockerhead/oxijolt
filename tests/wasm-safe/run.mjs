// Runs the safe-layer wasm32-unknown-unknown module built from this crate in Node, across export
// calls. Usage: node tests/wasm-safe/run.mjs <path to oxijolt_wasm_safe.wasm>
import { readFileSync } from "node:fs";

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

function checkImports(module) {
    const imports = WebAssembly.Module.imports(module);
    if (imports.length !== 0) {
        const names = imports.map((i) => `${i.module}.${i.name}`).join(", ");
        fail("imports", `the module imports ${names}`);
    }
}

function stepTimes(exports, count) {
    for (let i = 0; i < count; i++) {
        expect(`step_world #${i + 1}`, exports.step_world(), 0);
    }
}

function checkWorld(exports) {
    expect("default_is_single_threaded", exports.default_is_single_threaded(), 0);
    for (const count of [0, 1, 4, 64]) {
        expect(`worker_threads_refused(${count})`, exports.worker_threads_refused(count), 0);
    }
    expect("create_world", exports.create_world(), 0);
    expect("create_world with a world", exports.create_world(), 1);
    stepTimes(exports, 120);
    expect("ball_rests_on_floor", exports.ball_rests_on_floor(), 0);
    expect("save_restore_replays", exports.save_restore_replays(), 0);
    expect("destroy_world", exports.destroy_world(), 0);
    expect("destroy_world without a world", exports.destroy_world(), 1);
    expect("step_world without a world", exports.step_world(), 1);
}

// A panic aborts on this target: the call traps instead of returning.
function checkPanicTraps(module) {
    const exports = instantiate(module);
    expect("create_world for the panic", exports.create_world(), 0);
    try {
        exports.step_with_panicking_listener();
    } catch (error) {
        if (error instanceof WebAssembly.RuntimeError) {
            return;
        }
        throw error;
    }
    fail("step_with_panicking_listener", "returned instead of trapping");
}

function main(path) {
    if (!path) {
        fail("usage", "node run.mjs <module.wasm>");
    }
    const module = new WebAssembly.Module(readFileSync(path));
    checkImports(module);
    checkWorld(instantiate(module));
    checkPanicTraps(module);
    const count = WebAssembly.Module.exports(module).length;
    console.log(`wasm-safe: ok (${count} exports, no imports)`);
}

try {
    main(process.argv[2]);
} catch (error) {
    console.error(`wasm-safe: ${error.message}`);
    process.exit(1);
}
