// Runs the determinism scene of the oxijolt-wasm-safe module, one export call per tick, and returns
// its records (one line per body and tick, see src/scene.rs). Runs in Node and in a browser.

function expect(step, actual, expected = 0) {
    if (actual !== expected) {
        throw new Error(`${step}: returned ${actual}, expected ${expected}`);
    }
}

/** Instantiates the module from its bytes and returns the records of ticks 0 to the last. */
export async function sceneRecords(bytes) {
    const { instance } = await WebAssembly.instantiate(bytes, {});
    const exports = instance.exports;
    const decoder = new TextDecoder();
    const parts = [];
    const take = () => {
        const start = exports.scene_records_ptr() >>> 0;
        const length = exports.scene_records_len() >>> 0;
        parts.push(decoder.decode(new Uint8Array(exports.memory.buffer, start, length)));
    };
    expect("scene_create", exports.scene_create());
    take();
    const ticks = exports.scene_ticks();
    for (let tick = 1; tick <= ticks; tick++) {
        expect(`scene_step #${tick}`, exports.scene_step());
        take();
    }
    expect("scene_step after the last tick", exports.scene_step(), 4);
    expect("scene_settled", exports.scene_settled());
    expect("scene_destroy", exports.scene_destroy());
    return parts.join("");
}
