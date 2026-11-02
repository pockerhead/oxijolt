# oxijolt in the browser

A page that steps a pile of balls falling into a bin with `oxijolt` in a `wasm32-unknown-unknown`
module and draws it on a canvas, seen from the front. A slider sets the number of balls (50 to
1000) and the page shows the mean step time of the last 60 steps, measured in that browser.

## Run it

Rust 1.91 or newer and the target:

```bash
rustup target add wasm32-unknown-unknown
```

From the repository root:

```bash
cargo build --release -p oxijolt-web-example --target wasm32-unknown-unknown
python3 -m http.server 8000
```

Open `http://localhost:8000/examples/web/`. The page loads
`target/wasm32-unknown-unknown/release/oxijolt_web_example.wasm`; with another target directory,
pass the module's URL as `?wasm=<url>`. Chrome does not fetch the module for a page opened from
`file://`, hence the server.

The build takes Jolt and joltc from a release archive when one fits the checkout, otherwise it
compiles them with wasi-sdk (`WASI_SDK_PATH` and Ninja); [building](../../docs/building.md#webassembly)
has the details.

## How the page talks to the module

The module imports nothing (`oxijolt-sys` defines the WASI functions its C++ runtime calls), so
`main.js` instantiates it with `WebAssembly.instantiate(bytes, {})`: no wasm-bindgen, no WASI shim.
`src/lib.rs` exports plain `extern "C"` functions:

| Export | What it does |
|---|---|
| `create_scene(count)` | replaces the scene with an empty bin and `count` balls (up to 1000) above it |
| `step_scene()` | steps the scene by 1/60 s |
| `ball_count()`, `ball_radius()` | the number of balls and their radius in metres |
| `ball_poses()` | the address of `ball_count()` records of 7 floats in the module's memory: position x, y, z and rotation x, y, z, w |

The world stays in a thread-local between calls. `main.js` reads the poses through a
`Float32Array` over `memory.buffer`, made again every frame because the memory may have grown.
Worlds are single-threaded on this target and a panic traps the module.
