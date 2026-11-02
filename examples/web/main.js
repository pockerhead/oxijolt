// Loads the example module, steps it at 60 Hz and draws the balls from the front. The module
// imports nothing, so it is instantiated with an empty import object. Its path is the `wasm`
// query parameter, by default the release build of a repository served from its root.
const DEFAULT_MODULE = "../../target/wasm32-unknown-unknown/release/oxijolt_web_example.wasm";
const STEP_SECONDS = 1 / 60;
const MAX_STEPS_PER_FRAME = 4;
const FLOATS_PER_POSE = 7;
const PIXELS_PER_METRE = 40;
// Inside of the bin in metres, as built by `add_bin` in src/lib.rs.
const BIN_HALF_WIDTH = 6;
const BIN_HEIGHT = 20;

const canvas = document.getElementById("view");
const context = canvas.getContext("2d");
const status = document.getElementById("status");
const countInput = document.getElementById("count");
const countValue = document.getElementById("count-value");

const modulePath = new URLSearchParams(location.search).get("wasm") ?? DEFAULT_MODULE;
const response = await fetch(modulePath);
if (!response.ok) {
    status.textContent = `no module at ${modulePath} (HTTP ${response.status}): build it first, see README.md`;
    throw new Error(status.textContent);
}
const { instance } = await WebAssembly.instantiate(await response.arrayBuffer(), {});
const physics = instance.exports;

let tick = 0;
let stepTimes = [];
let pending = 0;
let lastFrame = performance.now();

function restart() {
    const code = physics.create_scene(Number(countInput.value));
    if (code !== 0) {
        throw new Error(`create_scene returned ${code}`);
    }
    tick = 0;
    stepTimes = [];
}

function step() {
    const start = performance.now();
    const code = physics.step_scene();
    stepTimes.push(performance.now() - start);
    if (stepTimes.length > 60) {
        stepTimes.shift();
    }
    if (code !== 0) {
        throw new Error(`step_scene returned ${code}`);
    }
    tick += 1;
}

// Canvas coordinates of a world point: x to the right, y up, the bin's floor near the bottom.
function toCanvas(x, y) {
    return [canvas.width / 2 + x * PIXELS_PER_METRE, canvas.height - 20 - y * PIXELS_PER_METRE];
}

// The ball's local x axis turned by its rotation (x, y, z, w), seen from the front.
function markerDirection(qx, qy, qz, qw) {
    return [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy + qw * qz)];
}

function drawBin() {
    const [left, top] = toCanvas(-BIN_HALF_WIDTH, BIN_HEIGHT);
    const [right, bottom] = toCanvas(BIN_HALF_WIDTH, 0);
    context.strokeStyle = "#555";
    context.lineWidth = 3;
    context.beginPath();
    context.moveTo(left, top);
    context.lineTo(left, bottom);
    context.lineTo(right, bottom);
    context.lineTo(right, top);
    context.stroke();
}

function drawBalls() {
    const count = physics.ball_count();
    const address = physics.ball_poses();
    // A new view every frame: the module's memory may have grown and detached the old one.
    const poses = new Float32Array(physics.memory.buffer, address, count * FLOATS_PER_POSE);
    const radius = physics.ball_radius() * PIXELS_PER_METRE;
    context.lineWidth = 1;
    for (let i = 0; i < count; i++) {
        const [x, y, z, qx, qy, qz, qw] = poses.subarray(i * FLOATS_PER_POSE, (i + 1) * FLOATS_PER_POSE);
        const [cx, cy] = toCanvas(x, y);
        // Nearer balls (larger z) are lighter.
        const shade = Math.round(120 + 40 * Math.max(-1.5, Math.min(1.5, z)));
        context.fillStyle = `rgb(${shade}, ${shade + 30}, 230)`;
        context.beginPath();
        context.arc(cx, cy, radius, 0, 2 * Math.PI);
        context.fill();
        const [dx, dy] = markerDirection(qx, qy, qz, qw);
        context.strokeStyle = "#123";
        context.beginPath();
        context.moveTo(cx, cy);
        context.lineTo(cx + dx * radius, cy - dy * radius);
        context.stroke();
    }
}

function frame(now) {
    pending = Math.min(pending + (now - lastFrame) / 1000, MAX_STEPS_PER_FRAME * STEP_SECONDS);
    lastFrame = now;
    while (pending >= STEP_SECONDS) {
        step();
        pending -= STEP_SECONDS;
    }
    context.clearRect(0, 0, canvas.width, canvas.height);
    drawBin();
    drawBalls();
    const mean = stepTimes.reduce((sum, time) => sum + time, 0) / Math.max(1, stepTimes.length);
    status.textContent = `tick ${tick}, ${physics.ball_count()} balls, `
        + `step ${mean.toFixed(2)} ms (mean of the last ${stepTimes.length})`;
    requestAnimationFrame(frame);
}

countInput.addEventListener("input", () => {
    countValue.textContent = countInput.value;
});
document.getElementById("restart").addEventListener("click", restart);
restart();
requestAnimationFrame(frame);
