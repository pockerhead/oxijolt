// Writes the determinism scene's records of the oxijolt-wasm-safe module to a file, run in Node or
// in headless Chrome. Usage: node tests/wasm-safe/scene_records.mjs <node|chrome> <module.wasm> <out>
// Chrome is the `CHROME` environment variable, else the first of google-chrome,
// google-chrome-stable, chromium and chromium-browser on PATH, else Chrome's Windows install path.
import { spawn } from "node:child_process";
import { existsSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { createServer } from "node:http";
import { tmpdir } from "node:os";
import { delimiter, join } from "node:path";
import { fileURLToPath } from "node:url";

import { sceneRecords } from "./scene.mjs";

const SCENE_MODULE = fileURLToPath(new URL("./scene.mjs", import.meta.url));
const CHROME_NAMES = ["google-chrome", "google-chrome-stable", "chromium", "chromium-browser"];
const WINDOWS_CHROME = "C:/Program Files/Google/Chrome/Application/chrome.exe";
const TIMEOUT_MS = 300_000;

function findChrome() {
    if (process.env.CHROME) {
        return process.env.CHROME;
    }
    for (const dir of (process.env.PATH ?? "").split(delimiter)) {
        for (const name of CHROME_NAMES) {
            if (dir && existsSync(join(dir, name))) {
                return join(dir, name);
            }
        }
    }
    if (process.platform === "win32" && existsSync(WINDOWS_CHROME)) {
        return WINDOWS_CHROME;
    }
    throw new Error("no Chrome found: set CHROME");
}

// Serves an empty page, the scene module and the wasm module on 127.0.0.1.
function serve(wasm) {
    const files = {
        "/": ["text/html", "<!DOCTYPE html><title>oxijolt scene</title>"],
        "/scene.mjs": ["text/javascript", readFileSync(SCENE_MODULE)],
        "/module.wasm": ["application/wasm", wasm],
    };
    const server = createServer((request, response) => {
        const file = files[request.url];
        response.writeHead(file ? 200 : 404, { "Content-Type": file ? file[0] : "text/plain" });
        response.end(file ? file[1] : "not found");
    });
    return new Promise((resolve) => server.listen(0, "127.0.0.1", () => resolve(server)));
}

// Starts headless Chrome with remote debugging: the process, its DevTools WebSocket URL and its
// end.
function launchChrome(profile) {
    // The sandbox is off because Ubuntu 24.04 blocks the user namespaces it needs; the browser
    // loads only the local page served above.
    const args = ["--headless=new", "--no-sandbox", "--no-first-run", "--no-default-browser-check",
        "--remote-debugging-port=0", `--user-data-dir=${profile}`, "about:blank"];
    const chrome = spawn(findChrome(), args, { stdio: ["ignore", "ignore", "pipe"] });
    const closed = new Promise((resolve) => chrome.once("close", resolve));
    const endpoint = new Promise((resolve, reject) => {
        let output = "";
        chrome.stderr.on("data", (chunk) => {
            output += chunk;
            const match = /DevTools listening on (ws:\/\/\S+)/.exec(output);
            if (match) {
                resolve(match[1]);
            }
        });
        chrome.on("error", reject);
        chrome.on("exit", (code) => reject(new Error(`Chrome exited with ${code}: ${output}`)));
    });
    return { chrome, endpoint, closed };
}

// A minimal Chrome DevTools Protocol client over Node's WebSocket.
async function connect(url) {
    const socket = new WebSocket(url);
    await new Promise((resolve, reject) => {
        socket.onopen = resolve;
        socket.onerror = () => reject(new Error(`cannot connect to ${url}`));
    });
    let nextId = 1;
    const pending = new Map();
    const waiters = [];
    socket.onmessage = (event) => {
        const message = JSON.parse(event.data);
        const call = pending.get(message.id);
        if (call) {
            pending.delete(message.id);
            if (message.error) {
                call.reject(new Error(`${call.method}: ${message.error.message}`));
            } else {
                call.resolve(message.result);
            }
            return;
        }
        for (const waiter of waiters.filter((w) => w.method === message.method)) {
            waiters.splice(waiters.indexOf(waiter), 1);
            waiter.resolve(message.params);
        }
    };
    return {
        send(method, params = {}, sessionId = undefined) {
            const id = nextId++;
            socket.send(JSON.stringify({ id, method, params, sessionId }));
            return new Promise((resolve, reject) => pending.set(id, { method, resolve, reject }));
        },
        next(method) {
            return new Promise((resolve) => waiters.push({ method, resolve }));
        },
        close() {
            socket.close();
        },
    };
}

async function chromeRecords(wasm) {
    const server = await serve(wasm);
    const page = `http://127.0.0.1:${server.address().port}/`;
    const profile = mkdtempSync(join(tmpdir(), "oxijolt-chrome-"));
    const { chrome, endpoint, closed } = launchChrome(profile);
    try {
        const browser = await connect(await endpoint);
        const { targetId } = await browser.send("Target.createTarget", { url: "about:blank" });
        const { sessionId } = await browser.send("Target.attachToTarget", { targetId, flatten: true });
        await browser.send("Page.enable", {}, sessionId);
        const loaded = browser.next("Page.loadEventFired");
        await browser.send("Page.navigate", { url: page }, sessionId);
        await loaded;
        const expression = `import("/scene.mjs").then(async (scene) =>
            scene.sceneRecords(await (await fetch("/module.wasm")).arrayBuffer()))`;
        const result = await browser.send("Runtime.evaluate",
            { expression, awaitPromise: true, returnByValue: true }, sessionId);
        browser.close();
        if (result.exceptionDetails) {
            const detail = result.exceptionDetails.exception?.description ?? result.exceptionDetails.text;
            throw new Error(`in Chrome: ${detail}`);
        }
        return result.result.value;
    } finally {
        chrome.removeAllListeners("exit");
        chrome.kill();
        server.close();
        await closed;
        rmSync(profile, { recursive: true, force: true, maxRetries: 5 });
    }
}

async function main([host, path, out]) {
    if (!["node", "chrome"].includes(host) || !path || !out) {
        throw new Error("usage: node scene_records.mjs <node|chrome> <module.wasm> <out>");
    }
    const wasm = readFileSync(path);
    const records = host === "node" ? await sceneRecords(wasm) : await chromeRecords(wasm);
    writeFileSync(out, records);
    const lines = records.split("\n").length - 1;
    console.log(`scene-records (${host}): ${lines} records in ${out}`);
}

const timeout = setTimeout(() => {
    console.error("scene-records: timed out");
    process.exit(1);
}, TIMEOUT_MS);
try {
    await main(process.argv.slice(2));
    clearTimeout(timeout);
} catch (error) {
    console.error(`scene-records: ${error.message}`);
    process.exit(1);
}
