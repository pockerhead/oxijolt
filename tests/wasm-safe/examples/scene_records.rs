//! Prints the determinism scene's records (`src/scene.rs`) for a native run, to compare with the
//! wasm module's. `scripts/check_wasm_determinism.sh` builds it with and without
//! `cross-platform-deterministic`.

use std::io::Write;
use std::process::ExitCode;

#[path = "../src/scene.rs"]
mod scene;

fn main() -> ExitCode {
    let Some(mut scene) = scene::Scene::new() else {
        eprintln!("scene_records: the scene could not be created");
        return ExitCode::FAILURE;
    };
    let mut records = String::new();
    for tick in 0..=scene::TICKS {
        if tick > 0 && !scene.step() {
            eprintln!("scene_records: step {tick} failed");
            return ExitCode::FAILURE;
        }
        if !scene.write_records(tick, &mut records) {
            eprintln!("scene_records: a body is missing at tick {tick}");
            return ExitCode::FAILURE;
        }
    }
    if !scene.settled() {
        eprintln!("scene_records: the pile did not land and fall asleep");
        return ExitCode::FAILURE;
    }
    if std::io::stdout().write_all(records.as_bytes()).is_err() {
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}
