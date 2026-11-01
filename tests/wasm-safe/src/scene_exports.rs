//! Exports that run the determinism scene (`scene.rs`) one tick per call; `scene.mjs` collects the
//! records of every tick for comparison with the native example `scene_records`.
//!
//! Every export that returns `i32` returns 0 on success and a nonzero code naming the failure.

use std::cell::RefCell;

use crate::scene::{Scene, TICKS};

/// The kept scene and the tick it is at.
struct Kept {
    scene: Scene,
    tick: u32,
}

thread_local! {
    static SCENE: RefCell<Option<Kept>> = const { RefCell::new(None) };
    static RECORDS: RefCell<String> = const { RefCell::new(String::new()) };
}

/// Ticks the scene runs after its creation.
#[no_mangle]
pub extern "C" fn scene_ticks() -> u32 {
    TICKS
}

/// Creates the scene and records tick 0: 1 when a scene exists already, 2 when a creation fails,
/// 3 when a body is missing from the records.
#[no_mangle]
pub extern "C" fn scene_create() -> i32 {
    SCENE.with(|kept| {
        let mut kept = kept.borrow_mut();
        if kept.is_some() {
            return 1;
        }
        let Some(scene) = Scene::new() else {
            return 2;
        };
        if !record(&scene, 0) {
            return 3;
        }
        *kept = Some(Kept { scene, tick: 0 });
        0
    })
}

/// Steps the scene once and records the new tick: 1 without a scene, 2 when the step fails, 3 when
/// a body is missing from the records, 4 after the last tick.
#[no_mangle]
pub extern "C" fn scene_step() -> i32 {
    SCENE.with(|kept| {
        let mut kept = kept.borrow_mut();
        let Some(kept) = kept.as_mut() else {
            return 1;
        };
        if kept.tick == TICKS {
            return 4;
        }
        if !kept.scene.step() {
            return 2;
        }
        kept.tick += 1;
        if record(&kept.scene, kept.tick) {
            0
        } else {
            3
        }
    })
}

/// Checks the scene's outcome after the last tick (`Scene::settled`): 1 without a scene, 2 when
/// the scene is not at its last tick or did not settle.
#[no_mangle]
pub extern "C" fn scene_settled() -> i32 {
    SCENE.with(|kept| match kept.borrow().as_ref() {
        None => 1,
        Some(kept) if kept.tick == TICKS && kept.scene.settled() => 0,
        Some(_) => 2,
    })
}

/// Drops the kept scene: 1 without a scene.
#[no_mangle]
pub extern "C" fn scene_destroy() -> i32 {
    SCENE.with(|kept| {
        if kept.borrow_mut().take().is_some() {
            0
        } else {
            1
        }
    })
}

/// Address of the last recorded tick's text (UTF-8), valid until the next `scene_*` call.
#[no_mangle]
pub extern "C" fn scene_records_ptr() -> *const u8 {
    RECORDS.with(|records| records.borrow().as_ptr())
}

/// Length in bytes of the last recorded tick's text.
#[no_mangle]
pub extern "C" fn scene_records_len() -> usize {
    RECORDS.with(|records| records.borrow().len())
}

/// Replaces the kept records with those of `tick`; `false` when a body is missing.
fn record(scene: &Scene, tick: u32) -> bool {
    RECORDS.with(|records| {
        let mut records = records.borrow_mut();
        records.clear();
        scene.write_records(tick, &mut records)
    })
}
