//! A safe-layer `oxijolt` module for wasm32-unknown-unknown, driven from Node by `run.mjs`.
//!
//! It checks what the safe layer adds to the raw layer's runtime contract on this target: a module
//! that links world creation, stepping and save/restore still imports nothing, worlds default to
//! the single-threaded job system and refuse worker threads, a world created in one export call
//! keeps stepping in the next ones, a saved state replays bit for bit, and a panic in a listener
//! traps.
//!
//! Every export returns 0 on success and a nonzero code naming the failed check; the codes are
//! listed on each function. On other targets the crate is empty.

#![cfg(all(target_arch = "wasm32", target_os = "unknown"))]

use std::cell::RefCell;
use std::ops::RangeInclusive;
use std::sync::Arc;

use oxijolt::{
    BodyId, BodySettings, ContactListener, ContactManifold, ContactSettings, JobSystemKind,
    PhysicsWorld, RVec3, Real, Shape, Vec3, WorldError, WorldSettings,
};

/// Step length in seconds.
const DELTA_TIME: f32 = 1.0 / 60.0;

/// Height the ball starts falling from, in metres.
const BALL_START_HEIGHT: Real = 2.0;

/// Radius of the ball, in metres.
const BALL_RADIUS: f32 = 0.5;

/// Heights of the ball's centre that count as resting on the floor: one radius, within 10 cm.
const RESTING_HEIGHTS: RangeInclusive<Real> = 0.4..=0.6;

/// Steps between the save and the comparison of the two runs.
const REPLAY_STEPS: u32 = 30;

/// The kept world and its falling ball.
struct World {
    world: PhysicsWorld,
    ball: BodyId,
}

thread_local! {
    static WORLD: RefCell<Option<World>> = const { RefCell::new(None) };
}

/// Runs `check` on the kept world; returns 1 without a world.
fn with_world(check: impl FnOnce(&mut World) -> i32) -> i32 {
    WORLD.with(|world| world.borrow_mut().as_mut().map_or(1, check))
}

/// Checks the default settings: 2 when they do not choose the single-threaded job system.
#[no_mangle]
pub extern "C" fn default_is_single_threaded() -> i32 {
    if WorldSettings::default().job_system_kind() == JobSystemKind::SingleThreaded {
        0
    } else {
        2
    }
}

/// Checks that a world with `count` worker threads is refused: 2 when it is created, 3 when it
/// fails with another error.
#[no_mangle]
pub extern "C" fn worker_threads_refused(count: u32) -> i32 {
    match PhysicsWorld::new(WorldSettings::default().worker_threads(count)) {
        Err(WorldError::InvalidValue(_)) => 0,
        Ok(_) => 2,
        Err(_) => 3,
    }
}

/// Creates a default world with a static floor and a ball above it, kept for the next calls:
/// 1 when a world exists already, 2 when a creation fails, 3 when the world is not
/// single-threaded.
#[no_mangle]
pub extern "C" fn create_world() -> i32 {
    WORLD.with(|kept| {
        let mut kept = kept.borrow_mut();
        if kept.is_some() {
            return 1;
        }
        let Some(world) = new_world() else {
            return 2;
        };
        if world.world.job_system_kind() != JobSystemKind::SingleThreaded
            || world.world.max_concurrency() != 1
        {
            return 3;
        }
        *kept = Some(world);
        0
    })
}

fn new_world() -> Option<World> {
    let mut world = PhysicsWorld::new(WorldSettings::default()).ok()?;
    let floor = Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).ok()?;
    world
        .create_body(
            &floor,
            &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)),
        )
        .ok()?;
    let ball = world
        .create_body(
            &Shape::new_sphere(BALL_RADIUS).ok()?,
            &BodySettings::new_dynamic().position(RVec3::new(0.0, BALL_START_HEIGHT, 0.0)),
        )
        .ok()?;
    Some(World { world, ball })
}

/// Drops the kept world: 1 without a world.
#[no_mangle]
pub extern "C" fn destroy_world() -> i32 {
    WORLD.with(|kept| {
        if kept.borrow_mut().take().is_some() {
            0
        } else {
            1
        }
    })
}

/// Steps the kept world once: 1 without a world, 2 when the step fails or is incomplete.
#[no_mangle]
pub extern "C" fn step_world() -> i32 {
    with_world(|world| match world.world.step(DELTA_TIME) {
        Ok(report) if report.is_complete() => 0,
        _ => 2,
    })
}

/// Checks that the ball rests on the floor: 1 without a world, 2 when its centre is not at
/// `RESTING_HEIGHTS`.
#[no_mangle]
pub extern "C" fn ball_rests_on_floor() -> i32 {
    with_world(|world| {
        let Ok(ball) = world.world.body(world.ball) else {
            return 2;
        };
        if RESTING_HEIGHTS.contains(&ball.position().y) {
            0
        } else {
            2
        }
    })
}

/// Throws the ball up, saves the world, steps it, restores the save and steps again: 1 without
/// a world, 2 when a call fails, 3 when the two runs end at different ball positions.
#[no_mangle]
pub extern "C" fn save_restore_replays() -> i32 {
    with_world(|world| {
        let Ok(mut ball) = world.world.body_mut(world.ball) else {
            return 2;
        };
        if ball.add_impulse(Vec3::new(0.0, 8.0, 0.0)).is_err() {
            return 2;
        }
        let state = world.world.save_state();
        let Some(first) = run_replay(world) else {
            return 2;
        };
        if world.world.restore_state(&state).is_err() {
            return 2;
        }
        match run_replay(world) {
            Some(second) if second.to_bits() == first.to_bits() => 0,
            Some(_) => 3,
            None => 2,
        }
    })
}

/// Steps `REPLAY_STEPS` times and returns the ball's height; `None` when a step fails or is
/// incomplete.
fn run_replay(world: &mut World) -> Option<Real> {
    for _ in 0..REPLAY_STEPS {
        if !world.world.step(DELTA_TIME).ok()?.is_complete() {
            return None;
        }
    }
    Some(world.world.body(world.ball).ok()?.position().y)
}

/// A contact listener that panics on every new contact.
struct Panicking;

impl ContactListener for Panicking {
    fn contact_added(&self, _: &ContactManifold, _: &mut ContactSettings) {
        panic!("contact listener panic");
    }
}

/// Installs a panicking contact listener, drops a second ball and steps until it lands. Traps on
/// the first contact; 1 without a world, 2 when every step returned.
#[no_mangle]
pub extern "C" fn step_with_panicking_listener() -> i32 {
    with_world(|world| {
        world.world.set_contact_listener(Some(Arc::new(Panicking)));
        let Ok(sphere) = Shape::new_sphere(BALL_RADIUS) else {
            return 2;
        };
        let settings =
            BodySettings::new_dynamic().position(RVec3::new(3.0, BALL_START_HEIGHT, 0.0));
        if world.world.create_body(&sphere, &settings).is_err() {
            return 2;
        }
        for _ in 0..240 {
            let _ = world.world.step(DELTA_TIME);
        }
        2
    })
}
