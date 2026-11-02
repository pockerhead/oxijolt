//! A browser example: balls falling into a bin, stepped by `oxijolt` in a wasm32-unknown-unknown
//! module and drawn on a canvas by `main.js`.
//!
//! The module imports nothing, so the page instantiates it with an empty import object and calls
//! the exports below; no binding generator is involved. The scene lives in a thread-local between
//! calls. On other targets the crate is empty.

#![cfg(all(target_arch = "wasm32", target_os = "unknown"))]

use std::cell::RefCell;

use oxijolt::{BodyId, BodySettings, PhysicsWorld, RVec3, Real, Shape, Vec3, WorldSettings};

/// Step length in seconds.
const DELTA_TIME: f32 = 1.0 / 60.0;

/// Radius of every ball, in metres.
const BALL_RADIUS: f32 = 0.3;

/// Most balls a scene takes; the page's slider stops here.
const MAX_BALLS: u32 = 1000;

/// Floats per ball in the pose buffer: position x, y, z, then rotation x, y, z, w.
const FLOATS_PER_POSE: usize = 7;

/// The world, its balls in creation order and the pose buffer the page reads.
struct Scene {
    world: PhysicsWorld,
    balls: Vec<BodyId>,
    poses: Vec<f32>,
}

thread_local! {
    static SCENE: RefCell<Option<Scene>> = const { RefCell::new(None) };
}

/// Replaces the scene with an empty bin and `count` balls above it: 0 on success, 1 when `count`
/// exceeds `MAX_BALLS`, 2 when the world or a body cannot be created (no scene is kept then).
#[no_mangle]
pub extern "C" fn create_scene(count: u32) -> i32 {
    if count > MAX_BALLS {
        return 1;
    }
    SCENE.with(|kept| {
        let mut kept = kept.borrow_mut();
        kept.take();
        *kept = new_scene(count);
        if kept.is_some() {
            0
        } else {
            2
        }
    })
}

fn new_scene(count: u32) -> Option<Scene> {
    let mut world = PhysicsWorld::new(WorldSettings::default()).ok()?;
    add_bin(&mut world)?;
    let ball = Shape::new_sphere(BALL_RADIUS).ok()?;
    let balls = (0..count)
        .map(|index| {
            let settings = BodySettings::new_dynamic().position(start_position(index));
            world.create_body(&ball, &settings).ok()
        })
        .collect::<Option<Vec<_>>>()?;
    Some(Scene {
        world,
        balls,
        poses: Vec::new(),
    })
}

/// The bin's static boxes as (half extents, centre) in metres: a floor whose top is at y = 0 and
/// four 20 m high walls around an inside 12 m wide (x) and 3 m deep (z).
fn add_bin(world: &mut PhysicsWorld) -> Option<()> {
    let parts = [
        (Vec3::new(7.0, 0.5, 2.5), RVec3::new(0.0, -0.5, 0.0)),
        (Vec3::new(0.5, 10.0, 2.5), RVec3::new(-6.5, 10.0, 0.0)),
        (Vec3::new(0.5, 10.0, 2.5), RVec3::new(6.5, 10.0, 0.0)),
        (Vec3::new(6.0, 10.0, 0.5), RVec3::new(0.0, 10.0, -2.0)),
        (Vec3::new(6.0, 10.0, 0.5), RVec3::new(0.0, 10.0, 2.0)),
    ];
    for (half_extents, position) in parts {
        let shape = Shape::new_box(half_extents).ok()?;
        world
            .create_body(&shape, &BodySettings::new_static().position(position))
            .ok()?;
    }
    Some(())
}

/// Start position of ball `index`: layers of 10 by 3 balls, each a little offset so the pile does
/// not stand as a column.
fn start_position(index: u32) -> RVec3 {
    let column = (index % 10) as Real;
    let row = (index / 10 % 3) as Real;
    let layer = (index / 30) as Real;
    let offset = (index * 37 % 11) as Real * 0.02;
    RVec3::new(
        -4.5 + column + offset,
        1.0 + layer * 0.7,
        -1.0 + row + offset,
    )
}

/// Steps the scene once: 0 on success, 1 without a scene, 2 when the step fails or is incomplete.
#[no_mangle]
pub extern "C" fn step_scene() -> i32 {
    SCENE.with(|kept| match kept.borrow_mut().as_mut() {
        None => 1,
        Some(scene) => match scene.world.step(DELTA_TIME) {
            Ok(report) if report.is_complete() => 0,
            _ => 2,
        },
    })
}

/// Number of balls in the scene; 0 without a scene.
#[no_mangle]
pub extern "C" fn ball_count() -> u32 {
    SCENE.with(|kept| {
        kept.borrow()
            .as_ref()
            .map_or(0, |scene| scene.balls.len() as u32)
    })
}

/// Radius of every ball, in metres.
#[no_mangle]
pub extern "C" fn ball_radius() -> f32 {
    BALL_RADIUS
}

/// Writes every ball's pose into the pose buffer and returns its address in the module's memory:
/// `ball_count()` records of `FLOATS_PER_POSE` floats. The address stays valid until the next call
/// of `ball_poses` or `create_scene`; null without a scene or when a ball is missing.
#[no_mangle]
pub extern "C" fn ball_poses() -> *const f32 {
    SCENE.with(|kept| {
        let mut kept = kept.borrow_mut();
        let Some(scene) = kept.as_mut() else {
            return std::ptr::null();
        };
        scene.poses.clear();
        scene.poses.reserve(scene.balls.len() * FLOATS_PER_POSE);
        for &id in &scene.balls {
            let Ok(ball) = scene.world.body(id) else {
                return std::ptr::null();
            };
            let (position, rotation) = (ball.position(), ball.rotation());
            scene.poses.extend([
                to_f32(position.x),
                to_f32(position.y),
                to_f32(position.z),
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ]);
        }
        scene.poses.as_ptr()
    })
}

/// A world coordinate as the page draws it.
// `Real` is `f32` without the `double-precision` feature.
#[allow(clippy::unnecessary_cast)]
fn to_f32(value: Real) -> f32 {
    value as f32
}
