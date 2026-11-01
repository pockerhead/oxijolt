//! Same-machine determinism of [`PhysicsWorld::active_body_poses`]: the poses of the awake
//! bodies, in the order the call returns them, must be bit-identical whether the world steps
//! with 1 or 4 worker threads or on a caller job system.
//!
//! The "wake" scene puts a sleeping 8 x 8 x 3 grid of small cubes on a floor and drops four
//! spheres onto it. The spheres wake the cubes they hit and the cubes wake their neighbours, from
//! several jobs at once, so Jolt's own active list fills in a thread-dependent order; later the
//! grid falls asleep again. Every tick records the returned ids and poses, and checks them
//! against the per-body readout.
//!
//! Each run happens in its own child process (this test binary, running the ignored
//! `body_poses_child` test), so no state leaks between runs; see `common::determinism`.

mod common;

use common::determinism::*;
use common::jobs::{self, JobChoice};
use common::{add_floor, DT};
use oxijolt::*;

/// Cubes along each horizontal side of the grid, and its layers.
const GRID_SIDE: usize = 8;
const GRID_LAYERS: usize = 3;
/// Half extent of a grid cube (its shape's half extent), metres.
const CUBE_HALF: Real = 0.125;
/// Gap between neighbouring cubes, metres.
const GAP: Real = 0.001;
const TICKS: usize = 300;

/// The sleeping cube grid and the four awake spheres above its quadrants, in creation order.
fn build_wake_scene(world: &mut PhysicsWorld) -> Vec<BodyId> {
    add_floor(world);
    let cube = Shape::new_box(Vec3::new(0.125, 0.125, 0.125)).unwrap();
    let pitch = 2.0 * CUBE_HALF + GAP;
    let offset = (GRID_SIDE - 1) as Real * pitch / 2.0;
    let mut ids = Vec::new();
    for layer in 0..GRID_LAYERS {
        for row in 0..GRID_SIDE {
            for column in 0..GRID_SIDE {
                let position = RVec3::new(
                    column as Real * pitch - offset,
                    CUBE_HALF + GAP + layer as Real * pitch,
                    row as Real * pitch - offset,
                );
                let settings = BodySettings::new_dynamic()
                    .position(position)
                    .activation(Activation::DontActivate);
                ids.push(world.create_body(&cube, &settings).unwrap());
            }
        }
    }
    let sphere = Shape::new_sphere(0.5).unwrap();
    for (x, z) in [(-0.5, -0.5), (0.5, -0.5), (-0.5, 0.5), (0.5, 0.5)] {
        let settings = BodySettings::new_dynamic().position(RVec3::new(x, 2.0, z));
        ids.push(world.create_body(&sphere, &settings).unwrap());
    }
    ids
}

/// Checks the returned poses against the per-body readout: ascending ids, exactly the awake
/// bodies of `created`, and the same bits as `world.body(id)`.
fn check_against_bodies(world: &PhysicsWorld, created: &[BodyId], poses: &[BodyPose]) {
    assert!(poses.windows(2).all(|pair| pair[0].id < pair[1].id));
    let mut awake: Vec<BodyId> = created
        .iter()
        .copied()
        .filter(|&id| world.body(id).unwrap().is_active())
        .collect();
    awake.sort_unstable();
    let ids: Vec<BodyId> = poses.iter().map(|pose| pose.id).collect();
    assert_eq!(ids, awake);
    for pose in poses {
        let body = world.body(pose.id).unwrap();
        assert_eq!(position_bits(pose.position), position_bits(body.position()));
        assert_eq!(rotation_bits(pose.rotation), rotation_bits(body.rotation()));
    }
}

fn position_bits(p: RVec3) -> [Vec<u8>; 3] {
    [p.x, p.y, p.z].map(|c| c.to_bits().to_le_bytes().to_vec())
}

fn rotation_bits(q: Quat) -> [u32; 4] {
    [q.x, q.y, q.z, q.w].map(f32::to_bits)
}

/// Runs the wake scene with `threads` workers and records the poses of every tick.
fn run_wake(threads: u32) -> Digest {
    let mut world = common::world(Vec3::new(0.0, -9.81, 0.0), threads);
    let created = build_wake_scene(&mut world);
    let mut digest = Digest::new();
    let mut poses = Vec::new();
    let mut peak = 0;
    for _ in 0..TICKS {
        assert!(world.step(DT).unwrap().is_complete());
        world.active_body_poses_into(&mut poses);
        check_against_bodies(&world, &created, &poses);
        peak = peak.max(poses.len());
        let state = &mut digest.push().state;
        state.extend((poses.len() as u32).to_le_bytes());
        for pose in &poses {
            state.extend(pose.id.to_raw().to_le_bytes());
            for bytes in position_bits(pose.position) {
                state.extend(bytes);
            }
            for bits in rotation_bits(pose.rotation) {
                state.extend(bits.to_le_bytes());
            }
        }
    }
    // Both waking and falling asleep happened, so the active list grew and shrank.
    assert!(peak > 150, "only {peak} bodies were awake at once");
    assert!(poses.len() < peak, "{} of {peak} still awake", poses.len());
    digest
}

#[test]
#[ignore = "run by the other tests of this file in a child process"]
fn body_poses_child() {
    let Some((scenario, threads, _variant)) = child_request() else {
        return;
    };
    let digest = match scenario.as_str() {
        "wake" => run_wake(threads),
        scenario => panic!("unknown scenario {scenario}"),
    };
    // Without this a job system choice that never reached the world would pass as the native run.
    match JobChoice::from_env() {
        JobChoice::Native | JobChoice::SingleThreaded => {
            assert_eq!(jobs::queued(), 0, "a caller job system was used")
        }
        choice => assert!(
            jobs::queued() > 0,
            "the {choice:?} job system was handed no job"
        ),
    }
    finish_child(&digest);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn poses_are_identical_across_thread_counts() {
    let one_thread = digest_in_child("body_poses_child", "wake", 1, "forward");
    let four_threads = digest_in_child("body_poses_child", "wake", 4, "forward");
    assert_eq!(one_thread.ticks.len(), TICKS);
    assert_same("wake, 1 vs 4 worker threads", &one_thread, &four_threads);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn poses_are_identical_with_caller_job_systems() {
    assert_caller_job_systems_agree("body_poses_child", "wake", "forward");
}
