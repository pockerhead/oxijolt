//! Solver step counts per world: bounds, read-back, their effect on a hinge chain, and equal
//! results with 1 and 4 worker threads.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::math::{add, norm, rotate, rvec3, sub, v3, V3};
use common::DT;
use oxijolt::*;

#[test]
fn step_counts_outside_jolts_range_are_refused() {
    let refused = [
        WorldSettings::default().velocity_steps(0),
        WorldSettings::default().velocity_steps(1),
        WorldSettings::default().velocity_steps(256),
        WorldSettings::default().velocity_steps(u32::MAX),
        WorldSettings::default().position_steps(256),
        WorldSettings::default().position_steps(u32::MAX),
    ];
    for settings in refused {
        assert!(
            matches!(
                PhysicsWorld::new(settings.clone()),
                Err(WorldError::InvalidValue(_))
            ),
            "{settings:?} was accepted"
        );
    }
}

#[test]
fn step_counts_read_back_from_jolt() {
    let world = PhysicsWorld::new(WorldSettings::default()).unwrap();
    assert_eq!((world.velocity_steps(), world.position_steps()), (10, 2));
    for (velocity, position) in [
        (WorldSettings::MIN_VELOCITY_STEPS, 0),
        (4, 2),
        (
            WorldSettings::MAX_SOLVER_STEPS,
            WorldSettings::MAX_SOLVER_STEPS,
        ),
    ] {
        let settings = WorldSettings::default()
            .velocity_steps(velocity)
            .position_steps(position);
        let world = PhysicsWorld::new(settings).unwrap();
        assert_eq!(
            (world.velocity_steps(), world.position_steps()),
            (velocity, position)
        );
    }
}

#[test]
fn a_stack_without_position_steps_stays_finite() {
    let settings = WorldSettings::default().velocity_steps(2).position_steps(0);
    let mut world = PhysicsWorld::new(settings).unwrap();
    common::add_floor(&mut world);
    let cubes: Vec<BodyId> = (0..5u8)
        .map(|i| common::add_cube(&mut world, RVec3::new(0.0, 0.5 + Real::from(i), 0.0)))
        .collect();
    common::step(&mut world, 60);
    for id in cubes {
        let body = world.body(id).unwrap();
        let position = v3(body.position());
        assert!(position.iter().all(|c| c.is_finite()), "{position:?}");
        assert!(position[1] > 0.0, "cube {id:?} fell through the floor");
    }
}

/// Links in the hinge chain.
const LINKS: usize = 10;

/// Height of the chain's static anchor, in metres.
const HEIGHT: f64 = 20.0;

/// A hinge chain hanging from a static anchor: small balls in a row, each joined to the one
/// before half a metre from its own centre, so the balls never touch and only the joints act.
struct Chain {
    world: PhysicsWorld,
    bodies: Vec<BodyId>,
    /// Per joint: the two bodies and the anchor relative to each body's centre at creation.
    joints: Vec<(BodyId, BodyId, V3, V3)>,
}

/// Builds the chain horizontally along +X, so gravity swings it down at once.
fn chain(settings: WorldSettings) -> Chain {
    let mut world = PhysicsWorld::new(settings).unwrap();
    let ball = Shape::new_sphere(0.1).unwrap();
    let anchor_centre = [0.0, HEIGHT, 0.0];
    let anchor = world
        .create_body(
            &ball,
            &BodySettings::new_static().position(rvec3(anchor_centre)),
        )
        .unwrap();
    let mut bodies = vec![anchor];
    let mut joints = Vec::new();
    let mut centres = vec![anchor_centre];
    for i in 0..LINKS {
        let centre = [0.5 + i as f64, HEIGHT, 0.0];
        let link = world
            .create_body(
                &ball,
                &BodySettings::new_dynamic()
                    .position(rvec3(centre))
                    .allow_sleeping(false),
            )
            .unwrap();
        let point = [i as f64, HEIGHT, 0.0];
        let previous = *bodies.last().unwrap();
        let hinge = HingeConstraintSettings::new(
            rvec3(point),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        );
        world.create_constraint(previous, link, &hinge).unwrap();
        joints.push((
            previous,
            link,
            sub(point, *centres.last().unwrap()),
            sub(point, centre),
        ));
        bodies.push(link);
        centres.push(centre);
    }
    Chain {
        world,
        bodies,
        joints,
    }
}

impl Chain {
    /// The largest distance between the two ends of any joint now.
    fn max_anchor_separation(&self) -> f64 {
        let anchor = |id: BodyId, local: V3| {
            let body = self.world.body(id).unwrap();
            add(v3(body.position()), rotate(body.rotation(), local))
        };
        self.joints
            .iter()
            .map(|&(a, b, local_a, local_b)| norm(sub(anchor(a, local_a), anchor(b, local_b))))
            .fold(0.0, f64::max)
    }

    fn step(&mut self) {
        assert!(self.world.step(DT).unwrap().is_complete());
    }
}

fn max_separation_over_60_ticks(velocity: u32, position: u32) -> f64 {
    let settings = WorldSettings::default()
        .velocity_steps(velocity)
        .position_steps(position);
    let mut chain = chain(settings);
    let mut max = 0.0f64;
    for _ in 0..60 {
        chain.step();
        max = max.max(chain.max_anchor_separation());
    }
    max
}

#[test]
fn fewer_solver_steps_let_a_hinge_chain_stretch_more() {
    let loose = max_separation_over_60_ticks(2, 0);
    let tight = max_separation_over_60_ticks(10, 2);
    assert!(
        loose > 2.0 * tight,
        "2/0 steps: {loose} m, 10/2 steps: {tight} m"
    );
}

/// The chain at 4 velocity and 2 position steps, every body's pose and velocities per tick.
fn run_chain(worker_threads: u32) -> Digest {
    let settings = WorldSettings::default()
        .velocity_steps(4)
        .position_steps(2)
        .worker_threads(worker_threads);
    let mut chain = chain(settings);
    let mut digest = Digest::new();
    for _ in 0..120 {
        chain.step();
        let tick = digest.push();
        for &id in &chain.bodies {
            let body = chain.world.body(id).unwrap();
            let rotation = body.rotation();
            let values = <[Real; 3]>::from(body.position())
                .map(|c| c.to_le_bytes().to_vec())
                .concat();
            tick.state.extend_from_slice(&values);
            for value in [rotation.x, rotation.y, rotation.z, rotation.w]
                .into_iter()
                .chain(<[f32; 3]>::from(body.linear_velocity()))
                .chain(<[f32; 3]>::from(body.angular_velocity()))
            {
                tick.state.extend_from_slice(&value.to_bits().to_le_bytes());
            }
        }
    }
    digest
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
#[ignore = "child process of the solver step determinism gate"]
fn solver_steps_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "chain");
    finish_child(&run_chain(threads));
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn chain_at_four_steps_is_identical_with_1_and_4_workers() {
    let one = digest_in_child("solver_steps_child", "chain", 1, "");
    let four = digest_in_child("solver_steps_child", "chain", 4, "");
    assert_eq!(one.ticks.len(), 120);
    assert_ne!(one.ticks[0], one.ticks[119], "the chain did not move");
    assert_same("chain at 4/2 steps, 1 vs 4 workers", &one, &four);
}
