//! Determinism of the body controls: a scene that uses every control at fixed ticks runs bit for
//! bit the same with 1 and 4 worker threads, in one process and in two.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::ragdoll::mul;
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const TICKS: usize = 240;

/// A floor, a pile of cubes, a kinematic platform carrying a cube past a static sensor, a
/// kinematic sensor, a `PLANE_2D` body, a body created asleep and a static body that may move.
struct Scene {
    world: PhysicsWorld,
    bodies: Vec<BodyId>,
    pile: Vec<BodyId>,
    platform: BodyId,
    plane: BodyId,
    sleeper: BodyId,
    movable: BodyId,
}

impl Scene {
    fn new(threads: u32) -> Self {
        let mut world = world(GRAVITY, threads);
        world.set_event_settings(
            EventSettings::default()
                .contacts(true)
                .persisted_contacts(true)
                .body_activation(true),
        );
        let mut bodies = vec![add_floor(&mut world)];
        let pile: Vec<BodyId> = (0..6)
            .map(|i| {
                let (column, layer) = ((i % 3) as Real, (i / 3) as Real);
                add_cube(
                    &mut world,
                    RVec3::new(1.1 * column, 0.5 + 1.05 * layer, 0.0),
                )
            })
            .collect();
        bodies.extend(&pile);
        let deck = Shape::new_box(Vec3::new(1.0, 0.1, 1.0)).unwrap();
        let platform = world
            .create_body(
                &Shape::new_offset_center_of_mass(&deck, Vec3::new(0.2, 0.0, 0.0)).unwrap(),
                &BodySettings::new_kinematic().position(RVec3::new(-6.0, 1.0, 0.0)),
            )
            .unwrap();
        let rider = add_cube(&mut world, RVec3::new(-6.0, 1.65, 0.0));
        let sensor = world
            .create_body(
                &Shape::new_box(Vec3::new(1.0, 2.0, 1.0)).unwrap(),
                &BodySettings::new_static()
                    .position(RVec3::new(-2.0, 2.0, 0.0))
                    .sensor(true),
            )
            .unwrap();
        let scanner = world
            .create_body(
                &Shape::new_box(Vec3::new(2.0, 1.0, 2.0)).unwrap(),
                &BodySettings::new_kinematic()
                    .position(RVec3::new(1.0, 1.0, 0.0))
                    .linear_velocity(Vec3::new(0.0, 0.0, 0.5))
                    .sensor(true),
            )
            .unwrap();
        let plane = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_dynamic()
                    .position(RVec3::new(5.0, 3.0, 0.0))
                    .linear_velocity(Vec3::new(-1.0, 0.0, 2.0))
                    .angular_velocity(Vec3::new(1.0, 2.0, 3.0))
                    .allowed_dofs(AllowedDofs::PLANE_2D),
            )
            .unwrap();
        let sleeper = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_dynamic()
                    .position(RVec3::new(8.0, 0.5, 0.0))
                    .activation(Activation::DontActivate),
            )
            .unwrap();
        let movable = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_static()
                    .position(RVec3::new(8.0, 3.0, 0.0))
                    .object_layer(ObjectLayer::MOVING)
                    .allow_dynamic_or_kinematic(true)
                    .user_data(99),
            )
            .unwrap();
        bodies.extend([platform, rider, sensor, scanner, plane, sleeper, movable]);
        Self {
            world,
            bodies,
            pile,
            platform,
            plane,
            sleeper,
            movable,
        }
    }

    /// The controls of `tick`, then one step.
    fn tick(&mut self, tick: usize) {
        let platform = self.world.body(self.platform).unwrap();
        let (at, rotation) = (platform.position(), platform.rotation());
        let turn = mul(quat_about(Vec3::new(0.0, 1.0, 0.0), 0.004), rotation);
        self.world
            .body_mut(self.platform)
            .unwrap()
            .move_kinematic(RVec3::new(at.x + 0.05, 1.0, at.z), turn, DT)
            .unwrap();
        let world = &mut self.world;
        match tick {
            10 => world
                .body_mut(self.pile[4])
                .unwrap()
                .add_impulse(Vec3::new(3.0, 2.0, -1.0))
                .unwrap(),
            15 => world
                .body_mut(self.pile[1])
                .unwrap()
                .add_angular_impulse(Vec3::new(0.0, 0.2, 0.1))
                .unwrap(),
            20 => {
                let at = world.body(self.pile[5]).unwrap().position();
                world
                    .body_mut(self.pile[5])
                    .unwrap()
                    .add_impulse_at_point(
                        Vec3::new(0.0, 5.0, 2.0),
                        RVec3::new(at.x + 0.4, at.y, at.z),
                    )
                    .unwrap();
            }
            25 => world
                .body_mut(self.movable)
                .unwrap()
                .set_motion_type(MotionType::Dynamic, Activation::Activate)
                .unwrap(),
            28 => world
                .body_mut(self.plane)
                .unwrap()
                .set_shape(
                    &Shape::new_sphere(0.6).unwrap(),
                    Some(2.0),
                    Activation::Activate,
                )
                .unwrap(),
            60 => world.body_mut(self.pile[0]).unwrap().deactivate().unwrap(),
            90 => world
                .activate_bodies_in_box(RVec3::new(7.0, 0.0, -1.0), RVec3::new(9.0, 1.0, 1.0))
                .unwrap(),
            120 => world.body_mut(self.pile[0]).unwrap().activate(),
            150 => world.body_mut(self.sleeper).unwrap().deactivate().unwrap(),
            _ => {}
        }
        step(&mut self.world, 1);
    }

    /// Every body's state and configuration, and the step's events.
    fn record(&mut self, digest: &mut Digest) {
        let tick = digest.push();
        for &id in &self.bodies {
            record_body(&self.world, id, &mut tick.state);
            let body = self.world.body(id).unwrap();
            let configuration = format!(
                "{} {} {:?} {} {:?}",
                body.is_sensor(),
                body.user_data(),
                body.allowed_dofs(),
                body.can_be_kinematic_or_dynamic(),
                body.motion_type(),
            );
            tick.state.extend(configuration.bytes());
        }
        // Body ids print without their world, and floats in their shortest exact form.
        let events = self.world.take_events();
        tick.state
            .extend(format!("{:?}{:?}", events.contacts, events.activations).bytes());
    }
}

/// The scene run for [`TICKS`] ticks with `threads` workers.
fn controls(threads: u32) -> Digest {
    let mut scene = Scene::new(threads);
    let mut digest = Digest::new();
    for tick in 1..=TICKS {
        scene.tick(tick);
        scene.record(&mut digest);
    }
    digest
}

#[test]
#[ignore = "child process of the body control determinism gate"]
fn body_controls_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "controls");
    finish_child(&controls(threads));
}

#[test]
fn controls_match_with_1_and_4_workers() {
    let one = controls(1);
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in one process", &one, &controls(4));
    // The scene moves and records sensor contacts and activation changes.
    assert_ne!(one.ticks[0].state, one.ticks[TICKS - 1].state);
    let text: String = one
        .ticks
        .iter()
        .map(|tick| String::from_utf8_lossy(&tick.state).into_owned())
        .collect();
    for needle in ["is_sensor: true", "Activated(", "Deactivated("] {
        assert!(text.contains(needle), "no {needle}");
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn controls_match_across_processes() {
    let one = digest_in_child("body_controls_child", "controls", 1, "");
    let four = digest_in_child("body_controls_child", "controls", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &controls(1), &one);
}
