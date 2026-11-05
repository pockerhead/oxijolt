//! Deactivating bodies in one batch: the same result as `BodyMut::deactivate` for each id in the
//! same order, and nothing changed by a refused call.

// The thread tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::events::add_cloth;
use common::*;
use oxijolt::*;

const SETTLE_TICKS: usize = 20;
const TICKS: usize = 90;

/// A world without gravity that records activation events.
fn activation_world() -> PhysicsWorld {
    let mut world = world(Vec3::ZERO, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    world
}

/// A dynamic unit cube at `position` with `velocity`, awake or asleep as `activation` says.
fn moving_cube(
    world: &mut PhysicsWorld,
    position: RVec3,
    velocity: Vec3,
    activation: Activation,
) -> BodyId {
    let shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let settings = BodySettings::new_dynamic()
        .position(position)
        .linear_velocity(velocity)
        .angular_velocity(Vec3::new(0.0, 0.7, 0.0))
        .activation(activation);
    world.create_body(&shape, &settings).unwrap()
}

/// A floor, two layers of cubes falling onto it, a cube thrown sideways into the lower layer and
/// two cubes created asleep in the air with velocities. After [`SETTLE_TICKS`] ticks the lower
/// layer, the sleepers and the floor are deactivated in a mixed order, one batch or one by one;
/// the upper layer and the thrown cube stay awake and wake parts of the lower layer again.
/// Records every body's state and the activation events of each tick.
fn deactivation_run(threads: u32, batched: bool) -> Vec<u8> {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), threads);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let floor = add_floor(&mut world);
    let mut lower = Vec::new();
    let mut upper = Vec::new();
    for x in 0..3 {
        for z in 0..3 {
            let (x, z) = (1.1 * x as Real, 1.1 * z as Real);
            lower.push(add_cube(&mut world, RVec3::new(x, 0.6, z)));
            upper.push(add_cube(&mut world, RVec3::new(x + 0.3, 2.0, z)));
        }
    }
    let thrown = moving_cube(
        &mut world,
        RVec3::new(-4.0, 0.6, 1.1),
        Vec3::new(6.0, 0.0, 0.0),
        Activation::Activate,
    );
    let sleepers = [
        moving_cube(
            &mut world,
            RVec3::new(8.0, 3.0, 0.0),
            Vec3::new(1.0, 2.0, 0.0),
            Activation::DontActivate,
        ),
        moving_cube(
            &mut world,
            RVec3::new(-8.0, 3.0, 0.0),
            Vec3::new(0.0, 0.0, -3.0),
            Activation::DontActivate,
        ),
    ];
    let mut all: Vec<BodyId> = world.body_ids().collect();
    all.retain(|&id| id != floor);

    let mut digest = Vec::new();
    let mut rewoken = false;
    for tick in 1..=TICKS {
        if tick == SETTLE_TICKS {
            let chosen = [
                lower[4],
                sleepers[1],
                lower[8],
                floor,
                lower[0],
                lower[2],
                sleepers[0],
                lower[6],
                lower[1],
                lower[7],
                lower[3],
                lower[5],
            ];
            if batched {
                world.deactivate_bodies(&chosen).unwrap();
            } else {
                for id in chosen {
                    world.body_mut(id).unwrap().deactivate().unwrap();
                }
            }
            for id in chosen.into_iter().filter(|&id| id != floor) {
                let body = world.body(id).unwrap();
                assert!(body.is_sleeping());
                assert_eq!(body.linear_velocity(), Vec3::ZERO);
                assert_eq!(body.angular_velocity(), Vec3::ZERO);
            }
            assert!(upper.iter().all(|&id| world.body(id).unwrap().is_active()));
            digest.extend(format!("{:?}", world.take_events().activations).bytes());
        }
        assert!(world.step(DT).unwrap().is_complete());
        for &id in &all {
            record_body(&world, id, &mut digest);
        }
        let activations = world.take_events().activations;
        rewoken |= tick >= SETTLE_TICKS
            && activations
                .iter()
                .any(|event| matches!(event, ActivationEvent::Activated(id) if lower.contains(id)));
        digest.extend(format!("{activations:?}").bytes());
    }
    assert!(rewoken, "nothing woke the deactivated layer again");
    assert_ne!(world.body(thrown).unwrap().position().x, -4.0);
    digest
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn deactivate_bodies_equals_deactivate_one_by_one_with_1_and_4_workers() {
    let one = deactivation_run(1, true);
    assert!(
        one == deactivation_run(1, false),
        "1 worker: batch and one by one differ"
    );
    let four = deactivation_run(4, true);
    assert!(
        four == deactivation_run(4, false),
        "4 workers: batch and one by one differ"
    );
    assert!(one == four, "1 and 4 workers differ");
}

#[test]
fn deactivate_bodies_reports_the_awake_bodies_in_the_given_order() {
    let mut world = activation_world();
    let a = add_cube(&mut world, RVec3::ZERO);
    let b = add_cube(&mut world, RVec3::new(3.0, 0.0, 0.0));
    let asleep = moving_cube(
        &mut world,
        RVec3::new(6.0, 0.0, 0.0),
        Vec3::ZERO,
        Activation::DontActivate,
    );
    let c = add_cube(&mut world, RVec3::new(9.0, 0.0, 0.0));
    let fixed = world
        .create_body(
            &Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap(),
            &BodySettings::new_static().position(RVec3::new(12.0, 0.0, 0.0)),
        )
        .unwrap();
    world.take_events();
    let ids = [c, fixed, a, asleep, b];
    world.deactivate_bodies(&ids).unwrap();
    assert_eq!(ids, [c, fixed, a, asleep, b]);
    assert_eq!(
        world.take_events().activations,
        [
            ActivationEvent::Deactivated(c),
            ActivationEvent::Deactivated(a),
            ActivationEvent::Deactivated(b),
        ]
    );
    assert!([a, b, c, asleep]
        .iter()
        .all(|&id| world.body(id).unwrap().is_sleeping()));
    // A second call finds them asleep and reports nothing.
    world.deactivate_bodies(&ids).unwrap();
    world.deactivate_bodies(&[]).unwrap();
    assert!(world.take_events().activations.is_empty());
}

#[test]
fn deactivate_bodies_zeroes_the_velocities_of_sleepers() {
    let mut world = activation_world();
    let asleep = moving_cube(
        &mut world,
        RVec3::ZERO,
        Vec3::new(4.0, 0.0, 0.0),
        Activation::DontActivate,
    );
    assert_eq!(
        world.body(asleep).unwrap().linear_velocity(),
        Vec3::new(4.0, 0.0, 0.0)
    );
    world.deactivate_bodies(&[asleep]).unwrap();
    let body = world.body(asleep).unwrap();
    assert!(body.is_sleeping());
    assert_eq!(body.linear_velocity(), Vec3::ZERO);
    assert_eq!(body.angular_velocity(), Vec3::ZERO);
    assert!(world.take_events().activations.is_empty());
    // Waking it does not bring the velocity back.
    world.body_mut(asleep).unwrap().activate();
    step(&mut world, 5);
    assert_eq!(world.body(asleep).unwrap().position(), RVec3::ZERO);
}

#[test]
fn deactivate_bodies_checks_every_id_first_and_changes_nothing_on_an_error() {
    let mut world = activation_world();
    let velocity = Vec3::new(1.0, 0.0, 0.0);
    let awake = moving_cube(&mut world, RVec3::ZERO, velocity, Activation::Activate);
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 5.0, 0.0), Quat::IDENTITY);
    let detached = world
        .create_body_detached(
            &Shape::new_sphere(0.5).unwrap(),
            &BodySettings::new_dynamic().position(RVec3::new(5.0, 0.0, 0.0)),
        )
        .unwrap();
    let removed = add_cube(&mut world, RVec3::new(-5.0, 0.0, 0.0));
    world.remove_body(removed).unwrap();
    let other = {
        let mut other_world = activation_world();
        add_cube(&mut other_world, RVec3::ZERO)
    };
    world.take_events();

    let cases = [
        ([awake, cloth], BodyError::NotRigidBody(cloth)),
        ([awake, detached], BodyError::Detached(detached)),
        ([awake, removed], BodyError::NotFound(removed)),
        ([awake, other], BodyError::WrongWorld(other)),
        ([awake, awake], BodyError::DuplicateId(awake)),
    ];
    for (ids, error) in cases {
        assert_eq!(world.deactivate_bodies(&ids), Err(error));
        let body = world.body(awake).unwrap();
        assert!(body.is_active());
        assert_eq!(body.linear_velocity(), velocity);
        assert!(world.body(cloth).unwrap().is_active());
        assert!(world.take_events().activations.is_empty());
    }
}
