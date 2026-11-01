//! Steps split into several collision steps: the bounds on the count and on each collision step's
//! length, that the count reaches Jolt and covers the whole `delta_time`, how forces clear, and
//! the single event batch of a call, and a vehicle's gravity override in every collision step.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::math::wide;
use common::*;
use oxijolt::*;

const MAX: u32 = PhysicsWorld::MAX_COLLISION_STEPS;

const NO_GRAVITY: Vec3 = Vec3::new(0.0, 0.0, 0.0);
const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// A dynamic unit cube of 1 kg with `velocity`, no damping and no sleeping.
fn add_free_cube(world: &mut PhysicsWorld, position: RVec3, velocity: Vec3) -> BodyId {
    let shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let settings = BodySettings::new_dynamic()
        .position(position)
        .linear_velocity(velocity)
        .linear_damping(0.0)
        .angular_damping(0.0)
        .mass(1.0)
        .allow_sleeping(false);
    world.create_body(&shape, &settings).unwrap()
}

fn record(world: &PhysicsWorld, id: BodyId) -> Vec<u8> {
    let mut bytes = Vec::new();
    record_body(world, id, &mut bytes);
    bytes
}

fn assert_relative(actual: f64, expected: f64, tolerance: f64, what: &str) {
    let error = ((actual - expected) / expected).abs();
    assert!(
        error <= tolerance,
        "{what}: {actual} vs {expected}, relative error {error}"
    );
}

#[test]
fn collision_step_counts_outside_the_bound_are_refused() {
    let mut world = world(GRAVITY, 1);
    let cube = add_cube(&mut world, RVec3::new(0.0, 2.0, 0.0));
    let before = record(&world, cube);
    for count in [0, MAX + 1, u32::MAX] {
        assert!(
            matches!(
                world.step_with_collision_steps(DT, count),
                Err(StepError::InvalidValue(_))
            ),
            "{count}"
        );
        assert_eq!(
            record(&world, cube),
            before,
            "the world advanced with {count}"
        );
    }
    for count in [1, MAX] {
        assert!(world
            .step_with_collision_steps(DT, count)
            .unwrap()
            .is_complete());
    }
    assert_ne!(record(&world, cube), before);
}

#[test]
fn bad_total_delta_times_are_refused_for_every_count() {
    let mut world = world(GRAVITY, 1);
    let cube = add_cube(&mut world, RVec3::new(0.0, 2.0, 0.0));
    let before = record(&world, cube);
    for count in 1..=MAX {
        for dt in [
            f32::NAN,
            f32::INFINITY,
            -1.0 / 60.0,
            0.0,
            -0.0,
            PhysicsWorld::MIN_DELTA_TIME.next_down(),
            PhysicsWorld::MAX_DELTA_TIME.next_up(),
        ] {
            assert_eq!(
                world.step_with_collision_steps(dt, count),
                Err(StepError::InvalidDeltaTime),
                "dt {dt}, {count} collision steps"
            );
            assert_eq!(record(&world, cube), before, "dt {dt}, {count}");
        }
    }
}

#[test]
fn collision_steps_shorter_than_min_delta_time_are_refused() {
    let mut world = world(GRAVITY, 1);
    let cube = add_cube(&mut world, RVec3::new(0.0, 2.0, 0.0));
    for count in 2..=MAX {
        let n = count as f32;
        // The smallest `x` whose collision step `x / n`, divided in `f32`, is long enough.
        let mut x = PhysicsWorld::MIN_DELTA_TIME * n;
        while x / n >= PhysicsWorld::MIN_DELTA_TIME {
            x = x.next_down();
        }
        while x / n < PhysicsWorld::MIN_DELTA_TIME {
            x = x.next_up();
        }
        let before = record(&world, cube);
        assert_eq!(
            world.step_with_collision_steps(x.next_down(), count),
            Err(StepError::InvalidDeltaTime),
            "{count}"
        );
        assert_eq!(record(&world, cube), before, "{count}");
        assert!(world.step_with_collision_steps(x, count).is_ok(), "{count}");
    }
}

/// A floor, a falling cube with a pending force and a resting cube with a pending contact cache
/// invalidation, after five steps, with contact events on and the events so far taken.
fn pending_world() -> (PhysicsWorld, Vec<BodyId>) {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().persisted_contacts(true));
    let floor = add_floor(&mut world);
    let falling = add_cube(&mut world, RVec3::new(3.0, 2.0, 0.0));
    let resting = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    step(&mut world, 5);
    world
        .body_mut(falling)
        .unwrap()
        .add_force(Vec3::new(20.0, 0.0, 0.0))
        .unwrap();
    world.body_mut(resting).unwrap().invalidate_contact_cache();
    let _ = world.take_events();
    (world, vec![floor, falling, resting])
}

#[test]
fn a_refused_call_consumes_nothing() {
    let (mut refused, ids) = pending_world();
    let (mut control, control_ids) = pending_world();
    assert!(matches!(
        refused.step_with_collision_steps(DT, 0),
        Err(StepError::InvalidValue(_))
    ));
    for world in [&mut refused, &mut control] {
        assert!(world
            .step_with_collision_steps(DT, MAX)
            .unwrap()
            .is_complete());
    }
    for (&id, &control_id) in ids.iter().zip(&control_ids) {
        assert_eq!(record(&refused, id), record(&control, control_id));
    }
    // Ids carry their world, so the events of two worlds compare through their Debug text, which
    // prints every value exactly.
    let events = format!("{:?}", refused.take_events());
    assert!(events.contains("Persisted"), "{events}");
    assert_eq!(events, format!("{:?}", control.take_events()));
}

#[test]
fn a_refused_call_keeps_the_queued_events() {
    let (mut refused, _) = pending_world();
    let (mut control, _) = pending_world();
    for world in [&mut refused, &mut control] {
        assert!(world.step(DT).unwrap().is_complete());
    }
    // A valid total whose collision steps are too short.
    let total = PhysicsWorld::MIN_DELTA_TIME * 2.0;
    assert_eq!(
        refused.step_with_collision_steps(total, MAX),
        Err(StepError::InvalidDeltaTime)
    );
    let events = format!("{:?}", refused.take_events());
    assert!(events.contains("Persisted"), "{events}");
    assert_eq!(events, format!("{:?}", control.take_events()));
}

#[test]
fn collision_steps_cover_the_whole_delta_time() {
    for count in 1..=MAX {
        let mut world = world(NO_GRAVITY, 1);
        let velocity = Vec3::new(3.0, 0.0, 0.0);
        let cube = add_free_cube(&mut world, RVec3::ZERO, velocity);
        assert!(world
            .step_with_collision_steps(DT, count)
            .unwrap()
            .is_complete());
        let body = world.body(cube).unwrap();
        let moved = wide(body.position().x);
        assert_relative(moved, 3.0 * f64::from(DT), 1e-5, &format!("{count}"));
        assert_eq!(body.linear_velocity(), velocity, "{count}");
    }
}

#[test]
fn the_collision_step_count_reaches_jolt() {
    for count in 1..=MAX {
        let mut world = world(GRAVITY, 1);
        let cube = add_free_cube(&mut world, RVec3::ZERO, Vec3::ZERO);
        assert!(world
            .step_with_collision_steps(DT, count)
            .unwrap()
            .is_complete());
        let body = world.body(cube).unwrap();
        // Semi-implicit Euler over `n` collision steps of `dt / n`: the velocity gains `g dt / n`
        // before each position update, so the cube falls `g dt² (n + 1) / (2n)`.
        let (g, dt, n) = (-9.81, f64::from(DT), f64::from(count));
        let fallen = wide(body.position().y);
        assert_relative(
            fallen,
            g * dt * dt * (n + 1.0) / (2.0 * n),
            1e-4,
            &format!("{count}"),
        );
        let velocity = f64::from(body.linear_velocity().y);
        assert_relative(velocity, g * dt, 1e-5, &format!("{count}"));
    }
}

#[test]
fn a_rigid_force_acts_over_all_collision_steps_and_clears_after_the_call() {
    let mut world = world(NO_GRAVITY, 1);
    let cube = add_free_cube(&mut world, RVec3::ZERO, Vec3::ZERO);
    let force = Vec3::new(2.0, 0.0, 0.0);
    world.body_mut(cube).unwrap().add_force(force).unwrap();
    assert!(world
        .step_with_collision_steps(DT, MAX)
        .unwrap()
        .is_complete());
    let mass = f64::from(world.body(cube).unwrap().mass().unwrap());
    let velocity = world.body(cube).unwrap().linear_velocity();
    assert_relative(
        f64::from(velocity.x),
        f64::from(force.x) / mass * f64::from(DT),
        1e-4,
        "velocity after the force",
    );
    assert!(world
        .step_with_collision_steps(DT, MAX)
        .unwrap()
        .is_complete());
    assert_eq!(world.body(cube).unwrap().linear_velocity(), velocity);
}

/// A 1500 kg wheeled vehicle without damping or ground, at rest at the origin (where `f32`
/// positions resolve its millimetre fall), under `GRAVITY` either as world gravity or as the
/// vehicle's override (world gravity zero), with a caller force on the chassis, after one call
/// of `collision_steps` collision steps on `settings`' job system. Returns the chassis' x and y
/// position and velocity.
fn falling_vehicle(
    settings: WorldSettings,
    collision_steps: u32,
    gravity_override: bool,
    force: Vec3,
) -> ([f64; 2], [f64; 2]) {
    let world_gravity = if gravity_override {
        NO_GRAVITY
    } else {
        GRAVITY
    };
    let mut world = PhysicsWorld::new(settings.gravity(world_gravity)).unwrap();
    let shape = Shape::new_box(Vec3::new(0.9, 0.3, 2.0)).unwrap();
    let body_settings = BodySettings::new_dynamic()
        .mass(1500.0)
        .linear_damping(0.0)
        .angular_damping(0.0)
        .allow_sleeping(false);
    let chassis = world.create_body(&shape, &body_settings).unwrap();
    let wheels = [(-0.9, -1.4), (0.9, -1.4), (-0.9, 1.4), (0.9, 1.4)]
        .into_iter()
        .map(|(x, z)| WheelSettings::new(Vec3::new(x, -0.1, z)))
        .collect();
    let car_settings = WheeledVehicleSettings::new(
        wheels,
        vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
        VehicleCollisionTester::ray(ObjectLayer::MOVING),
    );
    let car = world
        .create_wheeled_vehicle(chassis, &car_settings)
        .unwrap();
    if gravity_override {
        world
            .vehicle_mut(car)
            .unwrap()
            .set_gravity(GRAVITY)
            .unwrap();
    }
    world.body_mut(chassis).unwrap().add_force(force).unwrap();
    assert!(world
        .step_with_collision_steps(DT, collision_steps)
        .unwrap()
        .is_complete());
    let body = world.body(chassis).unwrap();
    let (position, velocity) = (body.position(), body.linear_velocity());
    (
        [wide(position.x), wide(position.y)],
        [f64::from(velocity.x), f64::from(velocity.y)],
    )
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn a_vehicle_gravity_override_acts_like_world_gravity_in_every_collision_step() {
    // 2 m/s² sideways and 1 m/s² up on 1500 kg, so the override must keep the caller's force.
    let force = Vec3::new(3000.0, 1500.0, 0.0);
    let acceleration = [2.0, -9.81 + 1.0];
    for settings in [
        WorldSettings::default().worker_threads(4),
        WorldSettings::default().single_threaded(),
    ] {
        for count in 1..=MAX {
            let what = format!("{count} collision steps, {settings:?}");
            let (moved, velocity) = falling_vehicle(settings.clone(), count, true, force);
            let (control_moved, control_velocity) =
                falling_vehicle(settings.clone(), count, false, force);
            // The semi-implicit Euler fall of `the_collision_step_count_reaches_jolt`.
            let (dt, n) = (f64::from(DT), f64::from(count));
            for (axis, a) in [(0, acceleration[0]), (1, acceleration[1])] {
                let expected_velocity = a * dt;
                let expected_moved = a * dt * dt * (n + 1.0) / (2.0 * n);
                for (actual, expected, part) in [
                    (velocity[axis], expected_velocity, "velocity"),
                    (
                        control_velocity[axis],
                        expected_velocity,
                        "world gravity velocity",
                    ),
                    (moved[axis], expected_moved, "displacement"),
                    (
                        control_moved[axis],
                        expected_moved,
                        "world gravity displacement",
                    ),
                ] {
                    assert_relative(actual, expected, 1e-4, &format!("{what}: {part} {axis}"));
                }
                assert_relative(
                    velocity[axis],
                    control_velocity[axis],
                    1e-6,
                    &format!("{what}: override against world gravity, axis {axis}"),
                );
            }
        }
    }
}

#[test]
fn a_soft_body_force_acts_in_the_first_collision_step_only() {
    let mut world = world(NO_GRAVITY, 1);
    let shared = soft_body::tetrahedral_cube(1.0).build().unwrap();
    let settings = SoftBodySettings::default()
        .linear_damping(0.0)
        .allow_sleeping(false);
    let id = world.create_soft_body(&shared, &settings).unwrap();
    // Large enough that the velocity Jolt derives from the vertex positions is exact to 1e-4.
    let force = Vec3::new(8000.0, 0.0, 0.0);
    world.body_mut(id).unwrap().add_force(force).unwrap();
    assert!(world
        .step_with_collision_steps(DT, MAX)
        .unwrap()
        .is_complete());
    let first = world.soft_body(id).unwrap().vertices();
    assert_eq!(first.len(), 8);
    let collision_step = f64::from(DT) / f64::from(MAX);
    for vertex in &first {
        let expected = f64::from(force.x) * f64::from(vertex.inverse_mass) / 8.0 * collision_step;
        assert_relative(
            f64::from(vertex.velocity.x),
            expected,
            1e-3,
            "vertex velocity",
        );
    }
    assert!(world
        .step_with_collision_steps(DT, MAX)
        .unwrap()
        .is_complete());
    let second = world.soft_body(id).unwrap().vertices();
    for (before, after) in first.iter().zip(&second) {
        assert_relative(
            f64::from(after.velocity.x),
            f64::from(before.velocity.x),
            1e-4,
            "vertex velocity after the call",
        );
    }
}

#[test]
fn one_collision_step_equals_step() {
    let mut stepped = world(GRAVITY, 1);
    let ids = build_stacks(&mut stepped);
    let mut split = world(GRAVITY, 1);
    let split_ids = build_stacks(&mut split);
    for tick in 0..120 {
        assert!(stepped.step(DT).unwrap().is_complete());
        assert!(split
            .step_with_collision_steps(DT, 1)
            .unwrap()
            .is_complete());
        for (&id, &split_id) in ids.iter().zip(&split_ids) {
            assert_eq!(
                record(&stepped, id),
                record(&split, split_id),
                "tick {tick}"
            );
        }
    }
}

#[test]
fn sub_stepped_events_come_in_one_canonical_batch() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().persisted_contacts(true));
    let floor = add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    assert!(world
        .step_with_collision_steps(DT, MAX)
        .unwrap()
        .is_complete());
    let events = world.take_events();
    let kinds: Vec<char> = events
        .contacts
        .iter()
        .map(|event| {
            let pair = event.pair();
            assert_eq!((pair.body1, pair.body2), (floor, cube), "{event:?}");
            match event {
                ContactEvent::Added { .. } => 'a',
                ContactEvent::Persisted { .. } => 'p',
                ContactEvent::Removed(_) => 'r',
            }
        })
        .collect();
    // The resting cube touches the floor in the first collision step and in each later one: one
    // Added and a Persisted per later collision step, every copy kept, Added sorted first.
    let mut expected = vec!['a'];
    expected.resize(MAX as usize, 'p');
    assert_eq!(kinds, expected);
}
