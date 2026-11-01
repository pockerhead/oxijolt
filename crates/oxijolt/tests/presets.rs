//! Settings presets: a humanoid character standing and walking on a floor, a car that drives,
//! brakes, steers and shifts, and a motorcycle that rides upright.

mod common;

use common::math::{f3, norm, rotate, v3};
use common::vehicle::{car_world, chassis_settings, CarLayers, GRAVITY as CAR_GRAVITY};
use common::vehicle_kinds::{
    behaviour_chassis, bike_chassis_shape, bike_tester, bike_vehicle_settings, BIKE_MASS,
};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// One character update with horizontal `velocity` plus 1 m/s down, under gravity.
fn walk_tick(world: &mut PhysicsWorld, id: CharacterId, velocity: Vec3) {
    world
        .character_mut(id)
        .unwrap()
        .set_linear_velocity(Vec3::new(velocity.x, velocity.y - 1.0, velocity.z))
        .unwrap();
    world
        .update_character(
            id,
            DT,
            GRAVITY,
            &ExtendedUpdateSettings::default(),
            &QueryFilter::new(),
        )
        .unwrap();
}

#[test]
fn a_humanoid_stands_on_a_floor_and_walks() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let settings = CharacterSettings::humanoid(1.8, 0.3).unwrap();
    let id = world
        .create_character(&settings, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    assert_eq!(
        world.character(id).unwrap().ground_state(),
        GroundState::OnGround
    );

    for _ in 0..60 {
        walk_tick(&mut world, id, Vec3::new(1.0, 0.0, 0.0));
    }
    let character = world.character(id).unwrap();
    assert_eq!(character.ground_state(), GroundState::OnGround);
    let position = character.position();
    assert!(
        (position.x - 1.0).abs() < 0.02,
        "walked to x = {}",
        position.x
    );
    assert!(
        position.y.abs() < 0.05,
        "stays on the floor at y = {}",
        position.y
    );
}

#[test]
fn humanoid_settings_outlive_their_clones() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let original = CharacterSettings::humanoid(1.8, 0.3).unwrap();
    let clone = original.clone();
    drop(original);
    let first = world
        .create_character(&clone, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    let second_clone = clone.clone();
    drop(clone);
    let second = world
        .create_character(&second_clone, RVec3::new(3.0, 0.0, 0.0), Quat::IDENTITY)
        .unwrap();
    drop(second_clone);
    for _ in 0..60 {
        walk_tick(&mut world, first, Vec3::new(0.0, 0.0, 1.0));
        walk_tick(&mut world, second, Vec3::new(0.0, 0.0, -1.0));
    }
    for id in [first, second] {
        assert_eq!(
            world.character(id).unwrap().ground_state(),
            GroundState::OnGround
        );
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn humanoid_settings_cross_threads() {
    fn send_and_sync<T: Send + Sync>() {}
    send_and_sync::<CharacterSettings<'static>>();

    let original = CharacterSettings::humanoid(1.8, 0.3).unwrap();
    let clone = original.clone();
    // The clone walks a character on another thread and is dropped there, while the original
    // is still alive here; then the original is used here after the clone is gone.
    let walker = std::thread::spawn(move || {
        let mut world = world(GRAVITY, 1);
        add_floor(&mut world);
        let id = world
            .create_character(&clone, RVec3::ZERO, Quat::IDENTITY)
            .unwrap();
        drop(clone);
        for _ in 0..60 {
            walk_tick(&mut world, id, Vec3::new(1.0, 0.0, 0.0));
        }
        world.character(id).unwrap().ground_state()
    });
    assert_eq!(walker.join().unwrap(), GroundState::OnGround);

    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let id = world
        .create_character(&original, RVec3::ZERO, Quat::IDENTITY)
        .unwrap();
    drop(original);
    for _ in 0..60 {
        walk_tick(&mut world, id, Vec3::new(0.0, 0.0, 1.0));
    }
    assert_eq!(
        world.character(id).unwrap().ground_state(),
        GroundState::OnGround
    );
}

#[test]
fn a_humanoid_accepts_a_borrowed_inner_body() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let bodies = world.body_count();
    {
        let inner_shape = Shape::new_capsule(0.6, 0.3).unwrap();
        let settings = CharacterSettings::humanoid(1.8, 0.3)
            .unwrap()
            .inner_body(Some(InnerBody {
                shape: &inner_shape,
                object_layer: ObjectLayer::MOVING,
            }));
        let id = world
            .create_character(&settings, RVec3::ZERO, Quat::IDENTITY)
            .unwrap();
        assert_eq!(world.body_count(), bodies + 1);
        world.remove_character(id).unwrap();
    }
    assert_eq!(world.body_count(), bodies);
}

/// The preset car of the test chassis with a sphere tester, on a 400 m box floor, level at rest.
fn preset_car() -> (PhysicsWorld, CarLayers, BodyId, VehicleId) {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    world
        .create_body(
            &Shape::new_box(Vec3::new(200.0, 1.0, 200.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(layers.ground),
        )
        .unwrap();
    let chassis = world
        .create_body(
            &common::vehicle::chassis_shape(),
            &chassis_settings(&layers, RVec3::new(0.0, 0.9, -150.0), Quat::IDENTITY),
        )
        .unwrap();
    let tester = VehicleCollisionTester::cast_sphere(layers.probe, 0.2);
    let settings = WheeledVehicleSettings::car(Vec3::new(0.9, -0.1, 1.4), 0.35, tester);
    let car = world.create_wheeled_vehicle(chassis, &settings).unwrap();
    drive_car(&mut world, car, DriverInput::default(), 30);
    (world, layers, chassis, car)
}

/// Drives `car` with `input` for `ticks` ticks under the fixture's gravity, which the chassis
/// takes from the vehicle only (gravity factor 0).
fn drive_car(world: &mut PhysicsWorld, car: VehicleId, input: DriverInput, ticks: usize) {
    for _ in 0..ticks {
        let mut vehicle = world.vehicle_mut(car).unwrap();
        vehicle.set_gravity(CAR_GRAVITY).unwrap();
        vehicle.set_driver_input(input).unwrap();
        assert!(world.step(DT).unwrap().is_complete());
    }
}

/// The chassis' heading about +Y, radians from +Z toward +X.
fn heading(world: &PhysicsWorld, chassis: BodyId) -> f64 {
    let forward = rotate(world.body(chassis).unwrap().rotation(), [0.0, 0.0, 1.0]);
    forward[0].atan2(forward[2])
}

#[test]
fn a_car_preset_drives_brakes_steers_and_shifts() {
    let full_throttle = DriverInput {
        forward: 1.0,
        ..DriverInput::default()
    };
    // Measured on this fixture: 14.5 m in 4 s of full throttle, in first gear.
    let (mut world, _, chassis, car) = preset_car();
    let start = v3(world.body(chassis).unwrap().position());
    drive_car(&mut world, car, full_throttle, 240);
    let end = v3(world.body(chassis).unwrap().position());
    assert!(end[2] - start[2] > 10.0, "moved from {start:?} to {end:?}");
    assert!(
        (end[0] - start[0]).abs() < 0.1,
        "moved from {start:?} to {end:?}"
    );
    assert_eq!(world.vehicle(car).unwrap().current_gear(), 1);

    // Measured: stopped after 114 ticks of full brake.
    let brake = DriverInput {
        brake: 1.0,
        ..DriverInput::default()
    };
    let stopped = (0..240).any(|_| {
        drive_car(&mut world, car, brake, 1);
        norm(f3(world.body(chassis).unwrap().linear_velocity())) < 0.1
    });
    assert!(
        stopped,
        "{:?}",
        world.body(chassis).unwrap().linear_velocity()
    );

    // Measured: the automatic transmission shifts into second gear at tick 949 of full
    // throttle, at the engine's maximum rpm, while the driven wheels slip below it.
    let (mut world, _, _, car) = preset_car();
    let shifted = (0..1200).any(|_| {
        drive_car(&mut world, car, full_throttle, 1);
        world.vehicle(car).unwrap().current_gear() == 2
    });
    assert!(
        shifted,
        "gear {}",
        world.vehicle(car).unwrap().current_gear()
    );

    // Steering right turns toward −X for forward +Z. Measured: −0.70 rad in 2 s.
    let (mut world, _, chassis, car) = preset_car();
    let steer_right = DriverInput {
        forward: 1.0,
        right: 1.0,
        ..DriverInput::default()
    };
    let before = heading(&world, chassis);
    drive_car(&mut world, car, steer_right, 120);
    let turned = heading(&world, chassis) - before;
    assert!(turned < -0.5, "heading changed by {turned}");
    let steer = world.vehicle(car).unwrap().wheel(0).unwrap().steer_angle;
    assert!(
        (steer + 30.0_f32.to_radians()).abs() < 1e-4,
        "steer angle {steer}"
    );
}

#[test]
fn car_preset_values_are_validated() {
    const POSITION_RULE: &str = "wheel position must be finite and within limits::MAX_SHAPE_EXTENT";
    const RADIUS_RULE: &str = "wheel radius must be positive and at most limits::MAX_SHAPE_EXTENT";
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let chassis = world
        .create_body(
            &common::vehicle::chassis_shape(),
            &chassis_settings(&layers, RVec3::new(0.0, 0.9, 0.0), Quat::IDENTITY),
        )
        .unwrap();
    let bodies = world.body_count();
    let tester = VehicleCollisionTester::ray(layers.probe);
    let cases = [
        (Vec3::new(f32::NAN, -0.1, 1.4), 0.35, POSITION_RULE),
        (
            Vec3::new(0.9, -0.1, limits::MAX_SHAPE_EXTENT * 2.0),
            0.35,
            POSITION_RULE,
        ),
        (Vec3::new(0.9, -0.1, 1.4), 0.0, RADIUS_RULE),
        (Vec3::new(0.9, -0.1, 1.4), -1.0, RADIUS_RULE),
    ];
    for (front_left, radius, rule) in cases {
        let settings = WheeledVehicleSettings::car(front_left, radius, tester);
        assert_eq!(
            world.create_wheeled_vehicle(chassis, &settings),
            Err(VehicleError::InvalidValue(rule)),
            "{front_left:?}, radius {radius}"
        );
        assert_eq!(world.body_count(), bodies);
        assert_eq!(world.vehicle_ids().count(), 0);
    }
}

#[test]
fn bike_preset_equals_the_motorcycle_sample() {
    let (_, layers) = car_world(Vec3::ZERO, 1);
    let tester = bike_tester(&layers);
    assert_eq!(
        MotorcycleSettings::bike(Vec3::new(0.0, -0.27, 0.75), 0.31, tester),
        MotorcycleSettings::new(bike_vehicle_settings(tester))
    );
}

/// A chassis of the sample bike at y = 2 over a box floor, under gravity.
fn bike_world() -> (PhysicsWorld, CarLayers, BodyId) {
    let (mut world, layers) = car_world(CAR_GRAVITY, 1);
    world
        .create_body(
            &Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(layers.ground),
        )
        .unwrap();
    let body = behaviour_chassis(
        &layers,
        BIKE_MASS,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    let chassis = world.create_body(&bike_chassis_shape(), &body).unwrap();
    (world, layers, chassis)
}

#[test]
fn a_bike_preset_rides_upright() {
    let (mut world, layers, chassis) = bike_world();
    let settings =
        MotorcycleSettings::bike(Vec3::new(0.0, -0.27, 0.75), 0.31, bike_tester(&layers));
    let bike = world.create_motorcycle(chassis, &settings).unwrap();
    let input = DriverInput {
        forward: 0.4,
        ..DriverInput::default()
    };
    for _ in 0..180 {
        let mut vehicle = world.vehicle_mut(bike).unwrap();
        vehicle.set_driver_input(input).unwrap();
        assert!(world.step(DT).unwrap().is_complete());
    }
    let body = world.body(chassis).unwrap();
    let speed = norm(f3(body.linear_velocity()));
    let up = rotate(body.rotation(), [0.0, 1.0, 0.0])[1];
    // Measured: 5.39 m/s with up.y 0.9986 after 3 s at 0.4 throttle.
    assert!(speed > 4.0, "speed {speed}");
    assert!(up > 0.99, "up.y {up}");
    let wheels = world.vehicle(bike).unwrap().wheels();
    assert!(wheels.iter().all(|wheel| wheel.contact.is_some()));
}

#[test]
fn bike_preset_values_are_validated() {
    const APART_RULE: &str = "a motorcycle's wheels must be apart along its forward";
    const RADIUS_RULE: &str = "wheel radius must be positive and at most limits::MAX_SHAPE_EXTENT";
    let (mut world, layers, chassis) = bike_world();
    let tester = bike_tester(&layers);
    // The front suspension is raked 30° and 0.5 long, so it ends 0.5 sin 30° ahead of its
    // attachment. At z = -0.25 sin 30° that end is level with the rear wheel at z = 0.25 sin 30°:
    // no wheel base. The sine is computed as the preset computes it, so both ends round alike.
    let rake = 30.0_f32.to_radians().tan();
    let sin_rake = rake / (1.0 + rake * rake).sqrt();
    let cases = [
        (Vec3::new(0.0, -0.27, -0.25 * sin_rake), 0.31, APART_RULE),
        (Vec3::new(0.0, -0.27, 0.75), 0.0, RADIUS_RULE),
    ];
    for (front, radius, rule) in cases {
        let settings = MotorcycleSettings::bike(front, radius, tester);
        assert_eq!(
            world.create_motorcycle(chassis, &settings),
            Err(VehicleError::InvalidValue(rule)),
            "{front:?}, radius {radius}"
        );
        assert_eq!(world.vehicle_ids().count(), 0);
    }
}
