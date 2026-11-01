//! Rollback with tracked vehicles and motorcycles: a tank, a motorcycle and a car replay bit for
//! bit after a save, a divergent detour and a restore, from a save before their first contact,
//! in a turn and during a gear shift, with 1 and 4 workers and across processes. Each vehicle's
//! engine and transmission change right after the save; a restore keeps them, so the replay
//! matches a straight run that changed them at the save. Jolt saves the tracked input, the track
//! speeds and the motorcycle's target lean; it does not save the motorcycle's integrated lean
//! angle, which the default integration coefficient of 0 leaves without effect.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::vehicle::{add_car, car_world, record_vehicle, CarLayers, GRAVITY};
use common::vehicle_kinds::*;
use common::*;
use oxijolt::*;

/// Ticks recorded after the save.
const AFTER_SAVE: usize = 60;
/// Ticks of the detour between the save and the restore.
const DETOUR: usize = 30;

/// Where a run is saved: before the first contact, in the motorcycle's turn, and two ticks
/// after the first gear shift (the tank's, from first to second gear).
#[derive(Clone, Copy, Debug)]
enum SavePoint {
    InTheAir,
    MidTurn,
    GearShift,
}

const SAVE_POINTS: [SavePoint; 3] = [
    SavePoint::InTheAir,
    SavePoint::MidTurn,
    SavePoint::GearShift,
];

impl SavePoint {
    fn name(self) -> &'static str {
        match self {
            Self::InTheAir => "air",
            Self::MidTurn => "turn",
            Self::GearShift => "shift",
        }
    }

    fn from_name(name: &str) -> Self {
        SAVE_POINTS
            .into_iter()
            .find(|point| point.name() == name)
            .unwrap_or_else(|| panic!("no save point {name}"))
    }
}

/// A tank and a motorcycle under world gravity and the test car under its gravity override, on
/// flat ground.
struct Scene {
    world: PhysicsWorld,
    tank: VehicleId<TrackedVehicle>,
    bike: VehicleId<Motorcycle>,
    car: VehicleId,
}

impl Scene {
    fn new(threads: u32) -> Self {
        let (mut world, layers) = car_world(GRAVITY, threads);
        add_ground(&mut world, &layers);
        let (_, tank) = add_tank(
            &mut world,
            &layers,
            RVec3::new(-6.0, 1.5, 0.0),
            Quat::IDENTITY,
        );
        let (_, bike) = add_bike(
            &mut world,
            &layers,
            RVec3::new(4.0, 1.5, 0.0),
            Quat::IDENTITY,
        );
        let (_, car) = add_car(
            &mut world,
            &layers,
            RVec3::new(14.0, 1.0, 0.0),
            Quat::IDENTITY,
        );
        world
            .vehicle_mut(car)
            .unwrap()
            .set_gravity(GRAVITY)
            .unwrap();
        Self {
            world,
            tank,
            bike,
            car,
        }
    }

    /// The scripted inputs of `tick`; the car takes the motorcycle's.
    fn inputs(tick: usize) -> (TrackedDriverInput, DriverInput) {
        let tank = if tick < 90 {
            tracks(1.0, 1.0, 1.0)
        } else {
            tracks(1.0, -0.5, 1.0)
        };
        let bike = if tick < 120 {
            ride(0.6, 0.0)
        } else {
            ride(0.6, 0.15)
        };
        (tank, bike)
    }

    fn set_inputs(&mut self, tank: TrackedDriverInput, bike: DriverInput) {
        self.world
            .vehicle_mut(self.tank)
            .unwrap()
            .set_driver_input(tank)
            .unwrap();
        self.world
            .vehicle_mut(self.bike)
            .unwrap()
            .set_driver_input(bike)
            .unwrap();
        self.world
            .vehicle_mut(self.car)
            .unwrap()
            .set_driver_input(bike)
            .unwrap();
    }

    /// Gives every vehicle a stronger engine and longer gears, with the gear counts and clutch
    /// release time of creation; the car a longer differential, the tank heavier tracks with
    /// stronger brakes.
    fn retune(&mut self) {
        let longer = |ratios: &[f32]| ratios.iter().map(|ratio| 1.1 * ratio).collect::<Vec<_>>();
        let mut tank = self.world.vehicle_mut(self.tank).unwrap();
        tank.set_engine(TrackedVehicleSettings::default_engine().max_torque(650.0))
            .unwrap();
        tank.set_transmission(
            TrackedVehicleSettings::default_transmission()
                .gear_ratios(longer(&[4.0, 3.0, 2.0, 1.0])),
        )
        .unwrap();
        let track = |x: f32| {
            tank_track(x)
                .inertia(10.5)
                .max_brake_torque(18000.0)
                .differential_ratio(6.5)
        };
        tank.set_tracks(track(1.7), track(-1.7)).unwrap();
        let mut bike = self.world.vehicle_mut(self.bike).unwrap();
        bike.set_engine(bike_engine().max_torque(170.0)).unwrap();
        bike.set_transmission(bike_transmission().gear_ratios(longer(&BIKE_GEAR_RATIOS)))
            .unwrap();
        let mut car = self.world.vehicle_mut(self.car).unwrap();
        car.set_engine(VehicleEngineSettings::default().max_torque(600.0))
            .unwrap();
        car.set_transmission(
            VehicleTransmissionSettings::default()
                .gear_ratios(longer(&[2.66, 1.78, 1.3, 1.0, 0.74])),
        )
        .unwrap();
        let differential =
            VehicleDifferentialSettings::new(Some(0), Some(1)).differential_ratio(3.8);
        car.set_differentials(vec![differential], 1.6).unwrap();
    }

    /// The inputs of `tick`, then one step.
    fn tick(&mut self, tick: usize) {
        let (tank, bike) = Self::inputs(tick);
        self.set_inputs(tank, bike);
        step(&mut self.world, 1);
    }

    /// Both vehicles, the tracks, the lean and the inputs as exact bits.
    fn record(&self, digest: &mut Digest) {
        let state = &mut digest.push().state;
        let push = |state: &mut Vec<u8>, value: f32| state.extend(value.to_bits().to_le_bytes());
        record_vehicle(&self.world, self.tank, state);
        let tank = self.world.vehicle(self.tank).unwrap();
        for track in tank.tracks() {
            push(state, track.angular_velocity);
        }
        let input = tank.driver_input();
        for value in [
            input.forward,
            input.left_ratio,
            input.right_ratio,
            input.brake,
        ] {
            push(state, value);
        }
        record_vehicle(&self.world, self.bike, state);
        let lean = self.world.vehicle(self.bike).unwrap().lean();
        for value in <[f32; 3]>::from(lean.target) {
            push(state, value);
        }
        push(state, lean.angle);
        record_vehicle(&self.world, self.car, state);
    }

    /// A run that differs from the straight one: the tank brakes and turns the other way, the
    /// motorcycle steers the other way, and the world's gravity changes.
    fn detour(&mut self) {
        self.world.set_gravity(Vec3::new(1.5, -12.0, 0.5)).unwrap();
        for tick in 0..DETOUR {
            let brake = TrackedDriverInput {
                forward: 0.5,
                left_ratio: 1.0,
                right_ratio: -1.0,
                brake: if tick < 10 { 1.0 } else { 0.0 },
            };
            self.set_inputs(brake, ride(0.2, -0.4));
            step(&mut self.world, 1);
        }
    }
}

/// A static box ground whose top face is the plane y = 0.
fn add_ground(world: &mut PhysicsWorld, layers: &CarLayers) -> BodyId {
    world
        .create_body(
            &Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap(),
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -1.0, 0.0))
                .object_layer(layers.ground),
        )
        .unwrap()
}

/// The tick a run is saved at.
fn save_tick(point: SavePoint) -> usize {
    match point {
        SavePoint::InTheAir => 2,
        SavePoint::MidTurn => 135,
        SavePoint::GearShift => first_shift() + 2,
    }
}

/// The tick at which the tank or the motorcycle first changes from one gear to another in a
/// straight run.
fn first_shift() -> usize {
    let mut scene = Scene::new(1);
    let gears = |scene: &Scene| {
        [
            scene.world.vehicle(scene.tank).unwrap().current_gear(),
            scene.world.vehicle(scene.bike).unwrap().current_gear(),
        ]
    };
    let mut before = gears(&scene);
    for tick in 0..400 {
        scene.tick(tick);
        let now = gears(&scene);
        if now
            .iter()
            .zip(&before)
            .any(|(&now, &before)| before > 0 && now != before)
        {
            return tick;
        }
        before = now;
    }
    panic!("neither vehicle shifts");
}

/// The ticks after the save point of a straight run.
fn straight(point: SavePoint, threads: u32) -> Digest {
    let mut scene = Scene::new(threads);
    let save = save_tick(point);
    for tick in 0..save {
        scene.tick(tick);
    }
    scene.retune();
    let mut digest = Digest::new();
    for tick in save..save + AFTER_SAVE {
        scene.tick(tick);
        scene.record(&mut digest);
    }
    digest
}

/// The same ticks replayed after a save, a detour and a restore, twice.
fn replay(point: SavePoint, threads: u32) -> Digest {
    let mut scene = Scene::new(threads);
    let save = save_tick(point);
    for tick in 0..save {
        scene.tick(tick);
    }
    let saved = scene.world.save_state();
    scene.retune();
    scene.detour();
    scene.world.restore_state(&saved).unwrap();
    // A first replay, abandoned half-way, then the recorded one.
    for tick in save..save + AFTER_SAVE / 2 {
        scene.tick(tick);
    }
    scene.world.restore_state(&saved).unwrap();
    let mut digest = Digest::new();
    for tick in save..save + AFTER_SAVE {
        scene.tick(tick);
        scene.record(&mut digest);
    }
    digest
}

#[test]
#[ignore = "child process of the vehicle kinds rollback gate"]
fn vehicle_kinds_state_child() {
    let Some((scenario, threads, variant)) = child_request() else {
        return;
    };
    let point = SavePoint::from_name(&variant);
    let digest = match scenario.as_str() {
        "straight" => straight(point, threads),
        "replay" => replay(point, threads),
        other => panic!("no scenario {other}"),
    };
    finish_child(&digest);
}

#[test]
fn vehicle_kinds_replay_after_a_detour() {
    for point in SAVE_POINTS {
        let reference = straight(point, 1);
        assert_eq!(reference.ticks.len(), AFTER_SAVE);
        for threads in [1, 4] {
            let what = format!("{point:?} with {threads} workers");
            assert_same(
                &format!("straight {what}"),
                &reference,
                &straight(point, threads),
            );
            assert_same(
                &format!("replay {what}"),
                &reference,
                &replay(point, threads),
            );
        }
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn vehicle_kinds_replay_across_processes() {
    for point in SAVE_POINTS {
        let reference = digest_in_child("vehicle_kinds_state_child", "straight", 1, point.name());
        let replayed = digest_in_child("vehicle_kinds_state_child", "replay", 4, point.name());
        assert_same(
            &format!("{point:?}: child straight vs child replay"),
            &reference,
            &replayed,
        );
        assert_same(
            &format!("{point:?}: in process vs child"),
            &straight(point, 1),
            &reference,
        );
    }
}

#[test]
fn the_save_points_are_where_they_claim() {
    // Before the first contact.
    let mut scene = Scene::new(1);
    for tick in 0..save_tick(SavePoint::InTheAir) {
        scene.tick(tick);
    }
    let wheels_down = |scene: &Scene| {
        let tank = scene.world.vehicle(scene.tank).unwrap().wheels();
        let bike = scene.world.vehicle(scene.bike).unwrap().wheels();
        tank.iter()
            .chain(&bike)
            .filter(|wheel| wheel.contact.is_some())
            .count()
    };
    assert_eq!(wheels_down(&scene), 0);
    // In the turn: steering and leaning.
    for tick in save_tick(SavePoint::InTheAir)..save_tick(SavePoint::MidTurn) {
        scene.tick(tick);
    }
    let lean = scene.world.vehicle(scene.bike).unwrap().lean();
    assert!(lean.angle > 0.02, "{lean:?}");
    assert!(scene.world.vehicle(scene.bike).unwrap().wheels()[0].steer_angle != 0.0);
}

#[test]
fn tracked_input_tracks_and_target_lean_are_restored() {
    let mut scene = Scene::new(1);
    for tick in 0..135 {
        scene.tick(tick);
    }
    let read = |scene: &Scene| {
        let tank = scene.world.vehicle(scene.tank).unwrap();
        let bike = scene.world.vehicle(scene.bike).unwrap();
        (
            tank.driver_input(),
            tank.tracks(),
            bike.driver_input(),
            bike.lean().target,
        )
    };
    let saved_reading = read(&scene);
    let saved = scene.world.save_state();
    scene.detour();
    assert_ne!(read(&scene), saved_reading);
    scene.world.restore_state(&saved).unwrap();
    assert_eq!(read(&scene), saved_reading);
}

#[test]
fn motorcycle_replays_at_the_default_integration_coefficient() {
    // An upright motorcycle at rest on flat ground: its lean impulse is exactly zero, so the
    // sign of a zero product with the unsaved integrated angle would show.
    let run = |rollback: bool| {
        let (mut world, layers) = car_world(GRAVITY, 1);
        add_ground(&mut world, &layers);
        let (chassis, bike) = add_bike(
            &mut world,
            &layers,
            RVec3::new(0.0, 1.0, 0.0),
            Quat::IDENTITY,
        );
        step(&mut world, 120);
        let saved = world.save_state();
        if rollback {
            // A detour that builds up a different integrated angle: a hard turn at speed.
            for _ in 0..90 {
                world
                    .vehicle_mut(bike)
                    .unwrap()
                    .set_driver_input(ride(1.0, 1.0))
                    .unwrap();
                step(&mut world, 1);
            }
            world.restore_state(&saved).unwrap();
        }
        let mut digest = Vec::new();
        for _ in 0..120 {
            world
                .vehicle_mut(bike)
                .unwrap()
                .set_driver_input(DriverInput::default())
                .unwrap();
            step(&mut world, 1);
            record_body(&world, chassis, &mut digest);
            record_vehicle(&world, bike, &mut digest);
        }
        digest
    };
    assert!(
        run(false) == run(true),
        "the rolled-back motorcycle replays differently"
    );
}

#[test]
fn a_refused_tank_or_motorcycle_keeps_a_state_restorable() {
    let mut scene = Scene::new(1);
    for tick in 0..30 {
        scene.tick(tick);
    }
    let (_, layers) = car_world(Vec3::ZERO, 1);
    let tank_chassis = scene
        .world
        .create_body(
            &tank_chassis_shape(),
            &behaviour_chassis(&layers, 4000.0, RVec3::new(-20.0, 1.5, 0.0), Quat::IDENTITY),
        )
        .unwrap();
    let bike_chassis = scene
        .world
        .create_body(
            &bike_chassis_shape(),
            &behaviour_chassis(
                &layers,
                BIKE_MASS,
                RVec3::new(20.0, 1.5, 0.0),
                Quat::IDENTITY,
            ),
        )
        .unwrap();
    let saved = scene.world.save_state();
    let tester = VehicleCollisionTester::ray(layers.probe);
    let empty_track = TrackedVehicleSettings::new(
        VehicleTrackSettings::new(Vec::new(), 0),
        tank_track(-1.7),
        tester,
    );
    assert!(scene
        .world
        .create_tracked_vehicle(tank_chassis, &empty_track)
        .is_err());
    let integrating = MotorcycleSettings::new(bike_vehicle_settings(bike_tester(&layers)))
        .lean_spring_integration_coefficient(1.0);
    assert_eq!(
        scene.world.create_motorcycle(bike_chassis, &integrating),
        Err(VehicleError::LeanSpringIntegrationNotSaved)
    );
    let stiff = MotorcycleSettings::new(bike_vehicle_settings(bike_tester(&layers)))
        .lean_spring_constant(1.0e30);
    assert!(scene.world.create_motorcycle(bike_chassis, &stiff).is_err());
    for tick in 30..40 {
        scene.tick(tick);
    }
    assert_eq!(scene.world.restore_state(&saved), Ok(()));
}

/// A vehicle's world up is not part of a state: in zero gravity a step keeps the world up it had,
/// so after a restore in zero gravity a motorcycle keeps the world up of the abandoned run. The
/// motorcycle uses the world up for its target lean even with the pitch and roll limit off.
#[test]
fn motorcycle_world_up_in_zero_gravity_is_not_restored() {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let settings = MotorcycleSettings::new(
        bike_vehicle_settings(bike_tester(&layers)).max_pitch_roll_angle(std::f32::consts::PI),
    );
    let body = behaviour_chassis(
        &layers,
        BIKE_MASS,
        RVec3::new(0.0, 5.0, 0.0),
        Quat::IDENTITY,
    );
    let (_, bike) = add_bike_with(&mut world, &body, &settings);
    step(&mut world, 2);
    let world_up = world.vehicle(bike).unwrap().world_up();
    let saved = world.save_state();
    let straight_target = {
        step(&mut world, 1);
        world.vehicle(bike).unwrap().lean().target
    };
    world.restore_state(&saved).unwrap();

    // The abandoned run has gravity along -X for a step, then zero again.
    world.set_gravity(Vec3::new(-9.81, 0.0, 0.0)).unwrap();
    step(&mut world, 1);
    let tilted = world.vehicle(bike).unwrap().world_up();
    assert_eq!(tilted, Vec3::new(1.0, 0.0, 0.0));
    world.restore_state(&saved).unwrap();
    assert_eq!(world.gravity(), Vec3::ZERO);
    assert_eq!(world.vehicle(bike).unwrap().world_up(), tilted);
    assert_ne!(tilted, world_up);
    step(&mut world, 1);
    let replayed_target = world.vehicle(bike).unwrap().lean().target;
    assert_ne!(replayed_target, straight_target);
}
