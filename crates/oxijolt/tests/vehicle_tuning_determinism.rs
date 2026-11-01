//! Determinism of tuned vehicles: 8 cars, 8 motorcycles and 8 tanks, each on its own patch of
//! flat ground, drive, steer and brake; cars and motorcycles cap their tire impulses with a
//! callback that depends only on its input, and at tick 30 every vehicle gets a new engine and
//! transmission, cars and motorcycles new differentials, tanks new tracks. The scene runs bit
//! for bit the same with 1 and 4 worker threads, in one process and in two.
//!
//! The 24 vehicle step listeners run in 2 jobs with 1 worker and 3 with 4
//! (`PhysicsSystem.cpp:243`: listeners / 8 per batch, capped by the job system's concurrency of
//! workers + 1).

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use common::determinism::{
    assert_same, child_request, digest_in_child, finish_child, first_divergence, Digest,
};
use common::vehicle::{
    car_settings, car_world, chassis_settings, chassis_shape, record_vehicle, GRAVITY,
};
use common::vehicle_kinds::*;
use common::*;
use oxijolt::*;

const TICKS: usize = 120;
/// The tick at which every vehicle's drivetrain changes.
const RETUNE_TICK: usize = 30;
const PER_KIND: usize = 8;
/// Distance between neighbouring vehicles, m, so that each stays in its own island.
const SPACING: Real = 25.0;

/// Tire friction that depends only on the callback's input: less grip with more slip (the
/// longitudinal slip is huge for a spinning wheel at rest, hence the bound), scaled by
/// `constant`.
fn slip_grip(
    constant: f32,
    calls: Arc<AtomicUsize>,
) -> impl Fn(&TireMaxImpulseInput) -> TireFriction + Send + Sync + 'static {
    move |input| {
        calls.fetch_add(1, Ordering::Relaxed);
        let friction = input.default_friction();
        TireFriction {
            longitudinal: constant * friction.longitudinal
                / (1.0 + 0.05 * input.longitudinal_slip.min(10.0)),
            lateral: constant * friction.lateral * (1.0 - 0.25 * input.lateral_slip),
        }
    }
}

struct Scene {
    world: PhysicsWorld,
    cars: Vec<VehicleId>,
    bikes: Vec<VehicleId<Motorcycle>>,
    tanks: Vec<VehicleId<TrackedVehicle>>,
    chassis: Vec<BodyId>,
    /// Calls of every tire callback, which do not affect the simulation.
    calls: Arc<AtomicUsize>,
}

/// The x of vehicle `index` of a row.
fn lane(index: usize) -> Real {
    (index as Real - 3.5) * SPACING
}

impl Scene {
    fn new(threads: u32, grip: f32) -> Self {
        let (mut world, layers) = car_world(GRAVITY, threads);
        world
            .create_body(
                &Shape::new_box(Vec3::new(150.0, 1.0, 150.0)).unwrap(),
                &BodySettings::new_static()
                    .position(RVec3::new(0.0, -1.0, 0.0))
                    .object_layer(layers.ground),
            )
            .unwrap();
        let calls = Arc::new(AtomicUsize::new(0));
        let mut chassis = Vec::new();
        let cars = (0..PER_KIND)
            .map(|i| {
                let body =
                    chassis_settings(&layers, RVec3::new(lane(i), 1.0, -60.0), Quat::IDENTITY);
                let body = world.create_body(&chassis_shape(), &body).unwrap();
                let settings = car_settings(VehicleCollisionTester::ray(layers.probe));
                let car = world.create_wheeled_vehicle(body, &settings).unwrap();
                let mut vehicle = world.vehicle_mut(car).unwrap();
                vehicle.set_gravity(GRAVITY).unwrap();
                vehicle.set_tire_max_impulse_callback(slip_grip(grip, Arc::clone(&calls)));
                chassis.push(body);
                car
            })
            .collect();
        let bikes = (0..PER_KIND)
            .map(|i| {
                let (body, bike) = add_bike(
                    &mut world,
                    &layers,
                    RVec3::new(lane(i), 1.0, 0.0),
                    Quat::IDENTITY,
                );
                world
                    .vehicle_mut(bike)
                    .unwrap()
                    .set_tire_max_impulse_callback(slip_grip(grip, Arc::clone(&calls)));
                chassis.push(body);
                bike
            })
            .collect();
        let tanks = (0..PER_KIND)
            .map(|i| {
                let (body, tank) = add_tank(
                    &mut world,
                    &layers,
                    RVec3::new(lane(i), 1.5, 60.0),
                    Quat::IDENTITY,
                );
                chassis.push(body);
                tank
            })
            .collect();
        Self {
            world,
            cars,
            bikes,
            tanks,
            chassis,
            calls,
        }
    }

    /// The wheeled input of vehicle `index` at `tick`: accelerate while steering, brake, then
    /// reverse steering the other way.
    fn wheeled_input(index: usize, tick: usize) -> DriverInput {
        let steer = 0.05 * (index as f32 - 3.5);
        match tick {
            0..60 => ride(0.5 + 0.05 * index as f32, steer),
            60..90 => DriverInput {
                brake: 1.0,
                right: steer,
                ..DriverInput::default()
            },
            _ => ride(-0.5, -steer),
        }
    }

    /// The tracked input of tank `index` at `tick`: drive while turning, brake, then reverse.
    fn tank_input(index: usize, tick: usize) -> TrackedDriverInput {
        let ratio = 1.0 - 0.05 * index as f32;
        match tick {
            0..60 => tracks(1.0, ratio, 1.0),
            60..90 => TrackedDriverInput {
                brake: 1.0,
                ..TrackedDriverInput::default()
            },
            _ => tracks(-1.0, 1.0, ratio),
        }
    }

    /// New engines, transmissions, differentials and tracks for every vehicle.
    fn retune(&mut self) {
        let longer = |ratios: &[f32]| ratios.iter().map(|ratio| 1.1 * ratio).collect::<Vec<_>>();
        for &car in &self.cars {
            let mut vehicle = self.world.vehicle_mut(car).unwrap();
            vehicle
                .set_engine(VehicleEngineSettings::default().max_torque(600.0))
                .unwrap();
            vehicle
                .set_transmission(
                    VehicleTransmissionSettings::default()
                        .gear_ratios(longer(&[2.66, 1.78, 1.3, 1.0, 0.74])),
                )
                .unwrap();
            let differential =
                VehicleDifferentialSettings::new(Some(0), Some(1)).differential_ratio(3.8);
            vehicle.set_differentials(vec![differential], 1.6).unwrap();
        }
        for &bike in &self.bikes {
            let mut vehicle = self.world.vehicle_mut(bike).unwrap();
            vehicle.set_engine(bike_engine().max_torque(170.0)).unwrap();
            vehicle
                .set_transmission(bike_transmission().gear_ratios(longer(&BIKE_GEAR_RATIOS)))
                .unwrap();
            let differential =
                VehicleDifferentialSettings::new(None, Some(1)).differential_ratio(5.5);
            vehicle.set_differentials(vec![differential], 1.4).unwrap();
        }
        let track = |x: f32| {
            tank_track(x)
                .inertia(10.5)
                .max_brake_torque(18000.0)
                .differential_ratio(6.5)
        };
        for &tank in &self.tanks {
            let mut vehicle = self.world.vehicle_mut(tank).unwrap();
            vehicle
                .set_engine(TrackedVehicleSettings::default_engine().max_torque(650.0))
                .unwrap();
            vehicle
                .set_transmission(
                    TrackedVehicleSettings::default_transmission()
                        .gear_ratios(longer(&[4.0, 3.0, 2.0, 1.0])),
                )
                .unwrap();
            vehicle.set_tracks(track(1.7), track(-1.7)).unwrap();
        }
    }

    /// The inputs of `tick` in vehicle id order, then one step.
    fn tick(&mut self, tick: usize) {
        if tick == RETUNE_TICK {
            self.retune();
        }
        for (index, &car) in self.cars.iter().enumerate() {
            let input = Self::wheeled_input(index, tick);
            self.world
                .vehicle_mut(car)
                .unwrap()
                .set_driver_input(input)
                .unwrap();
        }
        for (index, &bike) in self.bikes.iter().enumerate() {
            let input = Self::wheeled_input(index, tick);
            self.world
                .vehicle_mut(bike)
                .unwrap()
                .set_driver_input(input)
                .unwrap();
        }
        for (index, &tank) in self.tanks.iter().enumerate() {
            let input = Self::tank_input(index, tick);
            self.world
                .vehicle_mut(tank)
                .unwrap()
                .set_driver_input(input)
                .unwrap();
        }
        step(&mut self.world, 1);
    }

    /// Every vehicle (chassis, wheels, rpm and gear) and every tank's tracks, as exact bits.
    fn record(&self, digest: &mut Digest) {
        let state = &mut digest.push().state;
        for &car in &self.cars {
            record_vehicle(&self.world, car, state);
        }
        for &bike in &self.bikes {
            record_vehicle(&self.world, bike, state);
        }
        for &tank in &self.tanks {
            record_vehicle(&self.world, tank, state);
            for track in self.world.vehicle(tank).unwrap().tracks() {
                state.extend(track.angular_velocity.to_bits().to_le_bytes());
            }
        }
    }
}

/// The scene run for [`TICKS`] ticks with `threads` workers and tire grip `grip`.
fn tuning(threads: u32, grip: f32) -> Digest {
    let mut scene = Scene::new(threads, grip);
    let mut digest = Digest::new();
    for tick in 0..TICKS {
        scene.tick(tick);
        scene.record(&mut digest);
    }
    digest
}

const GRIP: f32 = 0.9;
const CHANGED_GRIP: f32 = 0.85;

#[test]
#[ignore = "child process of the vehicle tuning determinism gate"]
fn vehicle_tuning_child() {
    let Some((scenario, threads, variant)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "tuning");
    assert_eq!(variant, "");
    finish_child(&tuning(threads, GRIP));
}

#[test]
fn tuning_matches_with_1_and_4_workers() {
    let one = tuning(1, GRIP);
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in one process", &one, &tuning(4, GRIP));
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn tuning_matches_across_processes() {
    let one = digest_in_child("vehicle_tuning_child", "tuning", 1, "");
    let four = digest_in_child("vehicle_tuning_child", "tuning", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &tuning(1, GRIP), &one);
}

#[test]
fn tuning_detects_a_changed_input() {
    let plain = tuning(1, GRIP);
    let changed = tuning(1, CHANGED_GRIP);
    assert!(
        first_divergence(&plain, &changed).is_some(),
        "a changed callback constant changes the digest"
    );
}

#[test]
fn the_tuning_scene_moves_and_calls_back() {
    let mut scene = Scene::new(1, GRIP);
    let start: Vec<RVec3> = scene
        .chassis
        .iter()
        .map(|&body| scene.world.body(body).unwrap().position())
        .collect();
    // The farthest each vehicle got from its start: they drive out and reverse.
    let mut farthest: Vec<Real> = vec![0.0; start.len()];
    for tick in 0..TICKS {
        scene.tick(tick);
        for ((&body, start), farthest) in scene.chassis.iter().zip(&start).zip(&mut farthest) {
            let position = scene.world.body(body).unwrap().position();
            *farthest = farthest.max((position.x - start.x).hypot(position.z - start.z));
        }
    }
    for (index, farthest) in farthest.iter().enumerate() {
        assert!(*farthest > 1.0, "vehicle {index}: {farthest}");
    }
    assert!(scene.calls.load(Ordering::Relaxed) > 0);
    assert_eq!(scene.world.vehicle_count(), 3 * PER_KIND as u32);
}
