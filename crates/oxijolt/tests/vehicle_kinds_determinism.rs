//! Determinism of tracked vehicles and motorcycles: a scene of 12 tanks, 12 motorcycles and a
//! car driving scripted inputs over flat ground and a 12° ramp runs bit for bit the same with 1
//! and 4 worker threads, in one process and in two.
//!
//! The 25 vehicle step listeners (24 after a tank is removed) run in 2 jobs with 1 worker and 3
//! with 4 (`PhysicsSystem.cpp:243`: listeners / 8 per batch, capped by the job system's
//! concurrency of workers + 1).

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::determinism::{
    assert_same, child_request, digest_in_child, finish_child, first_divergence, Digest,
};
use common::vehicle::{
    add_car_with, car_world, chassis_settings, record_vehicle, CarLayers, GRAVITY,
};
use common::vehicle_kinds::*;
use common::*;
use oxijolt::*;

const TICKS: usize = 300;
/// The tick at which the scene removes a tank, and the nudged variant changes one input.
const REMOVAL_TICK: usize = 200;
const NUDGE_TICK: usize = 120;

/// A rotation of `degrees` about +X.
fn about_x(degrees: f32) -> Quat {
    let half = 0.5 * degrees.to_radians();
    Quat::from_xyzw(half.sin(), 0.0, 0.0, half.cos())
}

/// A chassis body under the vehicle's own gravity, which the scene sets every tick.
fn gate_chassis(layers: &CarLayers, mass: f32, position: RVec3) -> BodySettings {
    behaviour_chassis(layers, mass, position, Quat::IDENTITY).gravity_factor(0.0)
}

struct Scene {
    world: PhysicsWorld,
    tanks: Vec<Option<VehicleId<TrackedVehicle>>>,
    bikes: Vec<VehicleId<Motorcycle>>,
    car: VehicleId,
    chassis: Vec<BodyId>,
    nudged: bool,
}

impl Scene {
    fn new(threads: u32, nudged: bool) -> Self {
        let (mut world, layers) = car_world(Vec3::ZERO, threads);
        let ground = |world: &mut PhysicsWorld, half: Vec3, position: RVec3, rotation: Quat| {
            world
                .create_body(
                    &Shape::new_box(half).unwrap(),
                    &BodySettings::new_static()
                        .position(position)
                        .rotation(rotation)
                        .object_layer(layers.ground),
                )
                .unwrap();
        };
        ground(
            &mut world,
            Vec3::new(120.0, 1.0, 120.0),
            RVec3::new(0.0, -1.0, 0.0),
            Quat::IDENTITY,
        );
        // A 12° ramp across the tank lanes, rising from y = 0 at z = 4.
        ground(
            &mut world,
            Vec3::new(50.0, 1.0, 15.0),
            RVec3::new(-50.0, 2.141, 18.88),
            about_x(-12.0),
        );
        let mut chassis = Vec::new();
        let tanks = (0..12)
            .map(|i| {
                let body = gate_chassis(
                    &layers,
                    4000.0,
                    RVec3::new(-90.0 + 7.5 * i as Real, 1.2, 0.0),
                );
                let (body, tank) =
                    add_tank_with(&mut world, &body, VehicleCollisionTester::ray(layers.probe));
                chassis.push(body);
                Some(tank)
            })
            .collect();
        let bikes = (0..12)
            .map(|i| {
                let settings = MotorcycleSettings::new(bike_vehicle_settings(bike_tester(&layers)))
                    .lean_controller(i != 5);
                let body = gate_chassis(
                    &layers,
                    BIKE_MASS,
                    RVec3::new(5.0 + 3.0 * i as Real, 1.2, 0.0),
                );
                let (body, bike) = add_bike_with(&mut world, &body, &settings);
                chassis.push(body);
                bike
            })
            .collect();
        let (body, car) = add_car_with(
            &mut world,
            &chassis_settings(&layers, RVec3::new(70.0, 0.9, 0.0), Quat::IDENTITY),
            VehicleCollisionTester::ray(layers.probe),
        );
        chassis.push(body);
        Self {
            world,
            tanks,
            bikes,
            car,
            chassis,
            nudged,
        }
    }

    /// The input of tank `index` at `tick`: settle, then pivot (even tanks) or climb the ramp
    /// (odd tanks), brake, and reverse.
    fn tank_input(&self, index: usize, tick: usize) -> TrackedDriverInput {
        let ratio = 1.0 - 0.05 * index as f32;
        let mut input = match tick {
            0..60 => tracks(0.0, 1.0, 1.0),
            60..150 if index.is_multiple_of(2) => tracks(1.0, -ratio, ratio),
            60..150 => tracks(1.0, ratio, 1.0),
            150..200 => TrackedDriverInput {
                brake: 1.0,
                ..TrackedDriverInput::default()
            },
            _ => tracks(-1.0, 1.0, ratio),
        };
        if self.nudged && index == 3 && tick >= NUDGE_TICK {
            input.left_ratio = 0.9;
        }
        input
    }

    /// The input of motorcycle `index` at `tick`: accelerate, turn right, then left.
    fn bike_input(index: usize, tick: usize) -> DriverInput {
        let throttle = 0.3 + 0.02 * index as f32;
        match tick {
            0..180 => ride(throttle, 0.0),
            180..240 => ride(throttle, 0.15),
            _ => ride(throttle, -0.1),
        }
    }

    /// The inputs and gravity of `tick` in vehicle id order, then one step.
    fn tick(&mut self, tick: usize) {
        if tick == REMOVAL_TICK {
            let tank = self.tanks[6].take().unwrap();
            self.world.remove_vehicle(tank).unwrap();
        }
        for (index, tank) in self.tanks.iter().enumerate() {
            let Some(tank) = *tank else { continue };
            let input = self.tank_input(index, tick);
            let mut vehicle = self.world.vehicle_mut(tank).unwrap();
            vehicle.set_gravity(GRAVITY).unwrap();
            vehicle.set_driver_input(input).unwrap();
        }
        for (index, &bike) in self.bikes.iter().enumerate() {
            let mut vehicle = self.world.vehicle_mut(bike).unwrap();
            vehicle.set_gravity(GRAVITY).unwrap();
            vehicle
                .set_driver_input(Self::bike_input(index, tick))
                .unwrap();
        }
        let mut car = self.world.vehicle_mut(self.car).unwrap();
        car.set_gravity(GRAVITY).unwrap();
        car.set_driver_input(ride(0.5, 0.0)).unwrap();
        step(&mut self.world, 1);
    }

    /// Every vehicle: chassis, wheels, rpm and gear, then each tank's tracks and input and each
    /// motorcycle's lean and input, all as exact bits.
    fn record(&self, digest: &mut Digest) {
        let tick = digest.push();
        let state = &mut tick.state;
        let push = |state: &mut Vec<u8>, value: f32| state.extend(value.to_bits().to_le_bytes());
        for &tank in self.tanks.iter().flatten() {
            record_vehicle(&self.world, tank, state);
            let vehicle = self.world.vehicle(tank).unwrap();
            for track in vehicle.tracks() {
                state.extend(track.driven_wheel.to_le_bytes());
                push(state, track.angular_velocity);
            }
            let input = vehicle.driver_input();
            for value in [
                input.forward,
                input.left_ratio,
                input.right_ratio,
                input.brake,
            ] {
                push(state, value);
            }
        }
        for &bike in &self.bikes {
            record_vehicle(&self.world, bike, state);
            let vehicle = self.world.vehicle(bike).unwrap();
            let lean = vehicle.lean();
            for value in <[f32; 3]>::from(lean.target) {
                push(state, value);
            }
            push(state, lean.target_angle);
            push(state, lean.angle);
            let input = vehicle.driver_input();
            for value in [input.forward, input.right, input.brake, input.hand_brake] {
                push(state, value);
            }
        }
        record_vehicle(&self.world, self.car, state);
        // Chassis bodies of removed vehicles stay in the world.
        for &body in &self.chassis {
            record_body(&self.world, body, state);
        }
    }
}

/// The scene run for [`TICKS`] ticks with `threads` workers.
fn kinds(threads: u32, nudged: bool) -> Digest {
    let mut scene = Scene::new(threads, nudged);
    let mut digest = Digest::new();
    for tick in 0..TICKS {
        scene.tick(tick);
        scene.record(&mut digest);
    }
    digest
}

#[test]
#[ignore = "child process of the vehicle kinds determinism gate"]
fn vehicle_kinds_child() {
    let Some((scenario, threads, variant)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "kinds");
    finish_child(&kinds(threads, variant == "nudged"));
}

#[test]
fn vehicle_kinds_match_with_1_and_4_workers() {
    let one = kinds(1, false);
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in one process", &one, &kinds(4, false));
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn vehicle_kinds_match_across_processes() {
    let one = digest_in_child("vehicle_kinds_child", "kinds", 1, "");
    let four = digest_in_child("vehicle_kinds_child", "kinds", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &kinds(1, false), &one);
}

#[test]
fn vehicle_kinds_detect_a_changed_input() {
    let plain = kinds(1, false);
    let nudged = kinds(1, true);
    let divergence = first_divergence(&plain, &nudged).expect("a changed input changes the digest");
    // The change shows on the tick it is applied, not before.
    assert!(
        matches!(divergence, common::determinism::Divergence::Tick { tick, .. } if tick == NUDGE_TICK),
        "{divergence}"
    );
}

#[test]
fn the_scene_moves_every_kind() {
    let mut scene = Scene::new(1, false);
    let start: Vec<RVec3> = scene
        .chassis
        .iter()
        .map(|&body| scene.world.body(body).unwrap().position())
        .collect();
    let mut highest_climb = Real::MIN;
    for tick in 0..TICKS {
        scene.tick(tick);
        let climber = scene.world.body(scene.chassis[1]).unwrap().position();
        highest_climb = highest_climb.max(climber.y - start[1].y);
    }
    let world = &scene.world;
    // An odd tank climbed the ramp, an even one turned in place, the bikes rode off, one fell.
    assert!(highest_climb > 0.3, "{highest_climb}");
    let pivot = world.body(scene.chassis[0]).unwrap().rotation();
    assert!(pivot.y.abs() > 0.1, "{pivot:?}");
    for (index, &bike) in scene.bikes.iter().enumerate() {
        let body = world.body(world.vehicle(bike).unwrap().body()).unwrap();
        let travelled = body.position().z - start[12 + index].z;
        assert!(travelled > 5.0, "bike {index}: {travelled}");
    }
    let fallen = world.vehicle(scene.bikes[5]).unwrap().lean();
    assert!(fallen.angle.abs() > 0.5, "{fallen:?}");
    assert_eq!(world.vehicle_ids().count(), 24);
}
