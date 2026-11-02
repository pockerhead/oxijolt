//! Shape changes of static bodies that constraints use: the joints hold where they were, a gear
//! over hinges on a static support keeps coupling, the results are the same with 1 and 4 worker
//! threads in two processes, earlier states are refused and later ones replay. Kinematic and
//! dynamic bodies used by constraints are still refused.
//!
//! The world attachment points themselves are read back from Jolt for each kind, with the static
//! body on either side, by the unit tests in `src/constraint/shape_change_tests.rs`.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::constraint_rigs::{
    all_kinds_rig, anchor, assert_every_getter_loaded, cube, readout_bits, readout_coverage,
    shifted,
};
use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::{record_body, world, DT};
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
/// The tick before whose step the static bodies change shape; every readout of the rig carries a
/// load by then.
const EDIT_TICK: usize = 30;
/// The tick before whose step they change shape again, in the determinism gate.
const SECOND_EDIT_TICK: usize = 90;
const TICKS: usize = 150;
/// How far a dynamic body of the rig may drift from the run without the shape change, metres.
/// Measured: 4e-6 m over the 120 ticks after the change; without the constraints' notification the
/// bodies follow their anchors' centre-of-mass move of 0.54 m.
const DRIFT: Real = 1e-3;

/// The rig's anchor box (half extent 0.1 m) with its centre of mass moved off its middle; the
/// collision surface stays.
fn off_centre_anchor() -> Shape {
    let anchor = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    Shape::new_offset_center_of_mass(&anchor, Vec3::new(0.4, -0.3, 0.2)).unwrap()
}

/// The rig's anchor box with a second box beside it along -X and above it.
fn grown_anchor() -> Shape {
    let small = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    let block = Shape::new_box(Vec3::new(0.2, 0.1, 0.1)).unwrap();
    let child = |shape, x: f32, y: f32| CompoundChild {
        shape,
        position: Vec3::new(x, y, 0.0),
        rotation: Quat::IDENTITY,
        user_data: 0,
    };
    Shape::new_compound(&[child(&small, 0.0, 0.0), child(&block, -0.5, 0.3)]).unwrap()
}

/// The world of [`all_kinds_rig`], what it made and its static bodies that constraints use.
struct Rig {
    world: PhysicsWorld,
    /// Every body, in raw id order.
    bodies: Vec<BodyId>,
    constraints: Vec<AnyConstraintId>,
    supports: Vec<BodyId>,
}

impl Rig {
    fn new(threads: u32) -> Self {
        let mut world = world(GRAVITY, threads);
        let constraints = all_kinds_rig(&mut world);
        let mut bodies: Vec<BodyId> = world.body_ids().collect();
        bodies.sort_by_key(|id| id.to_raw());
        let supports = bodies
            .iter()
            .copied()
            .filter(|&id| world.body(id).unwrap().motion_type() == MotionType::Static)
            .filter(|&id| world.constraints_of_body(id).next().is_some())
            .collect();
        Self {
            world,
            bodies,
            constraints,
            supports,
        }
    }

    /// Gives every static body that constraints use `shape`.
    fn reshape_supports(&mut self, shape: &Shape) {
        for &id in &self.supports {
            self.world
                .body_mut(id)
                .unwrap()
                .set_shape(shape, None, Activation::DontActivate)
                .unwrap();
        }
    }

    fn step(&mut self) {
        assert!(self.world.step(DT).unwrap().is_complete());
    }

    /// Every body's state, then every constraint's readout bits.
    fn snapshot(&self) -> Vec<u8> {
        let mut bytes = Vec::new();
        for &id in &self.bodies {
            record_body(&self.world, id, &mut bytes);
        }
        for &id in &self.constraints {
            for bits in readout_bits(&self.world, id) {
                bytes.extend_from_slice(&bits.to_le_bytes());
            }
        }
        bytes
    }

    fn dynamic_positions(&self) -> Vec<RVec3> {
        self.bodies
            .iter()
            .filter(|&&id| self.world.body(id).unwrap().motion_type() == MotionType::Dynamic)
            .map(|&id| self.world.body(id).unwrap().position())
            .collect()
    }
}

fn distance(a: RVec3, b: RVec3) -> Real {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt()
}

#[test]
fn the_joints_of_every_kind_hold_through_a_static_shape_change() {
    let mut changed = Rig::new(1);
    let mut control = Rig::new(1);
    // Every kind that may have a static side uses one of the rig's anchors.
    assert_eq!(changed.supports.len(), 15);
    let shape = off_centre_anchor();
    let mut worst: Real = 0.0;
    for tick in 1..=TICKS {
        if tick == EDIT_TICK {
            assert_eq!(changed.snapshot(), control.snapshot());
            changed.reshape_supports(&shape);
        }
        changed.step();
        control.step();
        if tick == EDIT_TICK {
            let when = "the step after the shape change";
            assert_every_getter_loaded(&changed.world, &changed.constraints, when);
        }
        let positions = changed.dynamic_positions();
        for (a, b) in positions.iter().zip(control.dynamic_positions()) {
            worst = worst.max(distance(*a, b));
        }
    }
    println!("largest drift from the unchanged run: {worst:e} m");
    assert!(worst < DRIFT, "{worst}");
}

#[test]
fn a_gear_over_hinges_on_a_static_support_keeps_coupling_after_the_support_changes_shape() {
    let run = |ratio: f32, change: bool| {
        let mut world = world(GRAVITY, 1);
        let support = anchor(&mut world, RVec3::new(1.0, 0.0, 1.0));
        let centres = [RVec3::new(0.0, 0.0, 0.0), RVec3::new(2.0, 0.0, 0.0)];
        let discs = centres.map(|centre| cube(&mut world, centre, 2.0));
        let z = Vec3::new(0.0, 0.0, 1.0);
        let x = Vec3::new(1.0, 0.0, 0.0);
        let hinges = [0, 1].map(|i| {
            let hinge = HingeConstraintSettings::new(centres[i], z, x);
            world.create_constraint(support, discs[i], &hinge).unwrap()
        });
        let gear = GearConstraintSettings::new(z, z, ratio).constraints(hinges[0], hinges[1]);
        let gear = world.create_constraint(discs[0], discs[1], &gear).unwrap();
        let references = world.constraint(gear).unwrap();
        assert_eq!(references.gear1_constraint(), Some(hinges[0]));
        assert_eq!(references.gear2_constraint(), Some(hinges[1]));
        let mut motor = world.constraint_mut(hinges[0]).unwrap();
        motor.set_motor_state(MotorState::Velocity);
        motor.set_target_angular_velocity(1.0).unwrap();
        for tick in 1..=TICKS {
            if change && tick == EDIT_TICK {
                world
                    .body_mut(support)
                    .unwrap()
                    .set_shape(&grown_anchor(), None, Activation::DontActivate)
                    .unwrap();
            }
            assert!(world.step(DT).unwrap().is_complete());
        }
        let angles = hinges.map(|id| world.constraint(id).unwrap().current_angle());
        let offsets =
            [0, 1].map(|i| distance(world.body(discs[i]).unwrap().position(), centres[i]));
        (angles, offsets)
    };
    // Ratio 2 and the largest ratio a gear accepts.
    for ratio in [2.0, limits::MAX_GEAR_RATIO] {
        let (angles, offsets) = run(ratio, true);
        let (control_angles, _) = run(ratio, false);
        // 2.5 s at 1 rad/s turn the driven disc 2.5 rad and the other one `ratio` times less
        // the other way.
        assert!((angles[0] - 2.5).abs() < 0.05, "{ratio}: {angles:?}");
        assert!(
            (angles[0] + ratio * angles[1]).abs() < 1e-3,
            "{ratio}: {angles:?}"
        );
        for (angle, control) in angles.iter().zip(control_angles) {
            assert!(
                (angle - control).abs() < 1e-4,
                "{ratio}: {angles:?} {control_angles:?}"
            );
        }
        for offset in offsets {
            assert!(offset < DRIFT, "{ratio}: {offsets:?}");
        }
    }
}

#[test]
fn a_static_body_created_able_to_move_changes_shape_under_a_constraint() {
    let mut world = world(GRAVITY, 1);
    let shape = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    let support = world
        .create_body(
            &shape,
            &BodySettings::new_static()
                .allow_dynamic_or_kinematic(true)
                .position(RVec3::new(0.0, 0.0, 1.0)),
        )
        .unwrap();
    let held = cube(&mut world, RVec3::ZERO, 2.0);
    let weld = FixedConstraintSettings::new(
        RVec3::new(0.0, 0.0, 0.5),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    );
    world.create_constraint(support, held, &weld).unwrap();
    world
        .body_mut(support)
        .unwrap()
        .set_shape(&off_centre_anchor(), Some(5.0), Activation::Activate)
        .unwrap();
    for _ in 0..60 {
        assert!(world.step(DT).unwrap().is_complete());
    }
    assert!(distance(world.body(held).unwrap().position(), RVec3::ZERO) < DRIFT);
}

#[test]
fn a_disabled_weld_holds_where_it_was_once_enabled_after_shape_changes() {
    // Without gravity the cube stays where it was created while the weld is disabled.
    let mut world = world(Vec3::ZERO, 1);
    let support = anchor(&mut world, RVec3::new(0.0, 0.0, 1.0));
    let held = cube(&mut world, RVec3::ZERO, 2.0);
    let weld = FixedConstraintSettings::new(
        RVec3::new(0.0, 0.0, 0.5),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    );
    let weld = world.create_constraint(support, held, &weld).unwrap();
    world.constraint_mut(weld).unwrap().set_enabled(false);
    for shape in [grown_anchor(), off_centre_anchor()] {
        world
            .body_mut(support)
            .unwrap()
            .set_shape(&shape, None, Activation::DontActivate)
            .unwrap();
        assert!(world.step(DT).unwrap().is_complete());
    }
    world.constraint_mut(weld).unwrap().set_enabled(true);
    for _ in 0..60 {
        assert!(world.step(DT).unwrap().is_complete());
    }
    let offset = distance(world.body(held).unwrap().position(), RVec3::ZERO);
    assert!(offset < DRIFT, "{offset}");
}

/// A weld, a point pendulum and a rope pendulum, each with its static anchor as body 2: the
/// anchors, the cubes and the constraints.
fn side_two_rig(world: &mut PhysicsWorld) -> ([BodyId; 3], [BodyId; 3], Vec<AnyConstraintId>) {
    let x = Vec3::new(1.0, 0.0, 0.0);
    let y = Vec3::new(0.0, 1.0, 0.0);
    let pivots = [0.0, 8.0, 16.0].map(|x| RVec3::new(x, 10.0, -20.0));
    let anchors = pivots.map(|pivot| anchor(world, shifted(pivot, [0.0, 0.0, 1.0])));
    // The weld holds the cube beside its centre; the pendulums hang 0.3 rad off vertical.
    let below = |pivot| shifted(pivot, [0.75 * 0.3f32.sin(), -0.75 * 0.3f32.cos(), 0.0]);
    let cubes = [
        cube(world, pivots[0], 2.0),
        cube(world, below(pivots[1]), 2.0),
        cube(world, below(pivots[2]), 2.0),
    ];
    let weld = FixedConstraintSettings::new(shifted(pivots[0], [0.4, 0.25, 0.0]), x, y);
    let point = PointConstraintSettings::new(pivots[1]);
    let rope = DistanceConstraintSettings::new(below(pivots[2]), pivots[2]);
    let constraints = vec![
        world
            .create_constraint(cubes[0], anchors[0], &weld)
            .unwrap()
            .into(),
        world
            .create_constraint(cubes[1], anchors[1], &point)
            .unwrap()
            .into(),
        world
            .create_constraint(cubes[2], anchors[2], &rope)
            .unwrap()
            .into(),
    ];
    (anchors, cubes, constraints)
}

#[test]
fn loaded_joints_with_the_static_body_on_side_2_hold_through_its_shape_change() {
    let mut changed = world(GRAVITY, 1);
    let mut control = world(GRAVITY, 1);
    let (anchors, cubes, constraints) = side_two_rig(&mut changed);
    let (_, control_cubes, _) = side_two_rig(&mut control);
    let shape = off_centre_anchor();
    let mut worst: Real = 0.0;
    for tick in 1..=TICKS {
        if tick == EDIT_TICK {
            for &id in &anchors {
                changed
                    .body_mut(id)
                    .unwrap()
                    .set_shape(&shape, None, Activation::DontActivate)
                    .unwrap();
            }
        }
        assert!(changed.step(DT).unwrap().is_complete());
        assert!(control.step(DT).unwrap().is_complete());
        if tick == EDIT_TICK {
            let (_, unloaded) = readout_coverage(&changed, &constraints);
            assert!(unloaded.is_empty(), "no load in {unloaded:?}");
        }
        for (&id, &control_id) in cubes.iter().zip(&control_cubes) {
            let a = changed.body(id).unwrap().position();
            let b = control.body(control_id).unwrap().position();
            worst = worst.max(distance(a, b));
        }
    }
    println!("largest drift from the unchanged run: {worst:e} m");
    assert!(worst < DRIFT, "{worst}");
}

#[test]
fn constrained_bodies_keep_their_other_refusals() {
    let mut world = world(GRAVITY, 1);
    let support = anchor(&mut world, RVec3::new(0.0, 0.0, 1.0));
    let shape = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    let carrier = world
        .create_body(
            &shape,
            &BodySettings::new_kinematic().position(RVec3::new(3.0, 0.0, 0.0)),
        )
        .unwrap();
    let held = cube(&mut world, RVec3::ZERO, 2.0);
    let free = cube(&mut world, RVec3::new(3.0, -1.0, 0.0), 2.0);
    let point = |x: Real| PointConstraintSettings::new(RVec3::new(x, -0.5, 0.0));
    world.create_constraint(support, held, &point(0.0)).unwrap();
    world.create_constraint(carrier, free, &point(3.0)).unwrap();
    let saved = world.save_state();
    let other = grown_anchor();
    for id in [carrier, held, free] {
        assert_eq!(
            world
                .body_mut(id)
                .unwrap()
                .set_shape(&other, None, Activation::Activate),
            Err(BodyError::UsedByConstraint(id))
        );
    }
    world.restore_state(&saved).unwrap();
    assert_eq!(
        world
            .body_mut(support)
            .unwrap()
            .set_motion_type(MotionType::Static, Activation::DontActivate),
        Err(BodyError::UsedByConstraint(support))
    );
    assert_eq!(
        world.remove_body(support),
        Err(BodyError::UsedByConstraint(support))
    );
    world
        .body_mut(support)
        .unwrap()
        .set_shape(&other, None, Activation::Activate)
        .unwrap();
}

#[test]
fn earlier_states_are_refused_and_later_ones_replay() {
    let mut rig = Rig::new(1);
    for _ in 1..EDIT_TICK {
        rig.step();
    }
    let early = rig.world.save_state();
    rig.reshape_supports(&off_centre_anchor());
    rig.step();
    assert_eq!(
        rig.world.restore_state(&early),
        Err(StateError::WorldChanged)
    );
    // Setting the old shape back does not undo the change.
    let first = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    rig.reshape_supports(&first);
    rig.reshape_supports(&off_centre_anchor());
    assert_eq!(
        rig.world.restore_state(&early),
        Err(StateError::WorldChanged)
    );

    let late = rig.world.save_state();
    let at_save = rig.snapshot();
    let recording: Vec<Vec<u8>> = (0..60)
        .map(|_| {
            rig.step();
            rig.snapshot()
        })
        .collect();
    rig.world.restore_state(&late).unwrap();
    rig.world.set_gravity(Vec3::new(2.0, -5.0, 1.0)).unwrap();
    for tick in 0..40 {
        let body = rig.bodies[tick % rig.bodies.len()];
        if rig.world.body(body).unwrap().motion_type() == MotionType::Dynamic {
            rig.world
                .body_mut(body)
                .unwrap()
                .add_impulse(Vec3::new(0.5, 1.0, -0.5))
                .unwrap();
        }
        rig.step();
    }
    assert_ne!(rig.snapshot(), at_save, "the detour changes the world");
    rig.world.restore_state(&late).unwrap();
    rig.world.set_gravity(GRAVITY).unwrap();
    assert_eq!(rig.snapshot(), at_save);
    for (tick, expected) in recording.iter().enumerate() {
        rig.step();
        assert!(
            rig.snapshot() == *expected,
            "the replay differs at tick {tick}"
        );
    }
}

/// The rig with its supports changing shape twice, recorded every tick.
fn reshaped_rig(threads: u32) -> Digest {
    let mut rig = Rig::new(threads);
    let mut digest = Digest::new();
    for tick in 1..=TICKS {
        if tick == EDIT_TICK {
            rig.reshape_supports(&off_centre_anchor());
        }
        if tick == SECOND_EDIT_TICK {
            rig.reshape_supports(&grown_anchor());
        }
        rig.step();
        digest.push().state = rig.snapshot();
    }
    digest
}

#[test]
#[ignore = "child process of the static shape change determinism gate"]
fn static_shape_constraints_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "reshaped_rig");
    finish_child(&reshaped_rig(threads));
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn static_shape_changes_under_constraints_are_identical_with_1_and_4_workers() {
    let one = digest_in_child("static_shape_constraints_child", "reshaped_rig", 1, "");
    let four = digest_in_child("static_shape_constraints_child", "reshaped_rig", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
}
