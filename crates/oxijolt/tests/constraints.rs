//! World constraints: each kind's defining behaviour (limits hold, motors reach their targets,
//! springs oscillate and damp), typed ids, and the guards that keep constraint bodies alive.

mod common;

use std::f32::consts::{FRAC_PI_2, PI};

use common::math::wide;
use common::ragdoll::{humanoid_settings, ragdoll_world};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// A small static box at `position`, the world end of a constraint.
fn add_anchor(world: &mut PhysicsWorld, position: RVec3) -> BodyId {
    let shape = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    world
        .create_body(&shape, &BodySettings::new_static().position(position))
        .unwrap()
}

/// A dynamic box of half extents `half` at `position`.
fn add_box(world: &mut PhysicsWorld, half: Vec3, position: RVec3) -> BodyId {
    let shape = Shape::new_box(half).unwrap();
    world
        .create_body(&shape, &BodySettings::new_dynamic().position(position))
        .unwrap()
}

fn distance(a: RVec3, b: RVec3) -> f64 {
    let d = [a.x - b.x, a.y - b.y, a.z - b.z].map(f64::from);
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// `p + rotation * offset`.
fn attached(position: RVec3, rotation: Quat, offset: Vec3) -> RVec3 {
    let r = common::ragdoll::rotate(rotation, offset);
    RVec3::new(
        position.x + r.x as Real,
        position.y + r.y as Real,
        position.z + r.z as Real,
    )
}

/// Number of sign changes in `values`, ignoring exact zeros.
fn sign_changes(values: &[f64]) -> usize {
    let signs: Vec<bool> = values
        .iter()
        .filter(|v| **v != 0.0)
        .map(|v| *v > 0.0)
        .collect();
    signs.windows(2).filter(|w| w[0] != w[1]).count()
}

/// The largest magnitude in each half of `values`.
fn half_peaks(values: &[f64]) -> (f64, f64) {
    let (first, last) = values.split_at(values.len() / 2);
    let peak = |v: &[f64]| v.iter().fold(0.0_f64, |m, x| m.max(x.abs()));
    (peak(first), peak(last))
}

#[test]
fn fixed_constraint_holds_a_box_under_a_static_bar() {
    let mut world = world(GRAVITY, 1);
    let bar = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let start = RVec3::new(0.0, 4.0, 0.0);
    let cube = add_cube(&mut world, start);
    world
        .create_constraint(
            bar,
            cube,
            &FixedConstraintSettings::default().auto_detect_point(true),
        )
        .unwrap();
    let mut worst: f64 = 0.0;
    let mut worst_turn: f32 = 0.0;
    for _ in 0..120 {
        step(&mut world, 1);
        let body = world.body(cube).unwrap();
        worst = worst.max(distance(body.position(), start));
        worst_turn = worst_turn.max(common::ragdoll::angle_between(
            body.rotation(),
            Quat::IDENTITY,
        ));
    }
    // Measured: 0 m and 0 rad.
    assert!(worst < 1e-2, "moved {worst} m");
    assert!(worst_turn < 1e-2, "turned {worst_turn} rad");
}

#[test]
fn point_constraint_keeps_the_pendulum_pivot() {
    let mut world = world(GRAVITY, 1);
    let pivot = RVec3::new(0.0, 5.0, 0.0);
    let anchor = add_anchor(&mut world, pivot);
    let bob = add_cube(&mut world, RVec3::new(1.5, 5.0, 0.0));
    world
        .create_constraint(anchor, bob, &PointConstraintSettings::new(pivot))
        .unwrap();
    let mut worst: f64 = 0.0;
    for _ in 0..120 {
        step(&mut world, 1);
        let body = world.body(bob).unwrap();
        let end = attached(body.position(), body.rotation(), Vec3::new(-1.5, 0.0, 0.0));
        worst = worst.max(distance(end, pivot));
    }
    // The bob swings down; the pivot stays put. Measured: 0.0041 m.
    assert!(world.body(bob).unwrap().position().y < 4.0);
    assert!(worst < 1e-2, "pivot drifted {worst} m");
}

#[test]
fn distance_constraint_keeps_a_rod_length() {
    let mut world = world(GRAVITY, 1);
    let top = RVec3::new(0.0, 5.0, 0.0);
    let anchor = add_anchor(&mut world, top);
    let start = RVec3::new(2.0, 5.0, 0.0);
    let ball = add_box(&mut world, Vec3::new(0.2, 0.2, 0.2), start);
    let rod = world
        .create_constraint(anchor, ball, &DistanceConstraintSettings::new(top, start))
        .unwrap();
    let reading = world.constraint(rod).unwrap();
    assert_eq!((reading.min_distance(), reading.max_distance()), (2.0, 2.0));
    let mut worst: f64 = 0.0;
    let mut lowest = start.y;
    for _ in 0..120 {
        step(&mut world, 1);
        let position = world.body(ball).unwrap().position();
        worst = worst.max((distance(position, top) - 2.0).abs());
        lowest = lowest.min(position.y);
    }
    // Measured: 0.0050 m, lowest point 2.996 m.
    assert!(lowest < 3.1, "the ball did not swing down: {lowest}");
    assert!(worst < 1e-2, "length off by {worst} m");
    assert!(world
        .constraint(rod)
        .unwrap()
        .total_lambda_position()
        .is_finite());
}

#[test]
fn soft_distance_spring_oscillates_and_damps() {
    let mut world = world(Vec3::ZERO, 1);
    let anchor = add_anchor(&mut world, RVec3::ZERO);
    let ball = add_box(
        &mut world,
        Vec3::new(0.2, 0.2, 0.2),
        RVec3::new(2.2, 0.0, 0.0),
    );
    let spring = SpringSettings::FrequencyAndDamping {
        frequency: 2.0,
        damping: 0.1,
    };
    world
        .create_constraint(
            anchor,
            ball,
            &DistanceConstraintSettings::new(RVec3::ZERO, RVec3::new(2.2, 0.0, 0.0))
                .range(DistanceRange::Range { min: 2.0, max: 2.0 })
                .limits_spring(spring),
        )
        .unwrap();
    let mut offsets = Vec::new();
    for _ in 0..240 {
        step(&mut world, 1);
        offsets.push(distance(world.body(ball).unwrap().position(), RVec3::ZERO) - 2.0);
    }
    let (first, last) = half_peaks(&offsets);
    // Measured: 5 sign changes, peaks 0.19 m then 0.0004 m.
    assert!(sign_changes(&offsets) >= 2, "{offsets:?}");
    assert!(last < 0.5 * first, "peaks {first} then {last}");
}

/// A door: a static post and a 1 m panel hinged on its left edge about `axis`.
fn door(
    world: &mut PhysicsWorld,
    hinge: HingeConstraintSettings,
) -> (BodyId, ConstraintId<HingeConstraint>) {
    // The post stands apart: Jolt lets the two bodies of a constraint collide.
    let post = add_anchor(world, RVec3::new(-2.0, 0.0, 0.0));
    let panel = add_box(world, Vec3::new(0.5, 0.05, 0.5), RVec3::new(0.6, 0.0, 0.0));
    let id = world.create_constraint(post, panel, &hinge).unwrap();
    (panel, id)
}

#[test]
fn hinge_limit_holds_a_falling_door() {
    let mut world = world(GRAVITY, 1);
    // A horizontal hinge along Z: gravity swings the panel down, -Z rotations, until the limit.
    let (_, hinge) = door(
        &mut world,
        HingeConstraintSettings::new(RVec3::new(0.1, 0.0, 0.0), Z, X).limits(-0.5, 0.5),
    );
    step(&mut world, 180);
    let angle = world.constraint(hinge).unwrap().current_angle();
    // At tick 180 the panel rests on the limit; measured: -0.5006 rad.
    assert!(angle < -0.45, "the door did not fall: {angle}");
    assert!(angle > -0.5 - 1e-2, "past the limit: {angle}");
}

#[test]
fn hinge_velocity_motor_reaches_its_target_speed() {
    let mut world = world(Vec3::ZERO, 1);
    let (panel, hinge) = door(
        &mut world,
        HingeConstraintSettings::new(RVec3::new(0.1, 0.0, 0.0), Y, X),
    );
    let mut motor = world.constraint_mut(hinge).unwrap();
    motor.set_target_angular_velocity(2.0).unwrap();
    motor.set_motor_state(MotorState::Velocity);
    step(&mut world, 60);
    let hinge_reading = world.constraint(hinge).unwrap();
    assert_eq!(hinge_reading.motor_state(), MotorState::Velocity);
    assert_eq!(hinge_reading.target_angular_velocity(), 2.0);
    let spin = world.body(panel).unwrap().angular_velocity().y;
    // Measured: 2.00001 rad/s.
    assert!((spin - 2.0).abs() < 0.04, "{spin} rad/s");
}

#[test]
fn hinge_position_motor_reaches_its_target_angle() {
    let mut world = world(Vec3::ZERO, 1);
    let (_, hinge) = door(
        &mut world,
        HingeConstraintSettings::new(RVec3::new(0.1, 0.0, 0.0), Y, X),
    );
    let mut motor = world.constraint_mut(hinge).unwrap();
    motor.set_target_angle(1.0).unwrap();
    motor.set_motor_state(MotorState::Position);
    step(&mut world, 180);
    let angle = world.constraint(hinge).unwrap().current_angle();
    // Measured: 0.9979 rad.
    assert!((angle - 1.0).abs() < 0.02, "{angle}");
    assert_eq!(world.constraint(hinge).unwrap().target_angle(), 1.0);
}

#[test]
fn bodies_of_constraints_cannot_be_removed() {
    let mut world = world(GRAVITY, 1);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let cube = add_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
    let rope = world
        .create_constraint(
            anchor,
            cube,
            &DistanceConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0), RVec3::new(0.0, 3.0, 0.0)),
        )
        .unwrap();
    for body in [anchor, cube] {
        assert_eq!(
            world.remove_body(body),
            Err(BodyError::UsedByConstraint(body))
        );
        assert!(world.constraints_of_body(body).eq([rope.into()]));
    }
    step(&mut world, 10);
    world.remove_constraint(rope).unwrap();
    assert_eq!(world.constraints_of_body(cube).next(), None);
    world.remove_body(cube).unwrap();
    world.remove_body(anchor).unwrap();
    step(&mut world, 10);
}

#[test]
fn constraints_refuse_ragdoll_parts_inner_bodies_and_one_body() {
    let (mut world, layers) = ragdoll_world(1);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let point = PointConstraintSettings::new(RVec3::new(0.0, 4.0, 0.0));

    let ragdoll = world
        .create_ragdoll(
            &humanoid_settings(layers.ragdoll),
            None,
            Activation::Activate,
        )
        .unwrap();
    let part = world.ragdoll(ragdoll).unwrap().body_ids()[0];
    assert_eq!(
        world.create_constraint(anchor, part, &point),
        Err(ConstraintError::Body(BodyError::OwnedByRagdoll(part)))
    );

    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let character = world
        .create_character(
            &CharacterSettings::new(&capsule).inner_body(Some(InnerBody {
                shape: &capsule,
                object_layer: layers.ragdoll,
            })),
            RVec3::new(5.0, 1.0, 0.0),
            Quat::IDENTITY,
        )
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    assert_eq!(
        world.create_constraint(inner, anchor, &point),
        Err(ConstraintError::Body(BodyError::OwnedByCharacter(inner)))
    );

    assert!(matches!(
        world.create_constraint(anchor, anchor, &point),
        Err(ConstraintError::InvalidValue(_))
    ));
    let (mut other, _) = ragdoll_world(1);
    let foreign = add_anchor(&mut other, RVec3::ZERO);
    assert_eq!(
        world.create_constraint(anchor, foreign, &point),
        Err(ConstraintError::Body(BodyError::WrongWorld(foreign)))
    );
    assert_eq!(world.constraint_count(), 0);
}

#[test]
fn constraint_ids_are_typed_ordered_and_never_reused() {
    let mut world = world(GRAVITY, 1);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let cubes: Vec<BodyId> = (0..3)
        .map(|i| add_cube(&mut world, RVec3::new(2.0 * i as Real, 3.0, 0.0)))
        .collect();
    let point = |world: &mut PhysicsWorld, cube| {
        world
            .create_constraint(
                anchor,
                cube,
                &PointConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0)),
            )
            .unwrap()
    };
    let first = point(&mut world, cubes[0]);
    let hinge = world
        .create_constraint(
            anchor,
            cubes[1],
            &HingeConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0), Z, X),
        )
        .unwrap();
    let third = point(&mut world, cubes[2]);
    assert_eq!([first.to_raw(), hinge.to_raw(), third.to_raw()], [1, 2, 3]);
    assert!(first < third);
    assert!(world.contains_constraint(hinge));
    assert!(world.contains_constraint(AnyConstraintId::from(first)));
    world.remove_constraint(hinge).unwrap();
    assert!(!world.contains_constraint(hinge));
    let fourth = point(&mut world, cubes[1]);
    assert_eq!(fourth.to_raw(), 4);

    let ids: Vec<AnyConstraintId> = world.constraint_ids().collect();
    assert_eq!(ids, vec![first.into(), third.into(), fourth.into()]);
    assert_eq!(ids[0].kind(), ConstraintType::Point);
    assert_eq!(ids[0].downcast::<PointConstraint>(), Some(first));
    assert_eq!(ids[0].downcast::<HingeConstraint>(), None);
    assert_eq!(AnyConstraintId::from(hinge).kind(), ConstraintType::Hinge);
}

#[test]
fn wrong_world_and_removed_ids_are_rejected() {
    let mut world = world(GRAVITY, 1);
    let mut other = common::world(GRAVITY, 1);
    let make = |world: &mut PhysicsWorld| {
        let anchor = add_anchor(world, RVec3::new(0.0, 5.0, 0.0));
        let cube = add_cube(world, RVec3::new(0.0, 3.0, 0.0));
        world
            .create_constraint(
                anchor,
                cube,
                &HingeConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0), Z, X),
            )
            .unwrap()
    };
    let ours = make(&mut world);
    let theirs = make(&mut other);
    assert_eq!(ours.to_raw(), theirs.to_raw());
    assert_ne!(ours, theirs);
    assert!(world.contains_constraint(ours) && !world.contains_constraint(theirs));
    assert_eq!(
        world.constraint(theirs).err(),
        Some(ConstraintError::WrongWorld(theirs.into()))
    );
    assert_eq!(
        world.remove_constraint(theirs),
        Err(ConstraintError::WrongWorld(theirs.into()))
    );
    world.remove_constraint(ours).unwrap();
    assert_eq!(
        world.constraint_mut(ours).err().map(|e| e.to_string()),
        Some(ConstraintError::NotFound(ours.into()).to_string())
    );
    assert_eq!(
        world.remove_constraint(ours),
        Err(ConstraintError::NotFound(ours.into()))
    );
}

#[test]
fn dropping_a_world_with_constraints_is_clean() {
    for steps in [0, 30] {
        let mut world = world(GRAVITY, 1);
        let anchor = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
        let a = add_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
        let b = add_cube(&mut world, RVec3::new(2.0, 3.0, 0.0));
        world
            .create_constraint(
                anchor,
                a,
                &PointConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0)),
            )
            .unwrap();
        world
            .create_constraint(
                a,
                b,
                &FixedConstraintSettings::default().auto_detect_point(true),
            )
            .unwrap();
        world
            .create_constraint(
                anchor,
                b,
                &HingeConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0), Z, X),
            )
            .unwrap();
        step(&mut world, steps);
        drop(world);
    }
}

/// Asserts that `settings` are refused as invalid and leave the world's counts unchanged.
fn assert_creates_nothing<S: ConstraintSettings>(
    world: &mut PhysicsWorld,
    bodies: [BodyId; 2],
    settings: &S,
) {
    let body_count = world.body_count();
    let constraint_count = world.constraint_count();
    assert!(matches!(
        world.create_constraint(bodies[0], bodies[1], settings),
        Err(ConstraintError::InvalidValue(_))
    ));
    assert_eq!(world.body_count(), body_count);
    assert_eq!(world.constraint_count(), constraint_count);
}

#[test]
fn invalid_constraint_creates_nothing() {
    let mut world = world(GRAVITY, 1);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let cube = add_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
    let bodies = [anchor, cube];
    assert_creates_nothing(
        &mut world,
        bodies,
        &PointConstraintSettings::new(RVec3::new(Real::NAN, 0.0, 0.0)),
    );
    assert_creates_nothing(
        &mut world,
        bodies,
        &HingeConstraintSettings::new(RVec3::ZERO, Z, Z),
    );
    assert_creates_nothing(
        &mut world,
        bodies,
        &DistanceConstraintSettings::new(RVec3::ZERO, RVec3::ZERO)
            .range(DistanceRange::Range { min: 2.0, max: 1.0 }),
    );
    assert_creates_nothing(
        &mut world,
        bodies,
        &FixedConstraintSettings::default()
            .space(ConstraintSpace::LocalToBodyCom)
            .auto_detect_point(true),
    );
    let valid = world
        .create_constraint(
            anchor,
            cube,
            &PointConstraintSettings::new(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    assert_eq!(valid.to_raw(), 1);
}

#[test]
fn removing_a_constraint_wakes_its_bodies() {
    let mut world = world(GRAVITY, 1);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let cube = add_cube(&mut world, RVec3::new(0.0, 4.0, 0.0));
    let weld = world
        .create_constraint(
            anchor,
            cube,
            &FixedConstraintSettings::default().auto_detect_point(true),
        )
        .unwrap();
    let asleep = (0..600).any(|_| {
        step(&mut world, 1);
        world.body(cube).unwrap().is_sleeping()
    });
    assert!(asleep, "the welded cube never fell asleep");
    world.remove_constraint(weld).unwrap();
    assert!(!world.body(cube).unwrap().is_sleeping());
    step(&mut world, 30);
    assert!(world.body(cube).unwrap().position().y < 3.5);
}

#[test]
fn gears_racks_and_pulleys_need_two_dynamic_bodies() {
    let mut world = world(Vec3::ZERO, 1);
    let shape = Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap();
    let wall = world
        .create_body(&shape, &BodySettings::new_static())
        .unwrap();
    let conveyor = world
        .create_body(
            &shape,
            &BodySettings::new_kinematic()
                .position(RVec3::new(0.0, 3.0, 0.0))
                .angular_velocity(Z),
        )
        .unwrap();
    let wheel = add_box(
        &mut world,
        Vec3::new(0.2, 0.2, 0.2),
        RVec3::new(3.0, 0.0, 0.0),
    );
    let other = add_box(
        &mut world,
        Vec3::new(0.2, 0.2, 0.2),
        RVec3::new(6.0, 0.0, 0.0),
    );
    let gear = GearConstraintSettings::new(Z, Z, 2.0);
    let rack = RackAndPinionConstraintSettings::new(Z, X, 2.0);
    let pulley = PulleyConstraintSettings::new(
        RVec3::new(3.0, 0.0, 0.0),
        RVec3::new(3.0, 5.0, 0.0),
        RVec3::new(6.0, 0.0, 0.0),
        RVec3::new(6.0, 5.0, 0.0),
    )
    .length(PulleyLength::Range {
        min: 0.0,
        max: 12.0,
    });
    for not_dynamic in [wall, conveyor] {
        for pair in [[not_dynamic, wheel], [wheel, not_dynamic]] {
            let refused = |result: Result<AnyConstraintId, ConstraintError>| {
                assert!(
                    matches!(result, Err(ConstraintError::NotDynamic(id)) if id == not_dynamic),
                    "{result:?}"
                );
            };
            refused(
                world
                    .create_constraint(pair[0], pair[1], &gear)
                    .map(Into::into),
            );
            refused(
                world
                    .create_constraint(pair[0], pair[1], &rack)
                    .map(Into::into),
            );
            refused(
                world
                    .create_constraint(pair[0], pair[1], &pulley)
                    .map(Into::into),
            );
        }
    }
    // Nothing was created, and the world steps; the kinematic body keeps its own motion.
    assert_eq!(world.constraint_count(), 0);
    step(&mut world, 10);
    assert_eq!(world.body(conveyor).unwrap().angular_velocity(), Z);
    // Between two dynamic bodies all three are accepted.
    world.create_constraint(wheel, other, &gear).unwrap();
    world.create_constraint(wheel, other, &rack).unwrap();
    world.create_constraint(wheel, other, &pulley).unwrap();
    assert_eq!(world.constraint_count(), 3);
    step(&mut world, 10);
}

#[test]
fn a_new_constraint_wakes_its_sleeping_bodies() {
    // A box asleep on the ground gets a rope of at most 5 m to an anchor 10 m above it.
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 10.0, 0.0));
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    fall_asleep(&mut world, cube);
    let rope =
        DistanceConstraintSettings::new(RVec3::new(0.0, 10.0, 0.0), RVec3::new(0.0, 0.5, 0.0))
            .range(DistanceRange::Range { min: 0.0, max: 5.0 });
    world.create_constraint(anchor, cube, &rope).unwrap();
    assert!(!world.body(cube).unwrap().is_sleeping());
    step(&mut world, 120);
    // The rope lifted the box to within 5 m of the anchor; measured 4.999999 m.
    let y = wide(world.body(cube).unwrap().position().y);
    assert!(y > 4.99, "{y}");
}

/// Steps `world` until `body` sleeps, at most 900 ticks.
fn fall_asleep(world: &mut PhysicsWorld, body: BodyId) {
    let asleep = (0..900).any(|_| {
        step(world, 1);
        world.body(body).unwrap().is_sleeping()
    });
    assert!(asleep, "the body never fell asleep");
}

/// A world without gravity with [`anchored_box`]'s bodies joined by `settings`.
fn sleeping_pair<S: ConstraintSettings>(
    settings: &S,
) -> (PhysicsWorld, BodyId, ConstraintId<S::Kind>) {
    let mut world = world(Vec3::ZERO, 1);
    let (anchor, body) = anchored_box(&mut world);
    let id = world.create_constraint(anchor, body, settings).unwrap();
    (world, body, id)
}

/// Asserts that `call` on the constraint of a scene whose body has fallen asleep wakes it.
macro_rules! assert_wakes {
    ($scene:expr, |$constraint:ident| $call:expr) => {{
        let (mut world, body, id) = $scene;
        fall_asleep(&mut world, body);
        {
            let mut $constraint = world.constraint_mut(id).unwrap();
            $call.unwrap();
        }
        assert!(
            !world.body(body).unwrap().is_sleeping(),
            "{} left the body asleep",
            stringify!($call)
        );
    }};
}

#[test]
fn every_setter_wakes_the_constraint_bodies() {
    let soft = SpringSettings::FrequencyAndDamping {
        frequency: 2.0,
        damping: 0.5,
    };
    let motor = MotorSettings::default();
    let distance = DistanceConstraintSettings::new(RVec3::new(0.0, 10.0, 10.0), RVec3::ZERO);
    assert_wakes!(sleeping_pair(&distance), |c| c.set_limits_spring(soft));
    let slider = SliderConstraintSettings::new(RVec3::ZERO, Y, X);
    assert_wakes!(sleeping_pair(&slider), |c| c.set_motor_settings(motor));
    assert_wakes!(sleeping_pair(&slider), |c| c.set_limits(-1.0, 0.0));
    let limited = SliderConstraintSettings::new(RVec3::ZERO, Y, X).limits(-1.0, 0.0);
    assert_wakes!(sleeping_pair(&limited), |c| {
        c.remove_limits();
        Ok::<(), ConstraintError>(())
    });
    assert_wakes!(sleeping_pair(&slider), |c| c.set_limits_spring(soft));
    assert_wakes!(sleeping_pair(&slider), |c| c.set_max_friction_force(1.0));
    let hinge = HingeConstraintSettings::new(RVec3::ZERO, Z, X);
    assert_wakes!(sleeping_pair(&hinge), |c| c.set_motor_settings(motor));
    assert_wakes!(sleeping_pair(&hinge), |c| c.set_limits(-0.5, 0.5));
    assert_wakes!(sleeping_pair(&hinge), |c| c.set_limits_spring(soft));
    assert_wakes!(sleeping_pair(&hinge), |c| c.set_max_friction_torque(1.0));
    let swing_twist = SwingTwistConstraintSettings::new(RVec3::ZERO, X, Y);
    assert_wakes!(sleeping_pair(&swing_twist), |c| c
        .set_swing_motor_settings(motor));
    assert_wakes!(sleeping_pair(&swing_twist), |c| c
        .set_twist_motor_settings(motor));
    assert_wakes!(sleeping_pair(&swing_twist), |c| c
        .set_max_friction_torque(1.0));
    assert_wakes!(sleeping_pair(&six_dof_settings()), |c| c
        .set_motor_settings(SixDofConstraintAxis::RotationX, motor));
    let path = || {
        let mut world = world(Vec3::ZERO, 1);
        let (body, id) = path_scene(&mut world, RVec3::ZERO, Quat::IDENTITY, &s_curve());
        (world, body, id)
    };
    assert_wakes!(path(), |c| c.set_position_motor_settings(motor));
    assert_wakes!(path(), |c| c.set_max_friction_force(1.0));
}

#[test]
fn changed_limits_motors_and_friction_act_on_sleeping_bodies() {
    // Each body falls asleep where the old settings let it rest; the change moves it.
    // A slider that let the box fall to -2 m is narrowed to -1 m.
    let mut world = world(GRAVITY, 1);
    let (anchor, body) = anchored_box(&mut world);
    let slider = world
        .create_constraint(
            anchor,
            body,
            &SliderConstraintSettings::new(RVec3::ZERO, Y, X).limits(-2.0, 0.0),
        )
        .unwrap();
    fall_asleep(&mut world, body);
    let before = world.constraint(slider).unwrap().current_position();
    world
        .constraint_mut(slider)
        .unwrap()
        .set_limits(-1.0, 0.0)
        .unwrap();
    step(&mut world, 60);
    let after = world.constraint(slider).unwrap().current_position();
    // Measured: -2.0000 m, then -1.0000 m.
    assert!((before + 2.0).abs() < 1e-2, "{before}");
    assert!((after + 1.0).abs() < 1e-2, "{after}");

    // A door that fell to its -1 rad limit is narrowed to -0.5 rad.
    let door_resting_on = |limit: f32| {
        let mut world = common::world(GRAVITY, 1);
        let (panel, hinge) = door(
            &mut world,
            HingeConstraintSettings::new(RVec3::new(0.1, 0.0, 0.0), Z, X).limits(-limit, limit),
        );
        fall_asleep(&mut world, panel);
        (world, panel, hinge)
    };
    let angle = |world: &PhysicsWorld, hinge| world.constraint(hinge).unwrap().current_angle();
    let (mut world, _, hinge) = door_resting_on(1.0);
    let before = angle(&world, hinge);
    world
        .constraint_mut(hinge)
        .unwrap()
        .set_limits(-0.5, 0.5)
        .unwrap();
    step(&mut world, 60);
    let after = angle(&world, hinge);
    // Measured: -1.0021 rad, then -0.5062 rad.
    assert!((before + 1.0).abs() < 1e-2, "{before}");
    assert!((after + 0.5).abs() < 1e-2, "{after}");

    // A position motor without a spring does nothing; giving it one lifts the door.
    let (mut world, panel, hinge) = door_resting_on(1.0);
    let mut motor = world.constraint_mut(hinge).unwrap();
    motor
        .set_motor_settings(MotorSettings::default().spring(SpringSettings::default()))
        .unwrap();
    motor.set_target_angle(0.0).unwrap();
    motor.set_motor_state(MotorState::Position);
    fall_asleep(&mut world, panel);
    let before = angle(&world, hinge);
    world
        .constraint_mut(hinge)
        .unwrap()
        .set_motor_settings(MotorSettings::default())
        .unwrap();
    step(&mut world, 120);
    let after = angle(&world, hinge);
    // Measured: -1.0004 rad, then -0.3514 rad: the 2 Hz spring sags under the door's weight.
    assert!((before + 1.0).abs() < 1e-2, "{before}");
    assert!(after > -0.45, "{after}");

    // Friction that held the door level is released and the door falls to its limit.
    let mut world = common::world(GRAVITY, 1);
    let (panel, hinge) = door(
        &mut world,
        HingeConstraintSettings::new(RVec3::new(0.1, 0.0, 0.0), Z, X)
            .limits(-1.0, 1.0)
            .max_friction_torque(1.0e4),
    );
    fall_asleep(&mut world, panel);
    let before = angle(&world, hinge);
    world
        .constraint_mut(hinge)
        .unwrap()
        .set_max_friction_torque(0.0)
        .unwrap();
    step(&mut world, 120);
    let after = angle(&world, hinge);
    // Measured: -0.0003 rad, then -1.0021 rad.
    assert!(before.abs() < 1e-2, "{before}");
    assert!((after + 1.0).abs() < 1e-2, "{after}");
}

#[test]
fn hinge_setters_check_their_values() {
    let mut world = world(Vec3::ZERO, 1);
    let (_, hinge) = door(
        &mut world,
        HingeConstraintSettings::new(RVec3::new(0.1, 0.0, 0.0), Y, X),
    );
    let mut hinge = world.constraint_mut(hinge).unwrap();
    let invalid = |result: Result<(), ConstraintError>| {
        assert!(
            matches!(result, Err(ConstraintError::InvalidValue(_))),
            "{result:?}"
        );
    };
    invalid(hinge.set_target_angle(PI + 0.01));
    invalid(hinge.set_target_angle(f32::NAN));
    hinge.set_target_angle(-PI).unwrap();
    invalid(hinge.set_limits(0.1, 0.5));
    invalid(hinge.set_limits(0.0, 0.0));
    hinge.set_limits(-FRAC_PI_2, FRAC_PI_2).unwrap();
    invalid(hinge.set_max_friction_torque(-1.0));
    hinge.set_max_friction_torque(2.0).unwrap();
    invalid(hinge.set_motor_settings(MotorSettings::default().torque_limits(1.0, -1.0)));
    hinge
        .set_motor_settings(MotorSettings::default().torque_limits(-10.0, 10.0))
        .unwrap();
    let soft = SpringSettings::FrequencyAndDamping {
        frequency: 5.0,
        damping: 0.5,
    };
    hinge.set_limits_spring(soft).unwrap();
    hinge.set_limits(0.0, 0.0).unwrap();
    // Limits with min == max keep needing a soft spring.
    invalid(hinge.set_limits_spring(SpringSettings::default()));
    hinge.set_enabled(false);
    assert!(!hinge.is_enabled());
    let id = hinge.id();
    let reading = world.constraint(id).unwrap();
    assert_eq!(reading.limits(), (0.0, 0.0));
    assert_eq!(reading.max_friction_torque(), 2.0);
}

/// A static anchor far from `body`'s path and a dynamic 0.4 m box at the origin.
fn anchored_box(world: &mut PhysicsWorld) -> (BodyId, BodyId) {
    let anchor = add_anchor(world, RVec3::new(0.0, 10.0, 10.0));
    let body = add_box(world, Vec3::new(0.2, 0.2, 0.2), RVec3::ZERO);
    (anchor, body)
}

/// Twist about X and swing angles about Y and Z of a constraint-space rotation, with Jolt's
/// swing-twist decomposition (`Quat::GetSwingTwist`): twist first, swing angles as twice the
/// half angles `atan2(swing.y, swing.w)` and `atan2(swing.z, swing.w)`.
fn twist_and_swing(q: Quat) -> (f32, f32, f32) {
    let s = (q.w * q.w + q.x * q.x).sqrt();
    let twist = Quat::from_xyzw(q.x / s, 0.0, 0.0, q.w / s);
    let swing = common::ragdoll::mul(q, common::ragdoll::conj(twist));
    let half = |v: f32, w: f32| 2.0 * v.atan2(w);
    (
        half(twist.x, twist.w),
        half(swing.y, swing.w),
        half(swing.z, swing.w),
    )
}

#[test]
fn slider_limit_holds() {
    let mut world = world(GRAVITY, 1);
    let (anchor, body) = anchored_box(&mut world);
    let slider = world
        .create_constraint(
            anchor,
            body,
            &SliderConstraintSettings::new(RVec3::ZERO, Y, X).limits(-1.0, 0.0),
        )
        .unwrap();
    step(&mut world, 120);
    let position = world.constraint(slider).unwrap().current_position();
    let body = world.body(body).unwrap();
    // Measured: -1.0000 m; the box stays on the axis.
    assert!((position + 1.0).abs() < 1e-2, "{position}");
    assert!(body.position().x.abs() < 1e-3 && body.position().z.abs() < 1e-3);
    assert_eq!(
        world.constraint(slider).unwrap().limits(),
        Some((-1.0, 0.0))
    );
}

#[test]
fn slider_position_motor_reaches_its_target() {
    let mut world = world(Vec3::ZERO, 1);
    let (anchor, body) = anchored_box(&mut world);
    let slider = world
        .create_constraint(
            anchor,
            body,
            &SliderConstraintSettings::new(RVec3::ZERO, X, Y),
        )
        .unwrap();
    let mut motor = world.constraint_mut(slider).unwrap();
    motor.set_target_position(0.5).unwrap();
    motor.set_motor_state(MotorState::Position);
    step(&mut world, 180);
    let position = world.constraint(slider).unwrap().current_position();
    // Measured: 0.4999 m.
    assert!((position - 0.5).abs() < 1e-2, "{position}");
    let reading = world.constraint(slider).unwrap();
    assert_eq!(reading.motor_state(), MotorState::Position);
    assert_eq!(reading.target_position(), 0.5);
}

#[test]
fn slider_soft_limit_spring_oscillates_and_damps() {
    let mut world = world(Vec3::ZERO, 1);
    let (anchor, body) = anchored_box(&mut world);
    let spring = SpringSettings::FrequencyAndDamping {
        frequency: 2.0,
        damping: 0.1,
    };
    let slider = world
        .create_constraint(
            anchor,
            body,
            &SliderConstraintSettings::new(RVec3::ZERO, X, Y)
                .limits(0.0, 0.0)
                .limits_spring(spring),
        )
        .unwrap();
    world
        .body_mut(body)
        .unwrap()
        .set_linear_velocity(X)
        .unwrap();
    let mut offsets = Vec::new();
    for _ in 0..240 {
        step(&mut world, 1);
        offsets.push(f64::from(
            world.constraint(slider).unwrap().current_position(),
        ));
    }
    let (first, last) = half_peaks(&offsets);
    // Measured: 3 sign changes, peaks 0.059 m then 0.0005 m.
    assert!(sign_changes(&offsets) >= 2, "{offsets:?}");
    assert!(last < 0.5 * first, "peaks {first} then {last}");
}

#[test]
fn cone_holds_a_hanging_body_within_its_angle() {
    // Gravity tilted by atan(5 / 9.81) = 0.47 rad would hang the body outside the 0.3 rad cone,
    // so it comes to rest on the cone's edge.
    let mut world = world(Vec3::new(5.0, -9.81, 0.0), 1);
    let anchor = add_anchor(&mut world, RVec3::new(0.0, 10.0, 10.0));
    let body = add_box(
        &mut world,
        Vec3::new(0.1, 0.5, 0.1),
        RVec3::new(0.0, -0.5, 0.0),
    );
    let down = Vec3::new(0.0, -1.0, 0.0);
    let cone = world
        .create_constraint(
            anchor,
            body,
            &ConeConstraintSettings::new(RVec3::ZERO, down, 0.3),
        )
        .unwrap();
    let swing = |world: &PhysicsWorld| {
        let axis = common::ragdoll::rotate(world.body(body).unwrap().rotation(), down);
        (-axis.y).clamp(-1.0, 1.0).acos()
    };
    let mut peak: f32 = 0.0;
    for _ in 0..180 {
        step(&mut world, 1);
        peak = peak.max(swing(&world));
    }
    let settled = swing(&world);
    let reading = world.constraint(cone).unwrap();
    assert!((reading.half_cone_angle() - 0.3).abs() < 1e-5);
    // Measured at tick 180: 0.3037 rad; on the way the swing into the cone overshot to 0.3098.
    assert!((settled - 0.3).abs() < 1e-2, "{settled} rad");
    assert!(peak < 0.32, "overshoot to {peak} rad");
}

#[test]
fn swing_twist_twist_limit_holds() {
    let mut world = world(Vec3::ZERO, 1);
    let (anchor, body) = anchored_box(&mut world);
    let joint = world
        .create_constraint(
            anchor,
            body,
            &SwingTwistConstraintSettings::new(RVec3::ZERO, X, Y)
                .half_cone_angles(0.5, 0.5)
                .twist_limits(-0.3, 0.3),
        )
        .unwrap();
    // A velocity motor keeps twisting the body into the limit.
    let mut motor = world.constraint_mut(joint).unwrap();
    motor
        .set_target_angular_velocity_cs(Vec3::new(2.0, 0.0, 0.0))
        .unwrap();
    motor.set_twist_motor_state(MotorState::Velocity);
    let twist_now = |world: &PhysicsWorld| {
        let rotation = world
            .constraint(joint)
            .unwrap()
            .rotation_in_constraint_space();
        twist_and_swing(rotation).0
    };
    let mut peak: f32 = 0.0;
    for _ in 0..120 {
        step(&mut world, 1);
        peak = peak.max(twist_now(&world));
    }
    // Measured peak on the way into the limit: 0.3000 rad in the default build, 0.3213 rad with
    // `cross-platform-deterministic`.
    assert!(peak < 0.35, "overshoot to {peak} rad");
    let reading = world.constraint(joint).unwrap();
    let twist = twist_and_swing(reading.rotation_in_constraint_space()).0;
    assert_eq!(reading.twist_limits(), (-0.3, 0.3));
    // Measured at tick 120: 0.3000 rad.
    assert!((twist - 0.3).abs() < 1e-2, "{twist} rad");
}

#[test]
fn swing_twist_motor_reaches_target_orientation_with_a_rotated_parent() {
    let mut world = world(Vec3::ZERO, 1);
    let turn = quat_about(Vec3::new(0.48, 0.6, 0.64), 0.9);
    let shape = Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap();
    let dynamic = |position| {
        BodySettings::new_dynamic()
            .position(position)
            .rotation(turn)
    };
    let parent = world.create_body(&shape, &dynamic(RVec3::ZERO)).unwrap();
    let child = world
        .create_body(&shape, &dynamic(RVec3::new(1.0, 0.0, 0.0)))
        .unwrap();
    // Twist axis X and plane axis Z make the constraint frame the world axes (the normal axis
    // is plane × twist = Y). It is given in world space, so in both bodies' frames it is turn⁻¹.
    let joint = world
        .create_constraint(
            parent,
            child,
            &SwingTwistConstraintSettings::new(RVec3::new(0.5, 0.0, 0.0), X, Z)
                .half_cone_angles(1.0, 1.0)
                .twist_limits(-1.0, 1.0),
        )
        .unwrap();
    let target = common::ragdoll::mul(quat_about(Z, 0.5), quat_about(X, 0.4));
    let mut motor = world.constraint_mut(joint).unwrap();
    motor.set_target_orientation_cs(target).unwrap();
    motor.set_swing_motor_state(MotorState::Position);
    motor.set_twist_motor_state(MotorState::Position);
    step(&mut world, 240);

    let rotation = |id| world.body(id).unwrap().rotation();
    let relative = common::ragdoll::mul(common::ragdoll::conj(rotation(parent)), rotation(child));
    let wanted = common::ragdoll::mul(
        common::ragdoll::mul(common::ragdoll::conj(turn), target),
        turn,
    );
    let error = common::ragdoll::angle_between(relative, wanted);
    let reading = world.constraint(joint).unwrap();
    let in_constraint_space =
        common::ragdoll::angle_between(reading.rotation_in_constraint_space(), target);
    // Measured: 0.0012 rad for both; the relative rotation is 0.26 rad from the target itself.
    assert!(error < 2e-2, "{error} rad");
    assert!(in_constraint_space < 2e-2, "{in_constraint_space} rad");
    assert_eq!(reading.swing_motor_state(), MotorState::Position);
}

#[test]
fn swing_twist_reads_back_its_motor_settings() {
    let swing = MotorSettings::default().torque_limits(-5.0, 6.0);
    let twist = MotorSettings::default().spring(SpringSettings::FrequencyAndDamping {
        frequency: 4.0,
        damping: 0.7,
    });
    let (mut world, _, joint) = sleeping_pair(
        &SwingTwistConstraintSettings::new(RVec3::ZERO, X, Y)
            .swing_motor(swing)
            .twist_motor(twist),
    );
    let reading = world.constraint(joint).unwrap();
    assert_eq!(reading.swing_motor_settings(), swing);
    assert_eq!(reading.twist_motor_settings(), twist);
    let mut motors = world.constraint_mut(joint).unwrap();
    motors.set_swing_motor_settings(twist).unwrap();
    motors.set_twist_motor_settings(swing).unwrap();
    let reading = world.constraint(joint).unwrap();
    assert_eq!(reading.swing_motor_settings(), twist);
    assert_eq!(reading.twist_motor_settings(), swing);
}

/// A six-DOF joint at the origin with every translation fixed except as `settings` say.
fn six_dof(
    world: &mut PhysicsWorld,
    settings: SixDofConstraintSettings,
) -> (BodyId, ConstraintId<SixDofConstraint>) {
    let (anchor, body) = anchored_box(world);
    let id = world.create_constraint(anchor, body, &settings).unwrap();
    (body, id)
}

fn six_dof_settings() -> SixDofConstraintSettings {
    let mut settings = SixDofConstraintSettings::new(RVec3::ZERO, X, Y);
    for axis in [
        SixDofConstraintAxis::TranslationX,
        SixDofConstraintAxis::TranslationY,
        SixDofConstraintAxis::TranslationZ,
    ] {
        settings = settings.axis(axis, SixDofAxis::Fixed);
    }
    settings
}

#[test]
fn six_dof_translation_limit_holds() {
    let mut world = world(GRAVITY, 1);
    let (body, joint) = six_dof(
        &mut world,
        six_dof_settings().axis(
            SixDofConstraintAxis::TranslationY,
            SixDofAxis::Limited {
                min: -0.5,
                max: 0.0,
            },
        ),
    );
    step(&mut world, 120);
    let y = world.body(body).unwrap().position().y;
    // Measured: -0.5000 m.
    assert!((y + 0.5).abs() < 1e-2, "{y}");
    assert_eq!(
        world
            .constraint(joint)
            .unwrap()
            .limits(SixDofConstraintAxis::TranslationY),
        Some((-0.5, 0.0))
    );
}

#[test]
fn six_dof_translation_spring_oscillates_and_damps() {
    let mut world = world(Vec3::ZERO, 1);
    let y = SixDofConstraintAxis::TranslationY;
    let spring = SpringSettings::FrequencyAndDamping {
        frequency: 2.0,
        damping: 0.1,
    };
    let (body, _) = six_dof(
        &mut world,
        six_dof_settings()
            .axis(
                y,
                SixDofAxis::Limited {
                    min: -0.01,
                    max: 0.01,
                },
            )
            .limits_spring(y, spring),
    );
    world
        .body_mut(body)
        .unwrap()
        .set_linear_velocity(Y)
        .unwrap();
    let mut offsets = Vec::new();
    for _ in 0..240 {
        step(&mut world, 1);
        offsets.push(wide(world.body(body).unwrap().position().y));
    }
    let (first, last) = half_peaks(&offsets);
    // Measured: 5 sign changes, peaks 0.071 m then 0.0037 m.
    assert!(sign_changes(&offsets) >= 2, "{offsets:?}");
    assert!(last < 0.5 * first, "peaks {first} then {last}");
}

#[test]
fn six_dof_rotation_position_motor_reaches_its_target() {
    let mut world = world(Vec3::ZERO, 1);
    let mut settings = six_dof_settings();
    for axis in SIX_DOF_ROTATIONS {
        settings = settings.motor(axis, MotorSettings::default());
    }
    let (_, joint) = six_dof(&mut world, settings);
    let target = common::ragdoll::mul(quat_about(Y, 0.6), quat_about(X, -0.3));
    let mut motor = world.constraint_mut(joint).unwrap();
    motor.set_target_orientation_cs(target).unwrap();
    for axis in SIX_DOF_ROTATIONS {
        motor.set_motor_state(axis, MotorState::Position);
    }
    step(&mut world, 240);
    let reading = world.constraint(joint).unwrap();
    let error = common::ragdoll::angle_between(reading.rotation_in_constraint_space(), target);
    // Measured: 0.0012 rad.
    assert!(error < 2e-2, "{error} rad");
    assert_eq!(
        reading.motor_state(SixDofConstraintAxis::RotationY),
        MotorState::Position
    );
}

const SIX_DOF_ROTATIONS: [SixDofConstraintAxis; 3] = [
    SixDofConstraintAxis::RotationX,
    SixDofConstraintAxis::RotationY,
    SixDofConstraintAxis::RotationZ,
];

#[test]
fn six_dof_pyramid_holds_asymmetric_limits() {
    let limited = |min, max| SixDofAxis::Limited { min, max };
    // A velocity motor keeps turning the body into one swing limit, then the other.
    for (spin, bound) in [(1.0, 0.6), (-1.0, -0.2)] {
        let mut world = world(Vec3::ZERO, 1);
        let y = SixDofConstraintAxis::RotationY;
        let (_, joint) = six_dof(
            &mut world,
            six_dof_settings()
                .swing_type(SwingType::Pyramid)
                .axis(SixDofConstraintAxis::RotationX, SixDofAxis::Fixed)
                .axis(y, limited(-0.2, 0.6))
                .axis(SixDofConstraintAxis::RotationZ, limited(-0.1, 0.1)),
        );
        let mut motor = world.constraint_mut(joint).unwrap();
        motor
            .set_target_angular_velocity_cs(Vec3::new(0.0, spin, 0.0))
            .unwrap();
        motor.set_motor_state(y, MotorState::Velocity);
        let rotation = |world: &PhysicsWorld| {
            world
                .constraint(joint)
                .unwrap()
                .rotation_in_constraint_space()
        };
        let mut overshoot: f32 = 0.0;
        for _ in 0..120 {
            step(&mut world, 1);
            let swing_y = twist_and_swing(rotation(&world)).1;
            overshoot = overshoot.max((swing_y - bound) * spin);
        }
        // Measured peak past the limit: 0.0107 rad for both directions.
        assert!(overshoot < 0.02, "spin {spin}: overshoot {overshoot} rad");
        let (_, swing_y, swing_z) = twist_and_swing(rotation(&world));
        // Measured at tick 120: 0.6000 and -0.2000 rad, swing about z 0.
        assert!((swing_y - bound).abs() < 1e-2, "spin {spin}: {swing_y} rad");
        assert!(swing_z.abs() < 1e-2, "{swing_z} rad");
    }
}

/// `angle` wrapped into `(-π, π]`, as Jolt's `CenterAngleAroundZero` after `fmod`.
fn wrapped(angle: f32) -> f32 {
    let a = angle % (2.0 * PI);
    if a > PI {
        a - 2.0 * PI
    } else if a <= -PI {
        a + 2.0 * PI
    } else {
        a
    }
}

/// A disc of 1 m by 1 m at `x`, hinged about z to a static base far away.
fn hinged_disc(
    world: &mut PhysicsWorld,
    base: BodyId,
    x: Real,
) -> (BodyId, ConstraintId<HingeConstraint>) {
    let disc = add_box(world, Vec3::new(0.5, 0.5, 0.1), RVec3::new(x, 0.0, 0.0));
    let hinge = world
        .create_constraint(
            base,
            disc,
            &HingeConstraintSettings::new(RVec3::new(x, 0.0, 0.0), Z, X),
        )
        .unwrap();
    (disc, hinge)
}

/// Turns hinge `hinge` at `speed` rad/s with a velocity motor.
fn drive(world: &mut PhysicsWorld, hinge: ConstraintId<HingeConstraint>, speed: f32) {
    let mut motor = world.constraint_mut(hinge).unwrap();
    motor.set_target_angular_velocity(speed).unwrap();
    motor.set_motor_state(MotorState::Velocity);
}

struct GearPair {
    world: PhysicsWorld,
    discs: [BodyId; 2],
    hinges: [ConstraintId<HingeConstraint>; 2],
}

fn gear_pair() -> GearPair {
    let mut world = world(Vec3::ZERO, 1);
    let base = add_anchor(&mut world, RVec3::new(0.0, -10.0, 0.0));
    let (disc1, hinge1) = hinged_disc(&mut world, base, 0.0);
    let (disc2, hinge2) = hinged_disc(&mut world, base, 3.0);
    GearPair {
        world,
        discs: [disc1, disc2],
        hinges: [hinge1, hinge2],
    }
}

#[test]
fn gear_turns_the_second_hinge_at_the_ratio() {
    for ratio in [1.0, 2.0, 10.0] {
        let GearPair {
            mut world,
            discs,
            hinges,
        } = gear_pair();
        let gear = world
            .create_constraint(
                discs[0],
                discs[1],
                &GearConstraintSettings::new(Z, Z, ratio),
            )
            .unwrap();
        drive(&mut world, hinges[0], 2.0);
        step(&mut world, 60);
        let spin = world.body(discs[1]).unwrap().angular_velocity().z;
        let wanted = -2.0 / ratio;
        // Measured: -2.0000, -1.0000 and -0.2000 rad/s.
        assert!(
            ((spin - wanted) / wanted).abs() < 0.02,
            "ratio {ratio}: {spin} rad/s"
        );
        assert!(world.constraint(gear).unwrap().total_lambda().is_finite());
    }
}

/// Two discs like [`gear_pair`]'s, of `masses` kg, coupled by a gear of `ratio` about z.
fn weighted_gear_pair(masses: [f32; 2], ratio: f32) -> GearPair {
    let mut world = world(Vec3::ZERO, 1);
    let base = add_anchor(&mut world, RVec3::new(0.0, -10.0, 0.0));
    let shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.1)).unwrap();
    let [(disc1, hinge1), (disc2, hinge2)] = [0, 1].map(|i| {
        let position = RVec3::new(3.0 * i as Real, 0.0, 0.0);
        let disc = world
            .create_body(
                &shape,
                &BodySettings::new_dynamic()
                    .mass(masses[i])
                    .position(position),
            )
            .unwrap();
        let hinge = world
            .create_constraint(base, disc, &HingeConstraintSettings::new(position, Z, X))
            .unwrap();
        (disc, hinge)
    });
    world
        .create_constraint(disc1, disc2, &GearConstraintSettings::new(Z, Z, ratio))
        .unwrap();
    GearPair {
        world,
        discs: [disc1, disc2],
        hinges: [hinge1, hinge2],
    }
}

/// `|ω1 + ratio · ω2|` of the discs of `pair`.
fn gear_error(pair: &GearPair, ratio: f32) -> f32 {
    let spin = |i: usize| pair.world.body(pair.discs[i]).unwrap().angular_velocity().z;
    (spin(0) + ratio * spin(1)).abs()
}

#[test]
fn gear_keeps_its_velocity_relation_at_the_largest_ratio() {
    // Jolt's gear solver keeps up to `1 - 1/ratio` of the error per iteration (see
    // `limits::MAX_GEAR_RATIO`), worst with a heavy body 1. At the bound, every mass
    // distribution brings `ω1 + ratio · ω2` back within 2 % of its initial value within ten
    // steps, whether gear 2 is knocked or gear 1 is driven.
    let ratio = limits::MAX_GEAR_RATIO;
    let first_step_bound = (1.0 - 1.0 / ratio).powi(10);
    for masses in [[1.0e6, 1.0], [1.0, 1.0], [1.0, 1.0e6]] {
        // Gear 2 knocked to 1 rad/s.
        let mut pair = weighted_gear_pair(masses, ratio);
        pair.world
            .body_mut(pair.discs[1])
            .unwrap()
            .set_angular_velocity(Z)
            .unwrap();
        let initial = gear_error(&pair, ratio);
        let mut errors = Vec::new();
        for _ in 0..120 {
            step(&mut pair.world, 1);
            errors.push(gear_error(&pair, ratio) / initial);
        }
        let worst_after_ten = errors[9..].iter().fold(0.0_f32, |m, e| m.max(*e));
        // Measured after one step: 0.348, 0.315 and 0.000 of the initial error (bound 0.349);
        // worst from the tenth step on: 0.0051, 0.0030 and 0.0000.
        assert!(
            errors[0] <= first_step_bound + 1e-3,
            "masses {masses:?}: {} after one step",
            errors[0]
        );
        assert!(
            worst_after_ten < 0.02,
            "masses {masses:?}: {worst_after_ten} from the tenth step on"
        );

        // Gear 1 driven at 2 rad/s.
        let mut pair = weighted_gear_pair(masses, ratio);
        drive(&mut pair.world, pair.hinges[0], 2.0);
        let mut worst_after_ten: f32 = 0.0;
        for tick in 1..=120 {
            step(&mut pair.world, 1);
            let driven = pair.world.body(pair.discs[0]).unwrap().angular_velocity().z;
            if tick >= 10 {
                worst_after_ten = worst_after_ten.max(gear_error(&pair, ratio) / driven.abs());
            }
        }
        // Measured worst from the tenth step on: 0.0051, 0.0053 and 0.0161 of gear 1's rate.
        assert!(
            worst_after_ten < 0.02,
            "masses {masses:?}, driven: {worst_after_ten} from the tenth step on"
        );
    }
}

#[test]
fn gear_references_correct_drift() {
    let GearPair {
        mut world,
        discs,
        hinges,
    } = gear_pair();
    // Gear 2 starts turned by 0.3 rad, so the gear starts 0.6 rad out of mesh; only the hinge
    // references let Jolt correct that.
    world
        .body_mut(discs[1])
        .unwrap()
        .set_rotation(quat_about(Z, 0.3), Activation::Activate)
        .unwrap();
    world
        .create_constraint(
            discs[0],
            discs[1],
            &GearConstraintSettings::new(Z, Z, 2.0).constraints(hinges[0], hinges[1]),
        )
        .unwrap();
    drive(&mut world, hinges[0], 3.0);
    let angle =
        |world: &PhysicsWorld, i: usize| world.constraint(hinges[i]).unwrap().current_angle();
    let mesh_error = |world: &PhysicsWorld| wrapped(angle(world, 0) + 2.0 * angle(world, 1));
    let initial = mesh_error(&world);
    step(&mut world, 60);
    let mut worst: f32 = 0.0;
    let mut wraps = [0, 0];
    let mut last = [angle(&world, 0), angle(&world, 1)];
    for _ in 0..300 {
        step(&mut world, 1);
        let now = [angle(&world, 0), angle(&world, 1)];
        for i in 0..2 {
            if (now[i] - last[i]).abs() > PI {
                wraps[i] += 1;
            }
        }
        last = now;
        worst = worst.max(mesh_error(&world).abs());
    }
    // Both hinges wrapped at ±π after the first second; measured worst after it: 0.00002 rad,
    // and 0.6 rad without the references.
    assert!((initial - 0.6).abs() < 1e-3, "{initial}");
    assert!(wraps[0] >= 2 && wraps[1] >= 1, "{wraps:?}");
    assert!(worst < 1e-2, "{worst} rad");
}

#[test]
fn coupling_settings_name_their_references() {
    let GearPair { hinges, .. } = gear_pair();
    let RackScene { hinge, slider, .. } = rack_scene();

    let gear = GearConstraintSettings::new(Z, Z, 2.0);
    assert_eq!(gear.gear1_constraint(), None);
    assert_eq!(gear.gear2_constraint(), None);
    let gear = gear.constraints(hinges[0], hinges[1]);
    assert_eq!(gear.gear1_constraint(), Some(hinges[0]));
    assert_eq!(gear.gear2_constraint(), Some(hinges[1]));

    let rack = RackAndPinionConstraintSettings::new(Z, X, 4.0);
    assert_eq!(rack.pinion_constraint(), None);
    assert_eq!(rack.rack_constraint(), None);
    let rack = rack.constraints(hinge, slider);
    assert_eq!(rack.pinion_constraint(), Some(hinge));
    assert_eq!(rack.rack_constraint(), Some(slider));
}

#[test]
fn created_couplings_name_their_references() {
    let GearPair {
        mut world,
        discs,
        hinges,
    } = gear_pair();
    let bare = world
        .create_constraint(discs[0], discs[1], &GearConstraintSettings::new(Z, Z, 2.0))
        .unwrap();
    let referencing = world
        .create_constraint(
            discs[0],
            discs[1],
            &GearConstraintSettings::new(Z, Z, 2.0).constraints(hinges[0], hinges[1]),
        )
        .unwrap();
    let bare = world.constraint(bare).unwrap();
    assert_eq!(bare.gear1_constraint(), None);
    assert_eq!(bare.gear2_constraint(), None);
    let referencing = world.constraint(referencing).unwrap();
    assert_eq!(referencing.gear1_constraint(), Some(hinges[0]));
    assert_eq!(referencing.gear2_constraint(), Some(hinges[1]));

    let RackScene {
        mut world,
        pinion,
        rack,
        hinge,
        slider,
    } = rack_scene();
    let bare = world
        .create_constraint(
            pinion,
            rack,
            &RackAndPinionConstraintSettings::new(Z, X, 4.0),
        )
        .unwrap();
    let referencing = world
        .create_constraint(
            pinion,
            rack,
            &RackAndPinionConstraintSettings::new(Z, X, 4.0).constraints(hinge, slider),
        )
        .unwrap();
    let bare = world.constraint(bare).unwrap();
    assert_eq!(bare.pinion_constraint(), None);
    assert_eq!(bare.rack_constraint(), None);
    let coupling = world.constraint(referencing).unwrap();
    assert_eq!(coupling.pinion_constraint(), Some(hinge));
    assert_eq!(coupling.rack_constraint(), Some(slider));
    // The getters name the constraints the removal guard protects.
    let references: [AnyConstraintId; 2] = [
        coupling.pinion_constraint().unwrap().into(),
        coupling.rack_constraint().unwrap().into(),
    ];
    for reference in references {
        assert_eq!(
            world.remove_constraint(reference),
            Err(ConstraintError::UsedByConstraint(referencing.into()))
        );
    }
}

struct RackScene {
    world: PhysicsWorld,
    pinion: BodyId,
    rack: BodyId,
    hinge: ConstraintId<HingeConstraint>,
    slider: ConstraintId<SliderConstraint>,
}

fn rack_scene() -> RackScene {
    let mut world = world(Vec3::ZERO, 1);
    let base = add_anchor(&mut world, RVec3::new(0.0, -10.0, 0.0));
    let (pinion, hinge) = hinged_disc(&mut world, base, 0.0);
    let rack = add_box(
        &mut world,
        Vec3::new(1.0, 0.2, 0.2),
        RVec3::new(0.0, -2.0, 0.0),
    );
    let slider = world
        .create_constraint(
            base,
            rack,
            &SliderConstraintSettings::new(RVec3::new(0.0, -2.0, 0.0), X, Y),
        )
        .unwrap();
    RackScene {
        world,
        pinion,
        rack,
        hinge,
        slider,
    }
}

#[test]
fn rack_moves_at_the_pinion_rate() {
    let RackScene {
        mut world,
        pinion,
        rack,
        hinge,
        ..
    } = rack_scene();
    let coupling = world
        .create_constraint(
            pinion,
            rack,
            &RackAndPinionConstraintSettings::new(Z, X, 4.0),
        )
        .unwrap();
    drive(&mut world, hinge, 2.0);
    step(&mut world, 60);
    // Jolt's rack and pinion keeps rotation = ratio · translation: 2 rad/s / 4 rad/m.
    let speed = world.body(rack).unwrap().linear_velocity().x;
    // Measured: 0.5000 m/s.
    assert!(((speed - 0.5) / 0.5).abs() < 0.02, "{speed} m/s");
    assert!(world
        .constraint(coupling)
        .unwrap()
        .total_lambda()
        .is_finite());
}

#[test]
fn rack_references_correct_drift() {
    let RackScene {
        mut world,
        pinion,
        rack,
        hinge,
        slider,
    } = rack_scene();
    // The pinion starts turned by 0.5 rad, out of mesh; only the references let Jolt correct
    // that.
    world
        .body_mut(pinion)
        .unwrap()
        .set_rotation(quat_about(Z, 0.5), Activation::Activate)
        .unwrap();
    world
        .create_constraint(
            pinion,
            rack,
            &RackAndPinionConstraintSettings::new(Z, X, 4.0).constraints(hinge, slider),
        )
        .unwrap();
    drive(&mut world, hinge, 3.0);
    let mesh_error = |world: &PhysicsWorld| {
        let rotation = world.constraint(hinge).unwrap().current_angle();
        let translation = world.constraint(slider).unwrap().current_position();
        wrapped(rotation - 4.0 * translation)
    };
    let initial = mesh_error(&world);
    step(&mut world, 60);
    let mut worst: f32 = 0.0;
    for _ in 0..300 {
        step(&mut world, 1);
        worst = worst.max(mesh_error(&world).abs());
    }
    // The pinion turned about 2.9 times; measured worst after the first second: 0.000006 rad,
    // and 0.5 rad without the references.
    assert!((initial - 0.5).abs() < 1e-3, "{initial}");
    assert!(world.constraint(slider).unwrap().current_position() > 4.0);
    assert!(worst < 1e-2, "{worst} rad");
}

#[test]
fn a_referenced_hinge_cannot_be_removed() {
    let GearPair {
        mut world,
        discs,
        hinges,
    } = gear_pair();
    let gear = world
        .create_constraint(
            discs[0],
            discs[1],
            &GearConstraintSettings::new(Z, Z, 2.0).constraints(hinges[0], hinges[1]),
        )
        .unwrap();
    for hinge in hinges {
        assert_eq!(
            world.remove_constraint(hinge),
            Err(ConstraintError::UsedByConstraint(gear.into()))
        );
    }
    step(&mut world, 10);
    world.remove_constraint(gear).unwrap();
    for hinge in hinges {
        world.remove_constraint(hinge).unwrap();
    }
    step(&mut world, 10);
}

#[test]
fn unrelated_or_reversed_references_are_rejected() {
    let GearPair {
        mut world,
        discs,
        hinges,
    } = gear_pair();
    let base = world.constraint(hinges[0]).unwrap().bodies()[0];
    // A hinge of another disc.
    let (_, unrelated) = hinged_disc(&mut world, base, 6.0);
    // A hinge with disc 1 as its body 1.
    let reversed = world
        .create_constraint(
            discs[0],
            base,
            &HingeConstraintSettings::new(RVec3::ZERO, Z, X),
        )
        .unwrap();
    // A hinge of disc 1 about the opposite axis.
    let opposite = world
        .create_constraint(
            base,
            discs[0],
            &HingeConstraintSettings::new(RVec3::ZERO, Vec3::new(0.0, 0.0, -1.0), X),
        )
        .unwrap();
    let count = world.constraint_count();
    for first in [unrelated, reversed, opposite] {
        let result = world.create_constraint(
            discs[0],
            discs[1],
            &GearConstraintSettings::new(Z, Z, 2.0).constraints(first, hinges[1]),
        );
        assert!(
            matches!(result, Err(ConstraintError::InvalidValue(_))),
            "{result:?}"
        );
        assert_eq!(world.constraint_count(), count);
    }
    // The second hinge must hold body 2.
    let result = world.create_constraint(
        discs[0],
        discs[1],
        &GearConstraintSettings::new(Z, Z, 2.0).constraints(hinges[0], hinges[0]),
    );
    assert!(matches!(result, Err(ConstraintError::InvalidValue(_))));
    world
        .create_constraint(
            discs[0],
            discs[1],
            &GearConstraintSettings::new(Z, Z, 2.0).constraints(hinges[0], hinges[1]),
        )
        .unwrap();
    // A removed reference is not found.
    world.remove_constraint(unrelated).unwrap();
    assert_eq!(
        world
            .create_constraint(
                discs[0],
                discs[1],
                &GearConstraintSettings::new(Z, Z, 2.0).constraints(unrelated, hinges[1]),
            )
            .err(),
        Some(ConstraintError::NotFound(unrelated.into()))
    );
}

/// Two boxes of `masses` hanging from fixed points 3 m above them, at x = -1 and x = 1, joined
/// by a pulley of `ratio`.
fn hanging_pair(
    world: &mut PhysicsWorld,
    masses: [f32; 2],
    ratio: f32,
) -> ([BodyId; 2], ConstraintId<PulleyConstraint>) {
    let shape = Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap();
    let xs = [-1.0, 1.0];
    let bodies = [0, 1].map(|i| {
        world
            .create_body(
                &shape,
                &BodySettings::new_dynamic()
                    .mass(masses[i])
                    .position(RVec3::new(xs[i], 0.0, 0.0)),
            )
            .unwrap()
    });
    let pulley = world
        .create_constraint(
            bodies[0],
            bodies[1],
            &PulleyConstraintSettings::new(
                RVec3::new(-1.0, 0.0, 0.0),
                RVec3::new(-1.0, 3.0, 0.0),
                RVec3::new(1.0, 0.0, 0.0),
                RVec3::new(1.0, 3.0, 0.0),
            )
            .ratio(ratio),
        )
        .unwrap();
    (bodies, pulley)
}

#[test]
fn pulley_lifts_one_body_as_the_other_falls() {
    let mut world = world(GRAVITY, 1);
    let (bodies, pulley) = hanging_pair(&mut world, [2.0, 1.0], 1.0);
    let reading = world.constraint(pulley).unwrap();
    assert_eq!((reading.min_length(), reading.max_length()), (0.0, 6.0));
    let length = |world: &PhysicsWorld, i: usize, x: Real| {
        distance(
            world.body(bodies[i]).unwrap().position(),
            RVec3::new(x, 3.0, 0.0),
        )
    };
    let mut worst: f64 = 0.0;
    for _ in 0..60 {
        step(&mut world, 1);
        worst = worst.max(length(&world, 0, -1.0) + length(&world, 1, 1.0) - 6.0);
    }
    let y = |i: usize| world.body(bodies[i]).unwrap().position().y;
    // The heavier box went down 1.63 m and pulled the lighter one up as far; measured: the
    // rope stretched by at most 0.0000002 m.
    assert!(y(0) < -0.5 && y(1) > 0.5, "{} {}", y(0), y(1));
    assert!(worst < 1e-3, "{worst} m");
    assert!(world
        .constraint(pulley)
        .unwrap()
        .total_lambda_position()
        .is_finite());
}

#[test]
fn pulley_with_ratio_two_moves_half_as_far() {
    let mut world = world(GRAVITY, 1);
    let (bodies, pulley) = hanging_pair(&mut world, [3.0, 1.0], 2.0);
    assert_eq!(world.constraint(pulley).unwrap().ratio(), 2.0);
    step(&mut world, 60);
    let y = |i: usize| wide(world.body(bodies[i]).unwrap().position().y);
    // length1 + 2 · length2 stays 9 m: body 2 rises half as far as body 1 falls. Measured:
    // the ratio of the moves is 0.5000.
    assert!(y(0) < -0.5, "{}", y(0));
    let ratio = y(1) / -y(0);
    assert!((ratio - 0.5).abs() < 1e-2, "{ratio}");
}

/// A hinged door swinging down and, 5 m behind it, a hanging pulley pair, for the rebase
/// tests.
fn swinging_scene() -> (PhysicsWorld, Vec<BodyId>, ConstraintId<PulleyConstraint>) {
    let mut world = world(GRAVITY, 1);
    let post = add_anchor(&mut world, RVec3::new(-2.0, 2.0, 5.0));
    let panel = add_box(
        &mut world,
        Vec3::new(0.5, 0.05, 0.5),
        RVec3::new(0.6, 2.0, 5.0),
    );
    world
        .create_constraint(
            post,
            panel,
            &HingeConstraintSettings::new(RVec3::new(0.1, 2.0, 5.0), Z, X),
        )
        .unwrap();
    let (pair, pulley) = hanging_pair(&mut world, [2.0, 1.0], 1.0);
    (world, vec![post, panel, pair[0], pair[1]], pulley)
}

/// `p` mapped back from the frame `rotation * p + translation`.
fn mapped_back(p: RVec3, rotation: Quat, translation: RVec3) -> RVec3 {
    let shifted = Vec3::new(
        wide(p.x - translation.x) as f32,
        wide(p.y - translation.y) as f32,
        wide(p.z - translation.z) as f32,
    );
    let back = common::ragdoll::rotate(common::ragdoll::conj(rotation), shifted);
    RVec3::new(back.x as Real, back.y as Real, back.z as Real)
}

/// The largest distance over 60 ticks between the bodies of a swinging scene rebased at tick
/// 30 by `rotation` and `translation`, mapped back, and those of an un-rebased twin.
fn rebase_drift(rotation: Quat, translation: RVec3) -> f64 {
    let (mut rebased, bodies, pulley) = swinging_scene();
    let (mut twin, twin_bodies, _) = swinging_scene();
    step(&mut rebased, 30);
    step(&mut twin, 30);
    let length = rebased.constraint(pulley).unwrap().current_length();
    rebased.rebase(&bodies, rotation, translation).unwrap();
    let reading = rebased.constraint(pulley).unwrap();
    assert!((reading.current_length() - length).abs() < 1e-4);
    assert_eq!((reading.min_length(), reading.max_length()), (0.0, 6.0));
    let mut worst: f64 = 0.0;
    for _ in 0..60 {
        step(&mut rebased, 1);
        step(&mut twin, 1);
        for (&id, &twin_id) in bodies.iter().zip(&twin_bodies) {
            let back = mapped_back(rebased.body(id).unwrap().position(), rotation, translation);
            worst = worst.max(distance(back, twin.body(twin_id).unwrap().position()));
        }
    }
    worst
}

#[test]
fn rebase_moves_constraints_rigidly() {
    let translation = RVec3::new(30.0, -12.0, 7.0);
    let shifted = rebase_drift(Quat::IDENTITY, translation);
    let turned = rebase_drift(quat_about(Vec3::new(0.0, 0.6, 0.8), 0.4), translation);
    // Measured: 0.000007 m after a translation and 0.0023 m after a rotation. In the turned
    // frame Jolt's `f32` rope length rounds differently, and once it comes out just under the
    // pulley's maximum of 6 m (5.999999 m) the rope is slack for that step: no impulse, the
    // pair stretches the rope by 0.0044 m and the next steps pull it back, a drift that peaks
    // at 0.0023 m and decays to below 0.00002 m within 15 steps. In single precision this
    // happens in the first step after the rebase, in double precision in the 26th. The door
    // alone stays within 0.00001 m.
    assert!(shifted < 1e-4, "{shifted} m");
    assert!(turned < 5e-3, "{turned} m");
}

#[test]
fn rebase_with_a_pulley_is_atomic() {
    // The fixed points cannot leave a finite frame through a valid rebase, so a refused rebase
    // is made with an incomplete body list: nothing, the pulley included, changes.
    let (mut world, bodies, pulley) = swinging_scene();
    step(&mut world, 20);
    let snapshot = |world: &PhysicsWorld| {
        let reading = world.constraint(pulley).unwrap();
        let mut bits: Vec<u64> = [
            reading.current_length(),
            reading.min_length(),
            reading.max_length(),
            reading.total_lambda_position(),
        ]
        .iter()
        .map(|v| u64::from(v.to_bits()))
        .collect();
        for &id in &bodies {
            let p = world.body(id).unwrap().position();
            bits.extend([p.x, p.y, p.z].map(|c| wide(c).to_bits()));
        }
        bits
    };
    let before = snapshot(&world);
    let rotation = quat_about(Z, 0.3);
    assert!(matches!(
        world.rebase(&bodies[..3], rotation, RVec3::new(1.0, 2.0, 3.0)),
        Err(BodyError::InvalidValue(_))
    ));
    assert_eq!(snapshot(&world), before);
    world
        .rebase(&bodies, rotation, RVec3::new(1.0, 2.0, 3.0))
        .unwrap();
    assert_ne!(snapshot(&world), before);
}

#[test]
fn rebase_with_pulleys_is_repeatable() {
    let run = || {
        let (mut world, bodies, _) = swinging_scene();
        let mut digest = Vec::new();
        for tick in 0..90 {
            if tick % 30 == 29 {
                world
                    .rebase(&bodies, quat_about(Y, 0.2), RVec3::new(5.0, 0.0, -3.0))
                    .unwrap();
            }
            step(&mut world, 1);
            for &id in &bodies {
                record_body(&world, id, &mut digest);
            }
        }
        digest
    };
    assert_eq!(run(), run());
}

fn point(position: [f32; 3], tangent: [f32; 3]) -> HermitePathPoint {
    HermitePathPoint {
        position: Vec3::from(position),
        tangent: Vec3::from(tangent),
    }
}

/// The position on `path` at `fraction`, with Jolt's Hermite formula.
fn path_point(path: &HermitePath, fraction: f32) -> Vec3 {
    let points = path.points();
    let last = if path.is_looping() {
        points.len()
    } else {
        points.len() - 1
    };
    let index = (fraction.floor() as usize).min(last - 1);
    let t = fraction - index as f32;
    let (p1, p2) = (points[index], points[(index + 1) % points.len()]);
    let (t2, t3) = (t * t, t * t * t);
    let h = [
        2.0 * t3 - 3.0 * t2 + 1.0,
        t3 - 2.0 * t2 + t,
        -2.0 * t3 + 3.0 * t2,
        t3 - t2,
    ];
    let mix = |a: f32, b: f32, c: f32, d: f32| h[0] * a + h[1] * b + h[2] * c + h[3] * d;
    Vec3::new(
        mix(p1.position.x, p1.tangent.x, p2.position.x, p2.tangent.x),
        mix(p1.position.y, p1.tangent.y, p2.position.y, p2.tangent.y),
        mix(p1.position.z, p1.tangent.z, p2.position.z, p2.tangent.z),
    )
}

/// A descending S curve in the XY plane of its path space.
fn s_curve() -> HermitePath {
    HermitePath::new(
        Z,
        vec![
            point([0.0, 0.0, 0.0], [1.0, -0.3, 0.0]),
            point([1.0, -0.3, 0.0], [1.0, -0.5, 0.0]),
            point([2.0, -0.9, 0.0], [1.0, -0.5, 0.0]),
            point([3.0, -1.2, 0.0], [1.0, -0.3, 0.0]),
        ],
        false,
    )
    .unwrap()
}

/// A static anchor at `origin` turned by `turn`, and a small box at the start of a path that
/// begins 0.5 m along the anchor's x axis.
fn path_scene(
    world: &mut PhysicsWorld,
    origin: RVec3,
    turn: Quat,
    path: &HermitePath,
) -> (BodyId, ConstraintId<PathConstraint>) {
    let shape = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    let anchor = world
        .create_body(
            &shape,
            &BodySettings::new_static().position(origin).rotation(turn),
        )
        .unwrap();
    let offset = Vec3::new(0.5, 0.0, 0.0);
    let start = attached(origin, turn, offset);
    let body = world
        .create_body(&shape, &BodySettings::new_dynamic().position(start))
        .unwrap();
    let constraint = world
        .create_constraint(
            anchor,
            body,
            &PathConstraintSettings::new(path.clone()).path_position(offset),
        )
        .unwrap();
    (body, constraint)
}

/// `p` in the path space of a path at `offset` in the frame of a body at `origin` turned by
/// `turn`.
fn in_path_space(p: RVec3, origin: RVec3, turn: Quat, offset: Vec3) -> Vec3 {
    let local = Vec3::new(
        wide(p.x - origin.x) as f32,
        wide(p.y - origin.y) as f32,
        wide(p.z - origin.z) as f32,
    );
    let local = common::ragdoll::rotate(common::ragdoll::conj(turn), local);
    Vec3::new(local.x - offset.x, local.y - offset.y, local.z - offset.z)
}

#[test]
fn path_keeps_a_body_on_its_curve() {
    let mut world = world(GRAVITY, 1);
    let origin = RVec3::new(5.0, 3.0, -2.0);
    // Turned about the vertical, so gravity stays in the path's plane and pulls the box along.
    let turn = quat_about(Y, 0.5);
    let path = s_curve();
    let (body, constraint) = path_scene(&mut world, origin, turn, &path);
    let mut worst: f32 = 0.0;
    let mut last_fraction = 0.0;
    for _ in 0..70 {
        step(&mut world, 1);
        let position = world.body(body).unwrap().position();
        let local = in_path_space(position, origin, turn, Vec3::new(0.5, 0.0, 0.0));
        let reading = world.constraint(constraint).unwrap();
        let fraction = reading.closest_fraction(local, last_fraction).unwrap();
        let on_curve = path_point(&path, fraction);
        let off = Vec3::new(
            local.x - on_curve.x,
            local.y - on_curve.y,
            local.z - on_curve.z,
        );
        worst = worst.max(off.x.hypot(off.y).hypot(off.z));
        let sliding = reading.path_fraction();
        assert!(
            sliding >= last_fraction - 1e-4,
            "{sliding} after {last_fraction}"
        );
        last_fraction = sliding;
    }
    // While the box slides, before it reaches the end of the path; measured: at most 0.0007 m
    // off the curve, sliding to fraction 1.69.
    assert!(last_fraction > 1.0, "{last_fraction}");
    assert!(worst < 1e-2, "{worst} m");
}

#[test]
fn path_motor_drives_to_a_target_fraction() {
    let mut world = world(Vec3::ZERO, 1);
    let (_, constraint) = path_scene(&mut world, RVec3::ZERO, Quat::IDENTITY, &s_curve());
    let mut motor = world.constraint_mut(constraint).unwrap();
    motor.set_target_path_fraction(1.5).unwrap();
    motor.set_motor_state(MotorState::Position);
    step(&mut world, 180);
    let reading = world.constraint(constraint).unwrap();
    let fraction = reading.path_fraction();
    // Measured: 1.49997.
    assert!((fraction - 1.5).abs() < 1e-2, "{fraction}");
    assert_eq!(reading.target_path_fraction(), 1.5);
    assert_eq!(reading.max_fraction(), 3.0);
}

#[test]
fn path_reads_back_its_motor_looping_and_rotation_impulses() {
    let motor = MotorSettings::default()
        .spring(SpringSettings::StiffnessAndDamping {
            stiffness: 50.0,
            damping: 5.0,
        })
        .force_limits(-30.0, 40.0);
    let shape = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    // Body 2 knocked into a spin about x, then about z: a joint that only lets it turn about
    // the normal (z) resists the first through its hinge part, a fully constrained one the
    // second through its rotation part.
    for (rotation, spin) in [
        (PathRotationConstraint::ConstrainAroundNormal, X),
        (PathRotationConstraint::FullyConstrained, Z),
    ] {
        let mut world = world(Vec3::ZERO, 1);
        let anchor = world
            .create_body(&shape, &BodySettings::new_static())
            .unwrap();
        let body = world
            .create_body(
                &shape,
                &BodySettings::new_dynamic().position(RVec3::new(0.5, 0.0, 0.0)),
            )
            .unwrap();
        let path = world
            .create_constraint(
                anchor,
                body,
                &PathConstraintSettings::new(s_curve())
                    .path_position(Vec3::new(0.5, 0.0, 0.0))
                    .rotation_constraint(rotation)
                    .position_motor(motor),
            )
            .unwrap();
        world
            .body_mut(body)
            .unwrap()
            .set_angular_velocity(spin)
            .unwrap();
        step(&mut world, 1);
        let reading = world.constraint(path).unwrap();
        assert_eq!(reading.position_motor_settings(), motor);
        assert!(!reading.is_looping());
        let hinge = reading.total_lambda_rotation_hinge();
        let hinge = hinge[0].hypot(hinge[1]);
        let lambda = reading.total_lambda_rotation();
        let full = length(lambda);
        // Measured: hinge 0.0533 N·m·s, rotation 0; then hinge 0, rotation 0.0533 N·m·s about z.
        if rotation == PathRotationConstraint::FullyConstrained {
            assert_eq!(hinge, 0.0);
            assert!(full > 1e-3, "{full}");
            assert!(lambda.z.abs() > 0.99 * full, "{lambda:?}");
        } else {
            assert!(hinge > 1e-3, "{hinge}");
            assert_eq!(full, 0.0);
        }
        let other = MotorSettings::default().force_limits(-1.0, 1.0);
        world
            .constraint_mut(path)
            .unwrap()
            .set_position_motor_settings(other)
            .unwrap();
        assert_eq!(
            world.constraint(path).unwrap().position_motor_settings(),
            other
        );
    }
    let mut world = world(Vec3::ZERO, 1);
    let (_, looping) = path_scene(&mut world, RVec3::ZERO, Quat::IDENTITY, &circle());
    assert!(world.constraint(looping).unwrap().is_looping());
}

/// A loop through four points on a circle of radius 1, tangents of a quarter arc's length.
fn circle() -> HermitePath {
    let arc = std::f32::consts::FRAC_PI_2;
    HermitePath::new(
        Z,
        vec![
            point([1.0, 0.0, 0.0], [0.0, arc, 0.0]),
            point([0.0, 1.0, 0.0], [-arc, 0.0, 0.0]),
            point([-1.0, 0.0, 0.0], [0.0, -arc, 0.0]),
            point([0.0, -1.0, 0.0], [arc, 0.0, 0.0]),
        ],
        true,
    )
    .unwrap()
}

#[test]
fn looping_path_wraps() {
    let mut world = world(Vec3::ZERO, 1);
    let path = circle();
    assert_eq!(path.max_fraction(), 4.0);
    let (_, constraint) = path_scene(&mut world, RVec3::ZERO, Quat::IDENTITY, &path);
    let mut motor = world.constraint_mut(constraint).unwrap();
    motor.set_target_velocity(3.0).unwrap();
    motor.set_motor_state(MotorState::Velocity);
    let mut wraps = 0;
    let mut last = world.constraint(constraint).unwrap().path_fraction();
    for _ in 0..240 {
        step(&mut world, 1);
        let fraction = world.constraint(constraint).unwrap().path_fraction();
        if fraction < last - 2.0 {
            wraps += 1;
        }
        last = fraction;
    }
    // About 12 m along a loop of about 6.3 m: measured 1 wrap.
    assert!(wraps >= 1, "{wraps} wraps, at {last}");
    // A looping path's target fraction is within its range too.
    let mut motor = world.constraint_mut(constraint).unwrap();
    assert!(motor.set_target_path_fraction(4.0).is_ok());
    assert!(matches!(
        motor.set_target_path_fraction(4.0_f32.next_up()),
        Err(ConstraintError::InvalidValue(_))
    ));
}

#[test]
fn invalid_paths_are_rejected() {
    let invalid = |normal: Vec3, points: Vec<HermitePathPoint>, looping: bool| {
        matches!(
            HermitePath::new(normal, points, looping),
            Err(ConstraintError::InvalidValue(_))
        )
    };
    let straight = |n: usize| -> Vec<HermitePathPoint> {
        (0..n)
            .map(|i| point([i as f32 * 0.01, 0.0, 0.0], [0.01, 0.0, 0.0]))
            .collect()
    };
    assert!(HermitePath::new(Z, straight(2), false).is_ok());
    assert!(HermitePath::new(Z, straight(HermitePath::MAX_POINTS), false).is_ok());
    assert!(invalid(Z, straight(1), false));
    assert!(invalid(Z, straight(HermitePath::MAX_POINTS + 1), false));
    // Zero chord.
    let mut repeated = straight(3);
    repeated[1].position = repeated[0].position;
    assert!(invalid(Z, repeated, false));
    // A loop whose ends coincide.
    let square = vec![
        point([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        point([1.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        point([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
    ];
    assert!(invalid(Z, square, true));
    // A tangent against the chord.
    assert!(invalid(
        Z,
        vec![
            point([0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            point([1.0, 0.0, 0.0], [1.0, 0.0, 0.0])
        ],
        false
    ));
    // Tangents along the chord but so long that the curve runs backwards in the middle:
    // f(0.5) = -0.5 for a chord of 1 and tangent components of 4.
    assert!(invalid(
        Z,
        vec![
            point([0.0, 0.0, 0.0], [4.0, 0.0, 0.0]),
            point([1.0, 0.0, 0.0], [4.0, 0.0, 0.0])
        ],
        false
    ));
    // A tangent out of the plane.
    assert!(invalid(
        Z,
        vec![
            point([0.0, 0.0, 0.0], [1.0, 0.0, 0.6]),
            point([1.0, 0.0, 0.0], [1.0, 0.0, 0.0])
        ],
        false
    ));
    assert!(invalid(Vec3::new(0.0, 0.0, 1.1), straight(2), false));
    assert!(invalid(
        Z,
        vec![
            point([f32::NAN, 0.0, 0.0], [1.0, 0.0, 0.0]),
            point([1.0, 0.0, 0.0], [1.0, 0.0, 0.0])
        ],
        false
    ));
    let extent = limits::MAX_SHAPE_EXTENT;
    assert!(invalid(
        Z,
        vec![
            point([0.0, 0.0, 0.0], [extent.next_up(), 0.0, 0.0]),
            point([1.0, 0.0, 0.0], [1.0, 0.0, 0.0])
        ],
        false
    ));
}

#[test]
fn path_validation_accepts_its_boundary() {
    // Tangents (1, y, 0) along a chord of 1 along x, normal y: the derivative along the chord is
    // 1 everywhere and along the normal at most 2y, so y = 0.25 is the bound.
    let segment = |y: f32| {
        HermitePath::new(
            Y,
            vec![
                point([0.0, 0.0, 0.0], [1.0, y, 0.0]),
                point([1.0, 0.0, 0.0], [1.0, y, 0.0]),
            ],
            false,
        )
    };
    assert!(matches!(
        segment(0.25_f32.next_up()),
        Err(ConstraintError::InvalidValue(_))
    ));
    let path = segment(0.25).unwrap();
    // The accepted segment steps end to end with every rotation constraint.
    for rotation in [
        PathRotationConstraint::Free,
        PathRotationConstraint::ConstrainAroundTangent,
        PathRotationConstraint::ConstrainAroundNormal,
        PathRotationConstraint::ConstrainAroundBinormal,
        PathRotationConstraint::ConstrainToPath,
        PathRotationConstraint::FullyConstrained,
    ] {
        let mut world = world(Vec3::ZERO, 1);
        let shape = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
        let anchor = world
            .create_body(
                &shape,
                &BodySettings::new_static().position(RVec3::new(0.0, 0.0, -2.0)),
            )
            .unwrap();
        let body = add_box(&mut world, Vec3::new(0.1, 0.1, 0.1), RVec3::ZERO);
        let constraint = world
            .create_constraint(
                anchor,
                body,
                &PathConstraintSettings::new(path.clone())
                    .path_position(Vec3::new(0.0, 0.0, 2.0))
                    .rotation_constraint(rotation),
            )
            .unwrap();
        let mut motor = world.constraint_mut(constraint).unwrap();
        motor.set_target_velocity(2.0).unwrap();
        motor.set_motor_state(MotorState::Velocity);
        step(&mut world, 60);
        let fraction = world.constraint(constraint).unwrap().path_fraction();
        assert!(fraction > 0.99, "{rotation:?}: {fraction}");
    }
}
