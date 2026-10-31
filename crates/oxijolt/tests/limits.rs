//! The magnitude policy of `oxijolt::limits`: every bounded input is accepted at its bound
//! and rejected just beyond it without changing the world, and scenes with every input at its
//! bound step with finite state. In a native build with the `asserts` feature these scenes also
//! show that no Jolt assertion fires for them.

mod common;

use std::f32::consts::PI;

use common::controls::largest_accepted;
use common::math::{bits, f3, v3, wide};
use common::vehicle::*;
use common::*;
use oxijolt::*;

/// A world without gravity.
fn empty_world() -> PhysicsWorld {
    world(Vec3::ZERO, 1)
}

fn sphere() -> Shape {
    Shape::new_sphere(0.5).unwrap()
}

/// The axis-aligned vectors with `value` on one axis, in both directions.
fn on_axes(value: f32) -> Vec<Vec3> {
    let mut vectors = Vec::new();
    for axis in 0..3 {
        for sign in [1.0, -1.0] {
            let mut v = [0.0; 3];
            v[axis] = sign * value;
            vectors.push(Vec3::from(v));
        }
    }
    vectors
}

/// The axis-aligned positions with `value` on one axis, in both directions.
fn real_on_axes(value: Real) -> Vec<RVec3> {
    let mut vectors = Vec::new();
    for axis in 0..3 {
        for sign in [1.0, -1.0] {
            let mut v = [0.0; 3];
            v[axis] = sign * value;
            vectors.push(RVec3::from(v));
        }
    }
    vectors
}

const NON_FINITE: [f32; 3] = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY];

fn real_bits(v: RVec3) -> [u64; 3] {
    <[Real; 3]>::from(v).map(|c| wide(c).to_bits())
}

/// A body's pose and velocities as bits.
fn body_bits(world: &PhysicsWorld, id: BodyId) -> ([u64; 3], [u32; 4], [u32; 3], [u32; 3]) {
    let body = world.body(id).unwrap();
    (
        real_bits(body.position()),
        <[f32; 4]>::from(body.rotation()).map(f32::to_bits),
        bits(body.linear_velocity()),
        bits(body.angular_velocity()),
    )
}

fn assert_body_finite(world: &PhysicsWorld, id: BodyId, what: &str) {
    let body = world.body(id).unwrap();
    let state = [
        v3(body.position()),
        f3(body.linear_velocity()),
        f3(body.angular_velocity()),
    ];
    assert!(
        state.iter().flatten().all(|value| value.is_finite()),
        "{what}: {state:?}"
    );
}

fn body_invalid(result: Result<impl Sized, BodyError>) -> bool {
    matches!(result, Err(BodyError::InvalidValue(_)))
}

fn query_invalid(result: Result<impl Sized, QueryError>) -> bool {
    matches!(result, Err(QueryError::InvalidValue(_)))
}

fn character_invalid(result: Result<impl Sized, CharacterError>) -> bool {
    matches!(result, Err(CharacterError::InvalidValue(_)))
}

#[test]
fn world_gravity_is_bounded_by_max_acceleration() {
    let bound = limits::MAX_ACCELERATION;
    for gravity in on_axes(bound) {
        assert!(PhysicsWorld::new(WorldSettings::default().gravity(gravity)).is_ok());
    }
    let mut world = empty_world();
    for gravity in on_axes(bound) {
        world.set_gravity(gravity).unwrap();
    }
    let before = bits(world.gravity());
    let beyond = on_axes(bound.next_up()).into_iter().chain(
        NON_FINITE
            .into_iter()
            .map(|value| Vec3::new(0.0, value, 0.0)),
    );
    for gravity in beyond {
        assert!(matches!(
            PhysicsWorld::new(WorldSettings::default().gravity(gravity)),
            Err(WorldError::InvalidValue(_))
        ));
        assert!(matches!(
            world.set_gravity(gravity),
            Err(WorldError::InvalidValue(_))
        ));
        assert_eq!(bits(world.gravity()), before);
    }
}

#[test]
fn rebase_translation_is_bounded_by_twice_the_frame() {
    let mut world = empty_world();
    let id = world
        .create_body(&sphere(), &BodySettings::new_dynamic())
        .unwrap();
    let span = 2.0 * limits::MAX_POSITION;
    // The translation is bounded; the resulting position is only checked to be finite.
    world
        .rebase(&[id], Quat::IDENTITY, RVec3::new(span, 0.0, 0.0))
        .unwrap();
    let before = body_bits(&world, id);
    for translation in real_on_axes(span.next_up()) {
        assert!(body_invalid(world.rebase(
            &[id],
            Quat::IDENTITY,
            translation
        )));
        assert_eq!(body_bits(&world, id), before);
    }
}

#[test]
fn body_settings_are_bounded() {
    let mut world = empty_world();
    let shape = sphere();
    let dynamic = BodySettings::new_dynamic;
    let mut accepted = Vec::new();
    let mut rejected = Vec::new();
    for p in real_on_axes(limits::MAX_POSITION) {
        accepted.push(dynamic().position(p));
    }
    for p in real_on_axes(limits::MAX_POSITION.next_up()) {
        rejected.push(dynamic().position(p));
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY) {
        accepted.push(dynamic().linear_velocity(v));
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY.next_up()) {
        rejected.push(dynamic().linear_velocity(v));
    }
    for v in on_axes(limits::MAX_ANGULAR_VELOCITY) {
        accepted.push(dynamic().angular_velocity(v));
    }
    for v in on_axes(limits::MAX_ANGULAR_VELOCITY.next_up()) {
        rejected.push(dynamic().angular_velocity(v));
    }
    for restitution in [0.0, 1.0] {
        accepted.push(dynamic().restitution(restitution));
    }
    for restitution in [-0.1, 1.0f32.next_up(), f32::NAN] {
        rejected.push(dynamic().restitution(restitution));
    }
    for friction in [0.0, limits::MAX_FRICTION] {
        accepted.push(dynamic().friction(friction));
    }
    for friction in [-0.1, limits::MAX_FRICTION.next_up(), f32::MAX, f32::NAN] {
        rejected.push(dynamic().friction(friction));
    }
    let factor = limits::MAX_GRAVITY_FACTOR;
    for gravity_factor in [-factor, factor] {
        accepted.push(dynamic().gravity_factor(gravity_factor));
    }
    for gravity_factor in [(-factor).next_down(), factor.next_up(), f32::NAN] {
        rejected.push(dynamic().gravity_factor(gravity_factor));
    }
    for mass in [limits::MIN_MASS, limits::MAX_MASS] {
        accepted.push(dynamic().mass(mass));
    }
    for mass in [limits::MIN_MASS.next_down(), limits::MAX_MASS.next_up()] {
        rejected.push(dynamic().mass(mass));
    }
    for settings in &accepted {
        world.create_body(&shape, settings).unwrap();
    }
    let count = world.body_count();
    for settings in &rejected {
        assert!(
            body_invalid(world.create_body(&shape, settings)),
            "{settings:?}"
        );
    }
    assert_eq!(world.body_count(), count);
}

#[test]
fn computed_dynamic_mass_is_bounded_and_kinematic_mass_is_not() {
    let mut world = empty_world();
    // Jolt's density 1000 kg/m³: a 10 m cube weighs 1e6 kg, a 1 cm cube 1e-3 kg.
    let heavy = Shape::new_box(Vec3::new(6.0, 6.0, 6.0)).unwrap();
    let light = Shape::new_box_with_convex_radius(Vec3::new(0.004, 0.004, 0.004), 0.0).unwrap();
    for shape in [&heavy, &light] {
        assert!(body_invalid(
            world.create_body(shape, &BodySettings::new_dynamic())
        ));
        world
            .create_body(shape, &BodySettings::new_kinematic())
            .unwrap();
        world
            .create_body(shape, &BodySettings::new_dynamic().mass(1.0))
            .unwrap();
    }
}

#[test]
fn body_setters_are_bounded_and_rejection_changes_nothing() {
    let mut world = empty_world();
    let id = world
        .create_body(&sphere(), &BodySettings::new_dynamic())
        .unwrap();
    let mut body = world.body_mut(id).unwrap();
    for p in real_on_axes(limits::MAX_POSITION) {
        body.set_position(p, Activation::DontActivate).unwrap();
        body.set_position_and_rotation(p, Quat::IDENTITY, Activation::DontActivate)
            .unwrap();
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY) {
        body.set_linear_velocity(v).unwrap();
    }
    for v in on_axes(limits::MAX_ANGULAR_VELOCITY) {
        body.set_angular_velocity(v).unwrap();
    }
    let before = body_bits(&world, id);
    let mut body = world.body_mut(id).unwrap();
    for p in real_on_axes(limits::MAX_POSITION.next_up()) {
        assert!(body_invalid(body.set_position(p, Activation::DontActivate)));
        assert!(body_invalid(body.set_position_and_rotation(
            p,
            Quat::IDENTITY,
            Activation::DontActivate
        )));
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY.next_up()) {
        assert!(body_invalid(body.set_linear_velocity(v)));
    }
    for v in on_axes(limits::MAX_ANGULAR_VELOCITY.next_up()) {
        assert!(body_invalid(body.set_angular_velocity(v)));
    }
    assert_eq!(body_bits(&world, id), before);
}

/// The f32 vector along `direction` whose length by Jolt's own `Vec3::Length` is the largest
/// at most `bound`.
fn on_the_jolt_bound(direction: Vec3, bound: f32) -> Vec3 {
    let jolt_length = |v: Vec3| {
        let raw = oxijolt_sys::JPH_Vec3 {
            x: v.x,
            y: v.y,
            z: v.z,
        };
        // SAFETY: a pure function of a live local.
        unsafe { oxijolt_sys::JPH_Vec3_Length(&raw) }
    };
    let length = jolt_length(direction);
    let mut v = Vec3::new(
        direction.x * bound / length,
        direction.y * bound / length,
        direction.z * bound / length,
    );
    let largest = |v: Vec3| {
        if v.x.abs() >= v.y.abs() && v.x.abs() >= v.z.abs() {
            0
        } else if v.y.abs() >= v.z.abs() {
            1
        } else {
            2
        }
    };
    let axis = largest(v);
    let step = |v: Vec3, up: bool| {
        let mut c = <[f32; 3]>::from(v);
        let grow = (c[axis] >= 0.0) == up;
        c[axis] = if grow {
            c[axis].next_up()
        } else {
            c[axis].next_down()
        };
        Vec3::from(c)
    };
    while jolt_length(v) > bound {
        v = step(v, false);
    }
    while jolt_length(step(v, true)) <= bound {
        v = step(v, true);
    }
    v
}

#[test]
fn creation_velocities_agree_with_jolts_length_in_many_directions() {
    let mut world = empty_world();
    let shape = sphere();
    let mut seed: u64 = 0x2545_f491_4f6c_dd1d;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 40) as f32 / (1u64 << 24) as f32 * 2.0 - 1.0
    };
    for _ in 0..64 {
        let direction = Vec3::new(next(), next(), next());
        if direction.x == 0.0 && direction.y == 0.0 && direction.z == 0.0 {
            continue;
        }
        for (bound, angular) in [
            (limits::MAX_LINEAR_VELOCITY, false),
            (limits::MAX_ANGULAR_VELOCITY, true),
        ] {
            let at = on_the_jolt_bound(direction, bound);
            let mut beyond = <[f32; 3]>::from(at);
            let axis = (0..3)
                .max_by(|&a, &b| beyond[a].abs().total_cmp(&beyond[b].abs()))
                .unwrap();
            beyond[axis] = if beyond[axis] >= 0.0 {
                beyond[axis].next_up()
            } else {
                beyond[axis].next_down()
            };
            let settings = |v: Vec3| {
                if angular {
                    BodySettings::new_dynamic().angular_velocity(v)
                } else {
                    BodySettings::new_dynamic().linear_velocity(v)
                }
            };
            // With the `asserts` feature Jolt asserts `Length() <= max` on this creation.
            world.create_body(&shape, &settings(at)).unwrap();
            assert!(body_invalid(
                world.create_body(&shape, &settings(Vec3::from(beyond)))
            ));
        }
    }
}

#[test]
fn forces_are_bounded_by_the_acceleration_they_give() {
    let mut world = empty_world();
    // Mass 2 kg: Jolt's inverse mass 0.5 is exact.
    let id = world
        .create_body(&sphere(), &BodySettings::new_dynamic().mass(2.0))
        .unwrap();
    let half = limits::MAX_ACCELERATION;
    let mut body = world.body_mut(id).unwrap();
    // 2 * MAX_ACCELERATION newtons in total give exactly MAX_ACCELERATION.
    body.add_force(Vec3::new(half, 0.0, 0.0)).unwrap();
    body.add_force(Vec3::new(half, 0.0, 0.0)).unwrap();
    assert!(body_invalid(body.add_force(Vec3::new(1.0e3, 0.0, 0.0))));
    body.reset_forces();
    assert!(body_invalid(body.add_force(Vec3::new(
        0.0,
        -(2.0 * half).next_up(),
        0.0
    ))));
    body.add_force(Vec3::new(0.0, -2.0 * half, 0.0)).unwrap();
    for value in NON_FINITE {
        assert!(body_invalid(body.add_force(Vec3::new(value, 0.0, 0.0))));
    }
    assert!(world.step(DT).unwrap().is_complete());
    assert_body_finite(&world, id, "after the bound force");

    // Static and kinematic bodies ignore loads.
    for settings in [BodySettings::new_static(), BodySettings::new_kinematic()] {
        let other = world.create_body(&sphere(), &settings).unwrap();
        let mut body = world.body_mut(other).unwrap();
        body.add_force(Vec3::new(1.0e30, 0.0, 0.0)).unwrap();
        body.add_torque(Vec3::new(1.0e30, 0.0, 0.0)).unwrap();
    }
}

#[test]
fn torques_are_bounded_by_the_angular_acceleration_they_give() {
    let mut world = empty_world();
    // A sphere of mass m and radius r has the inertia 0.4 * m * r² about every axis: 1 kg·m² for
    // 2.5 kg and 1 m, within Jolt's rounding.
    let shape = Shape::new_sphere(1.0).unwrap();
    let id = world
        .create_body(&shape, &BodySettings::new_dynamic().mass(2.5))
        .unwrap();
    let bound = limits::MAX_ANGULAR_ACCELERATION;
    let mut body = world.body_mut(id).unwrap();
    body.add_torque(Vec3::new(0.0, 0.0, bound * 0.999)).unwrap();
    assert!(body_invalid(body.add_torque(Vec3::new(
        0.0,
        0.0,
        bound * 0.002
    ))));
    body.reset_forces();
    assert!(body_invalid(body.add_torque(Vec3::new(
        bound * 1.001,
        0.0,
        0.0
    ))));
    // A force of F newtons at 1 m from the centre adds F newton-metres.
    let point = RVec3::new(1.0, 0.0, 0.0);
    body.add_force_at_point(Vec3::new(0.0, bound * 0.999, 0.0), point)
        .unwrap();
    assert!(body_invalid(
        body.add_force_at_point(Vec3::new(0.0, bound * 0.002, 0.0), point)
    ));
    let corner = RVec3::new(limits::MAX_POSITION.next_up(), 0.0, 0.0);
    assert!(body_invalid(
        body.add_force_at_point(Vec3::new(0.0, 1.0, 0.0), corner)
    ));
    assert!(world.step(DT).unwrap().is_complete());
    assert_body_finite(&world, id, "after the bound torque");
}

#[test]
fn point_forces_along_a_long_lever_count_jolt_rounding() {
    // A needle of `MAX_MASS`, 2 km long and 0.2 nm thick along x, and a force at 7000 m exactly
    // parallel to the lever: the exact torque is 0, but Jolt's `f32` cross product, with one
    // product fused into the subtraction, keeps that product's rounding error.
    let mut world = empty_world();
    let shape =
        Shape::new_box_with_convex_radius(Vec3::new(1000.0, 1.0e-10, 1.0e-10), 0.0).unwrap();
    let id = world
        .create_body(
            &shape,
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, -3500.0, -3500.0))
                .mass(limits::MAX_MASS),
        )
        .unwrap();
    let point = RVec3::new(0.0, 3500.0, 3500.0);
    let along = |size: f32| Vec3::new(0.0, size, size);
    let mut body = world.body_mut(id).unwrap();
    assert!(body_invalid(body.add_force_at_point(along(7.0e12), point)));
    let largest = largest_accepted(7.0e12, |size| {
        let accepted = body.add_force_at_point(along(size), point).is_ok();
        body.reset_forces();
        accepted
    });
    assert!(body_invalid(
        body.add_force_at_point(along(largest.next_up()), point)
    ));
    body.add_force_at_point(along(largest), point).unwrap();
    for _ in 0..3 {
        assert!(world.step(DT).unwrap().is_complete());
    }
    assert_body_finite(&world, id, "after the largest force along the lever");
}

#[test]
fn character_settings_and_setters_are_bounded() {
    let mut world = empty_world();
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let base = || CharacterSettings::new(&capsule);
    let extent = limits::MAX_SHAPE_EXTENT;
    let accepted = [
        base().mass(0.0),
        base().mass(limits::MAX_MASS),
        base().shape_offset(Vec3::new(0.0, extent, 0.0)),
        base().predictive_contact_distance(extent),
        base().character_padding(extent),
        base().collision_tolerance(extent),
    ];
    for settings in &accepted {
        world
            .create_character(settings, RVec3::ZERO, Quat::IDENTITY)
            .unwrap();
    }
    let beyond = extent.next_up();
    let rejected = [
        base().mass(limits::MAX_MASS.next_up()),
        base().shape_offset(Vec3::new(0.0, beyond, 0.0)),
        base().predictive_contact_distance(beyond),
        base().character_padding(beyond),
        base().collision_tolerance(beyond),
    ];
    for settings in &rejected {
        assert!(character_invalid(world.create_character(
            settings,
            RVec3::ZERO,
            Quat::IDENTITY
        )));
    }

    let corner = RVec3::new(limits::MAX_POSITION, limits::MAX_POSITION, 0.0);
    let id = world
        .create_character(&base(), corner, Quat::IDENTITY)
        .unwrap();
    assert!(character_invalid(world.create_character(
        &base(),
        RVec3::new(limits::MAX_POSITION.next_up(), 0.0, 0.0),
        Quat::IDENTITY
    )));
    let mut character = world.character_mut(id).unwrap();
    for p in real_on_axes(limits::MAX_POSITION) {
        character.set_position(p).unwrap();
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY) {
        character.set_linear_velocity(v).unwrap();
    }
    for p in real_on_axes(limits::MAX_POSITION.next_up()) {
        assert!(character_invalid(character.set_position(p)));
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY.next_up()) {
        assert!(character_invalid(character.set_linear_velocity(v)));
    }
    for speed in [0.0, 1.0] {
        character.set_penetration_recovery_speed(speed).unwrap();
    }
    for speed in [
        -f32::MIN_POSITIVE,
        1.0 + f32::EPSILON,
        f32::NAN,
        f32::INFINITY,
        f32::NEG_INFINITY,
    ] {
        assert!(character_invalid(
            character.set_penetration_recovery_speed(speed)
        ));
    }
    let reached = world.character(id).unwrap();
    assert_eq!(reached.penetration_recovery_speed(), 1.0);
    assert_eq!(
        real_bits(reached.position()),
        real_bits(RVec3::new(0.0, 0.0, -limits::MAX_POSITION))
    );
    assert_eq!(
        bits(reached.linear_velocity()),
        bits(Vec3::new(0.0, 0.0, -limits::MAX_LINEAR_VELOCITY))
    );
}

#[test]
fn character_update_gravity_and_steps_are_bounded() {
    let mut world = empty_world();
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let id = world
        .create_character(
            &CharacterSettings::new(&capsule),
            RVec3::ZERO,
            Quat::IDENTITY,
        )
        .unwrap();
    let all = QueryFilter::new();
    let defaults = ExtendedUpdateSettings::default();
    let update = |world: &mut PhysicsWorld, gravity: Vec3, settings: &ExtendedUpdateSettings| {
        world.update_character(id, DT, gravity, settings, &all)
    };
    for gravity in on_axes(limits::MAX_ACCELERATION) {
        update(&mut world, gravity, &defaults).unwrap();
    }
    for gravity in on_axes(limits::MAX_ACCELERATION.next_up()) {
        assert!(character_invalid(update(&mut world, gravity, &defaults)));
    }
    let extent = limits::MAX_SHAPE_EXTENT;
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    let down = Vec3::new(0.0, -extent, 0.0);
    let accepted = [
        defaults.stick_to_floor_step_down(down),
        defaults.walk_stairs_step_up(Vec3::new(0.0, extent, 0.0)),
        defaults.walk_stairs_step_down_extra(down),
        defaults.walk_stairs_min_step_forward(extent),
        defaults.walk_stairs_step_forward_test(extent),
    ];
    for settings in &accepted {
        update(&mut world, gravity, settings).unwrap();
    }
    let beyond = extent.next_up();
    let rejected = [
        defaults.stick_to_floor_step_down(Vec3::new(0.0, -beyond, 0.0)),
        defaults.walk_stairs_step_up(Vec3::new(0.0, beyond, 0.0)),
        defaults.walk_stairs_step_down_extra(Vec3::new(beyond, 0.0, 0.0)),
        defaults.walk_stairs_min_step_forward(beyond),
        defaults.walk_stairs_step_forward_test(beyond),
    ];
    for settings in &rejected {
        assert!(character_invalid(update(&mut world, gravity, settings)));
    }
}

#[test]
fn constraint_frame_points_are_bounded() {
    let (_, layers) = common::ragdoll::ragdoll_layers();
    let skeleton = Skeleton::new(&[
        SkeletonJoint {
            name: "a",
            parent: None,
        },
        SkeletonJoint {
            name: "b",
            parent: Some(0),
        },
    ])
    .unwrap();
    let shape = sphere();
    let build = |anchor: RVec3| {
        let joint = HingeConstraintSettings::new(
            anchor,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        );
        let body = BodySettings::new_dynamic().object_layer(layers.ragdoll);
        let parts = [
            RagdollPart {
                shape: &shape,
                body: body.clone(),
                joint: None,
            },
            RagdollPart {
                shape: &shape,
                body,
                joint: Some(RagdollJoint::Hinge(joint)),
            },
        ];
        RagdollSettings::new(&skeleton, &parts)
    };
    for anchor in real_on_axes(limits::MAX_POSITION) {
        assert!(build(anchor).is_ok());
    }
    for anchor in real_on_axes(limits::MAX_POSITION.next_up()) {
        assert!(matches!(build(anchor), Err(RagdollError::InvalidValue(_))));
    }
}

#[test]
fn six_dof_translation_limits_at_the_bound_step_finitely() {
    // Two parts at the mass extremes, joined by a six-DOF joint whose translation limit along x
    // excludes where they start, so Jolt corrects them by about the extent bound each step.
    let skeleton = Skeleton::new(&[
        SkeletonJoint {
            name: "a",
            parent: None,
        },
        SkeletonJoint {
            name: "b",
            parent: Some(0),
        },
    ])
    .unwrap();
    let shape = sphere();
    let extent = limits::MAX_SHAPE_EXTENT;
    let tx = SixDofConstraintAxis::TranslationX;
    for (min, max) in [(extent.next_down(), extent), (-extent, (-extent).next_up())] {
        let (mut world, layers) = common::ragdoll::ragdoll_world(1);
        let joint = SixDofConstraintSettings::new(
            RVec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .axis(tx, SixDofAxis::Limited { min, max });
        let body = BodySettings::new_dynamic().object_layer(layers.ragdoll);
        let parts = [
            RagdollPart {
                shape: &shape,
                body: body.clone().mass(limits::MAX_MASS),
                joint: None,
            },
            RagdollPart {
                shape: &shape,
                body: body
                    .position(RVec3::new(0.0, 2.0, 0.0))
                    .mass(limits::MIN_MASS),
                joint: Some(RagdollJoint::SixDof(joint)),
            },
        ];
        let settings = RagdollSettings::new(&skeleton, &parts).unwrap();
        let id = world
            .create_ragdoll(&settings, None, Activation::Activate)
            .unwrap();
        let bodies = world.ragdoll(id).unwrap().body_ids().to_vec();
        step_at_both_extremes(&mut world, &bodies, &format!("limits {min}..{max}"));
    }
}

/// A flat heightfield of 4 x 4 samples, three cells of `scale` metres per side from `offset`
/// on x and z.
fn flat_field(offset: f32, scale: f32) -> Result<Shape, ShapeError> {
    Shape::new_height_field(
        4,
        &[0.0; 16],
        &HeightFieldSettings::default()
            .offset(Vec3::new(offset, 0.0, offset))
            .scale(Vec3::new(scale, 1.0, scale)),
    )
}

#[test]
fn height_field_extent_is_bounded() {
    let extent = limits::MAX_SHAPE_EXTENT;
    let field = |offset: f32, scale: f32| flat_field(offset, scale);
    // Three cells of 1333 m from -1999 m end at 2000 m.
    assert!(field(-1999.0, 1333.0).is_ok());
    assert!(matches!(
        field(-1999.0, 1333.0f32.next_up()),
        Err(ShapeError::InvalidValue(_))
    ));
    assert!(matches!(
        field(-extent.next_up(), 1.0),
        Err(ShapeError::InvalidValue(_))
    ));
}

#[test]
fn query_inputs_are_bounded_by_the_frame() {
    let mut world = empty_world();
    add_floor(&mut world);
    let all = QueryFilter::new();
    let ball = sphere();
    let bound = limits::MAX_POSITION;
    // `Real` is `f32` without the `double-precision` feature, so the cast is a no-op there.
    #[allow(clippy::unnecessary_cast)]
    let span = (2.0 * bound) as f32;
    let corner = RVec3::new(bound, bound, bound);
    let beyond = RVec3::new(bound.next_up(), 0.0, 0.0);
    let across = Vec3::new(-span, -span, -span);

    world.cast_ray(&RayCast::new(corner, across), &all).unwrap();
    assert!(query_invalid(world.cast_ray(
        &RayCast::new(beyond, Vec3::new(0.0, -1.0, 0.0)),
        &all
    )));

    let cast = |position: RVec3, direction: Vec3| {
        world.cast_shape(
            &ShapeCast::new(&ball, position, Quat::IDENTITY, direction),
            &all,
        )
    };
    cast(corner, across).unwrap();
    assert!(query_invalid(cast(beyond, Vec3::new(0.0, -1.0, 0.0))));
    assert!(query_invalid(cast(
        corner,
        Vec3::new(-span.next_up(), 0.0, 0.0)
    )));

    let extent = limits::MAX_SHAPE_EXTENT;
    let collide = |position: RVec3, separation: f32| {
        world.collide_shape(
            &CollideShape::new(&ball, position, Quat::IDENTITY).max_separation_distance(separation),
            &all,
        )
    };
    collide(corner, extent).unwrap();
    assert!(query_invalid(collide(beyond, 0.0)));
    assert!(query_invalid(collide(corner, extent.next_up())));
}

#[test]
fn queries_with_shapes_at_the_extent_bound_stay_finite() {
    let mut world = empty_world();
    let extent = limits::MAX_SHAPE_EXTENT;
    let slab = Shape::new_box(Vec3::new(extent, 1.0, extent)).unwrap();
    world
        .create_body(&slab, &BodySettings::new_static())
        .unwrap();
    let all = QueryFilter::new();
    let largest = Shape::new_sphere(extent).unwrap();
    let bound = limits::MAX_POSITION;
    // `Real` is `f32` without the `double-precision` feature, so the cast is a no-op there.
    #[allow(clippy::unnecessary_cast)]
    let span = (2.0 * bound) as f32;
    for sign in [1.0_f32, -1.0] {
        // From a frame corner across the frame, through the slab.
        let at = Real::from(sign) * bound;
        let corner = RVec3::new(at, at, at);
        let across = Vec3::new(-sign * span, -sign * span, -sign * span);
        let hit = world
            .cast_shape(
                &ShapeCast::new(&largest, corner, Quat::IDENTITY, across),
                &all,
            )
            .unwrap()
            .expect("the cast passes through the slab");
        let values = [
            v3(hit.point),
            f3(hit.normal),
            [hit.fraction, hit.distance, hit.penetration_depth].map(f64::from),
        ];
        assert!(
            values.iter().flatten().all(|value| value.is_finite()),
            "{hit:?}"
        );
    }
    // Deep overlap at the centre, reporting separations up to the extent bound.
    let hits = world
        .collide_shape(
            &CollideShape::new(&largest, RVec3::ZERO, Quat::IDENTITY)
                .max_separation_distance(extent),
            &all,
        )
        .unwrap();
    assert!(!hits.is_empty());
    for hit in &hits {
        let values = [
            v3(hit.point_on_shape),
            v3(hit.point_on_body),
            f3(hit.normal),
            [f64::from(hit.penetration_depth); 3],
        ];
        assert!(
            values.iter().flatten().all(|value| value.is_finite()),
            "{hit:?}"
        );
    }
}

#[test]
fn a_ray_with_a_huge_finite_direction_is_cast() {
    // Ray directions are only checked to be finite and not zero; this one reaches far beyond the
    // frame and is cast without overflow.
    let mut world = empty_world();
    let floor = add_floor(&mut world);
    let hit = world
        .cast_ray(
            &RayCast::new(RVec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -1.0e30, 0.0)),
            &QueryFilter::new(),
        )
        .unwrap()
        .expect("the ray hits the floor");
    assert_eq!(hit.body, floor);
    assert!(hit.fraction.is_finite() && hit.fraction > 0.0);
}

/// Steps `world` ten times at the largest and ten times at the smallest time step, checking
/// `bodies` finite after each step.
fn step_at_both_extremes(world: &mut PhysicsWorld, bodies: &[BodyId], what: &str) {
    for delta_time in [PhysicsWorld::MAX_DELTA_TIME, PhysicsWorld::MIN_DELTA_TIME] {
        for tick in 0..10 {
            let _ = world.step(delta_time).unwrap();
            for &id in bodies {
                assert_body_finite(world, id, &format!("{what}, dt {delta_time}, tick {tick}"));
            }
        }
    }
}

#[test]
fn bodies_at_every_bound_step_finitely() {
    let gravity = Vec3::new(0.0, -limits::MAX_ACCELERATION, 0.0);
    let mut world = world(gravity, 1);
    let bound = limits::MAX_POSITION;
    let floor_shape = Shape::new_box(Vec3::new(100.0, 1.0, 100.0)).unwrap();
    world
        .create_body(
            &floor_shape,
            &BodySettings::new_static().position(RVec3::new(bound - 100.0, -bound, 0.0)),
        )
        .unwrap();
    let ball = sphere();
    let speed = limits::MAX_LINEAR_VELOCITY;
    let spin = Vec3::new(0.0, limits::MAX_ANGULAR_VELOCITY, 0.0);
    let factor = limits::MAX_GRAVITY_FACTOR;
    let mut bodies = Vec::new();
    // Head-on pairs of the lightest and the heaviest body at a frame corner, both velocity
    // maxima and both gravity factor extremes.
    for (mass, gravity_factor, y) in [
        (limits::MIN_MASS, factor, bound),
        (limits::MAX_MASS, -factor, bound - 5.0),
        (limits::MAX_MASS, factor, bound - 10.0),
        (limits::MIN_MASS, -factor, bound - 15.0),
    ] {
        for (x, direction) in [(bound, -1.0), (bound - 1.5, 1.0)] {
            let settings = BodySettings::new_dynamic()
                .position(RVec3::new(x, y, bound))
                .mass(mass)
                .gravity_factor(gravity_factor)
                .linear_velocity(Vec3::new(direction * speed, 0.0, 0.0))
                .angular_velocity(spin)
                .allow_sleeping(false);
            bodies.push(world.create_body(&ball, &settings).unwrap());
        }
    }
    // A bouncing body with restitution 1, and two bodies with the largest friction sliding on
    // each other, on the floor.
    let on_floor = |x: Real, y: Real| RVec3::new(bound - 100.0 + x, -bound + y, 0.0);
    let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    bodies.push(
        world
            .create_body(
                &ball,
                &BodySettings::new_dynamic()
                    .position(on_floor(0.0, 3.0))
                    .restitution(1.0),
            )
            .unwrap(),
    );
    for (x, y, v) in [(10.0, 1.5, speed), (10.0, 2.6, -speed)] {
        bodies.push(
            world
                .create_body(
                    &cube,
                    &BodySettings::new_dynamic()
                        .position(on_floor(x, y))
                        .friction(limits::MAX_FRICTION)
                        .linear_velocity(Vec3::new(v, 0.0, 0.0)),
                )
                .unwrap(),
        );
    }
    // Loads at the acceleration bounds, one through a force at a frame corner.
    let loaded = world
        .create_body(&ball, &BodySettings::new_dynamic().mass(2.0))
        .unwrap();
    let pushed = world
        .create_body(
            &ball,
            &BodySettings::new_dynamic()
                .position(RVec3::new(bound - 1.0, bound - 1.0, bound - 1.0))
                .mass(limits::MAX_MASS)
                .gravity_factor(0.0),
        )
        .unwrap();
    let corner = RVec3::new(bound, bound, bound);
    bodies.extend([loaded, pushed]);
    for delta_time in [PhysicsWorld::MAX_DELTA_TIME, PhysicsWorld::MIN_DELTA_TIME] {
        for tick in 0..10 {
            let mut body = world.body_mut(loaded).unwrap();
            body.add_force(Vec3::new(2.0 * limits::MAX_ACCELERATION, 0.0, 0.0))
                .unwrap();
            // The torque bound depends on the inertia; Jolt's sphere inertia is 0.4 m r².
            let torque = 0.999 * limits::MAX_ANGULAR_ACCELERATION * 0.4 * 2.0 * 0.25;
            body.add_torque(Vec3::new(torque, 0.0, 0.0)).unwrap();
            // A force along y at the frame corner, sized so that its torque about the moving
            // centre stays just below the bound (Jolt's sphere inertia, 0.4 m r²).
            let mut body = world.body_mut(pushed).unwrap();
            let center = body.position();
            let lever =
                (wide(corner.x - center.x).powi(2) + wide(corner.z - center.z).powi(2)).sqrt();
            let inertia = 0.4 * f64::from(limits::MAX_MASS) * 0.25;
            let torque = 0.999 * f64::from(limits::MAX_ANGULAR_ACCELERATION) * inertia;
            let force = (torque / lever.max(1.0)) as f32;
            body.add_force_at_point(Vec3::new(0.0, force, 0.0), corner)
                .unwrap();
            let _ = world.step(delta_time).unwrap();
            for &id in &bodies {
                assert_body_finite(&world, id, &format!("dt {delta_time}, tick {tick}"));
            }
        }
    }
}

/// A character of [`limits::MAX_MASS`] whose capsule rests on the +x edge of a cube of
/// [`limits::MIN_MASS`] with half extent 0.03 m, in a world without gravity. That cube is about
/// the smallest one for which Jolt keeps its own inertia, so the weight impulse at the edge
/// turns it fastest.
fn character_on_the_edge_of_a_light_cube() -> (PhysicsWorld, BodyId, CharacterId) {
    let mut world = empty_world();
    let half = 0.03;
    let cube_shape = Shape::new_box_with_convex_radius(Vec3::new(half, half, half), 0.0).unwrap();
    let cube = world
        .create_body(
            &cube_shape,
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, -Real::from(half), 0.0))
                .mass(limits::MIN_MASS)
                .allow_sleeping(false),
        )
        .unwrap();
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let settings = CharacterSettings::new(&capsule)
        .mass(limits::MAX_MASS)
        .shape_offset(Vec3::new(0.0, 0.8, 0.0));
    // The lower sphere (radius 0.3) is centred 0.1 m beyond the edge and touches it.
    let sphere_height = (0.3f32 * 0.3 - 0.1 * 0.1).sqrt();
    let position = RVec3::new(Real::from(half + 0.1), Real::from(sphere_height - 0.3), 0.0);
    let id = world
        .create_character(&settings, position, Quat::IDENTITY)
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    (world, cube, id)
}

#[test]
fn character_weight_impulse_at_a_lever_arm_is_bounded() {
    let delta_time = PhysicsWorld::MAX_DELTA_TIME;
    let update = |world: &mut PhysicsWorld, id, gravity: f32| {
        world.update_character(
            id,
            delta_time,
            Vec3::new(0.0, -gravity, 0.0),
            &ExtendedUpdateSettings::default(),
            &QueryFilter::new(),
        )
    };
    // Mass times gravity times delta time is exactly the bound.
    let at_bound = limits::MAX_WEIGHT_IMPULSE / (limits::MAX_MASS * delta_time);
    let (mut world, cube, id) = character_on_the_edge_of_a_light_cube();
    update(&mut world, id, at_bound).unwrap();
    assert_eq!(world.character(id).unwrap().ground_body(), Some(cube));
    assert_body_finite(&world, cube, "weight impulse at the bound");
    // The impulse turns the cube up to Jolt's clamp; an overflow would have zeroed it.
    let spin = f3(world.body(cube).unwrap().angular_velocity())
        .iter()
        .map(|c| c * c)
        .sum::<f64>()
        .sqrt();
    assert!(
        spin >= 0.99 * f64::from(limits::MAX_ANGULAR_VELOCITY),
        "angular speed {spin}"
    );
    for gravity in [at_bound.next_up(), limits::MAX_ACCELERATION] {
        let (mut world, cube, id) = character_on_the_edge_of_a_light_cube();
        let position = world.character(id).unwrap().position();
        assert!(character_invalid(update(&mut world, id, gravity)));
        assert_eq!(
            real_bits(world.character(id).unwrap().position()),
            real_bits(position)
        );
        let body = world.body(cube).unwrap();
        assert_eq!(bits(body.linear_velocity()), bits(Vec3::ZERO));
        assert_eq!(bits(body.angular_velocity()), bits(Vec3::ZERO));
    }
}

#[test]
fn character_at_its_bounds_pushes_the_lightest_body() {
    let mut world = empty_world();
    add_floor(&mut world);
    let crate_shape = Shape::new_box_with_convex_radius(Vec3::new(0.2, 0.2, 0.2), 0.0).unwrap();
    let lightest = BodySettings::new_dynamic().mass(limits::MIN_MASS);
    let under = world
        .create_body(
            &crate_shape,
            &lightest.clone().position(RVec3::new(0.0, 0.2, 0.0)),
        )
        .unwrap();
    let ahead = world
        .create_body(&crate_shape, &lightest.position(RVec3::new(1.0, 0.6, 0.0)))
        .unwrap();
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let settings = CharacterSettings::new(&capsule)
        .mass(limits::MAX_MASS)
        .max_strength(f32::MAX)
        .shape_offset(Vec3::new(0.0, 0.8, 0.0));
    let id = world
        .create_character(&settings, RVec3::new(0.0, 0.4, 0.0), Quat::IDENTITY)
        .unwrap();
    let all = QueryFilter::new();
    world.refresh_character_contacts(id, &all).unwrap();
    let velocity = Vec3::new(limits::MAX_LINEAR_VELOCITY, 0.0, 0.0);
    for delta_time in [PhysicsWorld::MAX_DELTA_TIME, PhysicsWorld::MIN_DELTA_TIME] {
        let weight_bound = limits::MAX_WEIGHT_IMPULSE / (limits::MAX_MASS * delta_time);
        let gravity = Vec3::new(0.0, -limits::MAX_ACCELERATION.min(weight_bound), 0.0);
        for tick in 0..30 {
            world
                .character_mut(id)
                .unwrap()
                .set_linear_velocity(velocity)
                .unwrap();
            world
                .update_character(
                    id,
                    delta_time,
                    gravity,
                    &ExtendedUpdateSettings::default(),
                    &all,
                )
                .unwrap();
            let _ = world.step(delta_time).unwrap();
            let what = format!("dt {delta_time}, tick {tick}");
            for body in [under, ahead] {
                assert_body_finite(&world, body, &what);
            }
            let character = world.character(id).unwrap();
            let state = [v3(character.position()), f3(character.linear_velocity())];
            assert!(
                state.iter().flatten().all(|value| value.is_finite()),
                "{what}: {state:?}"
            );
        }
    }
}

#[test]
fn vehicle_gravity_at_the_bound_steps_finitely() {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let ground_shape = Shape::new_box(Vec3::new(50.0, 1.0, 50.0)).unwrap();
    world
        .create_body(
            &ground_shape,
            &BodySettings::new_static()
                .object_layer(layers.ground)
                .position(RVec3::new(0.0, -1.0, 0.0)),
        )
        .unwrap();
    let body = chassis_settings(&layers, RVec3::new(0.0, 0.9, 0.0), Quat::IDENTITY);
    let (chassis, car) = add_car_with(&mut world, &body, VehicleCollisionTester::ray(layers.probe));
    let mass = world.body(chassis).unwrap().mass().unwrap();
    for tick in 0..60 {
        world
            .vehicle_mut(car)
            .unwrap()
            .set_gravity(Vec3::new(0.0, -limits::MAX_ACCELERATION, 0.0))
            .unwrap();
        // Within the load bound: the force alone gives at most MAX_ACCELERATION.
        let force = 0.999 * limits::MAX_ACCELERATION * mass;
        world
            .body_mut(chassis)
            .unwrap()
            .add_force(Vec3::new(force, 0.0, 0.0))
            .unwrap();
        let _ = world.step(DT).unwrap();
        assert_body_finite(&world, chassis, &format!("tick {tick}"));
    }
}

#[test]
fn vehicle_springs_and_anti_roll_bar_at_their_bounds_step_finitely() {
    let (mut world, layers) = car_world(Vec3::ZERO, 1);
    let ground_shape = Shape::new_box(Vec3::new(50.0, 1.0, 50.0)).unwrap();
    world
        .create_body(
            &ground_shape,
            &BodySettings::new_static()
                .object_layer(layers.ground)
                .position(RVec3::new(0.0, -1.0, 0.0)),
        )
        .unwrap();
    let coefficient = limits::MAX_SPRING_COEFFICIENT;
    // The largest frequency whose stiffness stays within the bound for a chassis of MAX_MASS.
    let max_frequency = (f64::from(coefficient) / f64::from(limits::MAX_MASS)).sqrt()
        / (2.0 * std::f64::consts::PI);
    let springs = [
        SpringSettings::StiffnessAndDamping {
            stiffness: coefficient,
            damping: coefficient,
        },
        SpringSettings::FrequencyAndDamping {
            frequency: (0.999 * max_frequency) as f32,
            damping: 1.0,
        },
        WheelSettings::DEFAULT_SUSPENSION_SPRING,
        WheelSettings::DEFAULT_SUSPENSION_SPRING,
    ];
    let wheels = WHEEL_POSITIONS
        .iter()
        .zip(springs)
        .map(|(&position, spring)| {
            WheelSettings::new(position)
                .radius(WHEEL_RADIUS)
                .width(WHEEL_WIDTH)
                .suspension_min_length(SUSPENSION_MIN)
                .suspension_max_length(SUSPENSION_MAX)
                .suspension_spring(spring)
        })
        .collect();
    let settings = WheeledVehicleSettings::new(
        wheels,
        vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
        VehicleCollisionTester::ray(layers.probe),
    )
    .anti_roll_bars(vec![
        VehicleAntiRollBar::new(0, 1).stiffness(VehicleAntiRollBar::MAX_STIFFNESS),
        VehicleAntiRollBar::new(2, 3).stiffness(VehicleAntiRollBar::MAX_STIFFNESS),
    ]);
    // Tilted, so that the suspension lengths differ and the anti-roll bars push.
    let tilt = Quat::from_xyzw(0.0, 0.0, 0.05, (1.0_f32 - 0.0025).sqrt());
    let body = chassis_settings(&layers, RVec3::new(0.0, 0.9, 0.0), tilt);
    let chassis = world.create_body(&chassis_shape(), &body).unwrap();
    let car = world.create_wheeled_vehicle(chassis, &settings).unwrap();
    world
        .vehicle_mut(car)
        .unwrap()
        .set_gravity(GRAVITY)
        .unwrap();
    for tick in 0..60 {
        let _ = world.step(DT).unwrap();
        assert_body_finite(&world, chassis, &format!("tick {tick}"));
    }
    step_at_both_extremes(&mut world, &[chassis], "vehicle springs");
}

#[test]
fn friction_at_the_bound_keeps_contacts_finite() {
    // A spinning, sliding cube 1 cm above a floor, both at the largest friction: Jolt makes a
    // speculative contact whose normal impulse is zero, the case in which an infinite combined
    // friction (two bodies at `f32::MAX`) turned the cube's velocities into NaN within three
    // 60 Hz steps. Then the same cube resting on the floor under gravity.
    for (gravity, height) in [(Vec3::ZERO, 1.51), (Vec3::new(0.0, -9.81, 0.0), 1.5)] {
        let mut world = world(gravity, 1);
        let floor = Shape::new_box(Vec3::new(10.0, 1.0, 10.0)).unwrap();
        world
            .create_body(
                &floor,
                &BodySettings::new_static().friction(limits::MAX_FRICTION),
            )
            .unwrap();
        let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
        let id = world
            .create_body(
                &cube,
                &BodySettings::new_dynamic()
                    .position(RVec3::new(0.0, height, 0.0))
                    .friction(limits::MAX_FRICTION)
                    .linear_velocity(Vec3::new(1.0, 0.0, 0.0))
                    .angular_velocity(Vec3::new(0.0, 5.0, 0.0))
                    .allow_sleeping(false),
            )
            .unwrap();
        for tick in 0..30 {
            let _ = world.step(DT).unwrap();
            assert_body_finite(&world, id, &format!("gravity {gravity:?}, tick {tick}"));
        }
        step_at_both_extremes(&mut world, &[id], &format!("gravity {gravity:?}"));
    }
}

#[test]
fn largest_shapes_collide_finitely() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let extent = limits::MAX_SHAPE_EXTENT;
    let terrain = flat_field(-1999.0, 1333.0).unwrap();
    world
        .create_body(&terrain, &BodySettings::new_static())
        .unwrap();
    let heaviest = BodySettings::new_dynamic().mass(limits::MAX_MASS);
    let slab = Shape::new_box(Vec3::new(extent, 1.0, extent)).unwrap();
    let pole = Shape::new_capsule(extent - 1.0, 1.0).unwrap();
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &slab,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 1,
        },
        CompoundChild {
            shape: &pole,
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            user_data: 2,
        },
    ])
    .unwrap();
    let mut bodies = Vec::new();
    for (shape, y) in [
        (&slab, 2.0),
        (&pole, Real::from(extent) + 4.0),
        (&compound, 10.0),
    ] {
        bodies.push(
            world
                .create_body(shape, &heaviest.clone().position(RVec3::new(0.0, y, 0.0)))
                .unwrap(),
        );
    }
    step_at_both_extremes(&mut world, &bodies, "largest shapes");
}

/// A static anchor and a dynamic sphere of `mass` at the origin, hinged about z at the origin.
fn hinged_sphere(
    mass: f32,
    settings: HingeConstraintSettings,
) -> (
    PhysicsWorld,
    BodyId,
    Result<ConstraintId<HingeConstraint>, ConstraintError>,
) {
    let mut world = empty_world();
    let anchor = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    let body = world
        .create_body(&sphere(), &BodySettings::new_dynamic().mass(mass))
        .unwrap();
    let hinge = world.create_constraint(anchor, body, &settings);
    (world, body, hinge)
}

fn z_hinge() -> HingeConstraintSettings {
    HingeConstraintSettings::new(
        RVec3::ZERO,
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
}

fn constraint_invalid(result: Result<impl Sized, ConstraintError>) -> bool {
    matches!(result, Err(ConstraintError::InvalidValue(_)))
}

#[test]
fn world_constraint_springs_are_bounded_by_the_bodies_effective_mass() {
    // A sphere's largest principal moment, 0.1 m for a radius of 0.5 m, is below its mass, so
    // the bound is the mass as Jolt stores it: 1 / (1 / m) with the inverse in f32.
    let mass = limits::MAX_MASS;
    let bound = 1.0 / f64::from(1.0 / mass);
    let fits = |frequency: f32| {
        let omega = 2.0 * std::f64::consts::PI * f64::from(frequency);
        bound * omega * omega <= f64::from(limits::MAX_SPRING_COEFFICIENT)
    };
    let mut frequency = ((f64::from(limits::MAX_SPRING_COEFFICIENT) / bound).sqrt()
        / (2.0 * std::f64::consts::PI)) as f32;
    while !fits(frequency) {
        frequency = frequency.next_down();
    }
    while fits(frequency.next_up()) {
        frequency = frequency.next_up();
    }
    let spring = |frequency| SpringSettings::FrequencyAndDamping {
        frequency,
        damping: 0.0,
    };
    let motor = |frequency| MotorSettings::default().spring(spring(frequency));

    let (_, _, accepted) = hinged_sphere(mass, z_hinge().motor(motor(frequency)));
    assert!(accepted.is_ok());
    let (_, _, rejected) = hinged_sphere(mass, z_hinge().motor(motor(frequency.next_up())));
    assert!(constraint_invalid(rejected));

    let (mut world, _, hinge) = hinged_sphere(mass, z_hinge());
    let mut hinge = world.constraint_mut(hinge.unwrap()).unwrap();
    assert!(hinge.set_motor_settings(motor(frequency)).is_ok());
    assert!(constraint_invalid(
        hinge.set_motor_settings(motor(frequency.next_up()))
    ));
    assert!(hinge.set_limits_spring(spring(frequency)).is_ok());
    assert!(constraint_invalid(
        hinge.set_limits_spring(spring(frequency.next_up()))
    ));

    // A static body has no effective mass, so any valid frequency fits it; between two static
    // bodies nothing bounds a spring.
    let (mut world, body, _) = hinged_sphere(mass, z_hinge());
    let anchor = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(3.0, 0.0, 0.0)),
        )
        .unwrap();
    let other = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(-3.0, 0.0, 0.0)),
        )
        .unwrap();
    let rod =
        DistanceConstraintSettings::new(RVec3::new(3.0, 0.0, 0.0), RVec3::new(-3.0, 0.0, 0.0))
            .limits_spring(spring(f32::MAX));
    assert!(world.create_constraint(anchor, other, &rod).is_ok());
    let rod = DistanceConstraintSettings::new(RVec3::new(3.0, 0.0, 0.0), RVec3::ZERO);
    let rod = world.create_constraint(anchor, body, &rod).unwrap();
    let mut rod = world.constraint_mut(rod).unwrap();
    assert!(rod.set_limits_spring(spring(frequency)).is_ok());
    assert!(constraint_invalid(
        rod.set_limits_spring(spring(frequency.next_up()))
    ));
}

#[test]
fn constraint_targets_are_bounded() {
    let (mut world, body, hinge) = hinged_sphere(1.0, z_hinge());
    let mut hinge = world.constraint_mut(hinge.unwrap()).unwrap();
    for angle in [PI, -PI] {
        assert!(hinge.set_target_angle(angle).is_ok());
    }
    for angle in [PI.next_up(), (-PI).next_down(), f32::NAN] {
        assert!(constraint_invalid(hinge.set_target_angle(angle)));
    }
    let speed = limits::MAX_ANGULAR_VELOCITY;
    for velocity in [speed, -speed] {
        assert!(hinge.set_target_angular_velocity(velocity).is_ok());
    }
    for velocity in [speed.next_up(), (-speed).next_down(), f32::INFINITY] {
        assert!(constraint_invalid(
            hinge.set_target_angular_velocity(velocity)
        ));
    }

    let extent = limits::MAX_SHAPE_EXTENT;
    let anchor = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(3.0, 0.0, 0.0)),
        )
        .unwrap();
    let rod = |range| {
        DistanceConstraintSettings::new(RVec3::new(3.0, 0.0, 0.0), RVec3::ZERO).range(range)
    };
    for (min, max) in [(extent.next_up(), extent.next_up()), (-1.0e-6, 1.0)] {
        assert!(constraint_invalid(world.create_constraint(
            anchor,
            body,
            &rod(DistanceRange::Range { min, max })
        )));
    }
    let rod = world
        .create_constraint(
            anchor,
            body,
            &rod(DistanceRange::Range {
                min: 0.0,
                max: extent,
            }),
        )
        .unwrap();
    let mut rod = world.constraint_mut(rod).unwrap();
    assert!(rod.set_distance(extent, extent).is_ok());
    assert!(rod.set_distance(0.0, 0.0).is_ok());
    for (min, max) in [
        (0.0, extent.next_up()),
        (-1.0e-6, 0.0),
        (1.0, 0.5),
        (f32::NAN, 1.0),
    ] {
        assert!(constraint_invalid(rod.set_distance(min, max)));
    }

    // Constraint points follow the frame rule. A kinematic body has no lever-arm check, so
    // only the frame rule applies.
    let held = world
        .create_body(&sphere(), &BodySettings::new_kinematic())
        .unwrap();
    let point = |at| PointConstraintSettings::new(RVec3::ZERO).point2(at);
    for at in real_on_axes(limits::MAX_POSITION) {
        let id = world.create_constraint(anchor, held, &point(at)).unwrap();
        world.remove_constraint(id).unwrap();
    }
    for at in real_on_axes(limits::MAX_POSITION.next_up()) {
        assert!(constraint_invalid(world.create_constraint(
            anchor,
            held,
            &point(at)
        )));
    }
}

#[test]
fn constraint_friction_at_f32_max_steps_finitely() {
    // Jolt clamps a constraint's friction impulse to `dt` times the friction: a hinge of
    // unlimited friction on bodies of both mass extremes spinning at the velocity bound.
    for mass in [limits::MIN_MASS, limits::MAX_MASS] {
        let (mut world, body, hinge) = hinged_sphere(mass, z_hinge().max_friction_torque(f32::MAX));
        let hinge = hinge.unwrap();
        world
            .constraint_mut(hinge)
            .unwrap()
            .set_max_friction_torque(f32::MAX)
            .unwrap();
        world
            .body_mut(body)
            .unwrap()
            .set_angular_velocity(Vec3::new(0.0, 0.0, limits::MAX_ANGULAR_VELOCITY))
            .unwrap();
        for tick in 0..120 {
            let _ = world.step(DT).unwrap();
            assert_body_finite(&world, body, &format!("mass {mass}, tick {tick}"));
        }
        let lambda = world.constraint(hinge).unwrap().total_lambda_position();
        assert!(
            f3(lambda).iter().all(|value| value.is_finite()),
            "{lambda:?}"
        );
        step_at_both_extremes(&mut world, &[body], &format!("friction, mass {mass}"));
    }
}

/// A static anchor and a dynamic sphere of `mass` at the origin joined by `settings`.
fn joined_sphere<S: ConstraintSettings>(
    mass: f32,
    settings: &S,
) -> (
    PhysicsWorld,
    BodyId,
    Result<ConstraintId<S::Kind>, ConstraintError>,
) {
    let mut world = empty_world();
    let anchor = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    let body = world
        .create_body(&sphere(), &BodySettings::new_dynamic().mass(mass))
        .unwrap();
    let id = world.create_constraint(anchor, body, settings);
    (world, body, id)
}

/// As [`joined_sphere`], with a kinematic sphere: without a dynamic body no lever-arm check
/// applies, so only the settings' own ranges decide.
fn joined_kinematic_sphere<S: ConstraintSettings>(
    settings: &S,
) -> Result<ConstraintId<S::Kind>, ConstraintError> {
    let mut world = empty_world();
    let anchor = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    let body = world
        .create_body(&sphere(), &BodySettings::new_kinematic())
        .unwrap();
    world.create_constraint(anchor, body, settings)
}

/// Two dynamic spheres of 1 kg, 3 m apart along x, joined by `settings`: gears, racks and
/// pinions and pulleys need two dynamic bodies.
fn coupled_spheres<S: ConstraintSettings>(
    settings: &S,
) -> Result<ConstraintId<S::Kind>, ConstraintError> {
    let mut world = empty_world();
    let [first, second] = [0.0, 3.0].map(|x| {
        world
            .create_body(
                &sphere(),
                &BodySettings::new_dynamic().position(RVec3::new(x, 0.0, 0.0)),
            )
            .unwrap()
    });
    world.create_constraint(first, second, settings)
}

const AXIS_X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const AXIS_Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);

#[test]
fn slider_cone_swing_twist_and_six_dof_targets_are_bounded() {
    let extent = limits::MAX_SHAPE_EXTENT;
    let speed = limits::MAX_LINEAR_VELOCITY;
    let spin = limits::MAX_ANGULAR_VELOCITY;
    let slider = || SliderConstraintSettings::new(RVec3::ZERO, AXIS_X, AXIS_Y);
    for (min, max) in [(-extent, extent), (0.0, extent), (-extent, 0.0)] {
        assert!(joined_sphere(1.0, &slider().limits(min, max)).2.is_ok());
    }
    for (min, max) in [
        (-extent.next_up(), 0.0),
        (0.0, extent.next_up()),
        (0.1, 1.0),
    ] {
        assert!(constraint_invalid(
            joined_sphere(1.0, &slider().limits(min, max)).2
        ));
    }
    let (mut world, _, id) = joined_sphere(1.0, &slider());
    let mut slider = world.constraint_mut(id.unwrap()).unwrap();
    for position in [extent, -extent] {
        assert!(slider.set_target_position(position).is_ok());
    }
    for position in [extent.next_up(), (-extent).next_down(), f32::NAN] {
        assert!(constraint_invalid(slider.set_target_position(position)));
    }
    for velocity in [speed, -speed] {
        assert!(slider.set_target_velocity(velocity).is_ok());
    }
    for velocity in [speed.next_up(), (-speed).next_down()] {
        assert!(constraint_invalid(slider.set_target_velocity(velocity)));
    }
    assert!(slider.set_limits(-extent, extent).is_ok());
    assert!(constraint_invalid(
        slider.set_limits(-extent, extent.next_up())
    ));
    slider.remove_limits();
    assert_eq!(world.constraint(id.unwrap()).unwrap().limits(), None);

    let cone = |angle| ConeConstraintSettings::new(RVec3::ZERO, AXIS_X, angle);
    for angle in [0.0, PI] {
        assert!(joined_sphere(1.0, &cone(angle)).2.is_ok());
    }
    for angle in [-1.0e-6, PI.next_up(), f32::NAN] {
        assert!(constraint_invalid(joined_sphere(1.0, &cone(angle)).2));
    }
    let (mut world, _, id) = joined_sphere(1.0, &cone(0.5));
    let mut cone = world.constraint_mut(id.unwrap()).unwrap();
    assert!(cone.set_half_cone_angle(PI).is_ok());
    assert!(constraint_invalid(cone.set_half_cone_angle(PI.next_up())));

    let rotation_bound = |set: &mut dyn FnMut(Quat) -> Result<(), ConstraintError>| {
        assert!(set(Quat::from_xyzw(0.0, 0.6, 0.0, 0.8)).is_ok());
        assert!(constraint_invalid(set(Quat::from_xyzw(
            0.0, 0.6, 0.0, 0.81
        ))));
        assert!(constraint_invalid(set(Quat::from_xyzw(
            f32::NAN,
            0.0,
            0.0,
            1.0
        ))));
    };
    let swing_twist = SwingTwistConstraintSettings::new(RVec3::ZERO, AXIS_X, AXIS_Y)
        .half_cone_angles(1.0, 1.0)
        .twist_limits(-1.0, 1.0);
    let (mut world, _, id) = joined_sphere(1.0, &swing_twist);
    let mut joint = world.constraint_mut(id.unwrap()).unwrap();
    for velocity in on_axes(spin) {
        assert!(joint.set_target_angular_velocity_cs(velocity).is_ok());
    }
    for velocity in on_axes(spin.next_up()) {
        assert!(constraint_invalid(
            joint.set_target_angular_velocity_cs(velocity)
        ));
    }
    rotation_bound(&mut |q| joint.set_target_orientation_cs(q));

    let (mut world, _, id) = joined_sphere(1.0, &SixDofConstraintSettings::default());
    let mut joint = world.constraint_mut(id.unwrap()).unwrap();
    for velocity in on_axes(speed) {
        assert!(joint.set_target_velocity_cs(velocity).is_ok());
    }
    for velocity in on_axes(speed.next_up()) {
        assert!(constraint_invalid(joint.set_target_velocity_cs(velocity)));
    }
    for velocity in on_axes(spin) {
        assert!(joint.set_target_angular_velocity_cs(velocity).is_ok());
    }
    for velocity in on_axes(spin.next_up()) {
        assert!(constraint_invalid(
            joint.set_target_angular_velocity_cs(velocity)
        ));
    }
    for position in on_axes(extent) {
        assert!(joint.set_target_position_cs(position).is_ok());
    }
    for position in on_axes(extent.next_up()) {
        assert!(constraint_invalid(joint.set_target_position_cs(position)));
    }
    rotation_bound(&mut |q| joint.set_target_orientation_cs(q));
}

#[test]
fn slider_swing_twist_and_six_dof_motor_springs_are_bounded() {
    // As in `world_constraint_springs_are_bounded_by_the_bodies_effective_mass`, for the motor
    // setters of the other driven kinds.
    let mass = limits::MAX_MASS;
    let bound = 1.0 / f64::from(1.0 / mass);
    let fits = |frequency: f32| {
        let omega = 2.0 * std::f64::consts::PI * f64::from(frequency);
        bound * omega * omega <= f64::from(limits::MAX_SPRING_COEFFICIENT)
    };
    let mut frequency = ((f64::from(limits::MAX_SPRING_COEFFICIENT) / bound).sqrt()
        / (2.0 * std::f64::consts::PI)) as f32;
    while !fits(frequency) {
        frequency = frequency.next_down();
    }
    while fits(frequency.next_up()) {
        frequency = frequency.next_up();
    }
    let motor = |frequency| {
        MotorSettings::default().spring(SpringSettings::FrequencyAndDamping {
            frequency,
            damping: 0.0,
        })
    };
    let check = |set: &mut dyn FnMut(MotorSettings) -> Result<(), ConstraintError>| {
        assert!(set(motor(frequency)).is_ok());
        assert!(constraint_invalid(set(motor(frequency.next_up()))));
    };

    let slider = SliderConstraintSettings::new(RVec3::ZERO, AXIS_X, AXIS_Y);
    assert!(joined_sphere(mass, &slider.clone().motor(motor(frequency)))
        .2
        .is_ok());
    assert!(constraint_invalid(
        joined_sphere(mass, &slider.clone().motor(motor(frequency.next_up()))).2
    ));
    let (mut world, _, id) = joined_sphere(mass, &slider);
    let mut slider = world.constraint_mut(id.unwrap()).unwrap();
    check(&mut |m| slider.set_motor_settings(m));

    let (mut world, _, id) = joined_sphere(mass, &SwingTwistConstraintSettings::default());
    let mut joint = world.constraint_mut(id.unwrap()).unwrap();
    check(&mut |m| joint.set_swing_motor_settings(m));
    check(&mut |m| joint.set_twist_motor_settings(m));

    let (mut world, _, id) = joined_sphere(mass, &SixDofConstraintSettings::default());
    let mut joint = world.constraint_mut(id.unwrap()).unwrap();
    for axis in [
        SixDofConstraintAxis::TranslationX,
        SixDofConstraintAxis::RotationZ,
    ] {
        check(&mut |m| joint.set_motor_settings(axis, m));
    }
}

#[test]
fn slider_and_swing_twist_friction_at_f32_max_steps_finitely() {
    // As for the hinge: a slider sliding and a swing-twist joint spinning at the velocity
    // bounds, with unlimited friction, on bodies of both mass extremes.
    for mass in [limits::MIN_MASS, limits::MAX_MASS] {
        let slider =
            SliderConstraintSettings::new(RVec3::ZERO, AXIS_X, AXIS_Y).max_friction_force(f32::MAX);
        let (mut world, body, id) = joined_sphere(mass, &slider);
        world
            .constraint_mut(id.unwrap())
            .unwrap()
            .set_max_friction_force(f32::MAX)
            .unwrap();
        world
            .body_mut(body)
            .unwrap()
            .set_linear_velocity(Vec3::new(limits::MAX_LINEAR_VELOCITY, 0.0, 0.0))
            .unwrap();
        for tick in 0..120 {
            let _ = world.step(DT).unwrap();
            assert_body_finite(&world, body, &format!("slider, mass {mass}, tick {tick}"));
        }
        step_at_both_extremes(&mut world, &[body], &format!("slider, mass {mass}"));

        let joint = SwingTwistConstraintSettings::new(RVec3::ZERO, AXIS_X, AXIS_Y)
            .twist_limits(-PI, PI)
            .max_friction_torque(f32::MAX);
        let (mut world, body, id) = joined_sphere(mass, &joint);
        world
            .constraint_mut(id.unwrap())
            .unwrap()
            .set_max_friction_torque(f32::MAX)
            .unwrap();
        world
            .body_mut(body)
            .unwrap()
            .set_angular_velocity(Vec3::new(limits::MAX_ANGULAR_VELOCITY, 0.0, 0.0))
            .unwrap();
        for tick in 0..120 {
            let _ = world.step(DT).unwrap();
            assert_body_finite(
                &world,
                body,
                &format!("swing-twist, mass {mass}, tick {tick}"),
            );
        }
        step_at_both_extremes(&mut world, &[body], &format!("swing-twist, mass {mass}"));
    }
}

#[test]
fn coupling_ratios_are_bounded() {
    let z = Vec3::new(0.0, 0.0, 1.0);
    let max = limits::MAX_RATIO;
    let min = 1.0 / limits::MAX_RATIO;
    let gear = |ratio| GearConstraintSettings::new(z, z, ratio);
    let rack = |ratio| RackAndPinionConstraintSettings::new(z, AXIS_X, ratio);
    // A gear's ratio is between 1 and its own bound (see `GearConstraintSettings`).
    let gear_max = limits::MAX_GEAR_RATIO;
    for ratio in [1.0, gear_max] {
        assert!(coupled_spheres(&gear(ratio)).is_ok());
    }
    for ratio in [
        1.0_f32.next_down(),
        gear_max.next_up(),
        max,
        0.5,
        -1.0,
        -2.0,
        f32::NAN,
    ] {
        assert!(constraint_invalid(coupled_spheres(&gear(ratio))));
    }
    for ratio in [max, -max, min, -min] {
        assert!(coupled_spheres(&rack(ratio)).is_ok());
    }
    for ratio in [
        max.next_up(),
        -max.next_up(),
        min.next_down(),
        0.0,
        f32::NAN,
    ] {
        assert!(constraint_invalid(coupled_spheres(&rack(ratio))));
    }
    // Teeth give Jolt's ratios; zero teeth or a rack length out of range give none.
    assert!(coupled_spheres(&gear(1.0).teeth(10, 30)).is_ok());
    for (teeth1, teeth2) in [(0, 30), (30, 10), (1, 11)] {
        assert!(constraint_invalid(coupled_spheres(
            &gear(1.0).teeth(teeth1, teeth2)
        )));
    }
    assert!(coupled_spheres(&rack(1.0).teeth(20, 1.0, 10)).is_ok());
    for (teeth, length) in [
        (0, 1.0),
        (20, 0.0),
        (20, limits::MAX_SHAPE_EXTENT.next_up()),
    ] {
        assert!(constraint_invalid(coupled_spheres(
            &rack(1.0).teeth(teeth, length, 10)
        )));
    }
}

/// A static base far away and three spheres hinged to it about z at x = 0 and 3 and held on a
/// slider along x at x = -3, of masses `masses`.
fn coupling_scene(
    masses: [f32; 3],
) -> (
    PhysicsWorld,
    [BodyId; 3],
    [ConstraintId<HingeConstraint>; 2],
    ConstraintId<SliderConstraint>,
) {
    let z = Vec3::new(0.0, 0.0, 1.0);
    let mut world = empty_world();
    let base = world
        .create_body(
            &sphere(),
            &BodySettings::new_static().position(RVec3::new(0.0, -10.0, 0.0)),
        )
        .unwrap();
    let xs = [0.0, 3.0, -3.0];
    let bodies = [0, 1, 2].map(|i| {
        world
            .create_body(
                &sphere(),
                &BodySettings::new_dynamic()
                    .mass(masses[i])
                    .position(RVec3::new(xs[i], 0.0, 0.0)),
            )
            .unwrap()
    });
    let hinges = [0, 1].map(|i| {
        world
            .create_constraint(
                base,
                bodies[i],
                &HingeConstraintSettings::new(RVec3::new(xs[i], 0.0, 0.0), z, AXIS_X),
            )
            .unwrap()
    });
    let slider = world
        .create_constraint(
            base,
            bodies[2],
            &SliderConstraintSettings::new(RVec3::new(-3.0, 0.0, 0.0), AXIS_X, AXIS_Y),
        )
        .unwrap();
    (world, bodies, hinges, slider)
}

/// Steps 120 ticks and then both time step extremes, checking `bodies` finite.
fn step_coupling(world: &mut PhysicsWorld, bodies: &[BodyId], what: &str) {
    for tick in 0..120 {
        let _ = world.step(DT).unwrap();
        for &id in bodies {
            assert_body_finite(world, id, &format!("{what}, tick {tick}"));
        }
    }
    step_at_both_extremes(world, bodies, what);
}

#[test]
fn ratios_at_the_bound_step_finitely() {
    // Gears and racks and pinions at their ratio bounds between bodies of both mass extremes,
    // one of the coupled bodies spinning at the angular velocity bound.
    let z = Vec3::new(0.0, 0.0, 1.0);
    let spin = Vec3::new(0.0, 0.0, limits::MAX_ANGULAR_VELOCITY);
    let masses = [limits::MIN_MASS, limits::MAX_MASS];
    for mass1 in masses {
        for mass2 in masses {
            for spun in [0, 1] {
                for ratio in [1.0, limits::MAX_GEAR_RATIO] {
                    let (mut world, bodies, hinges, _) = coupling_scene([mass1, mass2, mass2]);
                    world
                        .create_constraint(
                            bodies[0],
                            bodies[1],
                            &GearConstraintSettings::new(z, z, ratio)
                                .constraints(hinges[0], hinges[1]),
                        )
                        .unwrap();
                    world
                        .body_mut(bodies[spun])
                        .unwrap()
                        .set_angular_velocity(spin)
                        .unwrap();
                    let what = format!("gear {ratio}, masses {mass1} {mass2}, body {spun} spun");
                    step_coupling(&mut world, &bodies[..2], &what);
                }
                let max = limits::MAX_RATIO;
                for ratio in [max, -max, 1.0 / max, -1.0 / max] {
                    let (mut world, bodies, hinges, slider) = coupling_scene([mass1, mass2, mass2]);
                    world
                        .create_constraint(
                            bodies[0],
                            bodies[2],
                            &RackAndPinionConstraintSettings::new(z, AXIS_X, ratio)
                                .constraints(hinges[0], slider),
                        )
                        .unwrap();
                    let driven = if spun == 0 { bodies[0] } else { bodies[2] };
                    let mut body = world.body_mut(driven).unwrap();
                    if spun == 0 {
                        body.set_angular_velocity(spin).unwrap();
                    } else {
                        body.set_linear_velocity(Vec3::new(limits::MAX_LINEAR_VELOCITY, 0.0, 0.0))
                            .unwrap();
                    }
                    let what = format!("rack {ratio}, masses {mass1} {mass2}, body {spun} driven");
                    step_coupling(&mut world, &[bodies[0], bodies[2]], &what);
                }
            }
        }
    }
}

/// A pulley between two dynamic spheres of `masses` hanging 3 m below their fixed points.
fn pulley_pair(
    gravity: Vec3,
    masses: [f32; 2],
    settings: impl Fn(PulleyConstraintSettings) -> PulleyConstraintSettings,
) -> (
    PhysicsWorld,
    [BodyId; 2],
    Result<ConstraintId<PulleyConstraint>, ConstraintError>,
) {
    let mut world = world(gravity, 1);
    let xs = [-2.0, 2.0];
    let bodies = [0, 1].map(|i| {
        world
            .create_body(
                &sphere(),
                &BodySettings::new_dynamic()
                    .mass(masses[i])
                    .position(RVec3::new(xs[i], 0.0, 0.0)),
            )
            .unwrap()
    });
    let pulley = settings(PulleyConstraintSettings::new(
        RVec3::new(-2.0, 0.0, 0.0),
        RVec3::new(-2.0, 3.0, 0.0),
        RVec3::new(2.0, 0.0, 0.0),
        RVec3::new(2.0, 3.0, 0.0),
    ));
    let id = world.create_constraint(bodies[0], bodies[1], &pulley);
    (world, bodies, id)
}

#[test]
fn pulley_ratio_and_lengths_are_bounded() {
    let max = limits::MAX_RATIO;
    let min = 1.0 / limits::MAX_RATIO;
    let create = |settings: &dyn Fn(PulleyConstraintSettings) -> PulleyConstraintSettings| {
        pulley_pair(Vec3::ZERO, [1.0, 1.0], settings).2
    };
    for ratio in [min, max] {
        assert!(create(&|s| s.ratio(ratio)).is_ok());
    }
    for ratio in [min.next_down(), max.next_up(), -1.0, 0.0, f32::NAN] {
        assert!(constraint_invalid(create(&|s| s.ratio(ratio))));
    }
    let longest = 3.0 * limits::MAX_SHAPE_EXTENT;
    let range = |min, max| PulleyLength::Range { min, max };
    for (lo, hi) in [(0.0, longest), (longest, longest), (0.0, 0.0)] {
        assert!(create(&|s| s.ratio(2.0).length(range(lo, hi))).is_ok());
    }
    for (lo, hi) in [(0.0, longest.next_up()), (-1.0e-6, 1.0), (2.0, 1.0)] {
        assert!(constraint_invalid(create(&|s| s
            .ratio(2.0)
            .length(range(lo, hi)))));
    }
    // Fixed points are world points under the frame rule, like body points.
    for point in real_on_axes(limits::MAX_POSITION.next_up()) {
        assert!(constraint_invalid(create(&|_| {
            PulleyConstraintSettings::new(
                RVec3::new(-2.0, 0.0, 0.0),
                point,
                RVec3::new(2.0, 0.0, 0.0),
                RVec3::new(2.0, 3.0, 0.0),
            )
        })));
    }

    let (mut world, _, id) = pulley_pair(Vec3::ZERO, [1.0, 1.0], |s| s.ratio(2.0));
    let mut pulley = world.constraint_mut(id.unwrap()).unwrap();
    assert!(pulley.set_length(0.0, longest).is_ok());
    assert!(pulley.set_length(longest, longest).is_ok());
    for (lo, hi) in [(0.0, longest.next_up()), (-1.0e-6, 1.0), (2.0, 1.0)] {
        assert!(constraint_invalid(pulley.set_length(lo, hi)));
    }
}

#[test]
fn pulleys_at_the_ratio_bound_step_finitely() {
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    let masses = [limits::MIN_MASS, limits::MAX_MASS];
    for ratio in [limits::MAX_RATIO, 1.0 / limits::MAX_RATIO] {
        for mass1 in masses {
            for mass2 in masses {
                let (mut world, bodies, id) =
                    pulley_pair(gravity, [mass1, mass2], |s| s.ratio(ratio));
                id.unwrap();
                let what = format!("pulley {ratio}, masses {mass1} {mass2}");
                step_coupling(&mut world, &bodies, &what);
            }
        }
    }
}

/// A dynamic cube of half extent `half` and `mass` at `position`.
fn cube(world: &mut PhysicsWorld, half: f32, mass: f32, position: RVec3) -> BodyId {
    world
        .create_body(
            &Shape::new_box(Vec3::new(half, half, half)).unwrap(),
            &BodySettings::new_dynamic().mass(mass).position(position),
        )
        .unwrap()
}

/// The lever at which a cube of half extent `half` has the lever-arm ratio `ratio`: a cube's
/// ratio is `2 · |r|² / k²` with `k² = (2 · half)² / 6`.
fn cube_lever(half: f32, ratio: f32) -> f32 {
    let k_squared = (2.0 * half) * (2.0 * half) / 6.0;
    (ratio * k_squared / 2.0).sqrt()
}

/// `direction` scaled to `length`, as a position.
fn at(direction: Vec3, length: f32) -> RVec3 {
    let unit = length
        / (direction.x * direction.x + direction.y * direction.y + direction.z * direction.z)
            .sqrt();
    RVec3::new(
        Real::from(direction.x * unit),
        Real::from(direction.y * unit),
        Real::from(direction.z * unit),
    )
}

#[test]
fn lever_arms_are_bounded_by_the_bodies_size() {
    let bound = limits::MAX_LEVER_ARM_RATIO;
    let lever = cube_lever(0.5, bound);
    let below = lever * 0.999;
    let above = lever * 1.001;
    let directions = [AXIS_X, Vec3::new(0.0, -1.0, 0.0), Vec3::new(1.0, 1.0, -1.0)];

    // A point in world space on a 1 kg, 1 m cube held by a static anchor, in several
    // directions; the anchor itself is not checked.
    let mut world = empty_world();
    let anchor = world
        .create_body(&sphere(), &BodySettings::new_static())
        .unwrap();
    let body = cube(&mut world, 0.5, 1.0, RVec3::new(0.0, 0.0, 0.0));
    for direction in directions {
        let id = world
            .create_constraint(
                anchor,
                body,
                &PointConstraintSettings::new(at(direction, below)),
            )
            .unwrap();
        world.remove_constraint(id).unwrap();
        assert!(constraint_invalid(world.create_constraint(
            anchor,
            body,
            &PointConstraintSettings::new(at(direction, above))
        )));
    }
    // The same in the bodies' own frames, on a turned body: the ratio does not depend on the
    // frame.
    let turned = world
        .create_body(
            &Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 30.0, 0.0))
                .rotation(Quat::from_xyzw(0.0, 0.6, 0.0, 0.8)),
        )
        .unwrap();
    let local = |length: f32| {
        let point = at(Vec3::new(1.0, 2.0, 3.0), length);
        HingeConstraintSettings::new(RVec3::ZERO, AXIS_X, AXIS_Y)
            .space(ConstraintSpace::LocalToBodyCom)
            .frame2(point, AXIS_X, AXIS_Y)
    };
    let id = world
        .create_constraint(anchor, turned, &local(below))
        .unwrap();
    world.remove_constraint(id).unwrap();
    assert!(constraint_invalid(world.create_constraint(
        anchor,
        turned,
        &local(above)
    )));
    // A rod held at its end has a ratio of 6 at any length.
    let rod = world
        .create_body(
            &Shape::new_box(Vec3::new(0.02, 1000.0, 0.02)).unwrap(),
            &BodySettings::new_dynamic().position(RVec3::new(0.0, -1000.0, 50.0)),
        )
        .unwrap();
    world
        .create_constraint(
            anchor,
            rod,
            &HingeConstraintSettings::new(RVec3::new(0.0, 0.0, 50.0), AXIS_X, AXIS_Y),
        )
        .unwrap();

    // An automatic point is where Jolt puts it: between the centres of mass, weighted by inverse
    // mass towards the lighter body, kinematic bodies included. A 1 kg, 1 m cube, dynamic or
    // kinematic, welded to a dynamic 1000 kg, 4 m cube holds the large cube at 1000/1001 of the
    // distance between them, which its ratio bounds.
    let weld = FixedConstraintSettings::default().auto_detect_point(true);
    let reach = cube_lever(2.0, bound) * 1.001;
    for light_settings in [BodySettings::new_dynamic(), BodySettings::new_kinematic()] {
        for (distance, accepted) in [(reach * 0.999, true), (reach * 1.001, false)] {
            let mut world = empty_world();
            let light = world
                .create_body(
                    &Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap(),
                    &light_settings.clone().mass(1.0),
                )
                .unwrap();
            let heavy = cube(&mut world, 2.0, 1000.0, at(AXIS_X, distance));
            let result = world.create_constraint(light, heavy, &weld);
            assert_eq!(result.is_ok(), accepted, "{distance}: {result:?}");
        }
    }
    // A 10 g, 6 cm part welded 3 cm from a 1000 kg cube is held at its own centre.
    for half in [0.5, 1.0, 2.0] {
        let mut world = empty_world();
        let big = cube(&mut world, half, 1000.0, RVec3::ZERO);
        let part = cube(&mut world, 0.03, 0.01, at(AXIS_X, half + 0.06));
        world.create_constraint(big, part, &weld).unwrap();
    }
    // Next to a static body it is the dynamic body's centre of mass.
    let far = cube(&mut world, 0.5, 1.0, RVec3::new(4000.0, 0.0, 0.0));
    world.create_constraint(anchor, far, &weld).unwrap();

    // A path holds body 1 anywhere along it: a path reaching beyond the bound from a dynamic
    // platform is refused, one within it accepted; body 2 sits at the path's start.
    let path = |length: f32| {
        PathConstraintSettings::new(
            HermitePath::new(
                Vec3::new(0.0, 0.0, 1.0),
                vec![
                    path_point([0.0, 0.0, 0.0], [length, 0.0, 0.0]),
                    path_point([length, 0.0, 0.0], [length, 0.0, 0.0]),
                ],
                false,
            )
            .unwrap(),
        )
    };
    for (length, accepted) in [(below * 0.7, true), (above, false)] {
        let mut world = empty_world();
        let platform = cube(&mut world, 0.5, 1.0, RVec3::ZERO);
        let car = cube(&mut world, 0.1, 1.0, RVec3::ZERO);
        let result = world.create_constraint(platform, car, &path(length));
        assert_eq!(result.is_ok(), accepted, "{length}: {result:?}");
    }
}

#[test]
fn far_and_light_constraint_points_are_refused() {
    // The three scenes in which a far point drove Jolt to a non-finite velocity in a release
    // build or to a failed assertion with the `asserts` feature.
    // Two 1 kg, 1 m cubes joined 4000 m away (ratio 1.9e8).
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let first = cube(&mut world, 0.5, 1.0, RVec3::ZERO);
    let second = cube(&mut world, 0.5, 1.0, RVec3::new(2.0, 0.0, 0.0));
    assert!(constraint_invalid(world.create_constraint(
        first,
        second,
        &PointConstraintSettings::new(RVec3::new(2400.0, 3200.0, 0.0))
    )));
    // A 1 g, 6 cm cube hinged 116 m away to a 1000 kg plate (4.5e7).
    let plate = world
        .create_body(
            &Shape::new_box(Vec3::new(0.5, 0.05, 0.5)).unwrap(),
            &BodySettings::new_dynamic()
                .mass(1000.0)
                .position(RVec3::new(0.0, 20.0, 0.0)),
        )
        .unwrap();
    let light = cube(
        &mut world,
        0.03,
        limits::MIN_MASS,
        RVec3::new(2.250432, 21.509048, -2.93712),
    );
    let hinge = HingeConstraintSettings::new(
        RVec3::new(-96.0364, 77.2034, 37.558807),
        Vec3::new(0.8384718, 0.37976828, -0.39082107),
        Vec3::new(0.5449333, -0.57961416, 0.6058838),
    );
    assert!(constraint_invalid(
        world.create_constraint(plate, light, &hinge)
    ));
    // A weld 3 m from a 1 g and 1.2 m from a 10 g cube of 6 cm (3.2e4 and 4400).
    let heavier = cube(&mut world, 0.03, 0.01, RVec3::new(0.0, 40.0, 0.0));
    let lighter = cube(
        &mut world,
        0.03,
        limits::MIN_MASS,
        RVec3::new(0.5571059, 42.71482, -2.755086),
    );
    let weld = FixedConstraintSettings::new(
        RVec3::new(0.68242, 40.884994, -0.28849602),
        Vec3::new(0.63742137, 0.31528664, 0.70305645),
        Vec3::new(0.52797884, -0.8432883, -0.10051466),
    );
    assert!(constraint_invalid(
        world.create_constraint(heavier, lighter, &weld)
    ));
    assert_eq!(world.constraint_count(), 0);
}

/// `direction` scaled to `length`.
fn scaled(direction: Vec3, length: f32) -> Vec3 {
    let unit = length
        / (direction.x * direction.x + direction.y * direction.y + direction.z * direction.z)
            .sqrt();
    Vec3::new(direction.x * unit, direction.y * unit, direction.z * unit)
}

/// Holds a cube of `half` and `mass` at the origin by `settings(point)`, the point `lever`
/// away: from a static anchor, or from a cube of the same size and ten times the mass on the
/// far side of the point. The cube moves and spins at the velocity bounds under gravity for
/// 120 ticks, then steps at both time step extremes.
fn step_held_cube<S: ConstraintSettings>(
    what: &str,
    half: f32,
    mass: f32,
    anchored: bool,
    settings: impl Fn(RVec3) -> S,
) {
    let direction = Vec3::new(0.1, 0.65, -0.7);
    let lever = cube_lever(half, limits::MAX_LEVER_ARM_RATIO * 0.999);
    let point = at(direction, lever);
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let other = if anchored {
        world
            .create_body(
                &sphere(),
                &BodySettings::new_static().position(at(direction, 3.0 * lever)),
            )
            .unwrap()
    } else {
        cube(&mut world, half, mass * 10.0, at(direction, 2.0 * lever))
    };
    let held = cube(&mut world, half, mass, RVec3::ZERO);
    world
        .create_constraint(other, held, &settings(point))
        .unwrap();
    let mut body = world.body_mut(held).unwrap();
    body.set_linear_velocity(scaled(
        Vec3::new(-0.45542818, 0.78363067, 0.42250222),
        limits::MAX_LINEAR_VELOCITY * 0.999,
    ))
    .unwrap();
    body.set_angular_velocity(scaled(
        Vec3::new(-0.59818655, -0.4488059, 0.6638871),
        limits::MAX_ANGULAR_VELOCITY * 0.999,
    ))
    .unwrap();
    let what = format!("{what}, cube of {half} m and {mass} kg");
    for tick in 0..120 {
        let _ = world.step(DT).unwrap();
        for id in [other, held] {
            assert_body_finite(&world, id, &format!("{what}, tick {tick}"));
        }
    }
    step_at_both_extremes(&mut world, &[other, held], &what);
}

#[test]
fn constraints_at_the_lever_arm_bound_step_finitely() {
    let axis = Vec3::new(0.63742137, 0.31528664, 0.70305645);
    let normal = Vec3::new(0.52797884, -0.8432883, -0.10051466);
    let locked = |point| {
        let mut settings = SixDofConstraintSettings::new(point, axis, normal);
        for which in [
            SixDofConstraintAxis::TranslationX,
            SixDofConstraintAxis::TranslationY,
            SixDofConstraintAxis::TranslationZ,
            SixDofConstraintAxis::RotationX,
            SixDofConstraintAxis::RotationY,
            SixDofConstraintAxis::RotationZ,
        ] {
            settings = settings.axis(which, SixDofAxis::Fixed);
        }
        settings
    };
    for (half, mass) in [(0.03, limits::MIN_MASS), (0.5, 1.0)] {
        step_held_cube("ball joint", half, mass, true, PointConstraintSettings::new);
        step_held_cube("hinge", half, mass, true, |p| {
            HingeConstraintSettings::new(p, axis, normal)
        });
        for anchored in [true, false] {
            step_held_cube("weld", half, mass, anchored, |p| {
                FixedConstraintSettings::new(p, axis, normal)
            });
            step_held_cube("locked six-DOF joint", half, mass, anchored, locked);
            step_held_cube(
                "swing-twist joint with zero ranges",
                half,
                mass,
                anchored,
                |p| SwingTwistConstraintSettings::new(p, axis, normal),
            );
        }
    }
}

fn path_point(position: [f32; 3], tangent: [f32; 3]) -> HermitePathPoint {
    HermitePathPoint {
        position: Vec3::from(position),
        tangent: Vec3::from(tangent),
    }
}

/// A straight path of one segment of `length` metres along x, in the plane of normal z.
fn straight_path(start: f32, length: f32) -> Result<HermitePath, ConstraintError> {
    HermitePath::new(
        Vec3::new(0.0, 0.0, 1.0),
        vec![
            path_point([start, 0.0, 0.0], [length, 0.0, 0.0]),
            path_point([start + length, 0.0, 0.0], [length, 0.0, 0.0]),
        ],
        false,
    )
}

#[test]
fn path_inputs_are_bounded() {
    let extent = limits::MAX_SHAPE_EXTENT;
    // Positions and tangents within the extent.
    assert!(straight_path(extent - 1.0, 1.0).is_ok());
    assert!(constraint_invalid(straight_path(extent, 1.0)));
    assert!(straight_path(0.0, extent).is_ok());
    assert!(constraint_invalid(straight_path(0.0, extent.next_up())));

    let path = straight_path(0.0, 1.0).unwrap();
    let settings = |f: &dyn Fn(PathConstraintSettings) -> PathConstraintSettings| {
        f(PathConstraintSettings::new(path.clone()))
    };
    // A kinematic body, so that the lever-arm check does not decide.
    let create = |settings: PathConstraintSettings| joined_kinematic_sphere(&settings);
    for offset in on_axes(extent) {
        assert!(create(settings(&|s| s.path_position(offset))).is_ok());
    }
    for offset in on_axes(extent.next_up()) {
        assert!(constraint_invalid(create(settings(
            &|s| s.path_position(offset)
        ))));
    }
    assert!(constraint_invalid(create(settings(&|s| {
        s.path_rotation(Quat::from_xyzw(0.0, 0.6, 0.0, 0.81))
    }))));
    for fraction in [0.0, 1.0] {
        assert!(create(settings(&|s| s.path_fraction(fraction))).is_ok());
    }
    for fraction in [-1.0e-6, 1.0_f32.next_up(), f32::NAN] {
        assert!(constraint_invalid(create(settings(
            &|s| s.path_fraction(fraction)
        ))));
    }
    assert!(constraint_invalid(create(settings(
        &|s| s.max_friction_force(-1.0)
    ))));

    let (mut world, _, id) = joined_sphere(1.0, &PathConstraintSettings::new(path.clone()));
    let id = id.unwrap();
    let reading = world.constraint(id).unwrap();
    assert!(reading
        .closest_fraction(Vec3::new(extent, 0.0, 0.0), 0.0)
        .is_ok());
    assert!(constraint_invalid(
        reading.closest_fraction(Vec3::new(extent.next_up(), 0.0, 0.0), 0.0)
    ));
    assert!(constraint_invalid(
        reading.closest_fraction(Vec3::ZERO, f32::NAN)
    ));
    let mut motor = world.constraint_mut(id).unwrap();
    let speed = limits::MAX_LINEAR_VELOCITY;
    for velocity in [speed, -speed] {
        assert!(motor.set_target_velocity(velocity).is_ok());
    }
    for velocity in [speed.next_up(), (-speed).next_down()] {
        assert!(constraint_invalid(motor.set_target_velocity(velocity)));
    }
    for fraction in [0.0, 1.0] {
        assert!(motor.set_target_path_fraction(fraction).is_ok());
    }
    for fraction in [-1.0e-6, 1.0_f32.next_up()] {
        assert!(constraint_invalid(motor.set_target_path_fraction(fraction)));
    }
    assert!(constraint_invalid(motor.set_max_friction_force(f32::NAN)));
}

#[test]
fn path_motor_springs_and_friction_are_bounded() {
    // As in `world_constraint_springs_are_bounded_by_the_bodies_effective_mass`.
    let mass = limits::MAX_MASS;
    let bound = 1.0 / f64::from(1.0 / mass);
    let fits = |frequency: f32| {
        let omega = 2.0 * std::f64::consts::PI * f64::from(frequency);
        bound * omega * omega <= f64::from(limits::MAX_SPRING_COEFFICIENT)
    };
    let mut frequency = ((f64::from(limits::MAX_SPRING_COEFFICIENT) / bound).sqrt()
        / (2.0 * std::f64::consts::PI)) as f32;
    while !fits(frequency) {
        frequency = frequency.next_down();
    }
    while fits(frequency.next_up()) {
        frequency = frequency.next_up();
    }
    let motor = |frequency| {
        MotorSettings::default().spring(SpringSettings::FrequencyAndDamping {
            frequency,
            damping: 0.0,
        })
    };
    let path = straight_path(0.0, 1.0).unwrap();
    let settings = PathConstraintSettings::new(path);
    assert!(
        joined_sphere(mass, &settings.clone().position_motor(motor(frequency)))
            .2
            .is_ok()
    );
    assert!(constraint_invalid(
        joined_sphere(
            mass,
            &settings.clone().position_motor(motor(frequency.next_up()))
        )
        .2
    ));
    let (mut world, _, id) = joined_sphere(mass, &settings);
    let mut path = world.constraint_mut(id.unwrap()).unwrap();
    assert!(path.set_position_motor_settings(motor(frequency)).is_ok());
    assert!(constraint_invalid(
        path.set_position_motor_settings(motor(frequency.next_up()))
    ));

    // Unlimited friction on a body sliding along the path at the velocity bound.
    for mass in [limits::MIN_MASS, limits::MAX_MASS] {
        let (mut world, body, id) =
            joined_sphere(mass, &settings.clone().max_friction_force(f32::MAX));
        world
            .constraint_mut(id.unwrap())
            .unwrap()
            .set_max_friction_force(f32::MAX)
            .unwrap();
        world
            .body_mut(body)
            .unwrap()
            .set_linear_velocity(Vec3::new(limits::MAX_LINEAR_VELOCITY, 0.0, 0.0))
            .unwrap();
        step_coupling(&mut world, &[body], &format!("path friction, mass {mass}"));
    }
}

fn soft_invalid(result: Result<SoftBodySharedSettings, SoftBodyError>) -> bool {
    matches!(result, Err(SoftBodyError::InvalidValue(_)))
}

/// A triangle of three vertices, the first at `first`, with one face.
fn soft_triangle(first: SoftBodyVertex) -> SoftBodySharedSettingsBuilder {
    let vertices = vec![
        first,
        SoftBodyVertex::new(Vec3::new(1.0, 0.0, 0.0)),
        SoftBodyVertex::new(Vec3::new(0.0, 0.0, 1.0)),
    ];
    SoftBodySharedSettings::builder(vertices, vec![[0, 2, 1]])
}

/// Four vertices of 1 kg on a unit square with two faces, every inverse mass `inverse_mass`.
fn soft_square(inverse_mass: f32) -> SoftBodySharedSettingsBuilder {
    let vertices = [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]]
        .map(|[x, z]| SoftBodyVertex {
            inverse_mass,
            ..SoftBodyVertex::new(Vec3::new(x, 0.0, z))
        })
        .to_vec();
    SoftBodySharedSettings::builder(vertices, vec![[0, 2, 3], [0, 3, 1]])
}

#[test]
fn soft_body_shared_settings_are_bounded() {
    let vertex = SoftBodyVertex::new;
    let mut accepted = Vec::new();
    let mut rejected = Vec::new();
    // The first vertex on the extent bound; the triangle keeps an area.
    for p in on_axes(limits::MAX_SHAPE_EXTENT) {
        accepted.push(soft_triangle(vertex(p)));
    }
    for p in on_axes(limits::MAX_SHAPE_EXTENT.next_up()) {
        rejected.push(soft_triangle(vertex(p)));
    }
    let away = Vec3::new(0.0, 0.0, -1.0);
    for v in on_axes(limits::MAX_LINEAR_VELOCITY) {
        accepted.push(soft_triangle(SoftBodyVertex {
            velocity: v,
            ..vertex(away)
        }));
    }
    for v in on_axes(limits::MAX_LINEAR_VELOCITY.next_up()) {
        rejected.push(soft_triangle(SoftBodyVertex {
            velocity: v,
            ..vertex(away)
        }));
    }
    let with_inverse_mass = |inverse_mass| {
        soft_triangle(SoftBodyVertex {
            inverse_mass,
            ..vertex(away)
        })
    };
    for inverse_mass in [0.0, limits::MAX_VERTEX_INVERSE_MASS, 2.0 / limits::MAX_MASS] {
        accepted.push(with_inverse_mass(inverse_mass));
    }
    for inverse_mass in [-0.5, limits::MAX_VERTEX_INVERSE_MASS.next_up(), 1.0e-7]
        .into_iter()
        .chain(NON_FINITE)
    {
        rejected.push(with_inverse_mass(inverse_mass));
    }
    // A face edge on the length bound and just below it.
    let edge = |length: f32| {
        let vertices = vec![
            vertex(Vec3::ZERO),
            vertex(Vec3::new(length, 0.0, 0.0)),
            vertex(Vec3::new(0.0, 0.0, 1.0)),
        ];
        SoftBodySharedSettings::builder(vertices, vec![[0, 2, 1]])
    };
    accepted.push(edge(limits::MIN_SOFT_BODY_EDGE_LENGTH));
    rejected.push(edge(limits::MIN_SOFT_BODY_EDGE_LENGTH.next_down()));
    let attributes = SoftBodyVertexAttributes::default;
    let generated = |attributes: SoftBodyVertexAttributes| {
        soft_square(1.0).create_constraints(SoftBodyBendType::Dihedral, attributes)
    };
    let lra = LongRangeAttachment::GeodesicDistance;
    for compliance in [0.0, limits::MAX_COMPLIANCE] {
        accepted.push(generated(attributes().compliance(compliance)));
        accepted.push(generated(attributes().shear_compliance(compliance)));
        accepted.push(generated(attributes().bend_compliance(Some(compliance))));
    }
    for compliance in [-f32::MIN_POSITIVE, limits::MAX_COMPLIANCE.next_up()]
        .into_iter()
        .chain(NON_FINITE)
    {
        rejected.push(generated(attributes().compliance(compliance)));
        rejected.push(generated(attributes().shear_compliance(compliance)));
        rejected.push(generated(attributes().bend_compliance(Some(compliance))));
    }
    for multiplier in [1.0, limits::MAX_RATIO] {
        accepted.push(generated(
            attributes().long_range_attachment(lra, multiplier),
        ));
    }
    for multiplier in [1.0f32.next_down(), limits::MAX_RATIO.next_up()]
        .into_iter()
        .chain(NON_FINITE)
    {
        rejected.push(generated(
            attributes().long_range_attachment(lra, multiplier),
        ));
    }
    for builder in accepted {
        let description = format!("{builder:?}");
        assert!(builder.build().is_ok(), "{description}");
    }
    for builder in rejected {
        let description = format!("{builder:?}");
        assert!(soft_invalid(builder.build()), "{description}");
    }
}

#[test]
fn soft_body_total_mass_is_bounded() {
    // Four vertices whose masses add up to the bound exactly (powers of two keep it exact in
    // f32 and f64).
    assert!(soft_square(4.0 / limits::MAX_MASS).build().is_ok());
    assert!(soft_invalid(
        soft_square((4.0 / limits::MAX_MASS).next_down()).build()
    ));
}

#[test]
fn soft_body_settings_are_bounded() {
    let mut world = empty_world();
    let shared = soft_square(1.0).build().unwrap();
    let base = SoftBodySettings::default;
    let mut accepted = Vec::new();
    let mut rejected = Vec::new();
    for p in real_on_axes(limits::MAX_POSITION) {
        accepted.push(base().position(p));
    }
    for p in real_on_axes(limits::MAX_POSITION.next_up()) {
        rejected.push(base().position(p));
    }
    for iterations in [1, SoftBodySettings::MAX_ITERATIONS] {
        accepted.push(base().num_iterations(iterations));
    }
    for iterations in [0, SoftBodySettings::MAX_ITERATIONS + 1] {
        rejected.push(base().num_iterations(iterations));
    }
    accepted.push(base().linear_damping(0.0));
    for damping in [-f32::MIN_POSITIVE].into_iter().chain(NON_FINITE) {
        rejected.push(base().linear_damping(damping));
    }
    for velocity in [f32::MIN_POSITIVE, limits::MAX_LINEAR_VELOCITY] {
        accepted.push(base().max_linear_velocity(velocity));
    }
    for velocity in [0.0, limits::MAX_LINEAR_VELOCITY.next_up(), f32::NAN] {
        rejected.push(base().max_linear_velocity(velocity));
    }
    for restitution in [0.0, 1.0] {
        accepted.push(base().restitution(restitution));
    }
    for restitution in [-0.1, 1.0f32.next_up(), f32::NAN] {
        rejected.push(base().restitution(restitution));
    }
    for friction in [0.0, limits::MAX_FRICTION] {
        accepted.push(base().friction(friction));
    }
    for friction in [-0.1, limits::MAX_FRICTION.next_up(), f32::NAN] {
        rejected.push(base().friction(friction));
    }
    // The open square encloses no volume, so it takes no pressure; see
    // `pressure_needs_a_volume_for_its_faces` for the bound on a closed body.
    accepted.push(base().pressure(0.0));
    rejected.push(base().pressure(f32::MIN_POSITIVE));
    for pressure in [-f32::MIN_POSITIVE, limits::MAX_SOFT_BODY_PRESSURE.next_up()]
        .into_iter()
        .chain(NON_FINITE)
    {
        rejected.push(base().pressure(pressure));
    }
    let factor = limits::MAX_GRAVITY_FACTOR;
    for gravity_factor in [-factor, factor] {
        accepted.push(base().gravity_factor(gravity_factor));
    }
    for gravity_factor in [(-factor).next_down(), factor.next_up(), f32::NAN] {
        rejected.push(base().gravity_factor(gravity_factor));
    }
    for radius in [0.0, limits::MAX_SHAPE_EXTENT] {
        accepted.push(base().vertex_radius(radius));
    }
    for radius in [
        -f32::MIN_POSITIVE,
        limits::MAX_SHAPE_EXTENT.next_up(),
        f32::NAN,
    ] {
        rejected.push(base().vertex_radius(radius));
    }
    rejected.push(base().rotation(Quat::from_xyzw(0.0, 0.0, 0.0, 2.0)));
    for settings in &accepted {
        world.create_soft_body(&shared, settings).unwrap();
    }
    let count = world.body_count();
    for settings in &rejected {
        assert!(
            body_invalid(world.create_soft_body(&shared, settings)),
            "{settings:?}"
        );
    }
    let unknown = ObjectLayer::new(99);
    assert_eq!(
        world.create_soft_body(&shared, &base().object_layer(unknown)),
        Err(BodyError::UnknownObjectLayer(unknown))
    );
    assert_eq!(world.body_count(), count);
}

/// The tetrahedron of the right corner at the origin with the edges `side` along x and z and
/// the fourth vertex at `(side, height, side)`, faces wound counter-clockwise seen from outside,
/// vertices of inverse mass `inverse_mass` joined by rigid edges.
fn soft_tetrahedron(side: f32, height: f32, inverse_mass: f32) -> SoftBodySharedSettingsBuilder {
    let vertices = [
        [0.0, 0.0, 0.0],
        [side, 0.0, 0.0],
        [0.0, 0.0, side],
        [side, height, side],
    ]
    .map(|p| SoftBodyVertex {
        inverse_mass,
        ..SoftBodyVertex::new(Vec3::from(p))
    })
    .to_vec();
    SoftBodySharedSettings::builder(vertices, vec![[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]])
        .create_constraints(SoftBodyBendType::None, SoftBodyVertexAttributes::default())
}

#[test]
fn pressure_needs_a_volume_for_its_faces() {
    let mut world = empty_world();
    let pressure = |value| SoftBodySettings::default().pressure(value);
    let max = limits::MAX_SOFT_BODY_PRESSURE;
    // Every face has an area and 1 m edges, but the six-volume is 1e-36: Jolt's pressure
    // coefficient `pressure * dt / volume` would overflow `f32` in the first step.
    let sliver = soft_tetrahedron(1.0, 1.0e-36, 1.0).build().unwrap();
    // The same tetrahedron wound inside out, and an open square: no positive volume.
    let inside_out = SoftBodySharedSettings::builder(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
        ]
        .map(|p| SoftBodyVertex::new(Vec3::from(p)))
        .to_vec(),
        vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
    )
    .build()
    .unwrap();
    let square = soft_square(1.0).build().unwrap();
    // A tetrahedron large enough for the largest pressure, and a unit one that takes less.
    let large = soft_tetrahedron(12.0, 12.0, 1.0).build().unwrap();
    let small = soft_tetrahedron(1.0, 1.0, 1.0).build().unwrap();
    for shared in [&sliver, &inside_out, &square, &large, &small] {
        world.create_soft_body(shared, &pressure(0.0)).unwrap();
    }
    world.create_soft_body(&large, &pressure(max)).unwrap();
    world.create_soft_body(&small, &pressure(5.0e4)).unwrap();
    let count = world.body_count();
    for shared in [&sliver, &inside_out, &square, &small] {
        assert!(body_invalid(world.create_soft_body(shared, &pressure(max))));
    }
    for shared in [&sliver, &inside_out, &square] {
        assert!(body_invalid(
            world.create_soft_body(shared, &pressure(f32::MIN_POSITIVE))
        ));
    }
    assert!(body_invalid(
        world.create_soft_body(&small, &pressure(2.0e5))
    ));
    assert_eq!(world.body_count(), count);
    step(&mut world, 60);
}

/// The smallest height up to `side` of a [`soft_tetrahedron`] that takes `pressure`, found by
/// bisection on `f32` heights.
fn smallest_pressurised_height(side: f32, pressure: f32, inverse_mass: f32) -> f32 {
    let mut world = empty_world();
    let mut accepts = |height: f32| {
        let shared = soft_tetrahedron(side, height, inverse_mass)
            .build()
            .unwrap();
        match world.create_soft_body(&shared, &SoftBodySettings::default().pressure(pressure)) {
            Ok(id) => {
                world.remove_body(id).unwrap();
                true
            }
            Err(_) => false,
        }
    };
    let (mut low, mut high) = (1.0e-36_f32, side);
    assert!(!accepts(low) && accepts(high));
    // Positive floats are ordered like their bit patterns.
    while high.to_bits() - low.to_bits() > 1 {
        let middle = f32::from_bits(low.to_bits() + (high.to_bits() - low.to_bits()) / 2);
        if accepts(middle) {
            high = middle;
        } else {
            low = middle;
        }
    }
    high
}

#[test]
fn a_sliver_at_the_pressure_bound_steps_finitely() {
    // The lightest vertices; the largest pressure, where the force term decides the volume
    // bound, and a pressure so small that the bound of Jolt's rounding decides.
    let inverse_mass = limits::MAX_VERTEX_INVERSE_MASS;
    for (side, pressure) in [(20.0, limits::MAX_SOFT_BODY_PRESSURE), (1.0, 1.0e-3)] {
        let height = smallest_pressurised_height(side, pressure, inverse_mass);
        for gravity in [Vec3::ZERO, GRAVITY] {
            let mut world = world(gravity, 1);
            let create = |world: &mut PhysicsWorld, height| {
                let shared = soft_tetrahedron(side, height, inverse_mass)
                    .build()
                    .unwrap();
                world.create_soft_body(&shared, &SoftBodySettings::default().pressure(pressure))
            };
            assert!(body_invalid(create(&mut world, height.next_down())));
            let id = create(&mut world, height).unwrap();
            for tick in 0..120 {
                let _ = world.step(DT).unwrap();
                for vertex in world.soft_body(id).unwrap().vertices() {
                    let p = vertex.position;
                    let [x, y, z] = <[f32; 3]>::from(vertex.velocity);
                    assert!(
                        p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
                        "pressure {pressure}, height {height}, tick {tick}"
                    );
                    assert!(
                        x.is_finite() && y.is_finite() && z.is_finite(),
                        "pressure {pressure}, height {height}, tick {tick}"
                    );
                }
            }
        }
    }
}

#[test]
fn unpinning_cannot_release_an_accumulated_force() {
    let bound = limits::MAX_ACCELERATION * limits::MAX_MASS;
    // Every vertex pinned: Jolt moves no vertex by the force, but keeps accumulating it.
    let mut world = empty_world();
    let shared = soft_square(0.0).build().unwrap();
    let id = world
        .create_soft_body(&shared, &SoftBodySettings::default())
        .unwrap();
    let mut body = world.body_mut(id).unwrap();
    assert!(body_invalid(body.add_force(Vec3::new(f32::MAX, 0.0, 0.0))));
    assert!(body_invalid(body.add_force(Vec3::new(
        bound.next_up(),
        0.0,
        0.0
    ))));
    body.add_force(Vec3::new(bound, 0.0, 0.0)).unwrap();
    assert!(body_invalid(body.add_force(Vec3::new(bound, 0.0, 0.0))));
    let before = world.soft_body(id).unwrap().vertices();
    let mut soft = world.soft_body_mut(id).unwrap();
    assert!(body_invalid(soft.set_vertex_inverse_mass(0, 1.0)));
    assert_eq!(world.soft_body(id).unwrap().vertices(), before);
    // Without the force the vertex may move.
    world.body_mut(id).unwrap().reset_forces();
    world
        .soft_body_mut(id)
        .unwrap()
        .set_vertex_inverse_mass(0, 1.0)
        .unwrap();
    step(&mut world, 2);

    // Vertices of 1 kg with the largest force for them: a lighter vertex is refused, a heavier
    // one and a pinned one are not.
    let mut world = empty_world();
    let shared = soft_square(1.0).build().unwrap();
    let id = world
        .create_soft_body(&shared, &SoftBodySettings::default())
        .unwrap();
    world
        .body_mut(id)
        .unwrap()
        .add_force(Vec3::new(4.0 * limits::MAX_ACCELERATION, 0.0, 0.0))
        .unwrap();
    let before = world.soft_body(id).unwrap().vertices();
    let mut soft = world.soft_body_mut(id).unwrap();
    assert!(body_invalid(soft.set_vertex_inverse_mass(0, 2.0)));
    assert_eq!(world.soft_body(id).unwrap().vertices(), before);
    let mut soft = world.soft_body_mut(id).unwrap();
    soft.set_vertex_inverse_mass(0, 0.5).unwrap();
    soft.set_vertex_inverse_mass(1, 0.0).unwrap();
    step(&mut world, 2);
    for vertex in world.soft_body(id).unwrap().vertices() {
        let [x, y, z] = <[f32; 3]>::from(vertex.velocity);
        assert!(x.is_finite() && y.is_finite() && z.is_finite());
    }
}

#[test]
fn soft_body_vertex_writes_are_bounded_and_rejection_changes_nothing() {
    let mut world = empty_world();
    let shared = soft_triangle(SoftBodyVertex::kinematic(Vec3::ZERO))
        .build()
        .unwrap();
    let id = world
        .create_soft_body(&shared, &SoftBodySettings::default())
        .unwrap();
    let before = world.soft_body(id).unwrap().vertices();
    let mut body = world.soft_body_mut(id).unwrap();
    for v in on_axes(limits::MAX_LINEAR_VELOCITY.next_up()) {
        assert!(body_invalid(body.set_vertex_velocity(1, v)));
    }
    for value in NON_FINITE {
        assert!(body_invalid(
            body.set_vertex_velocity(1, Vec3::new(value, 0.0, 0.0))
        ));
    }
    for inverse_mass in [-0.5, limits::MAX_VERTEX_INVERSE_MASS.next_up(), 1.0e-7]
        .into_iter()
        .chain(NON_FINITE)
    {
        assert!(body_invalid(body.set_vertex_inverse_mass(1, inverse_mass)));
    }
    // A kinematic move whose velocity passes the bound, and targets beyond the frame.
    let origin = before[0].position;
    let dt = 0.5;
    let reach = Real::from(1.01 * limits::MAX_LINEAR_VELOCITY * dt);
    let too_far = RVec3::new(origin.x + reach, origin.y, origin.z);
    assert!(body_invalid(body.move_kinematic_vertex(0, too_far, dt)));
    for p in real_on_axes(limits::MAX_POSITION.next_up()) {
        assert!(body_invalid(body.move_kinematic_vertex(0, p, dt)));
    }
    assert_eq!(world.soft_body(id).unwrap().vertices(), before);

    let mut body = world.soft_body_mut(id).unwrap();
    for v in on_axes(limits::MAX_LINEAR_VELOCITY) {
        body.set_vertex_velocity(1, v).unwrap();
    }
    // Vertex 2 pinned, so vertex 1 alone may take the largest mass.
    body.set_vertex_inverse_mass(2, 0.0).unwrap();
    for inverse_mass in [0.0, limits::MAX_VERTEX_INVERSE_MASS, 1.0 / limits::MAX_MASS] {
        body.set_vertex_inverse_mass(1, inverse_mass).unwrap();
    }
    // Two vertices of the largest mass pass the total bound.
    assert!(body_invalid(
        body.set_vertex_inverse_mass(2, 1.0 / limits::MAX_MASS)
    ));
    let reach = Real::from(0.99 * limits::MAX_LINEAR_VELOCITY * dt);
    let near = RVec3::new(origin.x + reach, origin.y, origin.z);
    body.move_kinematic_vertex(0, near, dt).unwrap();
}

#[test]
fn soft_body_forces_are_bounded_by_the_acceleration_of_a_vertex() {
    let mut world = empty_world();
    // Four vertices of 1 kg: Jolt gives each vertex `F / 4` per kg.
    let shared = soft_square(1.0).build().unwrap();
    let id = world
        .create_soft_body(&shared, &SoftBodySettings::default())
        .unwrap();
    let bound = 4.0 * limits::MAX_ACCELERATION;
    let mut body = world.body_mut(id).unwrap();
    for value in NON_FINITE {
        assert!(body_invalid(body.add_force(Vec3::new(value, 0.0, 0.0))));
    }
    assert!(body_invalid(body.add_force(Vec3::new(
        bound.next_up(),
        0.0,
        0.0
    ))));
    body.add_force(Vec3::new(bound, 0.0, 0.0)).unwrap();
    // The accumulated force counts.
    assert!(body_invalid(body.add_force(Vec3::new(1.0e3, 0.0, 0.0))));
    step(&mut world, 2);
    for vertex in world.soft_body(id).unwrap().vertices() {
        let [x, y, z] = <[f32; 3]>::from(vertex.velocity);
        assert!(x.is_finite() && y.is_finite() && z.is_finite());
        assert!(length(vertex.velocity) <= limits::MAX_LINEAR_VELOCITY * 1.0001);
    }
}

#[test]
fn soft_body_explicit_constraints_are_bounded() {
    // Two vertices `length` apart joined by an edge, and a bend over that edge.
    let pair = |length: f32| {
        let vertices = vec![
            SoftBodyVertex::new(Vec3::ZERO),
            SoftBodyVertex::new(Vec3::new(length, 0.0, 0.0)),
            SoftBodyVertex::new(Vec3::new(0.0, 0.0, 1.0)),
            SoftBodyVertex::new(Vec3::new(0.0, 0.0, -1.0)),
        ];
        SoftBodySharedSettings::builder(vertices, Vec::new())
    };
    let edge = |length, compliance| {
        pair(length)
            .edge(SoftBodyEdge {
                vertices: [0, 1],
                compliance,
            })
            .build()
    };
    let bend = |length| {
        pair(length)
            .dihedral_bend(SoftBodyDihedralBend {
                vertices: [0, 1, 2, 3],
                compliance: 0.0,
            })
            .build()
    };
    let min = limits::MIN_SOFT_BODY_EDGE_LENGTH;
    assert!(edge(min, limits::MAX_COMPLIANCE).is_ok());
    assert!(bend(min).is_ok());
    assert!(soft_invalid(edge(min.next_down(), 0.0)));
    assert!(soft_invalid(bend(min.next_down())));
    for compliance in [-f32::MIN_POSITIVE, limits::MAX_COMPLIANCE.next_up()]
        .into_iter()
        .chain(NON_FINITE)
    {
        assert!(soft_invalid(edge(1.0, compliance)));
    }
}

/// The vertices of an 11 × 11 cloth of 1 kg vertices, 0.1 m apart, centred `distance` from the
/// body origin along (1, 0, 1), its first vertex kinematic when `pinned`.
fn offset_cloth_vertices(distance: f32, pinned: bool) -> Vec<SoftBodyVertex> {
    let offset = distance / 2.0_f32.sqrt();
    let mut vertices = common::soft_body::Cloth::new(11, 0.1).vertices;
    for vertex in &mut vertices {
        vertex.position = Vec3::new(
            vertex.position.x + offset,
            vertex.position.y,
            vertex.position.z + offset,
        );
    }
    if pinned {
        vertices[0].inverse_mass = 0.0;
    }
    vertices
}

/// Shared settings of free particles (no faces, no constraints).
fn particles(vertices: Vec<SoftBodyVertex>) -> SoftBodySharedSettings {
    SoftBodySharedSettings::builder(vertices, Vec::new())
        .build()
        .unwrap()
}

#[test]
fn soft_body_inertia_is_checked_at_creation() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let turned = quat_about(Vec3::new(0.6, 0.0, 0.8), 1.0);
    // Jolt sums the inertia about the body origin: a cloth near its origin is accepted, also
    // with a rotation Jolt bakes into the vertices, and decomposed without an assertion.
    for distance in [0.0, 1.0, 5.0, 10.0] {
        for rotation in [Quat::IDENTITY, turned] {
            let shared = particles(offset_cloth_vertices(distance, false));
            let id = world
                .create_soft_body(&shared, &SoftBodySettings::default().rotation(rotation))
                .unwrap();
            step(&mut world, 2);
            world.remove_body(id).unwrap();
        }
    }
    // Far from it (Jolt's decomposition asserted at 50 m) it is refused, changing nothing.
    for distance in [15.0, 50.0, 100.0, 1000.0] {
        let shared = particles(offset_cloth_vertices(distance, false));
        for rotation in [Quat::IDENTITY, turned] {
            let settings = SoftBodySettings::default().rotation(rotation);
            assert!(body_invalid(world.create_soft_body(&shared, &settings)));
        }
        assert_eq!(world.body_count(), 0);
    }
    // A kinematic vertex gives the body infinite inertia without a decomposition.
    let pinned = particles(offset_cloth_vertices(1000.0, true));
    world
        .create_soft_body(&pinned, &SoftBodySettings::default())
        .unwrap();
    step(&mut world, 2);
    // Free vertices on one line have no inertia about it: refused on an axis and off it.
    let line = |direction: [f32; 3]| {
        particles(
            (1..=4)
                .map(|i| {
                    let [x, y, z] = direction.map(|c| c * i as f32);
                    SoftBodyVertex::new(Vec3::new(x, y, z))
                })
                .collect(),
        )
    };
    for direction in [[1.0, 0.0, 0.0], [0.6, 0.8, 0.0], [0.48, 0.6, 0.64]] {
        let settings = SoftBodySettings::default();
        assert!(body_invalid(
            world.create_soft_body(&line(direction), &settings)
        ));
    }
    // One vertex at the origin: Jolt gives the zero tensor the inertia of a unit sphere.
    let single = particles(vec![SoftBodyVertex::new(Vec3::ZERO)]);
    world
        .create_soft_body(&single, &SoftBodySettings::default())
        .unwrap();
    // Eight vertices within 1 cm of (10, 0, 0) with masses from 2 g to 50 t, on which Jolt's
    // decomposition asserted, are refused.
    let cluster = [
        ([10.001002, 0.0021947764, -0.00030142843], 0.024279153),
        ([9.998484, -0.0011584508, 0.004818453], 2.1364938e-5),
        ([10.0024, -0.0009066194, 0.0030977945], 144.5319),
        ([9.998568, 0.00448157, -0.0006148457], 0.017587047),
        ([9.999195, -0.0018897026, 0.003035497], 0.02039532),
        ([10.001087, 0.002987452, 0.000934242], 0.23255065),
        ([9.996244, -0.0009508091, 0.004369754], 0.17152376),
        ([9.995533, 0.0035093676, 0.0048835487], 0.0006062472),
    ]
    .map(|([x, y, z], inverse_mass)| SoftBodyVertex {
        inverse_mass,
        ..SoftBodyVertex::new(Vec3::new(x, y, z))
    })
    .to_vec();
    assert!(body_invalid(world.create_soft_body(
        &particles(cluster),
        &SoftBodySettings::default()
    )));
    step(&mut world, 2);
}

#[test]
fn unpinning_checks_the_inertia_at_the_current_positions() {
    // A cloth authored 50 m from its origin is accepted while a vertex is pinned; unpinning it
    // is refused and changes nothing. Near its origin the same unpinning is accepted.
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    for (distance, accepted) in [(50.0, false), (1.0, true)] {
        let shared = particles(offset_cloth_vertices(distance, true));
        let id = world
            .create_soft_body(&shared, &SoftBodySettings::default())
            .unwrap();
        let before = world.soft_body(id).unwrap().vertices();
        let result = world
            .soft_body_mut(id)
            .unwrap()
            .set_vertex_inverse_mass(0, 1.0);
        if accepted {
            result.unwrap();
            let mass = world.body(id).unwrap().mass().unwrap();
            assert!((mass - 121.0).abs() < 1.0e-3, "{mass}");
            step(&mut world, 2);
        } else {
            assert!(body_invalid(result));
            assert_eq!(world.soft_body(id).unwrap().vertices(), before);
            assert_eq!(world.body(id).unwrap().mass(), None);
        }
        world.remove_body(id).unwrap();
    }

    // Kinematic vertices carried about 50 m from an origin that stays put
    // (`update_position(false)`): the last unpinning is checked at the positions then.
    for update_position in [false, true] {
        let mut world = empty_world();
        let shared = particles(
            common::soft_body::Cloth::new(4, 0.1)
                .vertices
                .into_iter()
                .map(|vertex| SoftBodyVertex::kinematic(vertex.position))
                .collect(),
        );
        let id = world
            .create_soft_body(
                &shared,
                &SoftBodySettings::default().update_position(update_position),
            )
            .unwrap();
        for velocity in [Vec3::new(60.0, 0.0, 80.0), Vec3::ZERO] {
            for index in 0..16 {
                let mut soft = world.soft_body_mut(id).unwrap();
                soft.set_vertex_velocity(index, velocity).unwrap();
            }
            step(&mut world, 30);
        }
        for index in 0..15 {
            let mut soft = world.soft_body_mut(id).unwrap();
            soft.set_vertex_inverse_mass(index, 1.0).unwrap();
        }
        let last = world
            .soft_body_mut(id)
            .unwrap()
            .set_vertex_inverse_mass(15, 1.0);
        if update_position {
            last.unwrap();
            step(&mut world, 2);
        } else {
            assert!(body_invalid(last));
            let vertices = world.soft_body(id).unwrap().vertices();
            assert_eq!(vertices[15].inverse_mass, 0.0);
        }
    }
}

/// A seeded generator of `f32` in [0, 1).
fn uniform(seed: &mut u32) -> f32 {
    *seed ^= *seed << 13;
    *seed ^= *seed >> 17;
    *seed ^= *seed << 5;
    (*seed >> 8) as f32 / (1u32 << 24) as f32
}

#[test]
fn soft_body_inertia_at_its_bound_decomposes() {
    // Seeded clouds of free particles, flat or stretched, with masses from 1 g to 1 t, moved
    // away from the body origin until the inertia rule refuses them. The last accepted offset
    // is created, also with baked rotations; in the asserts build Jolt decomposes each inertia
    // there without an assertion.
    let mut world = empty_world();
    let mut seed = 0x9e37_79b9_u32;
    let mut created = 0;
    for _ in 0..24 {
        let count = 3 + (uniform(&mut seed) * 30.0) as usize;
        let extents = [0; 3].map(|_| 10.0_f32.powf(uniform(&mut seed) * 4.0 - 3.0));
        let positions: Vec<[f32; 3]> = (0..count)
            .map(|_| extents.map(|e| e * (uniform(&mut seed) - 0.5)))
            .collect();
        let inverse_masses: Vec<f32> = (0..count)
            .map(|_| 10.0_f32.powf(uniform(&mut seed) * 6.0 - 3.0))
            .collect();
        let direction = [0; 3].map(|_| uniform(&mut seed) - 0.5);
        let length = direction
            .iter()
            .map(|c| c * c)
            .sum::<f32>()
            .sqrt()
            .max(1.0e-3);
        let at = |distance: f32| {
            particles(
                positions
                    .iter()
                    .zip(&inverse_masses)
                    .map(|(p, &inverse_mass)| {
                        let [x, y, z] = [0, 1, 2].map(|k| p[k] + direction[k] / length * distance);
                        SoftBodyVertex {
                            inverse_mass,
                            ..SoftBodyVertex::new(Vec3::new(x, y, z))
                        }
                    })
                    .collect(),
            )
        };
        let accepts = |world: &mut PhysicsWorld, distance: f32| match world
            .create_soft_body(&at(distance), &SoftBodySettings::default())
        {
            Ok(id) => {
                world.remove_body(id).unwrap();
                true
            }
            Err(BodyError::InvalidValue(_)) => false,
            Err(other) => panic!("{other:?}"),
        };
        let (mut low, mut high) = (0.0_f32, 0.5 * limits::MAX_SHAPE_EXTENT);
        if !accepts(&mut world, low) || accepts(&mut world, high) {
            continue;
        }
        while high - low > 1.0e-6 * high {
            let middle = 0.5 * (low + high);
            if accepts(&mut world, middle) {
                low = middle;
            } else {
                high = middle;
            }
        }
        for _ in 0..4 {
            let [x, y, z, w] = [0; 4].map(|_| uniform(&mut seed) - 0.5);
            let length = (x * x + y * y + z * z + w * w).sqrt();
            let rotation = Quat::from_xyzw(x / length, y / length, z / length, w / length);
            for rotation in [Quat::IDENTITY, rotation] {
                let settings = SoftBodySettings::default().rotation(rotation);
                if let Ok(id) = world.create_soft_body(&at(low), &settings) {
                    step(&mut world, 1);
                    world.remove_body(id).unwrap();
                    created += 1;
                }
            }
        }
    }
    assert!(created >= 40, "{created}");
}

/// A slender shape of half length `length` and relative thickness `thin`: a box of half
/// extents `thin · length`, `1.5 · thin · length` and `length` (`kind` 0), or a capsule (1) or a
/// cylinder (2) of half height `length` and radius `thin · length`.
fn slender_shape(kind: u32, length: f32, thin: f32) -> Shape {
    let width = thin * length;
    match kind {
        0 => Shape::new_box_with_convex_radius(Vec3::new(width, 1.5 * width, length), 0.0),
        1 => Shape::new_capsule(length, width),
        _ => Shape::new_cylinder_with_convex_radius(length, width, 0.0),
    }
    .unwrap()
}

/// `v` rotated by the unit quaternion `q`.
fn rotated(q: Quat, v: [f32; 3]) -> [f32; 3] {
    let u = [q.x, q.y, q.z];
    let cross = |a: [f32; 3], b: [f32; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let t = cross(u, v).map(|c| 2.0 * c);
    let ut = cross(u, t);
    [0, 1, 2].map(|k| v[k] + q.w * t[k] + ut[k])
}

/// A seeded unit quaternion.
fn seeded_rotation(seed: &mut u32) -> Quat {
    let [x, y, z, w] = [0; 4].map(|_| uniform(seed) - 0.5);
    let length = (x * x + y * y + z * z + w * w).sqrt();
    Quat::from_xyzw(x / length, y / length, z / length, w / length)
}

#[test]
fn rigid_body_inertia_at_its_bound_decomposes() {
    // Seeded slender boxes, capsules and cylinders, 2 cm to 20 m long, as a rotated compound
    // child, as two rotated children side by side along their length, and with a centre of mass
    // moved nearly along their length; dynamic with computed or overridden masses, and
    // kinematic. Each is made thinner until the inertia rule refuses it; the thinnest accepted
    // one is created and stepped, and one just thinner is refused. In the asserts build Jolt
    // decomposes each inertia there without an assertion.
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let mut seed = 0x2545_f491_u32;
    // Boundaries found, by shape kind (rows) and arrangement (columns).
    let mut boundaries = [[0_u32; 3]; 3];
    let mut accepted_at_the_thin_end = 0;
    for case in 0..72_u32 {
        let kind = case % 3;
        let arrangement = (case / 3) % 3;
        let length = 10.0_f32.powf(uniform(&mut seed) * 3.0 - 2.0);
        let rotation = seeded_rotation(&mut seed);
        let tilt_size = 0.2 * uniform(&mut seed);
        let tilt = [0; 3].map(|_| (uniform(&mut seed) - 0.5) * tilt_size);
        let settings = match case % 4 {
            0 => BodySettings::new_kinematic(),
            1 => BodySettings::new_dynamic().mass(10.0_f32.powf(uniform(&mut seed) * 6.0 - 3.0)),
            _ => BodySettings::new_dynamic().mass(1.0),
        };
        let shape = |thin: f32| {
            let leaf = slender_shape(kind, length, thin);
            let child = |position: Vec3, user_data| CompoundChild {
                shape: &leaf,
                position,
                rotation,
                user_data,
            };
            // The long axis of the leaf, rotated.
            let axis = if kind == 0 {
                [0.0, 0.0, 1.0]
            } else {
                [0.0, 1.0, 0.0]
            };
            let along = rotated(rotation, axis).map(|c| c * length);
            let offset = [0, 1, 2].map(|k| 0.5 * (along[k] + tilt[k] * length));
            match arrangement {
                0 => Shape::new_compound(&[child(Vec3::ZERO, 0)]),
                1 => Shape::new_compound(&[
                    child(Vec3::from(along), 0),
                    child(Vec3::from(along.map(|c| -c)), 1),
                ]),
                _ => Shape::new_offset_center_of_mass(
                    &Shape::new_compound(&[child(Vec3::ZERO, 0)]).unwrap(),
                    Vec3::from(offset),
                ),
            }
            .unwrap()
        };
        let create = |world: &mut PhysicsWorld, thin: f32| match world
            .create_body(&shape(thin), &settings)
        {
            Ok(id) => Some(id),
            Err(BodyError::InvalidValue(_)) => None,
            Err(other) => panic!("{other:?}"),
        };
        let mut accepts = |thin: f32| {
            let id = create(&mut world, thin);
            id.map(|id| world.remove_body(id).unwrap()).is_some()
        };
        let thinnest = if accepts(THINNEST) {
            // The part of the centre of mass offset across the long axis adds a moment about
            // that axis that does not shrink with the thickness, so the thinnest shape passes.
            assert_eq!(arrangement, 2, "case {case}: thinnest shape accepted");
            accepted_at_the_thin_end += 1;
            THINNEST
        } else {
            let (thinnest, refused) = thinnest_accepted(accepts);
            assert!(create(&mut world, refused).is_none(), "case {case}");
            // Jolt computes no mass properties for a static body, so the rule does not apply.
            let fixed = world
                .create_body(&shape(THINNEST), &BodySettings::new_static())
                .unwrap();
            world.remove_body(fixed).unwrap();
            boundaries[kind as usize][arrangement as usize] += 1;
            thinnest
        };
        let id = create(&mut world, thinnest).unwrap();
        step(&mut world, 2);
        world.remove_body(id).unwrap();
    }
    // Every kind in every arrangement reaches its boundary in several of its 8 cases.
    assert!(
        boundaries.iter().flatten().all(|&count| count >= 3),
        "{boundaries:?}, {accepted_at_the_thin_end} accepted at the thin end"
    );
}

/// The thinnest relative thickness the boundary tests try.
const THINNEST: f32 = 1.0e-4;

/// A compound with one child, the slender shape of `kind` (see [`slender_shape`]) of half
/// length `length` and relative thickness `thin`, rotated by `rotation` about its centre.
fn rotated_slender_child(kind: u32, length: f32, thin: f32, rotation: Quat) -> Shape {
    let leaf = slender_shape(kind, length, thin);
    Shape::new_compound(&[CompoundChild {
        shape: &leaf,
        position: Vec3::ZERO,
        rotation,
        user_data: 0,
    }])
    .unwrap()
}

/// The thinnest relative thickness between [`THINNEST`] and 0.5 that `accepts`, to a relative
/// 1e-4, and the thickness just below it that it refuses. Panics unless `accepts` takes 0.5 and
/// refuses [`THINNEST`].
fn thinnest_accepted(mut accepts: impl FnMut(f32) -> bool) -> (f32, f32) {
    let (mut low, mut high) = (THINNEST, 0.5_f32);
    assert!(accepts(high), "thick shape refused");
    assert!(!accepts(low), "thinnest shape accepted");
    while high - low > 1.0e-4 * high {
        let middle = 0.5 * (low + high);
        if accepts(middle) {
            high = middle;
        } else {
            low = middle;
        }
    }
    (high, low)
}

#[test]
fn inner_body_inertia_at_its_bound_decomposes() {
    // Jolt creates a character's inner body as a kinematic body of the inner body shape. Seeded
    // slender boxes, capsules and cylinders, 2 cm to 20 m long, in a rotated compound child are
    // made thinner until the inertia rule refuses the character; the thinnest accepted one is
    // created and stepped, and one just thinner is refused. In the asserts build Jolt decomposes
    // each inertia there without an assertion.
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let mut seed = 0x1b87_3593_u32;
    for case in 0..12_u32 {
        let kind = case % 3;
        let length = 10.0_f32.powf(uniform(&mut seed) * 3.0 - 2.0);
        let rotation = seeded_rotation(&mut seed);
        let create = |world: &mut PhysicsWorld, thin: f32| {
            let inner = rotated_slender_child(kind, length, thin, rotation);
            let settings = CharacterSettings::new(&capsule).inner_body(Some(InnerBody {
                shape: &inner,
                object_layer: ObjectLayer::MOVING,
            }));
            match world.create_character(&settings, RVec3::ZERO, Quat::IDENTITY) {
                Ok(id) => Some(id),
                Err(CharacterError::InvalidValue(_)) => None,
                Err(other) => panic!("{other:?}"),
            }
        };
        let (thinnest, refused) = thinnest_accepted(|thin| {
            let id = create(&mut world, thin);
            id.map(|id| world.remove_character(id).unwrap()).is_some()
        });
        let id = create(&mut world, thinnest).unwrap();
        step(&mut world, 2);
        world.remove_character(id).unwrap();
        assert!(create(&mut world, refused).is_none(), "case {case}");
    }
}

#[test]
fn stabilized_ragdoll_inertia_at_its_bound_decomposes() {
    // A chain of three parts, each a seeded slender box, capsule or cylinder in a rotated
    // compound child, with masses that `Stabilize` redistributes: it scales each part's inertia
    // to its new mass and decomposes it, then rebuilds the parents' with raised moments; the
    // leaf part keeps its scaled tensor. The parts are made thinner until the inertia rule
    // refuses the settings; the ragdoll at the thinnest accepted shape is created and stepped,
    // and one just thinner is refused. In the asserts build Jolt decomposes each inertia there
    // without an assertion.
    let skeleton = Skeleton::new(&[
        SkeletonJoint {
            name: "root",
            parent: None,
        },
        SkeletonJoint {
            name: "middle",
            parent: Some(0),
        },
        SkeletonJoint {
            name: "leaf",
            parent: Some(1),
        },
    ])
    .unwrap();
    let mut seed = 0x68e3_1da4_u32;
    for case in 0..6_u32 {
        let kind = case % 3;
        let length = 10.0_f32.powf(uniform(&mut seed) * 2.0 - 1.0);
        let rotations = [0; 3].map(|_| seeded_rotation(&mut seed));
        let (mut world, layers) = common::ragdoll::ragdoll_world(1);
        let build = |thin: f32| {
            let shapes =
                rotations.map(|rotation| rotated_slender_child(kind, length, thin, rotation));
            let parts: Vec<RagdollPart> = [8.0, 2.0, 0.5]
                .into_iter()
                .enumerate()
                .map(|(index, mass)| {
                    let height = 3.0 * length * index as f32;
                    let joint = HingeConstraintSettings::new(
                        common::math::rvec3([0.0, f64::from(height - 1.5 * length), 0.0]),
                        Vec3::new(0.0, 0.0, 1.0),
                        Vec3::new(1.0, 0.0, 0.0),
                    );
                    RagdollPart {
                        shape: &shapes[index],
                        body: BodySettings::new_dynamic()
                            .object_layer(layers.ragdoll)
                            .position(common::math::rvec3([0.0, f64::from(height), 0.0]))
                            .mass(mass),
                        joint: (index > 0).then_some(RagdollJoint::Hinge(joint)),
                    }
                })
                .collect();
            match RagdollSettings::new_stabilized(&skeleton, &parts) {
                Ok(settings) => Some(settings),
                Err(RagdollError::InvalidValue(_)) => None,
                Err(other) => panic!("{other:?}"),
            }
        };
        let (thinnest, refused) = thinnest_accepted(|thin| build(thin).is_some());
        let id = world
            .create_ragdoll(&build(thinnest).unwrap(), None, Activation::Activate)
            .unwrap();
        step(&mut world, 2);
        world.remove_ragdoll(id).unwrap();
        assert!(build(refused).is_none(), "case {case}");
    }
}

/// The unit square of [`soft_square`] at the world origin, every vertex skinned to joint 0
/// with `max_distance`.
fn skinned_square(world: &mut PhysicsWorld, max_distance: Option<f32>) -> BodyId {
    let mut builder =
        soft_square(1.0).inv_bind(SoftBodyInvBind::new(0, Vec3::ZERO, Quat::IDENTITY));
    for vertex in 0..4 {
        let mut weights = [SoftBodySkinWeight::default(); 4];
        weights[0] = SoftBodySkinWeight::new(0, 1.0);
        builder = builder.skinned(SoftBodySkinned {
            max_distance,
            ..SoftBodySkinned::new(vertex, weights)
        });
    }
    world
        .create_soft_body(&builder.build().unwrap(), &SoftBodySettings::default())
        .unwrap()
}

/// A pose of one unturned joint at `x` on the X axis.
fn joint_on_x(x: Real) -> SkeletonPose {
    SkeletonPose {
        root_offset: RVec3::new(x, 0.0, 0.0),
        joints: vec![JointTransform {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        }],
    }
}

fn assert_soft_body_finite(world: &PhysicsWorld, id: BodyId, what: &str) {
    for vertex in world.soft_body(id).unwrap().vertices() {
        let state = [v3(vertex.position), f3(vertex.velocity)];
        assert!(
            state.iter().flatten().all(|value| value.is_finite()),
            "{what}: {state:?}"
        );
    }
}

#[test]
fn soft_body_skinning_is_bounded() {
    // Skin joints within MAX_SHAPE_EXTENT of the centre of mass, which is at the origin before
    // the first step.
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let id = skinned_square(&mut world, Some(0.0));
    let extent = Real::from(limits::MAX_SHAPE_EXTENT);
    let beyond = Real::from(limits::MAX_SHAPE_EXTENT.next_up());
    let mut body = world.soft_body_mut(id).unwrap();
    assert!(body_invalid(body.skin_vertices(&joint_on_x(beyond))));
    assert!(body_invalid(body.hard_skin_vertices(&joint_on_x(-beyond))));
    body.hard_skin_vertices(&joint_on_x(extent)).unwrap();
    step(&mut world, 2);
    assert_soft_body_finite(&world, id, "skinned at the extent");
    // The multiplier is within 0..=MAX_RATIO; at the bound the longest max distance stays
    // finite.
    let mut world = common::world(Vec3::new(0.0, -9.81, 0.0), 1);
    let id = skinned_square(&mut world, Some(limits::MAX_SHAPE_EXTENT));
    let mut body = world.soft_body_mut(id).unwrap();
    for multiplier in [-f32::MIN_POSITIVE, limits::MAX_RATIO.next_up()]
        .into_iter()
        .chain(NON_FINITE)
    {
        assert!(body_invalid(
            body.set_skinned_max_distance_multiplier(multiplier)
        ));
    }
    for multiplier in [0.0, limits::MAX_RATIO] {
        body.set_skinned_max_distance_multiplier(multiplier)
            .unwrap();
        assert_eq!(body.skinned_max_distance_multiplier(), multiplier);
    }
    body.skin_vertices(&joint_on_x(0.5)).unwrap();
    step(&mut world, 30);
    assert_soft_body_finite(&world, id, "multiplier at the bound");
}
