//! Body ids, creation, access, removal and forces.

mod common;

use common::*;
use oxijolt::*;

fn bits3(v: Vec3) -> [u32; 3] {
    <[f32; 3]>::from(v).map(f32::to_bits)
}

// `Real` is already `f64` with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn bits_r3(v: RVec3) -> [u64; 3] {
    <[Real; 3]>::from(v).map(|c| f64::from(c).to_bits())
}

fn bits4(q: Quat) -> [u32; 4] {
    <[f32; 4]>::from(q).map(f32::to_bits)
}

/// Creates three cubes, removes the second and creates one more; returns the four ids.
fn id_history(world: &mut PhysicsWorld) -> [BodyId; 4] {
    let a = add_cube(world, RVec3::new(0.0, 0.0, 0.0));
    let b = add_cube(world, RVec3::new(3.0, 0.0, 0.0));
    let c = add_cube(world, RVec3::new(6.0, 0.0, 0.0));
    world.remove_body(b).unwrap();
    let d = add_cube(world, RVec3::new(9.0, 0.0, 0.0));
    [a, b, c, d]
}

#[test]
fn ids_follow_insertion_order() {
    let mut first = world(Vec3::ZERO, 1);
    let ids = id_history(&mut first);
    let [a, b, c, d] = ids;

    for (n, id) in (0u32..).zip([a, b, c]) {
        assert_eq!(id.index(), n);
        assert_eq!(id.sequence(), 1);
        assert_eq!(id.to_raw(), (1 << 23) | n);
    }
    assert_eq!(d.index(), 1, "the freed index is reused");
    assert_eq!(d.sequence(), 2);
    assert_eq!(d.to_raw(), (2 << 23) | 1);
    assert_ne!(b, d);

    let mut second = world(Vec3::ZERO, 1);
    let second_ids = id_history(&mut second);
    for (mine, theirs) in ids.iter().zip(&second_ids) {
        assert_eq!(mine.to_raw(), theirs.to_raw());
        assert_ne!(mine, theirs, "ids of different worlds are different");
    }
}

#[test]
fn body_ids_list_every_body_in_id_order() {
    let mut world = world(Vec3::ZERO, 1);
    assert_eq!(world.body_ids().next(), None);
    let [a, _, c, d] = id_history(&mut world);
    // `d` reuses the freed index 1 with sequence 2, so its raw id sorts after `c`'s.
    assert!(world.body_ids().eq([a, c, d]));

    let inner_shape = Shape::new_capsule(0.6, 0.3).unwrap();
    let settings = CharacterSettings::humanoid(1.8, 0.3)
        .unwrap()
        .inner_body(Some(InnerBody {
            shape: &inner_shape,
            object_layer: ObjectLayer::MOVING,
        }));
    world
        .create_character(&settings, RVec3::new(20.0, 0.0, 0.0), Quat::IDENTITY)
        .unwrap();
    let ids: Vec<BodyId> = world.body_ids().collect();
    assert_eq!(ids.len() as u32, world.body_count());
    assert!(ids.windows(2).all(|pair| pair[0] < pair[1]));
    let inner: Vec<BodyId> = ids
        .iter()
        .copied()
        .filter(|id| ![a, c, d].contains(id))
        .collect();
    assert_eq!(inner.len(), 1, "the character's inner body is listed");
    assert!(world.body(inner[0]).is_ok());
}

#[test]
fn foreign_and_stale_ids_are_rejected() {
    let mut a = world(Vec3::ZERO, 1);
    let mut b = world(Vec3::ZERO, 1);
    let in_a = add_cube(&mut a, RVec3::new(0.0, 0.0, 0.0));
    let in_b = add_cube(&mut b, RVec3::new(0.0, 0.0, 0.0));
    assert_eq!(in_a.to_raw(), in_b.to_raw());

    assert!(!b.contains_body(in_a));
    assert_eq!(b.body(in_a).err(), Some(BodyError::WrongWorld(in_a)));
    assert_eq!(b.body_mut(in_a).err(), Some(BodyError::WrongWorld(in_a)));
    assert_eq!(b.remove_body(in_a), Err(BodyError::WrongWorld(in_a)));
    assert_eq!(b.body_count(), 1);

    a.remove_body(in_a).unwrap();
    assert_eq!(a.body_count(), 0);
    assert!(!a.contains_body(in_a));
    assert_eq!(a.body(in_a).err(), Some(BodyError::NotFound(in_a)));
    assert_eq!(a.body_mut(in_a).err(), Some(BodyError::NotFound(in_a)));
    assert_eq!(a.remove_body(in_a), Err(BodyError::NotFound(in_a)));

    // The new body takes the old index; the old id must still not resolve to it.
    let reused = add_cube(&mut a, RVec3::new(1.0, 2.0, 3.0));
    assert_eq!(reused.index(), in_a.index());
    assert!(!a.contains_body(in_a));
    assert_eq!(a.body(in_a).err(), Some(BodyError::NotFound(in_a)));
    assert_eq!(a.remove_body(in_a), Err(BodyError::NotFound(in_a)));
    assert!(a.contains_body(reused));
    assert_eq!(
        a.body(reused).unwrap().position(),
        RVec3::new(1.0, 2.0, 3.0)
    );
    assert_eq!(a.body_count(), 1);
}

#[test]
fn pose_and_velocity_read_back_as_exact_bits() {
    let mut world = world(Vec3::ZERO, 1);
    let position = RVec3::new(1.25, 3.1, -2.7);
    let rotation = Quat::from_xyzw(0.5, -0.5, 0.5, 0.5);
    let linear = Vec3::new(0.3, -1.7, 2.9);
    let angular = Vec3::new(-0.11, 0.7, 1.3);

    let id = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(position)
                .rotation(rotation)
                .linear_velocity(linear)
                .angular_velocity(angular),
        )
        .unwrap();
    let body = world.body(id).unwrap();
    assert_eq!(bits_r3(body.position()), bits_r3(position));
    assert_eq!(bits4(body.rotation()), bits4(rotation));
    assert_eq!(bits3(body.linear_velocity()), bits3(linear));
    assert_eq!(bits3(body.angular_velocity()), bits3(angular));
    assert_eq!(body.motion_type(), MotionType::Dynamic);

    let half = std::f32::consts::FRAC_1_SQRT_2;
    let position = RVec3::new(-7.3, 0.9, 11.1);
    let rotation = Quat::from_xyzw(0.0, half, 0.0, half);
    let linear = Vec3::new(-4.1, 0.2, 0.01);
    let angular = Vec3::new(2.3, -0.4, 0.6);
    let mut body = world.body_mut(id).unwrap();
    body.set_position(position, Activation::Activate).unwrap();
    body.set_rotation(rotation, Activation::Activate).unwrap();
    body.set_linear_velocity(linear).unwrap();
    body.set_angular_velocity(angular).unwrap();
    assert_eq!(bits_r3(body.position()), bits_r3(position));
    assert_eq!(bits4(body.rotation()), bits4(rotation));
    assert_eq!(bits3(body.linear_velocity()), bits3(linear));
    assert_eq!(bits3(body.angular_velocity()), bits3(angular));

    let position = RVec3::new(0.7, -0.3, 5.5);
    let rotation = Quat::from_xyzw(-0.5, -0.5, -0.5, 0.5);
    body.set_position_and_rotation(position, rotation, Activation::DontActivate)
        .unwrap();
    assert_eq!(bits_r3(body.position()), bits_r3(position));
    assert_eq!(bits4(body.rotation()), bits4(rotation));
}

#[test]
fn activation_decides_whether_a_pose_write_wakes_the_body() {
    let mut world = world(Vec3::ZERO, 1);
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic().activation(Activation::DontActivate),
        )
        .unwrap();
    assert!(!world.body(cube).unwrap().is_active());

    let mut body = world.body_mut(cube).unwrap();
    body.set_position(RVec3::new(1.0, 0.0, 0.0), Activation::DontActivate)
        .unwrap();
    body.set_position_and_rotation(
        RVec3::new(2.0, 0.0, 0.0),
        Quat::IDENTITY,
        Activation::DontActivate,
    )
    .unwrap();
    assert!(!body.is_active());
    body.set_rotation(Quat::from_xyzw(0.0, 1.0, 0.0, 0.0), Activation::Activate)
        .unwrap();
    assert!(body.is_active());
}

#[test]
fn sleeping_flag_is_readable() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let floor = add_floor(&mut world);
    let sleeper = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    let insomniac = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(5.0, 0.5, 0.0))
                .allow_sleeping(false),
        )
        .unwrap();

    let floor_body = world.body(floor).unwrap();
    assert!(!floor_body.is_sleeping());
    assert!(!floor_body.is_active());
    assert!(world.body(sleeper).unwrap().is_active());

    let mut slept = false;
    for _ in 0..180 {
        assert!(world.step(DT).unwrap().is_complete());
        slept |= world.body(sleeper).unwrap().is_sleeping();
        assert!(!world.body(insomniac).unwrap().is_sleeping());
    }
    assert!(slept, "the resting cube never fell asleep");
    assert!(world.body(sleeper).unwrap().is_sleeping());
    assert!(!world.body(floor).unwrap().is_sleeping());
}

#[test]
fn removal_wakes_bodies_resting_on_it() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    let bottom = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    let top = add_cube(&mut world, RVec3::new(0.0, 1.5, 0.0));

    let mut ticks = 0;
    while !(world.body(bottom).unwrap().is_sleeping() && world.body(top).unwrap().is_sleeping()) {
        assert!(world.step(DT).unwrap().is_complete());
        ticks += 1;
        assert!(ticks < 600, "the stack never fell asleep");
    }

    world.remove_body(bottom).unwrap();
    assert!(world.body(top).unwrap().is_active());
}

/// A sleeper resting within the speculative contact distance of the removed body is in Jolt's
/// contact range without touching its bounds; it wakes and falls.
#[test]
fn removal_wakes_a_sleeper_within_speculative_distance() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let floor = add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    common::controls::fall_asleep(&mut world, cube);
    let gap = 0.5 * world.speculative_contact_distance();
    let lifted = RVec3::new(0.0, 0.5 + Real::from(gap), 0.0);
    world
        .body_mut(cube)
        .unwrap()
        .set_position(lifted, Activation::DontActivate)
        .unwrap();
    assert!(world.body(cube).unwrap().is_sleeping());

    world.remove_body(floor).unwrap();
    assert!(world.body(cube).unwrap().is_active());
    step(&mut world, 30);
    assert!(
        world.body(cube).unwrap().position().y < 0.0,
        "the cube falls"
    );
}

/// Two sleeping cubes 3 m apart are moved by a rebase, which widens the broad phase's stored
/// bounds until its next maintenance; then the left one is removed. Returns whether the right
/// cube, 2 m outside the removed cube's bounds, was woken.
fn removal_wakes_the_far_cube(optimize_before_removal: bool) -> bool {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let floor = add_floor(&mut world);
    let left = add_cube(&mut world, RVec3::new(-3.0, 0.5, 0.0));
    let right = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    let mut ticks = 0;
    while !(world.body(left).unwrap().is_sleeping() && world.body(right).unwrap().is_sleeping()) {
        assert!(world.step(DT).unwrap().is_complete());
        ticks += 1;
        assert!(ticks < 600, "the cubes never fell asleep");
    }
    world
        .rebase(
            &[floor, left, right],
            Quat::IDENTITY,
            RVec3::new(3.0, 0.0, 0.0),
        )
        .unwrap();
    assert!(world.body(right).unwrap().is_sleeping());
    if optimize_before_removal {
        world.optimize_broad_phase();
    }
    world.remove_body(left).unwrap();
    !world.body(right).unwrap().is_sleeping()
}

#[test]
fn removal_wakes_the_same_bodies_whether_or_not_the_broad_phase_was_optimized() {
    for optimize in [false, true] {
        assert!(
            !removal_wakes_the_far_cube(optimize),
            "optimized {optimize}"
        );
    }
}

#[test]
fn invalid_body_settings_are_rejected() {
    let mut world = world(Vec3::ZERO, 1);
    let shape = cube_shape();
    let layer = ObjectLayer::new(2);
    assert_eq!(
        world.create_body(&shape, &BodySettings::new_dynamic().object_layer(layer)),
        Err(BodyError::UnknownObjectLayer(layer))
    );
    let invalid = [
        BodySettings::new_dynamic().position(RVec3::new(0.0, Real::NAN, 0.0)),
        BodySettings::new_dynamic().rotation(Quat::from_xyzw(0.0, 0.0, 0.0, 2.0)),
        BodySettings::new_dynamic().rotation(Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0)),
        BodySettings::new_dynamic().linear_velocity(Vec3::new(f32::INFINITY, 0.0, 0.0)),
        BodySettings::new_dynamic().angular_velocity(Vec3::new(0.0, f32::NAN, 0.0)),
        BodySettings::new_dynamic().mass(0.0),
        BodySettings::new_dynamic().mass(f32::NAN),
        BodySettings::new_dynamic().friction(-0.1),
        BodySettings::new_dynamic().restitution(f32::NAN),
        BodySettings::new_dynamic().gravity_factor(f32::INFINITY),
    ];
    for settings in &invalid {
        assert!(
            matches!(
                world.create_body(&shape, settings),
                Err(BodyError::InvalidValue(_))
            ),
            "{settings:?} was accepted"
        );
    }
    assert_eq!(world.body_count(), 0);

    let position = RVec3::new(1.0, 2.0, 3.0);
    let id = world
        .create_body(&shape, &BodySettings::new_dynamic().position(position))
        .unwrap();
    let nan = Vec3::new(f32::NAN, 0.0, 0.0);
    let not_unit = Quat::from_xyzw(0.0, 0.0, 0.0, 0.5);
    let mut body = world.body_mut(id).unwrap();
    assert!(body
        .set_position(RVec3::new(Real::NAN, 0.0, 0.0), Activation::Activate)
        .is_err());
    assert!(body.set_rotation(not_unit, Activation::Activate).is_err());
    assert!(body
        .set_position_and_rotation(position, not_unit, Activation::Activate)
        .is_err());
    assert!(body.set_linear_velocity(nan).is_err());
    assert!(body.set_angular_velocity(nan).is_err());
    assert!(body.add_force(nan).is_err());
    assert!(body.add_force_at_point(nan, position).is_err());
    assert!(body
        .add_force_at_point(Vec3::ZERO, RVec3::new(0.0, Real::INFINITY, 0.0))
        .is_err());
    assert!(body.add_torque(nan).is_err());

    assert_eq!(body.position(), position);
    assert_eq!(body.rotation(), Quat::IDENTITY);
    assert_eq!(body.linear_velocity(), Vec3::ZERO);
    assert_eq!(body.angular_velocity(), Vec3::ZERO);
    assert!(world.step(DT).unwrap().is_complete());
    let body = world.body(id).unwrap();
    assert_eq!(body.linear_velocity(), Vec3::ZERO, "no force was applied");
    assert_eq!(body.angular_velocity(), Vec3::ZERO, "no torque was applied");
}

#[test]
fn forces() {
    const MASS: f32 = 2.5;
    // Jolt's default linear damping, applied after the force: v *= 1 - c * dt.
    const LINEAR_DAMPING: f32 = 0.05;
    let mut world = world(Vec3::ZERO, 1);
    let shape = cube_shape();
    let heavy = BodySettings::new_dynamic().mass(MASS);
    let at_x = |x| heavy.clone().position(RVec3::new(x, 0.0, 0.0));
    let pushed = world.create_body(&shape, &heavy).unwrap();
    let spun = world.create_body(&shape, &at_x(5.0)).unwrap();
    let twisted = world.create_body(&shape, &at_x(10.0)).unwrap();
    let reset = world.create_body(&shape, &at_x(15.0)).unwrap();

    let force = Vec3::new(10.0, 0.0, -4.0);
    world.body_mut(pushed).unwrap().add_force(force).unwrap();
    world
        .body_mut(spun)
        .unwrap()
        .add_force_at_point(Vec3::new(10.0, 0.0, 0.0), RVec3::new(5.0, 0.5, 0.0))
        .unwrap();
    world
        .body_mut(twisted)
        .unwrap()
        .add_torque(Vec3::new(0.0, 3.0, 0.0))
        .unwrap();
    let mut body = world.body_mut(reset).unwrap();
    body.add_force(force).unwrap();
    body.add_torque(Vec3::new(1.0, 2.0, 3.0)).unwrap();
    body.reset_forces();
    assert!(world.step(DT).unwrap().is_complete());

    let velocity = world.body(pushed).unwrap().linear_velocity();
    for (actual, f) in [
        (velocity.x, force.x),
        (velocity.y, force.y),
        (velocity.z, force.z),
    ] {
        let expected = f / MASS * DT * (1.0 - LINEAR_DAMPING * DT);
        assert!(
            (actual - expected).abs() <= 1e-5 * expected.abs(),
            "velocity {actual}, expected {expected}"
        );
    }

    let spun = world.body(spun).unwrap();
    assert!(spun.linear_velocity().x > 0.0);
    assert!(
        spun.angular_velocity().z < 0.0,
        "a force along +x above the centre turns the body about -z"
    );
    assert!(world.body(twisted).unwrap().angular_velocity().y > 0.0);

    let reset = world.body(reset).unwrap();
    assert_eq!(bits3(reset.linear_velocity()), [0; 3]);
    assert_eq!(bits3(reset.angular_velocity()), [0; 3]);
}

#[test]
fn reset_forces_ignores_static_and_kinematic_bodies() {
    let mut world = world(Vec3::ZERO, 1);
    let shape = cube_shape();
    let floor = world
        .create_body(&shape, &BodySettings::new_static())
        .unwrap();
    let velocity = Vec3::new(1.0, 0.0, 0.0);
    let kinematic = world
        .create_body(
            &shape,
            &BodySettings::new_kinematic()
                .position(RVec3::new(5.0, 0.0, 0.0))
                .linear_velocity(velocity),
        )
        .unwrap();
    world.body_mut(floor).unwrap().reset_forces();
    world.body_mut(kinematic).unwrap().reset_forces();
    assert!(world.step(DT).unwrap().is_complete());

    let kinematic = world.body(kinematic).unwrap();
    assert_eq!(kinematic.linear_velocity(), velocity);
    assert_eq!(kinematic.motion_type(), MotionType::Kinematic);
    assert!(kinematic.position().x > 5.0);
    assert_eq!(world.body(floor).unwrap().position(), RVec3::ZERO);
}

#[test]
fn shape_can_be_dropped_after_body_creation() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    let id = {
        let shape = Shape::new_sphere(0.5).unwrap();
        world
            .create_body(
                &shape,
                &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
            )
            .unwrap()
    };
    step(&mut world, 120);
    let y = world.body(id).unwrap().position().y;
    assert!(
        (0.45..0.55).contains(&y),
        "the sphere rests on the floor, y = {y}"
    );
}

#[test]
fn mass_too_small_to_invert_is_rejected() {
    let mut world = world(Vec3::ZERO, 1);
    let cube = cube_shape();
    let tiny_sphere = Shape::new_sphere(1e-20).unwrap();
    // Its computed inertia about the long axis underflows to zero while the other two do not;
    // scaled to 1 kg, the long axis stays zero and has no finite inverse. Its own computed
    // mass is also below `limits::MIN_MASS`.
    let needle = Shape::new_box(Vec3::new(limits::MAX_SHAPE_EXTENT, 1e-17, 1e-17)).unwrap();
    let rejected = [
        (&cube, BodySettings::new_dynamic().mass(f32::from_bits(1))),
        (&cube, BodySettings::new_dynamic().mass(1e-39)),
        (&cube, BodySettings::new_dynamic().mass(1e-6)),
        (&tiny_sphere, BodySettings::new_dynamic()),
        (&tiny_sphere, BodySettings::new_kinematic()),
        (&needle, BodySettings::new_dynamic()),
        (&needle, BodySettings::new_dynamic().mass(1.0)),
    ];
    for (shape, settings) in &rejected {
        assert!(
            matches!(
                world.create_body(shape, settings),
                Err(BodyError::InvalidValue(_))
            ),
            "{settings:?} was accepted"
        );
    }
    assert_eq!(world.body_count(), 0);

    // Static bodies have no mass, so any shape is fine.
    let wall = world
        .create_body(&tiny_sphere, &BodySettings::new_static())
        .unwrap();
    assert_eq!(wall.to_raw(), 1 << 23, "rejected bodies used no id");

    let light = world
        .create_body(&cube, &BodySettings::new_dynamic().mass(limits::MIN_MASS))
        .unwrap();
    let mut body = world.body_mut(light).unwrap();
    body.add_force(Vec3::new(1.0, 0.0, 0.0)).unwrap();
    body.add_torque(Vec3::new(0.0, 1e-6, 0.0)).unwrap();
    assert!(world.step(DT).unwrap().is_complete());
    let body = world.body(light).unwrap();
    for value in [body.linear_velocity(), body.angular_velocity()] {
        assert!(
            value.x.is_finite() && value.y.is_finite() && value.z.is_finite(),
            "{value:?}"
        );
    }
}

#[test]
fn full_world_rejects_another_body() {
    let mut world = PhysicsWorld::new(WorldSettings::default().max_bodies(1)).unwrap();
    let shape = cube_shape();
    let position = RVec3::new(1.0, 2.0, 3.0);
    let first = world
        .create_body(&shape, &BodySettings::new_static().position(position))
        .unwrap();
    assert_eq!(
        world.create_body(&shape, &BodySettings::new_dynamic()),
        Err(BodyError::TooManyBodies)
    );
    assert_eq!(world.body_count(), 1);
    assert_eq!(world.body(first).unwrap().position(), position);
}

#[test]
fn gravity_factor_scales_the_fall() {
    let fall_speed = |factor: f32| {
        let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
        let id = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_dynamic().gravity_factor(factor),
            )
            .unwrap();
        step(&mut world, 30);
        world.body(id).unwrap().linear_velocity().y
    };
    let (full, none, half) = (fall_speed(1.0), fall_speed(0.0), fall_speed(0.5));
    assert!(full < -4.0, "full gravity fell at {full}");
    assert_eq!(none, 0.0);
    assert!(
        (half - full / 2.0).abs() < 1e-4,
        "half {half} vs full {full}"
    );
}
