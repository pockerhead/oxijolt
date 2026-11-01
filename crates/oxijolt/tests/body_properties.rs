//! Body properties changed on a live body: friction, restitution, gravity factor, damping and
//! motion quality read back as set, act on the next steps, wake nothing, and are refused on the
//! bodies Jolt keeps them from.

mod common;

use common::soft_body::Cloth;
use common::{add_cube, add_floor, step, world, DT};
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// A 4 x 4 cloth without constraints, every vertex free.
fn add_cloth(world: &mut PhysicsWorld, settings: SoftBodySettings) -> BodyId {
    let shared = Cloth::new(4, 0.2).builder().build().unwrap();
    world.create_soft_body(&shared, &settings).unwrap()
}

/// The mean vertex position of soft body `id`.
fn mean_position(world: &PhysicsWorld, id: BodyId) -> [f64; 3] {
    let vertices = world.soft_body(id).unwrap().vertices();
    let mut sum = [0.0; 3];
    for vertex in &vertices {
        let p = vertex.position;
        for (total, value) in sum.iter_mut().zip([p.x, p.y, p.z]) {
            *total += wide(value);
        }
    }
    sum.map(|total| total / vertices.len() as f64)
}

/// `value` as `f64`.
// `Real` is `f64` with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn wide(value: Real) -> f64 {
    f64::from(value)
}

/// Sets every value property of `id` and checks that each reads back exactly.
fn set_and_read_back(world: &mut PhysicsWorld, id: BodyId, rigid: bool) {
    let mut body = world.body_mut(id).unwrap();
    body.set_friction(0.7).unwrap();
    body.set_restitution(0.4).unwrap();
    body.set_gravity_factor(-0.5).unwrap();
    body.set_linear_damping(0.3).unwrap();
    assert_eq!(body.friction(), 0.7);
    assert_eq!(body.restitution(), 0.4);
    assert_eq!(body.gravity_factor(), Some(-0.5));
    assert_eq!(body.linear_damping(), Some(0.3));
    if rigid {
        body.set_angular_damping(0.6).unwrap();
        assert_eq!(body.angular_damping(), Some(0.6));
        for quality in [MotionQuality::LinearCast, MotionQuality::Discrete] {
            body.set_motion_quality(quality).unwrap();
            assert_eq!(body.motion_quality(), Some(quality));
        }
    }
}

#[test]
fn values_read_back_as_set() {
    let mut world = world(GRAVITY, 1);
    let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let dynamic = add_cube(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let body = world.body(dynamic).unwrap();
    assert_eq!(
        (
            body.friction(),
            body.restitution(),
            body.gravity_factor(),
            body.linear_damping(),
            body.angular_damping(),
            body.motion_quality()
        ),
        (
            0.2,
            0.0,
            Some(1.0),
            Some(0.05),
            Some(0.05),
            Some(MotionQuality::Discrete)
        ),
        "Jolt's defaults"
    );
    set_and_read_back(&mut world, dynamic, true);

    let kinematic = world
        .create_body(
            &cube,
            &BodySettings::new_kinematic().position(RVec3::new(3.0, 5.0, 0.0)),
        )
        .unwrap();
    set_and_read_back(&mut world, kinematic, true);

    // A static body that may move keeps the quality it was created with, which Jolt's own
    // interface getter would report as `Discrete`.
    let may_move = world
        .create_body(
            &cube,
            &BodySettings::new_static()
                .allow_dynamic_or_kinematic(true)
                .motion_quality(MotionQuality::LinearCast)
                .position(RVec3::new(6.0, 5.0, 0.0)),
        )
        .unwrap();
    assert_eq!(
        world.body(may_move).unwrap().motion_quality(),
        Some(MotionQuality::LinearCast)
    );
    set_and_read_back(&mut world, may_move, true);

    let cloth = add_cloth(
        &mut world,
        SoftBodySettings::default().position(RVec3::new(9.0, 5.0, 0.0)),
    );
    assert_eq!(world.body(cloth).unwrap().linear_damping(), Some(0.1));
    set_and_read_back(&mut world, cloth, false);
}

#[test]
fn kinds_without_motion_properties_read_none() {
    let mut world = world(GRAVITY, 1);
    let floor = add_floor(&mut world);
    let mut body = world.body_mut(floor).unwrap();
    assert_eq!(
        (
            body.gravity_factor(),
            body.linear_damping(),
            body.angular_damping(),
            body.motion_quality()
        ),
        (None, None, None, None)
    );
    // Materials apply to every body.
    body.set_friction(0.9).unwrap();
    body.set_restitution(0.1).unwrap();
    assert_eq!((body.friction(), body.restitution()), (0.9, 0.1));

    let cloth = add_cloth(&mut world, SoftBodySettings::default());
    let body = world.body(cloth).unwrap();
    assert_eq!(
        (body.angular_damping(), body.motion_quality()),
        (None, None)
    );
}

/// A unit cube resting on the floor at `z`, sliding along +x at 5 m/s, without friction or
/// damping.
fn add_slider(world: &mut PhysicsWorld, z: Real) -> BodyId {
    world
        .create_body(
            &Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.5, z))
                .linear_velocity(Vec3::new(5.0, 0.0, 0.0))
                .friction(0.0)
                .linear_damping(0.0)
                .angular_damping(0.0),
        )
        .unwrap()
}

#[test]
fn friction_change_stops_a_sliding_box() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let braked = add_slider(&mut world, 0.0);
    let control = add_slider(&mut world, 3.0);
    step(&mut world, 10);
    for id in [braked, control] {
        let speed = world.body(id).unwrap().linear_velocity().x;
        assert!(speed > 4.9, "slides freely before the change: {speed}");
    }
    // Combined with the floor's default 0.2: sqrt(0.2) decelerates by about 4.4 m/s².
    world.body_mut(braked).unwrap().set_friction(1.0).unwrap();
    step(&mut world, 120);
    let stopped = world.body(braked).unwrap().linear_velocity();
    assert!(common::length(stopped) < 0.05, "braked box: {stopped:?}");
    let sliding = world.body(control).unwrap().linear_velocity().x;
    assert!(sliding > 4.9, "control: {sliding}");
}

/// The highest point of a ball dropped from 3 m whose restitution becomes `restitution` before
/// it lands, over ticks 50 to 120 (it lands at about tick 43).
fn rebound_height(restitution: f32) -> Real {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let ball = world
        .create_body(
            &Shape::new_sphere(0.5).unwrap(),
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 3.0, 0.0)),
        )
        .unwrap();
    step(&mut world, 5);
    world
        .body_mut(ball)
        .unwrap()
        .set_restitution(restitution)
        .unwrap();
    step(&mut world, 45);
    let mut highest = Real::MIN;
    for _ in 50..120 {
        step(&mut world, 1);
        highest = highest.max(world.body(ball).unwrap().position().y);
    }
    highest
}

#[test]
fn restitution_change_makes_a_ball_bounce() {
    let dead = rebound_height(0.0);
    let bouncy = rebound_height(0.8);
    assert!(dead < 0.6, "no bounce without restitution: {dead}");
    // About 7 m/s at impact, so 0.8 brings the centre back up about 1.6 m.
    assert!(bouncy > 1.5, "the ball bounced to {bouncy} only");
}

#[test]
fn gravity_factor_change_mid_flight() {
    let mut world = world(GRAVITY, 1);
    let shape = Shape::new_sphere(0.5).unwrap();
    let falling = |x| {
        BodySettings::new_dynamic()
            .position(RVec3::new(x, 100.0, 0.0))
            .linear_damping(0.0)
    };
    let [control, floating, rising] =
        [0.0, 3.0, 6.0].map(|x| world.create_body(&shape, &falling(x)).unwrap());
    step(&mut world, 10);
    let before = [control, floating, rising].map(|id| world.body(id).unwrap().linear_velocity());
    world
        .body_mut(floating)
        .unwrap()
        .set_gravity_factor(0.0)
        .unwrap();
    world
        .body_mut(rising)
        .unwrap()
        .set_gravity_factor(-1.0)
        .unwrap();
    step(&mut world, 20);
    let after = [control, floating, rising].map(|id| world.body(id).unwrap().linear_velocity());
    let gained = 20.0 * DT * 9.81;
    assert!(
        (after[0].y - before[0].y + gained).abs() < 1e-3,
        "{after:?}"
    );
    assert_eq!(
        after[1], before[1],
        "no gravity, no damping: the velocity stays"
    );
    assert!(
        (after[2].y - before[2].y - gained).abs() < 1e-3,
        "{after:?}"
    );
}

#[test]
fn value_setters_wake_nothing() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let sleeper = world
        .create_body(
            &Shape::new_sphere(0.5).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 10.0, 0.0))
                .activation(Activation::DontActivate),
        )
        .unwrap();
    let mut body = world.body_mut(sleeper).unwrap();
    body.set_friction(0.5).unwrap();
    body.set_restitution(0.5).unwrap();
    body.set_gravity_factor(2.0).unwrap();
    body.set_linear_damping(0.0).unwrap();
    body.set_angular_damping(0.0).unwrap();
    body.set_motion_quality(MotionQuality::LinearCast).unwrap();
    assert!(body.is_sleeping());
    step(&mut world, 10);
    assert!(world.body(sleeper).unwrap().is_sleeping());
    assert_eq!(world.body(sleeper).unwrap().position().y, 10.0);
    assert!(world.take_events().activations.is_empty());

    // Woken, it falls with the factor set while it slept: twice the world's gravity.
    world.body_mut(sleeper).unwrap().activate();
    step(&mut world, 1);
    let fall = world.body(sleeper).unwrap().linear_velocity().y;
    assert!((fall + 2.0 * 9.81 * DT).abs() < 1e-4, "{fall}");
}

#[test]
fn damping_slows_as_jolt_integrates() {
    let mut world = world(Vec3::ZERO, 1);
    let ball = world
        .create_body(
            &Shape::new_sphere(0.5).unwrap(),
            &BodySettings::new_dynamic()
                .linear_velocity(Vec3::new(2.0, 0.0, 0.0))
                .angular_velocity(Vec3::new(0.0, 3.0, 0.0))
                .linear_damping(0.0)
                .angular_damping(0.0),
        )
        .unwrap();
    let mut body = world.body_mut(ball).unwrap();
    body.set_linear_damping(6.0).unwrap();
    body.set_angular_damping(12.0).unwrap();
    step(&mut world, 1);
    // `MotionProperties::ApplyForceTorqueAndDragInternal`: v *= max(0, 1 - c * dt).
    let body = world.body(ball).unwrap();
    let (linear, angular) = (body.linear_velocity().x, body.angular_velocity().y);
    assert!((linear - 2.0 * (1.0 - 6.0 * DT)).abs() < 1e-6, "{linear}");
    assert!(
        (angular - 3.0 * (1.0 - 12.0 * DT)).abs() < 1e-6,
        "{angular}"
    );

    // A damping above 1 / dt stops the body in one step.
    world
        .body_mut(ball)
        .unwrap()
        .set_linear_damping(1000.0)
        .unwrap();
    step(&mut world, 1);
    assert_eq!(world.body(ball).unwrap().linear_velocity(), Vec3::ZERO);
}

#[test]
fn soft_body_uses_its_live_gravity_damping_and_friction() {
    // Gravity factor and damping: three cloths fall from the same height; after the change one
    // feels a quarter of the gravity and one is damped hard.
    let mut world = world(GRAVITY, 1);
    let [plain, light, damped] = [0.0, 3.0, 6.0].map(|x| {
        add_cloth(
            &mut world,
            SoftBodySettings::default().position(RVec3::new(x, 50.0, 0.0)),
        )
    });
    step(&mut world, 5);
    world
        .body_mut(light)
        .unwrap()
        .set_gravity_factor(0.25)
        .unwrap();
    world
        .body_mut(damped)
        .unwrap()
        .set_linear_damping(30.0)
        .unwrap();
    let ids = [plain, light, damped];
    let start = ids.map(|id| mean_position(&world, id)[1]);
    step(&mut world, 30);
    let drop: [f64; 3] = std::array::from_fn(|i| start[i] - mean_position(&world, ids[i])[1]);
    assert!(drop[1] < 0.5 * drop[0], "lighter: {drop:?}");
    assert!(drop[2] < 0.5 * drop[0], "damped: {drop:?}");

    // Friction: two cloths resting on the floor without friction are pushed along +x; the one
    // given friction slides less.
    let mut world = common::world(GRAVITY, 1);
    let floor = add_floor(&mut world);
    world.body_mut(floor).unwrap().set_friction(1.0).unwrap();
    let [slippery, grippy] = [0.0, 3.0].map(|z| {
        add_cloth(
            &mut world,
            SoftBodySettings::default()
                .position(RVec3::new(0.0, 0.05, z))
                .friction(0.0)
                .linear_damping(0.0),
        )
    });
    step(&mut world, 30);
    world.body_mut(grippy).unwrap().set_friction(1.0).unwrap();
    let start = [slippery, grippy].map(|id| mean_position(&world, id)[0]);
    for id in [slippery, grippy] {
        let mut cloth = world.soft_body_mut(id).unwrap();
        for index in 0..cloth.vertex_count() as u32 {
            cloth
                .set_vertex_velocity(index, Vec3::new(3.0, 0.0, 0.0))
                .unwrap();
        }
        world.body_mut(id).unwrap().activate();
    }
    step(&mut world, 30);
    let slid = [slippery, grippy]
        .iter()
        .zip(start)
        .map(|(&id, x)| mean_position(&world, id)[0] - x)
        .collect::<Vec<_>>();
    assert!(slid[0] > 1.0, "the slippery cloth slid {slid:?}");
    assert!(slid[1] < 0.5 * slid[0], "friction slowed nothing: {slid:?}");
}

/// Where a 0.1 m ball shot at a 10 cm static wall at 300 m/s ends after 10 steps, created with
/// `created` and switched to `switched` before the first step, awake or asleep until then.
fn shoot_through_wall(created: MotionQuality, switched: MotionQuality, asleep: bool) -> Real {
    let mut world = world(Vec3::ZERO, 1);
    world
        .create_body(
            &Shape::new_box(Vec3::new(2.0, 2.0, 0.05)).unwrap(),
            &BodySettings::new_static(),
        )
        .unwrap();
    let activation = if asleep {
        Activation::DontActivate
    } else {
        Activation::Activate
    };
    let bullet = world
        .create_body(
            &Shape::new_sphere(0.1).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.0, -3.0))
                .linear_velocity(Vec3::new(0.0, 0.0, 300.0))
                .motion_quality(created)
                .activation(activation),
        )
        .unwrap();
    let mut body = world.body_mut(bullet).unwrap();
    body.set_motion_quality(switched).unwrap();
    assert_eq!(body.is_sleeping(), asleep);
    body.activate();
    step(&mut world, 10);
    world.body(bullet).unwrap().position().z
}

#[test]
fn motion_quality_change_stops_tunnelling() {
    let wall_back_plus_radius = 0.05 + 0.1;
    for asleep in [false, true] {
        let discrete =
            shoot_through_wall(MotionQuality::LinearCast, MotionQuality::Discrete, asleep);
        assert!(
            discrete > wall_back_plus_radius,
            "switched to discrete, the ball stopped at z = {discrete} (asleep: {asleep})"
        );
        let cast = shoot_through_wall(MotionQuality::Discrete, MotionQuality::LinearCast, asleep);
        assert!(
            cast < wall_back_plus_radius,
            "switched to linear cast, the ball tunnelled to z = {cast} (asleep: {asleep})"
        );
    }
}

/// Every value property of `id`, as bits where it is a float.
fn values(world: &PhysicsWorld, id: BodyId) -> [Option<u32>; 6] {
    let body = world.body(id).unwrap();
    [
        Some(body.friction().to_bits()),
        Some(body.restitution().to_bits()),
        body.gravity_factor().map(f32::to_bits),
        body.linear_damping().map(f32::to_bits),
        body.angular_damping().map(f32::to_bits),
        body.motion_quality().map(|quality| quality as u32),
    ]
}

#[test]
fn refused_kinds_change_nothing() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let floor = add_floor(&mut world);
    let cloth = add_cloth(
        &mut world,
        SoftBodySettings::default()
            .position(RVec3::new(0.0, 5.0, 0.0))
            .activation(Activation::DontActivate),
    );
    let sensor = world
        .create_body(
            &Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap(),
            &BodySettings::new_kinematic()
                .position(RVec3::new(5.0, 2.0, 0.0))
                .sensor(true)
                .activation(Activation::DontActivate),
        )
        .unwrap();
    let saved = world.save_state();
    let before = [floor, cloth, sensor].map(|id| values(&world, id));

    // A plain static body has no motion properties; the kind is refused before the value.
    let mut body = world.body_mut(floor).unwrap();
    assert_eq!(
        body.set_gravity_factor(f32::NAN),
        Err(BodyError::CannotMove(floor))
    );
    assert_eq!(
        body.set_gravity_factor(0.5),
        Err(BodyError::CannotMove(floor))
    );
    assert_eq!(
        body.set_linear_damping(0.5),
        Err(BodyError::CannotMove(floor))
    );
    assert_eq!(
        body.set_angular_damping(0.5),
        Err(BodyError::CannotMove(floor))
    );
    assert_eq!(
        body.set_motion_quality(MotionQuality::LinearCast),
        Err(BodyError::CannotMove(floor))
    );

    // A soft body has neither angular damping nor a motion quality.
    let mut body = world.body_mut(cloth).unwrap();
    assert_eq!(
        body.set_angular_damping(0.5),
        Err(BodyError::NotRigidBody(cloth))
    );
    assert_eq!(
        body.set_motion_quality(MotionQuality::LinearCast),
        Err(BodyError::NotRigidBody(cloth))
    );

    // Jolt integrates sensors discretely only.
    let mut body = world.body_mut(sensor).unwrap();
    assert_eq!(
        body.set_motion_quality(MotionQuality::LinearCast),
        Err(BodyError::InvalidValue(
            "a sensor uses MotionQuality::Discrete"
        ))
    );
    body.set_motion_quality(MotionQuality::Discrete).unwrap();

    assert_eq!([floor, cloth, sensor].map(|id| values(&world, id)), before);
    assert!(world.body(cloth).unwrap().is_sleeping());
    assert!(world.body(sensor).unwrap().is_sleeping());
    assert!(world.take_events().activations.is_empty());
    world.restore_state(&saved).unwrap();
}
