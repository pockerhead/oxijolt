//! Momentary inputs to bodies: impulses, kinematic moves and activation on demand.

mod common;

use common::controls::*;
use common::events::add_cloth;
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

#[test]
fn impulses_change_velocity_at_once_and_wake() {
    let mut world = world(Vec3::ZERO, 1);
    // 2 kg: Jolt's inverse mass 0.5 is exact, so the velocity change is exact too.
    let cube = add_sleeping(&mut world, &cube_shape(), 2.0, RVec3::ZERO);
    assert!(world.body(cube).unwrap().is_sleeping());
    let mut body = world.body_mut(cube).unwrap();
    body.add_impulse(Vec3::new(3.0, -1.0, 0.5)).unwrap();
    assert!(body.is_active());
    assert_eq!(body.linear_velocity(), Vec3::new(1.5, -0.5, 0.25));
    body.add_impulse(Vec3::new(1.0, 1.0, -0.5)).unwrap();
    assert_eq!(body.linear_velocity(), Vec3::new(2.0, 0.0, 0.0));

    // A zero impulse is accepted and still wakes a sleeping body, as in Jolt.
    let other = add_sleeping(&mut world, &cube_shape(), 2.0, RVec3::new(5.0, 0.0, 0.0));
    let mut body = world.body_mut(other).unwrap();
    body.add_impulse(Vec3::ZERO).unwrap();
    body.add_angular_impulse(Vec3::ZERO).unwrap();
    assert!(body.is_active());
    assert_eq!(body.linear_velocity(), Vec3::ZERO);
    step(&mut world, 1);
    assert!(world.body(cube).unwrap().position().x > 0.0);
}

#[test]
fn impulse_at_point_spins_the_body() {
    let mut world = world(Vec3::ZERO, 1);
    // A sphere of 2.5 kg and radius 1 m has an inertia of 1 kg·m² about every axis.
    let ball = add_sleeping(
        &mut world,
        &Shape::new_sphere(1.0).unwrap(),
        2.5,
        RVec3::ZERO,
    );
    let mut body = world.body_mut(ball).unwrap();
    body.add_impulse_at_point(Vec3::new(0.0, 1.0, 0.0), RVec3::new(1.0, 0.0, 0.0))
        .unwrap();
    assert!(body.is_active());
    assert_eq!(body.linear_velocity(), Vec3::new(0.0, 0.4, 0.0));
    let w = body.angular_velocity();
    assert!(w.x.abs() < 1.0e-6 && w.y.abs() < 1.0e-6, "{w:?}");
    assert!((w.z - 1.0).abs() < 1.0e-5, "{w:?}");
}

#[test]
fn impulses_are_bounded_by_the_velocity_change_they_give() {
    let mut world = world(Vec3::ZERO, 1);
    for (i, mass) in [limits::MIN_MASS, 2.0, limits::MAX_MASS]
        .into_iter()
        .enumerate()
    {
        let id = add_sleeping(
            &mut world,
            &cube_shape(),
            mass,
            RVec3::new(0.0, 0.0, 10.0 * i as Real),
        );
        // Jolt's inverse mass of a body whose mass is overridden.
        let inverse_mass = f64::from(1.0 / mass);
        let bound = f64::from(limits::MAX_VELOCITY_CHANGE);
        let largest = largest_accepted(f32::MAX, |j| f64::from(j) * inverse_mass <= bound);
        let mut body = world.body_mut(id).unwrap();
        let before = (body.linear_velocity(), body.is_active());
        assert!(invalid(body.add_impulse(Vec3::new(
            0.0,
            largest.next_up(),
            0.0
        ))));
        assert!(invalid(body.add_impulse_at_point(
            Vec3::new(0.0, -largest.next_up(), 0.0),
            body.position()
        )));
        for value in [f32::NAN, f32::INFINITY] {
            assert!(invalid(body.add_impulse(Vec3::new(value, 0.0, 0.0))));
        }
        assert_eq!((body.linear_velocity(), body.is_active()), before);
        body.add_impulse(Vec3::new(0.0, largest, 0.0)).unwrap();
        // Jolt clamps the new velocity to the speed bound.
        let speed = length(body.linear_velocity());
        assert!(
            (speed - limits::MAX_LINEAR_VELOCITY).abs() < 1.0e-3,
            "{speed}"
        );
        body.add_impulse(Vec3::new(0.0, -largest, 0.0)).unwrap();
        assert!(body.linear_velocity().y < -499.9);
    }
    step(&mut world, 60);
}

#[test]
fn angular_impulses_are_bounded_by_the_angular_velocity_change() {
    let mut world = world(Vec3::ZERO, 1);
    // A 1 g cube of 6 cm: its inertia is just above Jolt's near-zero threshold, which gives the
    // largest inverse inertia a body can have.
    let tiny = Shape::new_box(Vec3::new(0.03, 0.03, 0.03)).unwrap();
    let moment = limits::MIN_MASS * (2.0 * 0.06 * 0.06) / 12.0;
    let rotations = [
        Quat::IDENTITY,
        quat_about(Vec3::new(0.6, 0.0, 0.8), 0.7),
        quat_about(Vec3::new(0.0, 0.28, 0.96), 2.1),
    ];
    let directions = [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.6, 0.8),
        Vec3::new(0.48, 0.6, -0.64),
    ];
    let mut ids = Vec::new();
    for (i, (&rotation, &direction)) in rotations.iter().zip(&directions).enumerate() {
        let id = world
            .create_body(
                &tiny,
                &BodySettings::new_dynamic()
                    .position(RVec3::new(i as Real, 0.0, 0.0))
                    .rotation(rotation)
                    .mass(limits::MIN_MASS)
                    .angular_damping(0.0),
            )
            .unwrap();
        ids.push(id);
        let mut body = world.body_mut(id).unwrap();
        let kick = |body: &mut BodyMut<'_>, size: f32| {
            body.add_angular_impulse(Vec3::new(
                direction.x * size,
                direction.y * size,
                direction.z * size,
            ))
        };
        let largest = largest_accepted(1.0, |size| kick(&mut body, size).is_ok());
        // The bound is the angular velocity change over the largest inverse inertia.
        let expected = limits::MAX_ANGULAR_VELOCITY_CHANGE * moment;
        assert!(
            (largest / expected - 1.0).abs() < 1.0e-3,
            "{largest} {expected}"
        );
        let before = body.angular_velocity();
        assert!(invalid(kick(&mut body, largest.next_up())));
        assert_eq!(body.angular_velocity(), before);
        let point = body.position();
        let lever = RVec3::new(point.x, point.y + 0.03, point.z);
        assert!(invalid(body.add_impulse_at_point(
            Vec3::new(largest.next_up() * 2.0 / 0.03, 0.0, 0.0),
            lever
        )));
        kick(&mut body, largest).unwrap();
        // Jolt clamps the spin to the bound; an overflow would show as an exact zero.
        let spin = length(body.angular_velocity());
        assert!(
            (spin - limits::MAX_ANGULAR_VELOCITY).abs() < 1.0e-3,
            "{spin}"
        );
    }
    step(&mut world, 60);
    for id in ids {
        assert_finite(&world, id);
        let spin = length(world.body(id).unwrap().angular_velocity());
        assert!(spin > 40.0, "{spin}");
    }
}

#[test]
fn point_impulses_are_bounded_by_the_angular_impulse_of_their_lever() {
    let mut world = world(Vec3::ZERO, 1);
    let ball = add_sleeping(
        &mut world,
        &Shape::new_sphere(1.0).unwrap(),
        2.5,
        RVec3::ZERO,
    );
    let mut body = world.body_mut(ball).unwrap();
    // 10 N·s at the centre is accepted; at 50 m from it, it is a 500 N·m·s angular impulse on an
    // inertia of 1 kg·m², beyond the bound.
    let impulse = Vec3::new(0.0, 10.0, 0.0);
    assert!(invalid(
        body.add_impulse_at_point(impulse, RVec3::new(50.0, 0.0, 0.0))
    ));
    let outside = RVec3::new(limits::MAX_POSITION.next_up(), 0.0, 0.0);
    assert!(invalid(body.add_impulse_at_point(impulse, outside)));
    assert!(body.is_sleeping());
    body.add_impulse_at_point(impulse, RVec3::ZERO).unwrap();
    assert_eq!(body.angular_velocity(), Vec3::ZERO);
    body.add_impulse_at_point(impulse, RVec3::new(2.0, 0.0, 0.0))
        .unwrap();
    assert!((body.angular_velocity().z - 20.0).abs() < 1.0e-3);
    step(&mut world, 30);
    assert_finite(&world, ball);
}

/// A dynamic needle of `MAX_MASS`, 2 km long and `2 * thin` thick along x, asleep with its
/// centre at (0, -3500, -3500), and the point (0, 3500, 3500): the lever from the centre is
/// (0, 7000, 7000), so an impulse or force along (0, 1, 1) is exactly parallel to it.
fn add_needle(world: &mut PhysicsWorld, thin: f32) -> (BodyId, RVec3) {
    let shape = Shape::new_box_with_convex_radius(Vec3::new(1000.0, thin, thin), 0.0).unwrap();
    let centre = RVec3::new(0.0, -3500.0, -3500.0);
    let id = add_sleeping(world, &shape, limits::MAX_MASS, centre);
    (id, RVec3::new(0.0, 3500.0, 3500.0))
}

#[test]
fn point_impulses_along_a_long_lever_count_jolt_rounding() {
    // The exact angular impulse of an impulse parallel to its lever is 0, but Jolt's `f32`
    // cross product, with one product fused into the subtraction, keeps that product's rounding
    // error: about 1e5 N·m·s for 7e8 N·s at 7000 m, on a needle whose inverse inertia about x is
    // up to about 1e14.
    let along = |size: f32| Vec3::new(0.0, size, size);
    for thin in [1.0e-10, 1.0e-8, 1.0e-6] {
        let mut world = world(Vec3::ZERO, 1);
        let (needle, point) = add_needle(&mut world, thin);
        let mut body = world.body_mut(needle).unwrap();
        assert!(invalid(body.add_impulse_at_point(along(7.0e8), point)));
        assert!(body.is_sleeping());
        let largest = largest_accepted(7.0e8, |size| {
            body.set_angular_velocity(Vec3::ZERO).unwrap();
            body.add_impulse_at_point(along(size), point).is_ok()
        });
        body.set_angular_velocity(Vec3::ZERO).unwrap();
        body.add_impulse_at_point(along(largest), point).unwrap();
        // Jolt's change stays below the clamp, within the bound.
        let spin = length(body.angular_velocity());
        assert!(
            spin < limits::MAX_ANGULAR_VELOCITY,
            "{thin}: {largest} {spin}"
        );
        step(&mut world, 3);
        assert_finite(&world, needle);
    }
}

#[test]
fn impulses_respect_locked_axes() {
    let mut world = world(Vec3::ZERO, 1);
    let plane = add_locked_cube(&mut world, RVec3::ZERO, AllowedDofs::PLANE_2D);
    let mut body = world.body_mut(plane).unwrap();
    body.add_impulse(Vec3::new(0.0, 0.0, 50.0)).unwrap();
    body.add_angular_impulse(Vec3::new(5.0, 5.0, 0.0)).unwrap();
    assert_eq!(body.linear_velocity(), Vec3::ZERO);
    assert_eq!(body.angular_velocity(), Vec3::ZERO);
    body.add_impulse_at_point(Vec3::new(0.0, 0.0, 10.0), RVec3::new(0.5, 0.5, 0.0))
        .unwrap();
    let (v, w) = (body.linear_velocity(), body.angular_velocity());
    assert_eq!((v.z, w.x, w.y), (0.0, 0.0, 0.0));
    step(&mut world, 10);
    assert_eq!(world.body(plane).unwrap().position(), RVec3::ZERO);
}

type Mat3 = [[f32; 3]; 3];

/// A rotation of 0.7 rad about (1, 1, 1): no world axis stays a principal axis.
fn tilted() -> Quat {
    let (s, c) = (0.35_f32.sin() / 3.0_f32.sqrt(), 0.35_f32.cos());
    Quat::from_xyzw(s, s, s, c)
}

fn rotation_matrix(q: Quat) -> Mat3 {
    let Quat { x, y, z, w } = q;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn mul(a: Mat3, b: Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}

fn transpose(a: Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}

fn inverse(a: Mat3) -> Mat3 {
    let cofactor = |i: usize, j: usize| {
        let (r0, r1) = ((i + 1) % 3, (i + 2) % 3);
        let (c0, c1) = ((j + 1) % 3, (j + 2) % 3);
        a[r0][c0] * a[r1][c1] - a[r0][c1] * a[r1][c0]
    };
    let determinant: f32 = (0..3).map(|j| a[0][j] * cofactor(0, j)).sum();
    std::array::from_fn(|i| std::array::from_fn(|j| cofactor(j, i) / determinant))
}

/// `R · inverse(tensor) · Rᵀ`: the world inverse inertia of a body with every axis free.
fn world_inverse(rotation: Quat, tensor: Mat3) -> Mat3 {
    let r = rotation_matrix(rotation);
    mul(mul(r, inverse(tensor)), transpose(r))
}

fn times(a: Mat3, v: Vec3) -> Vec3 {
    let row = |i: usize| a[i][0] * v.x + a[i][1] * v.y + a[i][2] * v.z;
    Vec3::new(row(0), row(1), row(2))
}

fn assert_close(actual: Mat3, expected: Mat3, tolerance: f32) {
    for i in 0..3 {
        for j in 0..3 {
            assert!(
                (actual[i][j] - expected[i][j]).abs() <= tolerance,
                "[{i}][{j}]: {actual:?} vs {expected:?}"
            );
        }
    }
}

fn gap(a: Vec3, b: Vec3) -> f32 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt()
}

#[test]
fn inverse_inertia_is_the_rotated_principal_inverse() {
    let mut world = world(Vec3::ZERO, 1);
    let moments = Vec3::new(1.0, 2.0, 4.0);
    let id = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .mass_and_inertia(3.0, Inertia::Principal(moments))
                .rotation(tilted()),
        )
        .unwrap();
    let matrix = world.body(id).unwrap().inverse_inertia().unwrap();
    let tensor = [[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 4.0]];
    let expected = world_inverse(tilted(), tensor);
    assert!(expected[0][1].abs() > 0.05, "{expected:?}");
    assert_close(matrix, expected, 1.0e-5);
    assert_close(matrix, transpose(matrix), 1.0e-6);
}

#[test]
fn inverse_inertia_masks_locked_rotation_axes_as_impulses_do() {
    let mut world = world(Vec3::ZERO, 1);
    // Off-diagonal products of inertia: the principal axes are not the body axes either.
    let tensor = [[2.0, -0.5, 0.3], [-0.5, 3.0, -0.4], [0.3, -0.4, 4.0]];
    let only_yaw = AllowedDofs::TRANSLATION_X
        | AllowedDofs::TRANSLATION_Y
        | AllowedDofs::TRANSLATION_Z
        | AllowedDofs::ROTATION_Y;
    let id = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .mass_and_inertia(3.0, Inertia::Tensor(tensor))
                .rotation(tilted())
                .allowed_dofs(only_yaw),
        )
        .unwrap();
    let matrix = world.body(id).unwrap().inverse_inertia().unwrap();
    let unmasked = world_inverse(tilted(), tensor);
    for i in [0, 2] {
        assert_eq!(matrix[i], [0.0; 3], "{matrix:?}");
        assert_eq!([matrix[0][i], matrix[1][i], matrix[2][i]], [0.0; 3]);
    }
    assert!((matrix[1][1] - unmasked[1][1]).abs() <= 1.0e-5 * unmasked[1][1]);

    let impulse = Vec3::new(0.3, 0.5, -0.2);
    let mut body = world.body_mut(id).unwrap();
    body.add_angular_impulse(impulse).unwrap();
    let w = body.angular_velocity();
    assert!(gap(w, times(matrix, impulse)) <= 1.0e-6, "{w:?}");
    // The unmasked tensor would predict a spin about the locked axes.
    assert!(gap(w, times(unmasked, impulse)) > 0.05, "{w:?}");
}

#[test]
fn inverse_inertia_is_zero_with_every_rotation_locked() {
    let mut world = world(Vec3::ZERO, 1);
    let sliding =
        AllowedDofs::TRANSLATION_X | AllowedDofs::TRANSLATION_Y | AllowedDofs::TRANSLATION_Z;
    let id = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .rotation(tilted())
                .allowed_dofs(sliding),
        )
        .unwrap();
    assert_eq!(
        world.body(id).unwrap().inverse_inertia(),
        Some([[0.0; 3]; 3])
    );
}

#[test]
fn inverse_inertia_is_none_for_static_kinematic_and_soft_bodies() {
    let mut world = world(Vec3::ZERO, 1);
    let fixed = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().allow_dynamic_or_kinematic(true),
        )
        .unwrap();
    let platform = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic().position(RVec3::new(3.0, 0.0, 0.0)),
        )
        .unwrap();
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 5.0, 0.0), Quat::IDENTITY);
    for id in [fixed, platform, cloth] {
        assert_eq!(world.body(id).unwrap().inverse_inertia(), None);
    }
}

#[test]
fn impulses_ignore_static_and_kinematic_bodies_and_refuse_soft_bodies() {
    let mut world = world(Vec3::ZERO, 1);
    let fixed = world
        .create_body(&cube_shape(), &BodySettings::new_static())
        .unwrap();
    let platform = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic().position(RVec3::new(3.0, 0.0, 0.0)),
        )
        .unwrap();
    for id in [fixed, platform] {
        let mut body = world.body_mut(id).unwrap();
        let huge = Vec3::new(1.0e30, 0.0, 0.0);
        body.add_impulse(huge).unwrap();
        body.add_angular_impulse(huge).unwrap();
        body.add_impulse_at_point(huge, RVec3::new(0.0, 4.0, 0.0))
            .unwrap();
        assert!(invalid(body.add_impulse(Vec3::new(f32::NAN, 0.0, 0.0))));
        assert_eq!(body.linear_velocity(), Vec3::ZERO);
        assert_eq!(body.angular_velocity(), Vec3::ZERO);
    }
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 5.0, 0.0), Quat::IDENTITY);
    let mut body = world.body_mut(cloth).unwrap();
    assert_eq!(
        body.add_impulse(Vec3::ZERO),
        Err(BodyError::NotRigidBody(cloth))
    );
    assert_eq!(
        body.add_angular_impulse(Vec3::ZERO),
        Err(BodyError::NotRigidBody(cloth))
    );
    assert_eq!(
        body.add_impulse_at_point(Vec3::ZERO, RVec3::ZERO),
        Err(BodyError::NotRigidBody(cloth))
    );
    step(&mut world, 5);
}

#[test]
fn a_kinematic_platform_moves_toward_its_target() {
    let mut world = world(GRAVITY, 1);
    let block = Shape::new_box(Vec3::new(1.0, 0.2, 1.0)).unwrap();
    let off_centre = Shape::new_offset_center_of_mass(&block, Vec3::new(0.3, 0.0, -0.2)).unwrap();
    for (i, shape) in [&block, &off_centre].into_iter().enumerate() {
        let start = RVec3::new(10.0 * i as Real, 0.0, 0.0);
        let platform = add_kinematic(&mut world, shape, start, Activation::Activate);
        let target = RVec3::new(start.x + 0.5, 0.2, -0.1);
        let turn = quat_about(Vec3::new(0.0, 1.0, 0.0), 0.03);
        world
            .body_mut(platform)
            .unwrap()
            .move_kinematic(target, turn, DT)
            .unwrap();
        step(&mut world, 1);
        let body = world.body(platform).unwrap();
        assert!(distance(body.position(), target) < 1.0e-5, "{i}");
        // Jolt's small-angle angular velocity turns by 2 sin(θ/2) instead of θ.
        let error = angle_between(body.rotation(), turn);
        assert!(error < 2.0e-6, "{i}: {error}");
    }
}

#[test]
fn a_kinematic_move_keeps_its_velocity_after_the_step() {
    let mut world = world(Vec3::ZERO, 1);
    let platform = add_kinematic(&mut world, &cube_shape(), RVec3::ZERO, Activation::Activate);
    world
        .body_mut(platform)
        .unwrap()
        .move_kinematic(RVec3::new(0.1, 0.0, 0.0), Quat::IDENTITY, DT)
        .unwrap();
    let velocity = world.body(platform).unwrap().linear_velocity();
    step(&mut world, 2);
    let body = world.body(platform).unwrap();
    assert_eq!(body.linear_velocity(), velocity);
    assert!(distance(body.position(), RVec3::new(0.2, 0.0, 0.0)) < 1.0e-5);
}

#[test]
fn a_tiny_move_does_not_wake_a_sleeping_kinematic_body() {
    let mut world = world(Vec3::ZERO, 1);
    let platform = add_kinematic(
        &mut world,
        &cube_shape(),
        RVec3::ZERO,
        Activation::DontActivate,
    );
    let mut body = world.body_mut(platform).unwrap();
    // 1e-9 m in 1/60 s: a squared speed of 3.6e-15 m²/s², below Jolt's 1e-12.
    body.move_kinematic(RVec3::new(1.0e-9, 0.0, 0.0), Quat::IDENTITY, DT)
        .unwrap();
    assert!(body.is_sleeping());
    step(&mut world, 1);
    assert_eq!(world.body(platform).unwrap().position(), RVec3::ZERO);
    let mut body = world.body_mut(platform).unwrap();
    body.move_kinematic(RVec3::new(1.0e-3, 0.0, 0.0), Quat::IDENTITY, DT)
        .unwrap();
    assert!(body.is_active());
}

#[test]
fn kinematic_moves_respect_locked_axes() {
    let mut world = world(Vec3::ZERO, 1);
    let rail = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic()
                .allowed_dofs(AllowedDofs::TRANSLATION_X | AllowedDofs::ROTATION_Y),
        )
        .unwrap();
    let turn = quat_about(Vec3::new(0.6, 0.8, 0.0), 0.2);
    world
        .body_mut(rail)
        .unwrap()
        .move_kinematic(RVec3::new(1.0, 2.0, 3.0), turn, DT)
        .unwrap();
    let body = world.body(rail).unwrap();
    let (v, w) = (body.linear_velocity(), body.angular_velocity());
    assert_eq!((v.y, v.z, w.x, w.z), (0.0, 0.0, 0.0, 0.0));
    assert!(v.x > 59.0 && w.y > 0.0, "{v:?} {w:?}");
    step(&mut world, 1);
    let p = world.body(rail).unwrap().position();
    assert_eq!((p.y, p.z), (0.0, 0.0));
}

#[test]
fn a_kinematic_platform_carries_a_resting_cube() {
    let mut world = world(GRAVITY, 1);
    let deck = Shape::new_box(Vec3::new(2.0, 0.2, 2.0)).unwrap();
    let platform = add_kinematic(&mut world, &deck, RVec3::ZERO, Activation::Activate);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.7, 0.0));
    step(&mut world, 30);
    for tick in 1..=60 {
        let target = RVec3::new(tick as Real / 60.0, 0.0, 0.0);
        world
            .body_mut(platform)
            .unwrap()
            .move_kinematic(target, Quat::IDENTITY, DT)
            .unwrap();
        step(&mut world, 1);
    }
    let platform_x = world.body(platform).unwrap().position().x;
    let cube_x = world.body(cube).unwrap().position().x;
    assert!((platform_x - 1.0).abs() < 1.0e-4, "{platform_x}");
    // Friction accelerates the cube to the platform's 1 m/s within about half a second.
    assert!(cube_x > 0.7, "{cube_x}");
    let cube_speed = world.body(cube).unwrap().linear_velocity().x;
    assert!((cube_speed - 1.0).abs() < 0.05, "{cube_speed}");
}

#[test]
fn kinematic_moves_are_bounded_by_the_velocities_they_imply() {
    let mut world = world(Vec3::ZERO, 1);
    // dt = 1/64: a move of 7.8125 m is exactly 500 m/s.
    let dt = 1.0 / 64.0;
    let platform = add_kinematic(&mut world, &cube_shape(), RVec3::ZERO, Activation::Activate);
    let along = |distance: f32| RVec3::new(Real::from(distance), 0.0, 0.0);
    let mut body = world.body_mut(platform).unwrap();
    assert!(invalid(body.move_kinematic(
        along(7.8125_f32.next_up()),
        Quat::IDENTITY,
        dt
    )));
    body.move_kinematic(along(7.8125), Quat::IDENTITY, dt)
        .unwrap();
    assert_eq!(body.linear_velocity(), Vec3::new(500.0, 0.0, 0.0));
    assert!(world.step(dt).unwrap().is_complete());
    assert_eq!(world.body(platform).unwrap().position(), along(7.8125));

    // The largest turn about z the bound accepts, found on the angle's bits.
    let spinner = add_kinematic(
        &mut world,
        &cube_shape(),
        RVec3::new(0.0, 5.0, 0.0),
        Activation::Activate,
    );
    let mut body = world.body_mut(spinner).unwrap();
    let here = body.position();
    let about_z = |angle: f32| quat_about(Z, angle);
    let largest = largest_accepted(1.5, |angle| {
        body.move_kinematic(here, about_z(angle), dt).is_ok()
    });
    assert!(
        (largest - limits::MAX_ANGULAR_VELOCITY * dt).abs() < 1.0e-3,
        "{largest}"
    );
    assert!(invalid(body.move_kinematic(
        here,
        about_z(largest.next_up()),
        dt
    )));
    body.move_kinematic(here, about_z(largest), dt).unwrap();
    let spin = length(body.angular_velocity());
    assert!(
        spin <= limits::MAX_ANGULAR_VELOCITY * (1.0 + 1.0e-6),
        "{spin}"
    );
    assert!(world.step(dt).unwrap().is_complete());
    assert_finite(&world, spinner);

    // Invalid inputs change nothing.
    let mut body = world.body_mut(platform).unwrap();
    let before = (body.linear_velocity(), body.angular_velocity());
    let far = RVec3::new(limits::MAX_POSITION.next_up(), 0.0, 0.0);
    assert!(invalid(body.move_kinematic(far, Quat::IDENTITY, dt)));
    let skewed = Quat::from_xyzw(0.0, 0.0, 0.0, 2.0);
    assert!(invalid(body.move_kinematic(along(8.0), skewed, dt)));
    for bad_dt in [0.0, -dt, f32::NAN, PhysicsWorld::MAX_DELTA_TIME * 2.0] {
        assert!(invalid(body.move_kinematic(
            along(8.0),
            Quat::IDENTITY,
            bad_dt
        )));
    }
    assert_eq!((body.linear_velocity(), body.angular_velocity()), before);

    let ball = add_cube(&mut world, RVec3::new(0.0, -5.0, 0.0));
    let fixed = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(0.0, -10.0, 0.0)),
        )
        .unwrap();
    for id in [ball, fixed] {
        assert_eq!(
            world
                .body_mut(id)
                .unwrap()
                .move_kinematic(RVec3::ZERO, Quat::IDENTITY, dt),
            Err(BodyError::NotKinematic(id))
        );
    }
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 20.0, 0.0), Quat::IDENTITY);
    assert_eq!(
        world
            .body_mut(cloth)
            .unwrap()
            .move_kinematic(RVec3::ZERO, Quat::IDENTITY, dt),
        Err(BodyError::NotRigidBody(cloth))
    );
    step(&mut world, 1);
}

/// A world without gravity that records activation events.
fn activation_world() -> PhysicsWorld {
    let mut world = world(Vec3::ZERO, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    world
}

/// The activation events recorded since the last call.
fn activations(world: &mut PhysicsWorld) -> Vec<ActivationEvent> {
    world.take_events().activations
}

#[test]
fn deactivate_zeroes_velocities_and_reports_an_event() {
    let mut world = activation_world();
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .linear_velocity(Vec3::new(1.0, 2.0, 3.0))
                .angular_velocity(Vec3::new(0.5, 0.0, 0.0)),
        )
        .unwrap();
    activations(&mut world);
    let mut body = world.body_mut(cube).unwrap();
    body.deactivate().unwrap();
    assert!(body.is_sleeping());
    assert_eq!(body.linear_velocity(), Vec3::ZERO);
    assert_eq!(body.angular_velocity(), Vec3::ZERO);
    assert_eq!(
        activations(&mut world),
        [ActivationEvent::Deactivated(cube)]
    );
    step(&mut world, 10);
    assert_eq!(world.body(cube).unwrap().position(), RVec3::ZERO);
}

#[test]
fn deactivate_zeroes_the_velocities_of_a_body_already_asleep() {
    let mut world = activation_world();
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .linear_velocity(Vec3::new(4.0, 0.0, 0.0))
                .activation(Activation::DontActivate),
        )
        .unwrap();
    assert_eq!(
        world.body(cube).unwrap().linear_velocity(),
        Vec3::new(4.0, 0.0, 0.0)
    );
    let mut body = world.body_mut(cube).unwrap();
    body.deactivate().unwrap();
    assert_eq!(body.linear_velocity(), Vec3::ZERO);
    assert!(body.is_sleeping());
    assert!(activations(&mut world).is_empty());
    // Waking it does not bring the velocity back.
    world.body_mut(cube).unwrap().activate();
    step(&mut world, 5);
    assert_eq!(world.body(cube).unwrap().position(), RVec3::ZERO);
}

#[test]
fn repeated_deactivation_reports_once() {
    let mut world = activation_world();
    let cube = add_cube(&mut world, RVec3::ZERO);
    let fixed = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(5.0, 0.0, 0.0)),
        )
        .unwrap();
    activations(&mut world);
    for _ in 0..3 {
        world.body_mut(cube).unwrap().deactivate().unwrap();
        world.body_mut(fixed).unwrap().deactivate().unwrap();
    }
    assert_eq!(
        activations(&mut world),
        [ActivationEvent::Deactivated(cube)]
    );
}

#[test]
fn activate_wakes_a_sleeping_body_and_reports_it() {
    let mut world = activation_world();
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic().activation(Activation::DontActivate),
        )
        .unwrap();
    let fixed = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(5.0, 0.0, 0.0)),
        )
        .unwrap();
    assert!(world.body(cube).unwrap().is_sleeping());
    world.body_mut(cube).unwrap().activate();
    world.body_mut(cube).unwrap().activate();
    world.body_mut(fixed).unwrap().activate();
    assert!(world.body(cube).unwrap().is_active());
    assert!(!world.body(fixed).unwrap().is_active());
    assert_eq!(activations(&mut world), [ActivationEvent::Activated(cube)]);
}

#[test]
fn deactivate_refuses_soft_bodies() {
    let mut world = activation_world();
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 5.0, 0.0), Quat::IDENTITY);
    activations(&mut world);
    assert_eq!(
        world.body_mut(cloth).unwrap().deactivate(),
        Err(BodyError::NotRigidBody(cloth))
    );
    assert!(world.body(cloth).unwrap().is_active());
    assert!(activations(&mut world).is_empty());
}

/// Bodies around a box from (0, 0, 0) to (2, 2, 2), all asleep: one inside, one touching its
/// +x face, one 1 mm beyond it, a static one inside, and one moved away from inside, whose
/// broad-phase bounds Jolt has widened over both places.
struct BoxScene {
    world: PhysicsWorld,
    inside: BodyId,
    touching: BodyId,
    beyond: BodyId,
    moved: BodyId,
}

fn box_scene(optimize: bool) -> BoxScene {
    let mut world = activation_world();
    let asleep = |x: Real| {
        BodySettings::new_dynamic()
            .position(RVec3::new(x, 1.0, 1.0))
            .activation(Activation::DontActivate)
    };
    // Created in an order that differs from body-id order of the result: ids follow creation.
    let moved = world.create_body(&cube_shape(), &asleep(1.0)).unwrap();
    let beyond = world.create_body(&cube_shape(), &asleep(2.501)).unwrap();
    let touching = world.create_body(&cube_shape(), &asleep(2.5)).unwrap();
    let inside = world.create_body(&cube_shape(), &asleep(0.5)).unwrap();
    world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(1.5, 1.0, 1.0)),
        )
        .unwrap();
    world
        .body_mut(moved)
        .unwrap()
        .set_position(RVec3::new(10.0, 1.0, 1.0), Activation::DontActivate)
        .unwrap();
    if optimize {
        world.optimize_broad_phase();
    }
    activations(&mut world);
    BoxScene {
        world,
        inside,
        touching,
        beyond,
        moved,
    }
}

#[test]
fn activate_bodies_in_box_wakes_exact_overlaps_in_body_id_order() {
    for optimize in [false, true] {
        let mut scene = box_scene(optimize);
        let world = &mut scene.world;
        world
            .activate_bodies_in_box(RVec3::ZERO, RVec3::new(2.0, 2.0, 2.0))
            .unwrap();
        let woken = activations(world);
        // Body-id order: `touching` was created before `inside`.
        assert_eq!(
            woken,
            [
                ActivationEvent::Activated(scene.touching),
                ActivationEvent::Activated(scene.inside)
            ],
            "optimize: {optimize}"
        );
        for id in [scene.beyond, scene.moved] {
            assert!(
                world.body(id).unwrap().is_sleeping(),
                "optimize: {optimize}"
            );
        }

        // A degenerate box, a point on the face the moved body left.
        let mut scene = box_scene(optimize);
        let world = &mut scene.world;
        let point = RVec3::new(1.5, 1.0, 1.0);
        world.activate_bodies_in_box(point, point).unwrap();
        assert!(activations(world).is_empty(), "optimize: {optimize}");
        let face = RVec3::new(1.0, 1.0, 1.0);
        world.activate_bodies_in_box(face, face).unwrap();
        assert_eq!(
            activations(world),
            [ActivationEvent::Activated(scene.inside)]
        );
    }
}

#[test]
fn activation_box_is_validated() {
    let mut scene = box_scene(false);
    let world = &mut scene.world;
    let far = limits::MAX_POSITION.next_up();
    for (min, max) in [
        (RVec3::new(2.0, 0.0, 0.0), RVec3::new(0.0, 2.0, 2.0)),
        (RVec3::ZERO, RVec3::new(far, 2.0, 2.0)),
        (RVec3::new(-far, 0.0, 0.0), RVec3::new(2.0, 2.0, 2.0)),
        (RVec3::new(Real::NAN, 0.0, 0.0), RVec3::new(2.0, 2.0, 2.0)),
    ] {
        assert!(invalid(world.activate_bodies_in_box(min, max)));
    }
    assert!(activations(world).is_empty());
    assert!(world.body(scene.inside).unwrap().is_sleeping());
}

#[test]
fn box_activation_compares_in_caller_precision() {
    // Only `f64` positions can name a box edge between two `f32` values.
    if std::mem::size_of::<Real>() != 8 {
        return;
    }
    let mut world = activation_world();
    // A cube whose bounds end exactly at x = 1 and start exactly at x = -1.
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.5, 0.0, 0.0))
                .activation(Activation::DontActivate),
        )
        .unwrap();
    let other = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(-0.5, 0.0, 3.0))
                .activation(Activation::DontActivate),
        )
        .unwrap();
    let just_beyond = 1.0 + 1.0e-8;
    world
        .activate_bodies_in_box(
            RVec3::new(just_beyond, -1.0, -1.0),
            RVec3::new(2.0, 1.0, 1.0),
        )
        .unwrap();
    world
        .activate_bodies_in_box(
            RVec3::new(-2.0, -1.0, 2.0),
            RVec3::new(-just_beyond, 1.0, 4.0),
        )
        .unwrap();
    assert!(activations(&mut world).is_empty());
    world
        .activate_bodies_in_box(RVec3::new(1.0, -1.0, -1.0), RVec3::new(2.0, 1.0, 1.0))
        .unwrap();
    world
        .activate_bodies_in_box(RVec3::new(-2.0, -1.0, 2.0), RVec3::new(-1.0, 1.0, 4.0))
        .unwrap();
    assert_eq!(
        activations(&mut world),
        [
            ActivationEvent::Activated(cube),
            ActivationEvent::Activated(other)
        ]
    );
}
