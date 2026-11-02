//! Centre-of-mass readouts: a shape's in its own space, a body's in world space.

mod common;

use common::math::{add, f3, norm, rotate, sub, v3};
use common::soft_body::Cloth;
use common::*;
use oxijolt::*;

/// A unit cube at the origin and a cube of half extent 1 at x = 3: 1 and 8 m³ at the same
/// density, so the centre of mass is at x = 8 · 3 / 9.
fn offset_compound() -> Shape {
    let small = cube_shape();
    let large = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    Shape::new_compound(&[
        child(&small, Vec3::ZERO, 0),
        child(&large, Vec3::new(3.0, 0.0, 0.0), 1),
    ])
    .unwrap()
}

const COMPOUND_CENTER: [f64; 3] = [8.0 / 3.0, 0.0, 0.0];

/// A turn of 0.7 rad about (1, 2, 3).
fn tilted() -> Quat {
    let axis = [1.0f64, 2.0, 3.0];
    let length = norm(axis);
    let (sin, cos) = (0.35f64.sin(), 0.35f64.cos());
    Quat::from_xyzw(
        (axis[0] / length * sin) as f32,
        (axis[1] / length * sin) as f32,
        (axis[2] / length * sin) as f32,
        cos as f32,
    )
}

fn assert_near(actual: [f64; 3], expected: [f64; 3], tolerance: f64) {
    let gap = norm(sub(actual, expected));
    assert!(gap <= tolerance, "{actual:?} vs {expected:?}: {gap}");
}

#[test]
fn an_offset_compound_reads_its_mass_weighted_center() {
    let compound = offset_compound();
    assert_near(f3(compound.center_of_mass()), COMPOUND_CENTER, 1e-5);
    assert_eq!(cube_shape().center_of_mass(), Vec3::ZERO);
}

#[test]
fn a_rotated_translated_body_reads_position_plus_rotated_shape_center() {
    let mut world = world(Vec3::ZERO, 1);
    let position = RVec3::new(10.0, -2.0, 5.0);
    let body = world
        .create_body(
            &offset_compound(),
            &BodySettings::new_dynamic()
                .position(position)
                .rotation(tilted()),
        )
        .unwrap();
    let body = world.body(body).unwrap();
    let expected = add(v3(position), rotate(tilted(), COMPOUND_CENTER));
    assert_near(v3(body.center_of_mass_position()), expected, 1e-4);
    // The origin stays where it was set; only the centre of mass is offset from it.
    assert_near(v3(body.position()), v3(position), 1e-4);
}

#[test]
fn a_moving_body_carries_its_center_of_mass() {
    let mut world = world(Vec3::ZERO, 1);
    let body = world
        .create_body(&offset_compound(), &BodySettings::new_dynamic())
        .unwrap();
    world
        .body_mut(body)
        .unwrap()
        .set_angular_velocity(Vec3::new(0.0, 1.0, 0.0))
        .unwrap();
    step(&mut world, 30);
    let body = world.body(body).unwrap();
    let expected = add(
        v3(body.position()),
        rotate(body.rotation(), COMPOUND_CENTER),
    );
    assert_near(v3(body.center_of_mass_position()), expected, 1e-4);
    // Jolt spins the body about its centre of mass, which therefore stays put.
    assert_near(v3(body.center_of_mass_position()), COMPOUND_CENTER, 1e-4);
}

#[test]
fn static_and_kinematic_bodies_read_their_shape_center() {
    let mut world = world(Vec3::ZERO, 1);
    let position = RVec3::new(-4.0, 1.0, 2.0);
    let expected = add(v3(position), rotate(tilted(), COMPOUND_CENTER));
    for settings in [BodySettings::new_static(), BodySettings::new_kinematic()] {
        let body = world
            .create_body(
                &offset_compound(),
                &settings.position(position).rotation(tilted()),
            )
            .unwrap();
        let body = world.body(body).unwrap();
        assert_eq!(body.mass(), None);
        assert_near(v3(body.center_of_mass_position()), expected, 1e-4);
    }
}

#[test]
fn a_soft_body_reads_its_position_at_the_center_of_its_bounds() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    // A 100 kg corner pulls the mass-weighted centre far from the middle of the cloth.
    let mut cloth = Cloth::new(6, 0.2);
    cloth.vertices[0].inverse_mass = 0.01;
    let settings = cloth
        .builder()
        .create_constraints(SoftBodyBendType::None, SoftBodyVertexAttributes::default())
        .build()
        .unwrap();
    let body = world
        .create_soft_body(
            &settings,
            &SoftBodySettings::default().position(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    step(&mut world, 10);

    let vertices = world.soft_body(body).unwrap().vertices();
    let (mut low, mut high) = ([f64::MAX; 3], [f64::MIN; 3]);
    let (mut weighted, mut mass) = ([0.0; 3], 0.0);
    for vertex in &vertices {
        let p = v3(vertex.position);
        let m = 1.0 / f64::from(vertex.inverse_mass);
        for axis in 0..3 {
            low[axis] = low[axis].min(p[axis]);
            high[axis] = high[axis].max(p[axis]);
            weighted[axis] += m * p[axis];
        }
        mass += m;
    }
    let bounds_center = [0, 1, 2].map(|axis| 0.5 * (low[axis] + high[axis]));
    let mass_center = weighted.map(|value| value / mass);

    let body = world.body(body).unwrap();
    let center = v3(body.center_of_mass_position());
    assert_eq!(center, v3(body.position()));
    assert!(center[1] < 5.0, "the cloth fell: {center:?}");
    assert_near(center, bounds_center, 1e-4);
    assert!(norm(sub(center, mass_center)) > 0.1, "{mass_center:?}");
}
