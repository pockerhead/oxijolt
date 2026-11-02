//! A shape's own mass and inertia (`Shape::mass_properties`): analytic values, compounds, the
//! round trip through a provided mass and inertia, and shapes without mass.

mod common;

use common::meshes::grid;
use common::*;
use oxijolt::*;

/// Jolt's density of convex shapes, kg/m³.
const DENSITY: f64 = 1000.0;

/// Asserts that `actual` is within a relative `1e-6` of `expected`.
fn assert_close(actual: f32, expected: f64) {
    let error = (f64::from(actual) - expected).abs();
    assert!(error <= 1.0e-6 * expected.abs(), "{actual} vs {expected}");
}

/// Asserts that `tensor` is diagonal with the `moments` on its diagonal.
fn assert_diagonal(tensor: [[f32; 3]; 3], moments: [f64; 3]) {
    for (i, row) in tensor.iter().enumerate() {
        for (j, &value) in row.iter().enumerate() {
            if i == j {
                assert_close(value, moments[i]);
            } else {
                assert_eq!(value, 0.0, "{tensor:?}");
            }
        }
    }
}

#[test]
fn a_box_has_its_volume_times_the_density_and_the_solid_box_inertia() {
    let (a, b, c) = (0.4_f64, 0.25, 1.5);
    let shape = Shape::new_box(Vec3::new(a as f32, b as f32, c as f32)).unwrap();
    let properties = shape.mass_properties();
    let mass = 8.0 * a * b * c * DENSITY;
    assert_close(properties.mass, mass);
    assert_diagonal(
        properties.inertia,
        [
            mass * (b * b + c * c) / 3.0,
            mass * (a * a + c * c) / 3.0,
            mass * (a * a + b * b) / 3.0,
        ],
    );
}

#[test]
fn a_sphere_has_its_volume_times_the_density_and_the_solid_sphere_inertia() {
    let r = 0.7_f64;
    let properties = Shape::new_sphere(r as f32).unwrap().mass_properties();
    let mass = 4.0 / 3.0 * std::f64::consts::PI * r * r * r * DENSITY;
    assert_close(properties.mass, mass);
    let moment = 0.4 * mass * r * r;
    assert_diagonal(properties.inertia, [moment; 3]);
}

#[test]
fn a_compound_weighs_the_sum_of_its_children() {
    let block = Shape::new_box(Vec3::new(0.4, 0.2, 0.3)).unwrap();
    let ball = Shape::new_sphere(0.3).unwrap();
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &block,
            position: Vec3::new(0.2, 0.0, 0.0),
            rotation: quat_about(Vec3::new(0.0, 0.6, 0.8), 0.5),
            user_data: 0,
        },
        child(&ball, Vec3::new(-0.3, 0.4, 0.1), 1),
    ])
    .unwrap();
    let properties = compound.mass_properties();
    assert_eq!(
        properties.mass,
        block.mass_properties().mass + ball.mass_properties().mass
    );
    // About the compound's centre of mass the rotated, offset block makes the tensor full.
    let tensor = properties.inertia;
    assert!(tensor[0][1] != 0.0 && tensor[0][2] != 0.0 && tensor[1][2] != 0.0);
}

#[test]
fn a_scaled_shape_scales_its_inner_shape() {
    let cube = cube_shape();
    let stretched = Shape::new_scaled(&cube, Vec3::new(2.0, 1.0, 1.0)).unwrap();
    let scaled = stretched.mass_properties();
    let long = Shape::new_box(Vec3::new(1.0, 0.5, 0.5))
        .unwrap()
        .mass_properties();
    assert_close(scaled.mass, f64::from(long.mass));
    assert_diagonal(
        scaled.inertia,
        [0, 1, 2].map(|i| f64::from(long.inertia[i][i])),
    );
}

/// The body's mass and world inverse inertia as bits.
fn mass_and_inverse_inertia(world: &PhysicsWorld, id: BodyId) -> Vec<u32> {
    let body = world.body(id).unwrap();
    let mut bits = vec![body.mass().unwrap().to_bits()];
    let inverse = body.inverse_inertia().unwrap();
    bits.extend(inverse.iter().flatten().map(|value| value.to_bits()));
    bits
}

#[test]
fn providing_a_shapes_mass_properties_gives_the_body_it_gets_by_default() {
    let block = Shape::new_box(Vec3::new(0.4, 0.2, 0.3)).unwrap();
    let ball = Shape::new_sphere(0.3).unwrap();
    let offset = Shape::new_compound(&[
        child(&block, Vec3::new(0.2, 0.0, 0.0), 0),
        child(&ball, Vec3::new(-0.3, 0.4, 0.1), 1),
    ])
    .unwrap();
    let turned = Shape::new_compound(&[
        CompoundChild {
            shape: &block,
            position: Vec3::new(0.2, 0.0, 0.0),
            rotation: quat_about(Vec3::new(0.0, 0.6, 0.8), 0.5),
            user_data: 0,
        },
        child(&ball, Vec3::new(-0.3, 0.4, 0.1), 1),
    ])
    .unwrap();
    let rotation = quat_about(Vec3::new(0.48, 0.6, -0.64), 0.7);
    let mut world = world(Vec3::ZERO, 1);
    for shape in [&block, &ball, &offset, &turned] {
        let properties = shape.mass_properties();
        let default = BodySettings::new_dynamic().rotation(rotation);
        let provided = default
            .clone()
            .mass_and_inertia(properties.mass, Inertia::Tensor(properties.inertia));
        let by_default = world.create_body(shape, &default).unwrap();
        let by_hand = world.create_body(shape, &provided).unwrap();
        assert_eq!(
            mass_and_inverse_inertia(&world, by_hand),
            mass_and_inverse_inertia(&world, by_default)
        );
    }
}

#[test]
fn a_tensor_below_the_provided_inertia_floor_is_read_but_not_taken_back() {
    // A 1 cm sphere: 4.19 g and moments of 1.68e-7 kg·m², below `limits::MIN_PROVIDED_INERTIA`.
    let shape = Shape::new_sphere(0.01).unwrap();
    let properties = shape.mass_properties();
    let r = 0.01_f64;
    let mass = 4.0 / 3.0 * std::f64::consts::PI * r * r * r * DENSITY;
    assert_close(properties.mass, mass);
    assert_diagonal(properties.inertia, [0.4 * mass * r * r; 3]);
    assert!(properties.inertia[0][0] < limits::MIN_PROVIDED_INERTIA);

    let mut world = world(Vec3::ZERO, 1);
    // By default Jolt finds the moments near zero and gives the body a unit sphere's inertia.
    let by_default = world
        .create_body(&shape, &BodySettings::new_dynamic())
        .unwrap();
    let inverse = world.body(by_default).unwrap().inverse_inertia().unwrap();
    assert_close(inverse[0][0], 2.5 / f64::from(properties.mass));
    // Provided back, the tensor is refused.
    let provided = BodySettings::new_dynamic()
        .mass_and_inertia(properties.mass, Inertia::Tensor(properties.inertia));
    assert_eq!(
        world.create_body(&shape, &provided),
        Err(BodyError::InvalidValue(
            "provided inertia needs principal moments of at least limits::MIN_PROVIDED_INERTIA"
        ))
    );
}

#[test]
fn meshes_height_fields_and_planes_have_zero_mass_and_inertia() {
    let (vertices, triangles) = grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let plane = Shape::new_plane(Vec3::new(0.0, 1.0, 0.0), 0.0, 10.0).unwrap();
    for shape in [&mesh, &flat_height_field(), &plane] {
        let properties = shape.mass_properties();
        assert_eq!(properties.mass, 0.0);
        assert_eq!(properties.inertia, [[0.0; 3]; 3]);
    }
    // A compound counts only its children with mass.
    let cube = cube_shape();
    let mixed = Shape::new_compound(&[
        child(&mesh, Vec3::ZERO, 0),
        child(&cube, Vec3::new(0.0, 2.0, 0.0), 1),
    ])
    .unwrap();
    assert_eq!(mixed.mass_properties().mass, cube.mass_properties().mass);
}
