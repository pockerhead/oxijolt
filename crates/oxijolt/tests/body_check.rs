//! `PhysicsWorld::check_body`: the refusals of `create_body` without creating anything, and the
//! promise that settings it accepts `create_body` takes unless the world is full.

mod common;

use common::meshes::grid;
use common::*;
use oxijolt::*;

/// A thin rod 2 m long, rotated in a compound: its inertia is too badly conditioned for a body
/// that can move.
fn thin_rotated_rod() -> Shape {
    let rod = Shape::new_box_with_convex_radius(Vec3::new(1.0, 0.002, 0.002), 0.0).unwrap();
    Shape::new_compound(&[CompoundChild {
        shape: &rod,
        position: Vec3::new(0.3, 0.1, 0.0),
        rotation: quat_about(Vec3::new(0.0, 0.6, 0.8), 0.5),
        user_data: 0,
    }])
    .unwrap()
}

/// A world without gravity holding one sleeping dynamic cube.
fn world_with_a_cube() -> PhysicsWorld {
    let mut world = world(Vec3::ZERO, 1);
    let cube = add_cube(&mut world, RVec3::ZERO);
    world.body_mut(cube).unwrap().deactivate().unwrap();
    world
}

/// Asserts that `check_body` refuses `shape` and `settings` with `expected`, creates nothing and
/// keeps a state saved before restorable, and that `create_body` refuses with the same error.
fn assert_refused(
    world: &mut PhysicsWorld,
    shape: &Shape,
    settings: &BodySettings,
    expected: BodyError,
) {
    let saved = world.save_state();
    let count = world.body_count();
    assert_eq!(world.check_body(shape, settings), Err(expected));
    assert_eq!(world.body_count(), count);
    world.restore_state(&saved).unwrap();
    assert_eq!(world.create_body(shape, settings), Err(expected));
    assert_eq!(world.body_count(), count);
}

/// Asserts that `check_body` accepts `shape` and `settings` without creating or waking
/// anything, keeps a state saved before restorable, and that `create_body` then succeeds.
fn assert_accepted(world: &mut PhysicsWorld, shape: &Shape, settings: &BodySettings) {
    world.set_event_settings(EventSettings::default().body_activation(true));
    world.take_events();
    let saved = world.save_state();
    let count = world.body_count();
    world.check_body(shape, settings).unwrap();
    assert_eq!(world.body_count(), count);
    assert!(world.take_events().activations.is_empty());
    world.restore_state(&saved).unwrap();
    world.create_body(shape, settings).unwrap();
    assert_eq!(world.body_count(), count + 1);
}

#[test]
fn every_refusal_of_create_body_is_returned_by_check_body() {
    let mut world = world_with_a_cube();
    let cube = cube_shape();
    let (vertices, triangles) = grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let invalid = BodyError::InvalidValue;
    let mass_rule = invalid("mass must be between limits::MIN_MASS and limits::MAX_MASS");
    let cases = [
        // Bad setting
        (
            &cube,
            BodySettings::new_dynamic().friction(-1.0),
            invalid("friction must be finite and between 0 and limits::MAX_FRICTION"),
        ),
        // Bad layer
        (
            &cube,
            BodySettings::new_dynamic().object_layer(ObjectLayer::new(99)),
            BodyError::UnknownObjectLayer(ObjectLayer::new(99)),
        ),
        // Static with restricted DOFs
        (
            &cube,
            BodySettings::new_static().allowed_dofs(AllowedDofs::PLANE_2D),
            invalid("a static body that cannot move keeps all DOFs"),
        ),
        // Mass setting out of range
        (
            &cube,
            BodySettings::new_dynamic().mass(limits::MAX_MASS * 2.0),
            mass_rule,
        ),
        // Kinematic mesh without a mass
        (
            &mesh,
            BodySettings::new_kinematic(),
            invalid("a kinematic body with a mesh shape needs a mass"),
        ),
        // Sensor with a static-only shape
        (
            &mesh,
            BodySettings::new_static().sensor(true),
            invalid("a sensor cannot use a shape for static bodies only"),
        ),
    ];
    for (shape, settings, expected) in cases {
        assert_refused(&mut world, shape, &settings, expected);
    }

    let height_field = flat_height_field();
    assert_refused(
        &mut world,
        &height_field,
        &BodySettings::new_dynamic(),
        invalid("this shape can only be used by static bodies"),
    );
    assert_refused(
        &mut world,
        &mesh,
        &BodySettings::new_dynamic().mass(1.0),
        invalid("mesh shapes cannot be used by dynamic bodies"),
    );
    // Computed masses outside the dynamic range: a 1 cm ball and a 20 m box.
    let light = Shape::new_sphere(0.005).unwrap();
    let heavy = Shape::new_box(Vec3::new(10.0, 10.0, 10.0)).unwrap();
    for shape in [&light, &heavy] {
        assert_refused(&mut world, shape, &BodySettings::new_dynamic(), mass_rule);
    }
    assert_refused(
        &mut world,
        &thin_rotated_rod(),
        &BodySettings::new_dynamic(),
        invalid("mass and shape must give a finite inverse mass and a well-conditioned inertia"),
    );
}

#[test]
fn accepted_settings_are_taken_by_create_body() {
    let mut world = world_with_a_cube();
    let (vertices, triangles) = grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let ball = Shape::new_sphere(0.5).unwrap();
    let above = RVec3::new(0.0, 3.0, 0.0);
    assert_accepted(
        &mut world,
        &ball,
        &BodySettings::new_dynamic().position(above),
    );
    assert_accepted(
        &mut world,
        &mesh,
        &BodySettings::new_kinematic()
            .mass(10.0)
            .position(RVec3::new(0.0, -3.0, 0.0)),
    );
    assert_accepted(
        &mut world,
        &thin_rotated_rod(),
        &BodySettings::new_static().position(RVec3::new(5.0, 0.0, 0.0)),
    );
    assert_accepted(
        &mut world,
        &flat_height_field(),
        &BodySettings::new_static().position(RVec3::new(0.0, -10.0, 0.0)),
    );
    step(&mut world, 5);
}

#[test]
fn a_full_world_is_not_a_refusal_of_check_body() {
    let mut world = PhysicsWorld::new(WorldSettings::default().max_bodies(1)).unwrap();
    let shape = cube_shape();
    world
        .create_body(&shape, &BodySettings::new_static())
        .unwrap();
    let settings = BodySettings::new_dynamic().position(RVec3::new(0.0, 3.0, 0.0));
    assert_eq!(world.check_body(&shape, &settings), Ok(()));
    assert_eq!(
        world.create_body(&shape, &settings),
        Err(BodyError::TooManyBodies)
    );
    assert_eq!(world.body_count(), 1);
}
