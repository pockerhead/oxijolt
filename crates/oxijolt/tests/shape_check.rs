//! `BodyMut::check_shape`: the refusals of `set_shape` without the change, and the promise that
//! a shape it accepts `set_shape` takes.

mod common;

use common::constraint_kinds::create_every_kind;
use common::events::add_cloth;
use common::meshes::grid;
use common::ragdoll::{humanoid_settings, ragdoll_world};
use common::vehicle::{add_car, car_world};
use common::*;
use oxijolt::*;

/// What the world shows of `id`: pose, velocities, sleep, centre of mass, mass and inertia.
fn snapshot(world: &PhysicsWorld, id: BodyId) -> String {
    let mut digest = Vec::new();
    record_body(world, id, &mut digest);
    let body = world.body(id).unwrap();
    format!(
        "{digest:?} {:?} {:?} {:?}",
        body.center_of_mass_position(),
        body.mass(),
        body.inverse_inertia()
    )
}

/// Asserts that `check_shape` refuses `shape` and `mass` on `id` with `expected` and changes
/// nothing: the body reads the same, a state saved before still restores, and no body wakes.
/// Then asserts that `set_shape` refuses with the same error.
fn assert_refused(
    world: &mut PhysicsWorld,
    id: BodyId,
    shape: &Shape,
    mass: Option<f32>,
    expected: BodyError,
) {
    world.set_event_settings(EventSettings::default().body_activation(true));
    world.take_events();
    let saved = world.save_state();
    let before = snapshot(world, id);
    let checked = world.body_mut(id).unwrap().check_shape(shape, mass);
    assert_eq!(checked, Err(expected));
    assert_eq!(snapshot(world, id), before);
    assert!(world.take_events().activations.is_empty());
    world.restore_state(&saved).unwrap();
    let set = world
        .body_mut(id)
        .unwrap()
        .set_shape(shape, mass, Activation::Activate);
    assert_eq!(set, Err(expected));
}

/// Asserts that `check_shape` accepts `shape` and `mass` on `id` without changing anything,
/// and that `set_shape` then succeeds.
fn assert_accepted(world: &mut PhysicsWorld, id: BodyId, shape: &Shape, mass: Option<f32>) {
    world.set_event_settings(EventSettings::default().body_activation(true));
    world.take_events();
    let saved = world.save_state();
    let before = snapshot(world, id);
    world
        .body_mut(id)
        .unwrap()
        .check_shape(shape, mass)
        .unwrap();
    assert_eq!(snapshot(world, id), before);
    assert!(world.take_events().activations.is_empty());
    world.restore_state(&saved).unwrap();
    world
        .body_mut(id)
        .unwrap()
        .set_shape(shape, mass, Activation::DontActivate)
        .unwrap();
}

/// A thin rod 2 m long, rotated in a compound: what is left of a compound when the children
/// that gave it bulk split off. Its inertia is too badly conditioned for a dynamic body.
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

/// A world without gravity with a sleeping dynamic cube at the origin and a sleeping one beside
/// it.
fn sleeping_cubes() -> (PhysicsWorld, BodyId, BodyId) {
    let mut world = world(Vec3::ZERO, 1);
    let cube = add_cube(&mut world, RVec3::ZERO);
    let neighbour = add_cube(&mut world, RVec3::new(1.05, 0.0, 0.0));
    for id in [cube, neighbour] {
        world.body_mut(id).unwrap().deactivate().unwrap();
    }
    (world, cube, neighbour)
}

#[test]
fn a_thin_rotated_rod_remainder_is_refused_on_a_dynamic_body() {
    let (mut world, cube, _) = sleeping_cubes();
    let rod = thin_rotated_rod();
    for mass in [None, Some(5.0)] {
        assert_refused(
            &mut world,
            cube,
            &rod,
            mass,
            BodyError::InvalidValue(
                "mass and shape must give a finite inverse mass and a well-conditioned inertia",
            ),
        );
    }
    // A static body takes it.
    let wall = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(-3.0, 0.0, 0.0)),
        )
        .unwrap();
    assert_accepted(&mut world, wall, &rod, None);
}

#[test]
fn bad_masses_and_static_only_shapes_are_refused_as_set_shape_refuses_them() {
    let (mut world, cube, _) = sleeping_cubes();
    let (vertices, triangles) = grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let shape = cube_shape();
    let mass_rule =
        BodyError::InvalidValue("mass must be between limits::MIN_MASS and limits::MAX_MASS");
    for mass in [
        limits::MIN_MASS / 2.0,
        limits::MAX_MASS * 2.0,
        f32::NAN,
        -1.0,
    ] {
        assert_refused(&mut world, cube, &shape, Some(mass), mass_rule);
    }
    let error = world
        .body_mut(cube)
        .unwrap()
        .set_shape(&mesh, Some(1.0), Activation::Activate);
    assert!(
        matches!(error, Err(BodyError::InvalidValue(_))),
        "{error:?}"
    );
    assert_refused(&mut world, cube, &mesh, Some(1.0), error.unwrap_err());
}

#[test]
fn pending_forces_are_refused_until_reset() {
    let (mut world, cube, _) = sleeping_cubes();
    let shape = thin_rotated_rod();
    let lopsided =
        Shape::new_offset_center_of_mass(&cube_shape(), Vec3::new(0.2, 0.0, 0.0)).unwrap();
    world
        .body_mut(cube)
        .unwrap()
        .add_force(Vec3::new(1.0, 0.0, 0.0))
        .unwrap();
    let pending = world
        .body_mut(cube)
        .unwrap()
        .set_shape(&shape, None, Activation::Activate)
        .unwrap_err();
    assert!(matches!(pending, BodyError::InvalidValue(_)));
    assert_refused(&mut world, cube, &lopsided, None, pending);
    world.body_mut(cube).unwrap().reset_forces();
    assert_accepted(&mut world, cube, &lopsided, None);
}

#[test]
fn held_and_soft_bodies_are_refused() {
    let shape = Shape::new_sphere(0.4).unwrap();
    let mut world = world(Vec3::ZERO, 1);
    let first = add_cube(&mut world, RVec3::ZERO);
    let second = add_cube(&mut world, RVec3::new(1.5, 0.0, 0.0));
    let support = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(0.0, -2.0, 0.0)),
        )
        .unwrap();
    let created = create_every_kind(&mut world, first, second);
    assert!(created.iter().all(Result::is_ok), "{created:?}");
    world
        .create_constraint(support, first, &FixedConstraintSettings::default())
        .unwrap();
    for id in [first, second] {
        assert_refused(
            &mut world,
            id,
            &shape,
            None,
            BodyError::UsedByConstraint(id),
        );
    }
    // A static body that constraints use may change shape.
    assert_accepted(&mut world, support, &shape, None);

    let (mut world, layers) = ragdoll_world(1);
    let ragdoll = world
        .create_ragdoll(
            &humanoid_settings(layers.ragdoll),
            None,
            Activation::Activate,
        )
        .unwrap();
    let part = world.ragdoll(ragdoll).unwrap().body_ids()[0];
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
    let cloth = add_cloth(&mut world, RVec3::new(-5.0, 1.0, 0.0), Quat::IDENTITY);
    for (id, error) in [
        (part, BodyError::OwnedByRagdoll(part)),
        (inner, BodyError::OwnedByCharacter(inner)),
        (cloth, BodyError::NotRigidBody(cloth)),
    ] {
        assert_refused(&mut world, id, &shape, None, error);
    }

    let (mut world, layers) = car_world(Vec3::new(0.0, -9.81, 0.0), 1);
    let (chassis, _) = add_car(
        &mut world,
        &layers,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    assert_refused(
        &mut world,
        chassis,
        &shape,
        None,
        BodyError::UsedByVehicle(chassis),
    );
}

#[test]
fn accepted_shapes_are_taken_by_set_shape_on_every_motion_type() {
    let (mut world, cube, neighbour) = sleeping_cubes();
    let (vertices, triangles) = grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let kinematic = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic().position(RVec3::new(0.0, 5.0, 0.0)),
        )
        .unwrap();
    let ground = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(0.0, -5.0, 0.0)),
        )
        .unwrap();
    let ball = Shape::new_sphere(0.6).unwrap();
    assert_accepted(&mut world, cube, &ball, None);
    assert_accepted(&mut world, cube, &ball, Some(3.0));
    assert_accepted(&mut world, kinematic, &mesh, Some(10.0));
    assert_accepted(&mut world, ground, &flat_height_field(), None);
    // `check_shape` woke nothing; `set_shape` with `DontActivate` woke the neighbour it touches.
    assert!(world.body(neighbour).unwrap().is_active());
    step(&mut world, 5);
}
