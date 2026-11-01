//! Shape changes of bodies: the mass properties they give, the bodies they wake, and the bodies
//! and shapes they refuse.

mod common;

use common::constraint_kinds::create_every_kind;
use common::controls::*;
use common::events::add_cloth;
use common::meshes::grid;
use common::ragdoll::{humanoid_settings, ragdoll_world};
use common::vehicle::{add_car, car_world};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// A compound of a box and a sphere, its centre of mass off its origin and its inertia not
/// diagonal.
fn lopsided() -> Shape {
    let block = Shape::new_box(Vec3::new(0.4, 0.2, 0.3)).unwrap();
    let ball = Shape::new_sphere(0.3).unwrap();
    Shape::new_compound(&[
        CompoundChild {
            shape: &block,
            position: Vec3::new(0.2, 0.0, 0.0),
            rotation: quat_about(Vec3::new(0.0, 0.6, 0.8), 0.5),
            user_data: 0,
        },
        CompoundChild {
            shape: &ball,
            position: Vec3::new(-0.3, 0.4, 0.1),
            rotation: Quat::IDENTITY,
            user_data: 1,
        },
    ])
    .unwrap()
}

/// What a dynamic body's mass and inertia show: its mass, and its velocities after the same
/// impulses, as bits.
fn mass_response(world: &mut PhysicsWorld, id: BodyId) -> Vec<u32> {
    let mut body = world.body_mut(id).unwrap();
    let at = body.position();
    body.add_angular_impulse(Vec3::new(0.3, -0.2, 0.5)).unwrap();
    body.add_impulse_at_point(
        Vec3::new(1.0, 2.0, -0.5),
        RVec3::new(at.x + 0.1, at.y - 0.2, at.z + 0.3),
    )
    .unwrap();
    let (v, w) = (body.linear_velocity(), body.angular_velocity());
    let mut bits = vec![body.mass().unwrap().to_bits()];
    bits.extend([v.x, v.y, v.z, w.x, w.y, w.z].map(f32::to_bits));
    bits
}

/// A world without gravity holding one body of `settings` and `shape`.
fn single(shape: &Shape, settings: &BodySettings) -> (PhysicsWorld, BodyId) {
    let mut world = world(Vec3::ZERO, 1);
    let id = world.create_body(shape, settings).unwrap();
    (world, id)
}

#[test]
fn set_shape_gives_the_mass_properties_of_a_fresh_body() {
    let posed = |settings: BodySettings| {
        settings
            .position(RVec3::new(1.0, 2.0, 3.0))
            .rotation(quat_about(Vec3::new(0.48, 0.6, -0.64), 0.7))
    };
    let new_shape = lopsided();
    for mass in [None, Some(3.0)] {
        let with_mass = |settings: BodySettings| match mass {
            Some(mass) => settings.mass(mass),
            None => settings,
        };
        let (mut fresh_world, fresh) =
            single(&new_shape, &with_mass(posed(BodySettings::new_dynamic())));
        let expected = mass_response(&mut fresh_world, fresh);
        step(&mut fresh_world, 1);
        let expected_motion = motion(&fresh_world, fresh);

        // A dynamic body is inspected directly.
        let (mut world, id) = single(&cube_shape(), &posed(BodySettings::new_dynamic().mass(7.0)));
        world
            .body_mut(id)
            .unwrap()
            .set_shape(&new_shape, mass, Activation::Activate)
            .unwrap();
        assert_eq!(mass_response(&mut world, id), expected, "{mass:?}");
        step(&mut world, 1);
        assert_eq!(motion(&world, id), expected_motion, "{mass:?}");

        // A kinematic body and a static one that may move are made dynamic after the change.
        for start in [
            BodySettings::new_kinematic(),
            BodySettings::new_static()
                .object_layer(ObjectLayer::MOVING)
                .allow_dynamic_or_kinematic(true),
        ] {
            let (mut world, id) = single(&cube_shape(), &posed(start));
            let mut body = world.body_mut(id).unwrap();
            body.set_shape(&new_shape, mass, Activation::Activate)
                .unwrap();
            body.set_motion_type(MotionType::Dynamic, Activation::Activate)
                .unwrap();
            assert_eq!(mass_response(&mut world, id), expected, "{mass:?}");
        }
    }
}

#[test]
fn set_shape_with_the_same_shape_and_a_new_mass_changes_the_mass_and_refuses_earlier_states() {
    let shape = cube_shape();
    let (mut world, id) = single(&shape, &BodySettings::new_dynamic().mass(2.0));
    let (mut fresh_world, fresh) = single(&shape, &BodySettings::new_dynamic().mass(5.0));
    let saved = world.save_state();
    world
        .body_mut(id)
        .unwrap()
        .set_shape(&shape, Some(5.0), Activation::Activate)
        .unwrap();
    assert_eq!(
        mass_response(&mut world, id),
        mass_response(&mut fresh_world, fresh)
    );
    assert_eq!(world.restore_state(&saved), Err(StateError::WorldChanged));
    // The very same shape and mass are a change too: Jolt does not save shapes.
    let saved = world.save_state();
    world
        .body_mut(id)
        .unwrap()
        .set_shape(&shape, Some(5.0), Activation::DontActivate)
        .unwrap();
    assert_eq!(world.restore_state(&saved), Err(StateError::WorldChanged));
}

#[test]
fn set_shape_with_none_restores_the_shapes_own_mass() {
    let shape = cube_shape();
    let (mut world, id) = single(&shape, &BodySettings::new_dynamic().mass(2.0));
    world
        .body_mut(id)
        .unwrap()
        .set_shape(&shape, None, Activation::Activate)
        .unwrap();
    let (mut fresh_world, fresh) = single(&shape, &BodySettings::new_dynamic());
    assert_eq!(
        mass_response(&mut world, id),
        mass_response(&mut fresh_world, fresh)
    );
}

#[test]
fn set_shape_keeps_the_origin_and_moves_the_centre_of_mass() {
    let (mut world, id) = single(
        &cube_shape(),
        &BodySettings::new_dynamic()
            .position(RVec3::new(1.0, 0.0, 0.0))
            .angular_damping(0.0),
    );
    let off_centre =
        Shape::new_offset_center_of_mass(&cube_shape(), Vec3::new(0.5, 0.0, 0.0)).unwrap();
    let mut body = world.body_mut(id).unwrap();
    body.set_shape(&off_centre, None, Activation::Activate)
        .unwrap();
    assert_eq!(body.position(), RVec3::new(1.0, 0.0, 0.0));
    // Spinning about its centre of mass at x = 1.5 swings the origin around it.
    body.set_angular_velocity(Vec3::new(0.0, 0.0, 1.0)).unwrap();
    step(&mut world, 30);
    let p = world.body(id).unwrap().position();
    let radius = distance(p, RVec3::new(1.5, 0.0, 0.0));
    assert!((radius - 0.5).abs() < 1.0e-4, "{radius}");
    assert!(p.y < -0.1, "{p:?}");
}

#[test]
fn set_shape_keeps_the_allowed_dofs() {
    let (mut world, id) = single(
        &cube_shape(),
        &BodySettings::new_dynamic().allowed_dofs(AllowedDofs::PLANE_2D),
    );
    let mut body = world.body_mut(id).unwrap();
    body.set_shape(&lopsided(), Some(2.0), Activation::Activate)
        .unwrap();
    assert_eq!(body.allowed_dofs(), AllowedDofs::PLANE_2D);
    body.add_angular_impulse(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    let w = body.angular_velocity();
    assert_eq!((w.x, w.y), (0.0, 0.0));
    assert!(w.z != 0.0);
}

/// A static 10 x 1 x 10 slab with its top at y = 0, and a cube resting asleep on it.
fn resting_cube(world: &mut PhysicsWorld) -> (BodyId, BodyId) {
    let slab = Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap();
    let floor = world
        .create_body(
            &slab,
            &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
        )
        .unwrap();
    let cube = add_cube(world, RVec3::new(0.0, 0.5, 0.0));
    for _ in 0..600 {
        if world.body(cube).unwrap().is_sleeping() {
            break;
        }
        step(world, 1);
    }
    assert!(world.body(cube).unwrap().is_sleeping());
    world.take_events();
    (floor, cube)
}

#[test]
fn a_growing_shape_pushes_a_resting_cube_up() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let (floor, cube) = resting_cube(&mut world);
    let thick = Shape::new_box(Vec3::new(5.0, 0.8, 5.0)).unwrap();
    world
        .body_mut(floor)
        .unwrap()
        .set_shape(&thick, None, Activation::Activate)
        .unwrap();
    assert_eq!(
        world.take_events().activations,
        [ActivationEvent::Activated(cube)]
    );
    step(&mut world, 60);
    let y = world.body(cube).unwrap().position().y;
    assert!(y > 0.7, "{y}");
}

#[test]
fn a_shrinking_static_floor_wakes_the_cube_on_it() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let (floor, cube) = resting_cube(&mut world);
    let thin = Shape::new_box(Vec3::new(5.0, 0.2, 5.0)).unwrap();
    world.take_events();
    world
        .body_mut(floor)
        .unwrap()
        .set_shape(&thin, None, Activation::Activate)
        .unwrap();
    assert_eq!(
        world.take_events().activations,
        [ActivationEvent::Activated(cube)]
    );
    step(&mut world, 30);
    // The thin slab's top is at y = -0.3, where the cube lands.
    let y = world.body(cube).unwrap().position().y;
    assert!((y - 0.2).abs() < 0.03, "{y}");
}

/// A sleeper resting within the speculative contact distance of the reshaped body is in Jolt's
/// contact range without touching its bounds; it wakes and falls onto the thinner slab.
#[test]
fn set_shape_wakes_a_sleeper_within_speculative_distance() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let (floor, cube) = resting_cube(&mut world);
    let gap = 0.5 * world.speculative_contact_distance();
    let lifted = RVec3::new(0.0, 0.5 + Real::from(gap), 0.0);
    world
        .body_mut(cube)
        .unwrap()
        .set_position(lifted, Activation::DontActivate)
        .unwrap();
    assert!(world.body(cube).unwrap().is_sleeping());
    world.take_events();
    let thin = Shape::new_box(Vec3::new(5.0, 0.2, 5.0)).unwrap();
    world
        .body_mut(floor)
        .unwrap()
        .set_shape(&thin, None, Activation::DontActivate)
        .unwrap();
    assert_eq!(
        world.take_events().activations,
        [ActivationEvent::Activated(cube)]
    );
    step(&mut world, 30);
    // The thin slab's top is at y = -0.3, where the cube lands.
    let y = world.body(cube).unwrap().position().y;
    assert!((y - 0.2).abs() < 0.03, "{y}");
}

#[test]
fn set_shape_wakes_bodies_between_old_and_new_bounds() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    add_floor(&mut world);
    let post_at = |x| {
        let post = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
        Shape::new_compound(&[child(&post, Vec3::new(x, 0.5, 0.0), 0)]).unwrap()
    };
    let post = world
        .create_body(&post_at(-10.0), &BodySettings::new_static())
        .unwrap();
    // Touches neither post, but lies inside the box enclosing both.
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    fall_asleep(&mut world, cube);
    world.take_events();
    world
        .body_mut(post)
        .unwrap()
        .set_shape(&post_at(10.0), None, Activation::DontActivate)
        .unwrap();
    assert_eq!(
        world.take_events().activations,
        [ActivationEvent::Activated(cube)]
    );
}

#[test]
fn set_shape_wakes_the_same_bodies_whether_or_not_the_broad_phase_was_optimized() {
    let woken = |optimize: bool| {
        let mut world = world(GRAVITY, 1);
        world.set_event_settings(EventSettings::default().body_activation(true));
        let (floor, _) = resting_cube(&mut world);
        // A sleeping cube moved away from the floor: Jolt's broad phase may still hold its old
        // bounds until the next maintenance.
        let moved = add_cube(&mut world, RVec3::new(3.0, 0.5, 0.0));
        for _ in 0..600 {
            if world.body(moved).unwrap().is_sleeping() {
                break;
            }
            step(&mut world, 1);
        }
        world
            .body_mut(moved)
            .unwrap()
            .set_position(RVec3::new(30.0, 0.5, 0.0), Activation::DontActivate)
            .unwrap();
        if optimize {
            world.optimize_broad_phase();
        }
        world.take_events();
        let thin = Shape::new_box(Vec3::new(5.0, 0.2, 5.0)).unwrap();
        world
            .body_mut(floor)
            .unwrap()
            .set_shape(&thin, None, Activation::Activate)
            .unwrap();
        // Raw ids: the two runs are different worlds.
        let events = world.take_events().activations;
        events
            .iter()
            .map(|event| {
                (
                    matches!(event, ActivationEvent::Activated(_)),
                    event.body().to_raw(),
                )
            })
            .collect::<Vec<_>>()
    };
    let plain = woken(false);
    assert_eq!(plain.len(), 1);
    assert_eq!(plain, woken(true));
}

#[test]
fn set_shape_applies_the_creation_rules_per_motion_type() {
    let mut world = world(Vec3::ZERO, 1);
    let terrain = flat_height_field();
    let (vertices, triangles) = grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let tiny = Shape::new_sphere(1.0e-20).unwrap();
    let dynamic = add_cube(&mut world, RVec3::ZERO);
    let kinematic = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic().position(RVec3::new(3.0, 0.0, 0.0)),
        )
        .unwrap();
    let movable = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static()
                .position(RVec3::new(6.0, 0.0, 0.0))
                .allow_dynamic_or_kinematic(true),
        )
        .unwrap();
    let sensor = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static()
                .position(RVec3::new(9.0, 0.0, 0.0))
                .sensor(true),
        )
        .unwrap();
    let plain = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static().position(RVec3::new(12.0, 0.0, 0.0)),
        )
        .unwrap();
    let saved = world.save_state();
    let refused: [(BodyId, &Shape, Option<f32>); 12] = [
        (dynamic, &terrain, Some(1.0)),
        (kinematic, &terrain, Some(1.0)),
        (movable, &terrain, Some(1.0)),
        (dynamic, &mesh, Some(1.0)),
        (kinematic, &mesh, None),
        (movable, &mesh, None),
        (dynamic, &tiny, None),
        (kinematic, &tiny, None),
        (dynamic, &cube_shape(), Some(limits::MIN_MASS / 2.0)),
        (plain, &cube_shape(), Some(limits::MAX_MASS * 2.0)),
        (sensor, &mesh, None),
        (sensor, &terrain, None),
    ];
    for (id, shape, mass) in refused {
        let mut before = Vec::new();
        record_body(&world, id, &mut before);
        assert!(
            invalid(
                world
                    .body_mut(id)
                    .unwrap()
                    .set_shape(shape, mass, Activation::Activate)
            ),
            "{id:?} {mass:?}"
        );
        let mut after = Vec::new();
        record_body(&world, id, &mut after);
        assert_eq!(after, before);
    }
    world.restore_state(&saved).unwrap();
    // A plain static body takes static-only shapes; a kinematic one a mesh with a mass.
    world
        .body_mut(plain)
        .unwrap()
        .set_shape(&terrain, None, Activation::Activate)
        .unwrap();
    world
        .body_mut(kinematic)
        .unwrap()
        .set_shape(&mesh, Some(10.0), Activation::Activate)
        .unwrap();
    step(&mut world, 5);
}

#[test]
fn set_shape_refuses_pending_forces() {
    let (mut world, id) = single(&cube_shape(), &BodySettings::new_dynamic());
    let mut body = world.body_mut(id).unwrap();
    body.add_force(Vec3::new(1.0, 0.0, 0.0)).unwrap();
    assert!(invalid(body.set_shape(
        &lopsided(),
        None,
        Activation::Activate
    )));
    body.reset_forces();
    body.add_torque(Vec3::new(0.0, 1.0, 0.0)).unwrap();
    assert!(invalid(body.set_shape(
        &lopsided(),
        None,
        Activation::Activate
    )));
    body.reset_forces();
    body.set_shape(&lopsided(), None, Activation::Activate)
        .unwrap();
}

#[test]
fn set_shape_refuses_owned_bodies() {
    let other = lopsided();
    let mut world = world(Vec3::ZERO, 1);
    let first = add_cube(&mut world, RVec3::ZERO);
    let second = add_cube(&mut world, RVec3::new(1.5, 0.0, 0.0));
    let created = create_every_kind(&mut world, first, second);
    assert!(created.iter().all(Result::is_ok), "{created:?}");
    let saved = world.save_state();
    for id in [first, second] {
        assert_eq!(
            world
                .body_mut(id)
                .unwrap()
                .set_shape(&other, None, Activation::Activate),
            Err(BodyError::UsedByConstraint(id))
        );
    }
    world.restore_state(&saved).unwrap();

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
    let saved = world.save_state();
    for (id, error) in [
        (part, BodyError::OwnedByRagdoll(part)),
        (inner, BodyError::OwnedByCharacter(inner)),
        (cloth, BodyError::NotRigidBody(cloth)),
    ] {
        assert_eq!(
            world
                .body_mut(id)
                .unwrap()
                .set_shape(&other, None, Activation::Activate),
            Err(error)
        );
    }
    world.restore_state(&saved).unwrap();

    let (mut world, layers) = car_world(GRAVITY, 1);
    let (chassis, _) = add_car(
        &mut world,
        &layers,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    assert_eq!(
        world
            .body_mut(chassis)
            .unwrap()
            .set_shape(&other, None, Activation::Activate),
        Err(BodyError::UsedByVehicle(chassis))
    );
}

#[test]
fn a_dropped_shape_lives_on_in_its_body() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
    {
        let ball = Shape::new_sphere(0.7).unwrap();
        world
            .body_mut(cube)
            .unwrap()
            .set_shape(&ball, None, Activation::Activate)
            .unwrap();
    }
    step(&mut world, 120);
    let y = world.body(cube).unwrap().position().y;
    assert!((y - 0.7).abs() < 0.03, "{y}");
    let hit = world
        .cast_ray(
            &RayCast::new(RVec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -10.0, 0.0)),
            &QueryFilter::new(),
        )
        .unwrap()
        .unwrap();
    assert_eq!(hit.body, cube);
}
