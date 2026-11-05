//! Detached bodies (`create_body_detached`, `add_body`, `destroy_body`) and batches
//! (`add_bodies`, `remove_bodies`): the detached state across the API, all-or-nothing checks,
//! caller slices, the batched neighbour wake, ids, world states and a batch-added pile.

mod common;

use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

fn cube_shape() -> Shape {
    Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap()
}

/// A dynamic unit cube at `position`, created and not added.
fn detached_cube(world: &mut PhysicsWorld, position: RVec3) -> BodyId {
    let settings = BodySettings::new_dynamic().position(position);
    world
        .create_body_detached(&cube_shape(), &settings)
        .unwrap()
}

/// A static box of half extent `half` at `position`, added.
fn static_box(world: &mut PhysicsWorld, half: Vec3, position: RVec3) -> BodyId {
    let shape = Shape::new_box(half).unwrap();
    let settings = BodySettings::new_static().position(position);
    world.create_body(&shape, &settings).unwrap()
}

/// A character with an inner body at `position`; returns the inner body's id.
fn character_inner_body(world: &mut PhysicsWorld, position: RVec3) -> BodyId {
    let inner_shape = Shape::new_capsule(0.6, 0.3).unwrap();
    let settings = CharacterSettings::humanoid(1.8, 0.3)
        .unwrap()
        .inner_body(Some(InnerBody {
            shape: &inner_shape,
            object_layer: ObjectLayer::MOVING,
        }));
    let character = world
        .create_character(&settings, position, Quat::IDENTITY)
        .unwrap();
    world.character(character).unwrap().inner_body().unwrap()
}

#[test]
fn a_detached_body_takes_a_place_but_is_not_in_the_world() {
    let mut world = world(GRAVITY, 1);
    let added = add_cube(&mut world, RVec3::new(5.0, 0.0, 0.0));
    let start = RVec3::new(0.0, 3.0, 0.0);
    let detached = detached_cube(&mut world, start);

    assert_eq!(world.body_count(), 2);
    assert!(world.body_ids().eq([added, detached]));
    assert!(!world.contains_body(detached));
    let refused = Err(BodyError::Detached(detached));
    assert_eq!(
        world.body(detached).err(),
        Some(BodyError::Detached(detached))
    );
    assert_eq!(
        world.body_mut(detached).err(),
        Some(BodyError::Detached(detached))
    );
    assert_eq!(world.remove_body(detached), refused);
    assert_eq!(
        world
            .create_constraint(added, detached, &PointConstraintSettings::new(start))
            .err(),
        Some(ConstraintError::Body(BodyError::Detached(detached)))
    );
    assert_eq!(
        world.save_state_of(BodySelection::Only(&[detached])).err(),
        Some(StateError::Body(BodyError::Detached(detached)))
    );
    assert_eq!(
        world.rebase(
            &[added, detached],
            Quat::IDENTITY,
            RVec3::new(1.0, 0.0, 0.0)
        ),
        refused
    );
    let down = RayCast::new(RVec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -20.0, 0.0));
    assert!(world
        .cast_ray(&down, &QueryFilter::new())
        .unwrap()
        .is_none());

    // Steps leave it where it was created; once added, it falls.
    step(&mut world, 10);
    world.add_body(detached, Activation::Activate).unwrap();
    assert!(world.contains_body(detached));
    assert_eq!(world.body(detached).unwrap().position(), start);
    assert_eq!(
        world.add_body(detached, Activation::Activate),
        Err(BodyError::AlreadyAdded(detached))
    );
    assert_eq!(
        world.destroy_body(detached),
        Err(BodyError::AlreadyAdded(detached))
    );
    assert_eq!(
        world
            .cast_ray(&down, &QueryFilter::new())
            .unwrap()
            .map(|hit| hit.body),
        Some(detached)
    );
    step(&mut world, 10);
    assert!(world.body(detached).unwrap().position().y < start.y);
}

#[test]
fn add_body_activation_decides_whether_the_body_wakes() {
    let mut world = world(GRAVITY, 1);
    let asleep = detached_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
    let awake = detached_cube(&mut world, RVec3::new(3.0, 3.0, 0.0));
    world.add_body(asleep, Activation::DontActivate).unwrap();
    world.add_body(awake, Activation::Activate).unwrap();
    assert!(!world.body(asleep).unwrap().is_active());
    assert!(world.body(awake).unwrap().is_active());
}

#[test]
fn destroy_body_frees_the_id_of_a_detached_body() {
    let mut world = world(GRAVITY, 1);
    let first = detached_cube(&mut world, RVec3::ZERO);
    let second = detached_cube(&mut world, RVec3::new(2.0, 0.0, 0.0));
    world.destroy_body(first).unwrap();
    assert_eq!(world.body_count(), 1);
    assert!(world.body_ids().eq([second]));
    assert_eq!(world.destroy_body(first), Err(BodyError::NotFound(first)));
    assert_eq!(
        world.add_body(first, Activation::Activate),
        Err(BodyError::NotFound(first))
    );

    let reused = detached_cube(&mut world, RVec3::ZERO);
    assert_eq!(reused.index(), first.index());
    assert_eq!(reused.sequence(), first.sequence() + 1);

    let mut other = common::world(GRAVITY, 1);
    assert_eq!(
        other.destroy_body(second),
        Err(BodyError::WrongWorld(second))
    );
    assert_eq!(
        other.add_body(second, Activation::Activate),
        Err(BodyError::WrongWorld(second))
    );
}

#[test]
fn detached_creation_checks_like_create_body() {
    let mut world = PhysicsWorld::new(WorldSettings::default().max_bodies(1)).unwrap();
    let bad = BodySettings::new_dynamic().friction(-1.0);
    assert_eq!(
        world.create_body_detached(&cube_shape(), &bad).err(),
        world.check_body(&cube_shape(), &bad).err()
    );
    assert_eq!(world.body_count(), 0);
    detached_cube(&mut world, RVec3::ZERO);
    assert_eq!(
        world
            .create_body_detached(&cube_shape(), &BodySettings::new_dynamic())
            .err(),
        Some(BodyError::TooManyBodies),
        "a detached body takes a place"
    );
}

#[test]
fn add_bodies_checks_every_id_first_and_adds_nothing_on_an_error() {
    let mut world = world(GRAVITY, 1);
    let mut foreign_world = common::world(GRAVITY, 1);
    let foreign = detached_cube(&mut foreign_world, RVec3::ZERO);
    let a = detached_cube(&mut world, RVec3::new(0.0, 1.0, 0.0));
    let b = detached_cube(&mut world, RVec3::new(2.0, 1.0, 0.0));
    let added = add_cube(&mut world, RVec3::new(4.0, 1.0, 0.0));
    let stale = detached_cube(&mut world, RVec3::new(6.0, 1.0, 0.0));
    world.destroy_body(stale).unwrap();

    for (ids, error) in [
        (vec![a, added], BodyError::AlreadyAdded(added)),
        (vec![b, a, b], BodyError::DuplicateId(b)),
        (vec![a, foreign], BodyError::WrongWorld(foreign)),
        (vec![a, stale, b], BodyError::NotFound(stale)),
    ] {
        assert_eq!(world.add_bodies(&ids, Activation::Activate), Err(error));
        assert!(
            !world.contains_body(a) && !world.contains_body(b),
            "{error:?}"
        );
    }

    world.add_bodies(&[], Activation::Activate).unwrap();
    let ids = [b, a];
    world.add_bodies(&ids, Activation::Activate).unwrap();
    assert_eq!(ids, [b, a], "the caller's slice is not reordered");
    assert!(ids.iter().all(|&id| world.body(id).unwrap().is_active()));
    assert_eq!(
        world.add_bodies(&[a], Activation::Activate),
        Err(BodyError::AlreadyAdded(a))
    );
}

#[test]
fn add_bodies_without_activation_leaves_them_asleep() {
    let mut world = world(GRAVITY, 1);
    let ids = [
        detached_cube(&mut world, RVec3::new(0.0, 3.0, 0.0)),
        detached_cube(&mut world, RVec3::new(2.0, 3.0, 0.0)),
    ];
    world.add_bodies(&ids, Activation::DontActivate).unwrap();
    assert!(ids.iter().all(|&id| !world.body(id).unwrap().is_active()));
}

/// A batch add wakes its bodies in ascending id order, whatever the order of the slice, and
/// skips static bodies.
#[test]
fn add_bodies_wakes_in_body_id_order() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let first = detached_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
    let static_shape = Shape::new_box(Vec3::new(1.0, 0.1, 1.0)).unwrap();
    let ground = world
        .create_body_detached(&static_shape, &BodySettings::new_static())
        .unwrap();
    let last = detached_cube(&mut world, RVec3::new(3.0, 3.0, 0.0));
    world
        .add_bodies(&[last, ground, first], Activation::Activate)
        .unwrap();
    assert_eq!(
        world.take_events().activations,
        vec![
            ActivationEvent::Activated(first),
            ActivationEvent::Activated(last)
        ]
    );
}

#[test]
fn remove_bodies_checks_every_id_first_and_removes_nothing_on_an_error() {
    let mut world = world(GRAVITY, 1);
    let mut foreign_world = common::world(GRAVITY, 1);
    let foreign = add_cube(&mut foreign_world, RVec3::ZERO);
    let a = add_cube(&mut world, RVec3::new(0.0, 1.0, 0.0));
    let b = add_cube(&mut world, RVec3::new(2.0, 1.0, 0.0));
    let detached = detached_cube(&mut world, RVec3::new(4.0, 1.0, 0.0));
    let anchor = static_box(
        &mut world,
        Vec3::new(0.5, 0.5, 0.5),
        RVec3::new(8.0, 5.0, 0.0),
    );
    let hanging = add_cube(&mut world, RVec3::new(8.0, 3.0, 0.0));
    world
        .create_constraint(
            anchor,
            hanging,
            &PointConstraintSettings::new(RVec3::new(8.0, 4.0, 0.0)),
        )
        .unwrap();
    let inner = character_inner_body(&mut world, RVec3::new(20.0, 0.0, 0.0));
    let stale = add_cube(&mut world, RVec3::new(12.0, 1.0, 0.0));
    world.remove_body(stale).unwrap();
    let count = world.body_count();

    for (ids, error) in [
        (vec![a, detached], BodyError::Detached(detached)),
        (vec![b, a, b], BodyError::DuplicateId(b)),
        (vec![a, hanging], BodyError::UsedByConstraint(hanging)),
        (vec![a, inner], BodyError::OwnedByCharacter(inner)),
        (vec![a, stale], BodyError::NotFound(stale)),
        (vec![a, foreign], BodyError::WrongWorld(foreign)),
    ] {
        assert_eq!(world.remove_bodies(&ids), Err(error));
        assert!(
            world.contains_body(a) && world.contains_body(b),
            "{error:?}"
        );
        assert_eq!(world.body_count(), count, "{error:?}");
    }

    world.remove_bodies(&[]).unwrap();
    let ids = [b, a];
    world.remove_bodies(&ids).unwrap();
    assert_eq!(ids, [b, a], "the caller's slice is not reordered");
    assert_eq!(world.body_count(), count - 2);
    assert_eq!(world.body(a).err(), Some(BodyError::NotFound(a)));
}

/// Two platforms side by side with a sleeping cube on each and one across both, and a sleeping
/// cube far away. Removing both platforms in one batch wakes each nearby cube once, in body-id
/// order, whatever the order of the slice; the far cube sleeps on.
#[test]
fn remove_bodies_wakes_the_union_once_in_body_id_order() {
    let mut world = world(GRAVITY, 1);
    let half = Vec3::new(1.0, 0.25, 1.0);
    let left = static_box(&mut world, half, RVec3::new(-1.0, -0.25, 0.0));
    let right = static_box(&mut world, half, RVec3::new(1.0, -0.25, 0.0));
    static_box(&mut world, half, RVec3::new(30.0, -0.25, 0.0));
    let on_left = add_cube(&mut world, RVec3::new(-1.5, 0.5, 0.0));
    let across = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    let on_right = add_cube(&mut world, RVec3::new(1.5, 0.5, 0.0));
    let far = add_cube(&mut world, RVec3::new(30.0, 0.5, 0.0));
    for cube in [on_left, across, on_right, far] {
        common::controls::fall_asleep(&mut world, cube);
    }
    world.set_event_settings(EventSettings::default().body_activation(true));
    world.take_events();

    let platforms = [right, left];
    world.remove_bodies(&platforms).unwrap();
    assert_eq!(platforms, [right, left]);
    assert_eq!(
        world.take_events().activations,
        vec![
            ActivationEvent::Activated(on_left),
            ActivationEvent::Activated(across),
            ActivationEvent::Activated(on_right),
        ]
    );
    assert!(world.body(far).unwrap().is_sleeping());
    step(&mut world, 30);
    for cube in [on_left, across, on_right] {
        assert!(
            world.body(cube).unwrap().position().y < 0.0,
            "the cubes fall"
        );
    }
}

/// The ids of a create/remove history, with each step done one by one or in batches.
fn id_history(batched: bool) -> Vec<u32> {
    let mut world = world(GRAVITY, 1);
    let create = |world: &mut PhysicsWorld, count: usize| -> Vec<BodyId> {
        let positions = (0..count).map(|i| RVec3::new(3.0 * i as Real, 1.0, 0.0));
        if batched {
            let ids: Vec<BodyId> = positions.map(|p| detached_cube(world, p)).collect();
            world.add_bodies(&ids, Activation::Activate).unwrap();
            ids
        } else {
            positions.map(|p| add_cube(world, p)).collect()
        }
    };
    let mut all = create(&mut world, 6);
    let removed = [all[4], all[1], all[2]];
    if batched {
        world.remove_bodies(&removed).unwrap();
    } else {
        for id in removed {
            world.remove_body(id).unwrap();
        }
    }
    all.extend(create(&mut world, 4));
    all.iter().map(|id| id.to_raw()).collect()
}

#[test]
fn batch_ids_equal_one_by_one_ids() {
    let one_by_one = id_history(false);
    assert_eq!(id_history(true), one_by_one);
    // The removed indices are reused, last freed first.
    let index = |raw: u32| raw & 0x7f_ffff;
    assert_eq!(
        one_by_one[6..9]
            .iter()
            .map(|&raw| index(raw))
            .collect::<Vec<_>>(),
        [2, 1, 4]
    );
}

#[test]
fn every_structure_change_refuses_earlier_states_and_refusals_keep_them() {
    let mut world = world(GRAVITY, 1);
    let mut foreign_world = common::world(GRAVITY, 1);
    let foreign = detached_cube(&mut foreign_world, RVec3::ZERO);
    let added = add_cube(&mut world, RVec3::new(4.0, 1.0, 0.0));

    let before = world.save_state();
    let detached = detached_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
    assert_eq!(world.restore_state(&before), Err(StateError::WorldChanged));

    // A state with a detached body restores, also after refused calls.
    let with_detached = world.save_state();
    step(&mut world, 5);
    assert_eq!(
        world.add_body(foreign, Activation::Activate),
        Err(BodyError::WrongWorld(foreign))
    );
    assert!(world
        .add_bodies(&[detached, detached], Activation::Activate)
        .is_err());
    assert!(world.remove_bodies(&[added, detached]).is_err());
    assert!(world.destroy_body(added).is_err());
    assert!(world
        .create_body_detached(&cube_shape(), &BodySettings::new_dynamic().friction(-1.0))
        .is_err());
    world.restore_state(&with_detached).unwrap();
    assert_eq!(
        world.body(added).unwrap().position(),
        RVec3::new(4.0, 1.0, 0.0)
    );

    let changes: [&dyn Fn(&mut PhysicsWorld); 4] = [
        &|world| world.add_body(detached, Activation::Activate).unwrap(),
        &|world| {
            let extra = detached_cube(world, RVec3::new(8.0, 3.0, 0.0));
            let state = world.save_state();
            world.add_bodies(&[extra], Activation::Activate).unwrap();
            assert_eq!(world.restore_state(&state), Err(StateError::WorldChanged));
        },
        &|world| {
            let extra = detached_cube(world, RVec3::new(12.0, 3.0, 0.0));
            let state = world.save_state();
            world.destroy_body(extra).unwrap();
            assert_eq!(world.restore_state(&state), Err(StateError::WorldChanged));
        },
        &|world| world.remove_bodies(&[detached]).unwrap(),
    ];
    for change in changes {
        let state = world.save_state();
        change(&mut world);
        assert_eq!(world.restore_state(&state), Err(StateError::WorldChanged));
    }
}

/// A floor and a 4 x 4 grid of stacks three cubes high, created detached and added in one
/// batch, optionally followed by a broad-phase optimization. Steps until every cube sleeps and
/// returns the resting heights.
fn settle_batch_added_stacks(optimize: bool) -> Vec<Real> {
    let mut world = world(GRAVITY, 2);
    add_floor(&mut world);
    let mut cubes = Vec::new();
    for x in 0..4 {
        for z in 0..4 {
            for layer in 0..3 {
                let position =
                    RVec3::new(1.5 * x as Real, 0.5 + 1.05 * layer as Real, 1.5 * z as Real);
                cubes.push(detached_cube(&mut world, position));
            }
        }
    }
    world.add_bodies(&cubes, Activation::Activate).unwrap();
    if optimize {
        world.optimize_broad_phase();
    }
    let mut ticks = 0;
    while !cubes
        .iter()
        .all(|&id| world.body(id).unwrap().is_sleeping())
    {
        step(&mut world, 1);
        ticks += 1;
        assert!(
            ticks < 600,
            "the stacks never fell asleep (optimize {optimize})"
        );
    }
    cubes
        .iter()
        .map(|&id| world.body(id).unwrap().position().y)
        .collect()
}

#[test]
fn a_batch_added_pile_collides_falls_and_sleeps() {
    for optimize in [false, true] {
        let heights = settle_batch_added_stacks(optimize);
        for (index, height) in heights.iter().enumerate() {
            let expected = 0.5 + (index % 3) as Real;
            assert!(
                (height - expected).abs() < 0.05,
                "cube {index} rests at {height}, expected about {expected} (optimize {optimize})"
            );
        }
    }
}
