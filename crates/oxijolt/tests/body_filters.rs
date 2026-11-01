//! Object layer and collision group changes on live bodies (`BodyMut::set_object_layer`,
//! `BodyMut::set_collision_group`): what collides afterwards, what queries see, which bodies
//! wake, how the group's table is kept alive, and the refusals.

mod common;

use common::controls::{contact_kinds, fall_asleep};
use common::events::add_cloth;
use common::ragdoll::{humanoid_settings, ragdoll_world};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// The layers of [`filter_world`].
struct Layers {
    /// Static geometry, broad-phase layer 0.
    fixed: ObjectLayer,
    /// Moving bodies, broad-phase layer 1; collide with each other and with `fixed`.
    moving: ObjectLayer,
    /// Broad-phase layer 1; collides with nothing.
    ghost: ObjectLayer,
}

/// A world with gravity, contact and activation events, and the layers of [`Layers`].
fn filter_world() -> (PhysicsWorld, Layers) {
    let mut layers = CollisionLayers::new(2);
    let fixed = layers.add_object_layer(BroadPhaseLayer::new(0));
    let moving = layers.add_object_layer(BroadPhaseLayer::new(1));
    let ghost = layers.add_object_layer(BroadPhaseLayer::new(1));
    layers.enable_collision(moving, fixed);
    layers.enable_collision(moving, moving);
    let mut world = PhysicsWorld::new(jobs::with_threads(
        WorldSettings::default().gravity(GRAVITY).layers(layers),
        2,
    ))
    .unwrap();
    world.set_event_settings(
        EventSettings::default()
            .contacts(true)
            .body_activation(true),
    );
    (
        world,
        Layers {
            fixed,
            moving,
            ghost,
        },
    )
}

fn table(sub_groups: u32) -> GroupFilterTable {
    GroupFilterTableBuilder::new(sub_groups).unwrap().build()
}

fn group(table: &GroupFilterTable, group_id: u32, sub_group_id: u32) -> CollisionGroup {
    CollisionGroup::new(table, group_id, sub_group_id).unwrap()
}

fn height(world: &PhysicsWorld, id: BodyId) -> f64 {
    common::math::wide(world.body(id).unwrap().position().y)
}

fn activated(events: &WorldEvents) -> Vec<BodyId> {
    events
        .activations
        .iter()
        .filter_map(|event| match event {
            ActivationEvent::Activated(id) => Some(*id),
            ActivationEvent::Deactivated(_) => None,
        })
        .collect()
}

#[test]
fn layer_change_stops_a_collision() {
    let (mut world, layers) = filter_world();
    add_floor(&mut world);
    let ghost = add_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
    let control = add_cube(&mut world, RVec3::new(3.0, 3.0, 0.0));
    step(&mut world, 10);
    let mut body = world.body_mut(ghost).unwrap();
    body.set_object_layer(layers.ghost, Activation::Activate)
        .unwrap();
    assert_eq!(body.object_layer(), layers.ghost);
    step(&mut world, 120);
    assert!(height(&world, ghost) < -1.0, "the ghost passed through");
    assert!(
        (height(&world, control) - 0.5).abs() < 0.05,
        "the control rests"
    );
}

#[test]
fn layer_change_across_broad_phase_layers_is_seen_by_queries() {
    let (mut world, layers) = filter_world();
    let cube = add_cube(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let ray = RayCast::new(RVec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -20.0, 0.0));
    let hit_in = |world: &PhysicsWorld, layer: ObjectLayer| {
        world
            .cast_ray(&ray, &QueryFilter::new().object_layers(&[layer]))
            .unwrap()
            .map(|hit| hit.body)
    };
    assert_eq!(hit_in(&world, layers.moving), Some(cube));
    assert_eq!(hit_in(&world, layers.fixed), None);
    // From broad-phase layer 1 to broad-phase layer 0, before any step.
    world
        .body_mut(cube)
        .unwrap()
        .set_object_layer(layers.fixed, Activation::DontActivate)
        .unwrap();
    assert_eq!(hit_in(&world, layers.moving), None);
    assert_eq!(hit_in(&world, layers.fixed), Some(cube));
    // And back.
    world
        .body_mut(cube)
        .unwrap()
        .set_object_layer(layers.moving, Activation::DontActivate)
        .unwrap();
    assert_eq!(hit_in(&world, layers.moving), Some(cube));
    assert_eq!(hit_in(&world, layers.fixed), None);
}

#[test]
fn support_layer_change_wakes_sleepers_with_either_activation() {
    for activation in [Activation::Activate, Activation::DontActivate] {
        let (mut world, layers) = filter_world();
        let floor = add_floor(&mut world);
        let bottom: Vec<BodyId> = [-3.0, 0.0, 3.0]
            .into_iter()
            .map(|x| add_cube(&mut world, RVec3::new(x, 0.5, 0.0)))
            .collect();
        let top = add_cube(&mut world, RVec3::new(0.0, 1.5, 0.0));
        fall_asleep(&mut world, top);
        for &id in bottom.iter().chain([&top]) {
            assert!(world.body(id).unwrap().is_sleeping());
        }
        world.take_events();

        world
            .body_mut(floor)
            .unwrap()
            .set_object_layer(layers.ghost, activation)
            .unwrap();
        // The cubes touching the floor wake, in body-id order; the top cube touches only a cube.
        assert_eq!(activated(&world.take_events()), bottom, "{activation:?}");
        step(&mut world, 1);
        let contacts = world.take_events().contacts;
        for &id in &bottom {
            assert_eq!(contact_kinds(&contacts, floor, id), "r", "{activation:?}");
        }
        step(&mut world, 60);
        for &id in bottom.iter().chain([&top]) {
            assert!(
                height(&world, id) < -1.0,
                "{activation:?}: everything falls"
            );
        }
    }
}

#[test]
fn sleeping_changed_body_stays_asleep_without_activation() {
    for activation in [Activation::DontActivate, Activation::Activate] {
        let (mut world, layers) = filter_world();
        add_floor(&mut world);
        let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
        fall_asleep(&mut world, cube);
        let resting = world.body(cube).unwrap().position();
        world
            .body_mut(cube)
            .unwrap()
            .set_object_layer(layers.ghost, activation)
            .unwrap();
        step(&mut world, 60);
        let body = world.body(cube).unwrap();
        if activation == Activation::Activate {
            assert!(height(&world, cube) < -1.0, "it falls through the floor");
        } else {
            assert!(body.is_sleeping());
            assert_eq!(body.position(), resting);
        }
    }
}

#[test]
fn same_filter_wakes_nothing() {
    let (mut world, layers) = filter_world();
    let floor = add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    fall_asleep(&mut world, cube);
    world.take_events();
    let mut body = world.body_mut(floor).unwrap();
    body.set_object_layer(layers.fixed, Activation::Activate)
        .unwrap();
    body.set_collision_group(None, Activation::Activate)
        .unwrap();
    let mut body = world.body_mut(cube).unwrap();
    body.set_object_layer(layers.moving, Activation::Activate)
        .unwrap();
    body.set_collision_group(None, Activation::Activate)
        .unwrap();
    assert!(world.take_events().activations.is_empty());
    assert!(world.body(cube).unwrap().is_sleeping());
}

#[test]
fn group_change_stops_and_starts_collisions() {
    let mut world = world(GRAVITY, 2);
    world.set_event_settings(common::events::contacts());
    add_floor(&mut world);
    let crew = table(1);
    let member = group(&crew, 1, 0);
    let lower = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.5, 0.0))
                .collision_group(member.clone()),
        )
        .unwrap();
    let upper = add_cube(&mut world, RVec3::new(0.0, 1.6, 0.0));
    let before = common::controls::contact_events(&mut world, 30);
    assert!(contact_kinds(&before, lower, upper).starts_with('a'));
    assert!(
        height(&world, upper) > 1.4,
        "the upper cube rests on the lower one"
    );

    world
        .body_mut(upper)
        .unwrap()
        .set_collision_group(Some(&member), Activation::Activate)
        .unwrap();
    let excluded = common::controls::contact_events(&mut world, 10);
    assert_eq!(contact_kinds(&excluded, lower, upper), "r");
    assert!(
        height(&world, upper) < 1.4,
        "the upper cube sinks into the lower one"
    );

    world
        .body_mut(upper)
        .unwrap()
        .set_collision_group(None, Activation::Activate)
        .unwrap();
    let again = common::controls::contact_events(&mut world, 30);
    assert!(contact_kinds(&again, lower, upper).starts_with('a'));
}

#[test]
fn group_readback() {
    let mut world = world(GRAVITY, 1);
    let cube = add_cube(&mut world, RVec3::new(0.0, 5.0, 0.0));
    let crew = table(2);
    let member = group(&crew, 1, 0);
    assert!(world.body(cube).unwrap().collision_group_is(None));
    assert!(!world.body(cube).unwrap().collision_group_is(Some(&member)));

    world
        .body_mut(cube)
        .unwrap()
        .set_collision_group(Some(&member), Activation::Activate)
        .unwrap();
    let body = world.body(cube).unwrap();
    assert!(body.collision_group_is(Some(&member)));
    assert!(body.collision_group_is(Some(&member.clone())));
    assert!(!body.collision_group_is(None));
    let other_table = table(2);
    assert!(!body.collision_group_is(Some(&group(&other_table, 1, 0))));
    assert!(!body.collision_group_is(Some(&group(&crew, 1, 1))));
    assert!(!body.collision_group_is(Some(&group(&crew, 2, 0))));

    let (mut world, layers) = ragdoll_world(1);
    let ragdoll = world
        .create_ragdoll(
            &humanoid_settings(layers.ragdoll),
            None,
            Activation::Activate,
        )
        .unwrap();
    let part = world.ragdoll(ragdoll).unwrap().body_ids()[0];
    let body = world.body(part).unwrap();
    assert!(!body.collision_group_is(None));
    for sub_group in 0..2 {
        assert!(!body.collision_group_is(Some(&group(
            &crew,
            CollisionGroup::MAX_GROUP_ID,
            sub_group
        ))));
    }
}

#[test]
fn body_keeps_its_group_after_the_caller_drops_it() {
    let mut world = world(GRAVITY, 2);
    add_floor(&mut world);
    let lower = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    let upper = add_cube(&mut world, RVec3::new(0.0, 1.6, 0.0));
    {
        let crew = table(1);
        let member = group(&crew, 1, 0);
        for id in [lower, upper] {
            world
                .body_mut(id)
                .unwrap()
                .set_collision_group(Some(&member), Activation::Activate)
                .unwrap();
        }
    }
    // New tables reuse the memory a wrongly freed table would have left.
    for _ in 0..1000 {
        drop(table(1));
    }
    step(&mut world, 60);
    assert!(
        height(&world, upper) < 1.0,
        "the pair still does not collide"
    );
}

#[test]
fn group_change_on_a_soft_body() {
    let fallen = |grouped: bool| {
        let mut world = world(GRAVITY, 2);
        let crew = table(1);
        let member = group(&crew, 1, 0);
        world
            .create_body(
                &Shape::new_box(Vec3::new(2.0, 0.5, 2.0)).unwrap(),
                &BodySettings::new_static()
                    .position(RVec3::new(0.0, -0.5, 0.0))
                    .collision_group(member.clone()),
            )
            .unwrap();
        let cloth = add_cloth(&mut world, RVec3::new(0.0, 1.0, 0.0), Quat::IDENTITY);
        if grouped {
            world
                .body_mut(cloth)
                .unwrap()
                .set_collision_group(Some(&member), Activation::Activate)
                .unwrap();
            assert!(world.body(cloth).unwrap().collision_group_is(Some(&member)));
        }
        step(&mut world, 90);
        height(&world, cloth)
    };
    assert!(fallen(false) > -0.5, "the cloth lies on the box");
    assert!(fallen(true) < -2.0, "the cloth falls through the box");
}

#[test]
fn refusals_change_nothing() {
    let (mut world, layers) = filter_world();
    let floor = add_floor(&mut world);
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    fall_asleep(&mut world, cube);
    world.take_events();
    let saved = world.save_state();
    let unknown = ObjectLayer::new(3);
    assert_eq!(
        world
            .body_mut(floor)
            .unwrap()
            .set_object_layer(unknown, Activation::Activate),
        Err(BodyError::UnknownObjectLayer(unknown))
    );
    assert_eq!(world.body(floor).unwrap().object_layer(), layers.fixed);
    assert!(world.take_events().activations.is_empty());
    assert!(world.body(cube).unwrap().is_sleeping());
    world.restore_state(&saved).unwrap();

    let (mut world, layers) = ragdoll_world(1);
    let ragdoll = world
        .create_ragdoll(
            &humanoid_settings(layers.ragdoll),
            None,
            Activation::DontActivate,
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
    world.take_events();
    let saved = world.save_state();
    for layer in [layers.fixed, layers.ragdoll] {
        assert_eq!(
            world
                .body_mut(inner)
                .unwrap()
                .set_object_layer(layer, Activation::Activate),
            Err(BodyError::OwnedByCharacter(inner))
        );
    }
    assert_eq!(world.body(inner).unwrap().object_layer(), layers.ragdoll);
    let crew = table(1);
    for wanted in [None, Some(&group(&crew, 1, 0))] {
        assert_eq!(
            world
                .body_mut(part)
                .unwrap()
                .set_collision_group(wanted, Activation::Activate),
            Err(BodyError::OwnedByRagdoll(part))
        );
    }
    assert!(!world.body(part).unwrap().collision_group_is(None));
    assert!(world.body(part).unwrap().is_sleeping());
    assert!(world.take_events().activations.is_empty());
    world.restore_state(&saved).unwrap();

    // A character's inner body may take a group, and a ragdoll part a layer.
    let member = group(&crew, 1, 0);
    world
        .body_mut(inner)
        .unwrap()
        .set_collision_group(Some(&member), Activation::DontActivate)
        .unwrap();
    assert!(world.body(inner).unwrap().collision_group_is(Some(&member)));
    world
        .body_mut(part)
        .unwrap()
        .set_object_layer(layers.fixed, Activation::DontActivate)
        .unwrap();
    assert_eq!(world.body(part).unwrap().object_layer(), layers.fixed);
}
