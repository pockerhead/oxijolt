//! Contact, activation and soft body events observed through `PhysicsWorld::take_events`.

mod common;

use std::sync::Arc;

use common::events::*;
use common::*;
use oxijolt::*;

#[test]
fn a_default_world_reports_nothing() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    assert_eq!(world.event_settings(), EventSettings::default());
    add_two_box_floor(&mut world);
    add_small_cube(&mut world, RVec3::new(-1.0, 0.3, 0.0));
    step(&mut world, 30);
    assert!(world.take_events().is_empty());
}

#[test]
fn contacts_are_added_persisted_and_removed_with_their_sub_shapes() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    world.set_event_settings(EventSettings::default().persisted_contacts(true));
    let floor = add_two_box_floor(&mut world);
    let cube = add_small_cube(&mut world, RVec3::new(1.0, 0.3, 0.0));
    let mut kinds_seen = String::new();
    let mut added = None;
    for _ in 0..30 {
        step(&mut world, 1);
        let events = world.take_events();
        kinds_seen.extend(kinds(&events));
        for event in &events.contacts {
            if let ContactEvent::Added {
                manifold, settings, ..
            } = event
            {
                added = Some(manifold.clone());
                assert!(!settings.is_sensor());
            }
        }
    }
    assert!(kinds_seen.starts_with("ap"), "{kinds_seen}");
    assert!(!kinds_seen.contains('r'), "{kinds_seen}");
    let manifold = added.expect("the cube lands");
    let pair = manifold.pair;
    assert!(pair.body1.to_raw() < pair.body2.to_raw());
    assert_eq!((pair.body1, pair.body2), (floor, cube));
    let child = world
        .compound_sub_shape(floor, pair.sub_shape1)
        .unwrap()
        .unwrap();
    assert_eq!(child.user_data, 11, "the cube lands on the +x child");
    // The normal moves body 2 (the cube) out of body 1 (the floor): up.
    assert!(manifold.normal.y > 0.99, "{manifold:?}");
    assert!(!manifold.points.is_empty());
    for point in &manifold.points {
        assert!(
            point.point_on1.y.abs() < 0.1 && point.point_on2.y.abs() < 0.1,
            "{point:?}"
        );
    }
    assert_eq!(manifold.materials, [None, None]);

    lift(&mut world, cube, RVec3::new(1.0, 5.0, 0.0));
    step(&mut world, 1);
    let events = world.take_events();
    assert_eq!(events.contacts, vec![ContactEvent::Removed(pair)]);
}

#[test]
fn persisted_contacts_are_opt_in() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_event_settings(contacts());
    assert!(!world.event_settings().reports_persisted_contacts());
    add_two_box_floor(&mut world);
    add_small_cube(&mut world, RVec3::new(-1.0, 0.3, 0.0));
    step(&mut world, 30);
    let kinds = kinds(&world.take_events());
    assert!(kinds.contains(&'a') && !kinds.contains(&'p'), "{kinds:?}");
    let settings = EventSettings::default()
        .persisted_contacts(true)
        .contacts(false);
    assert_eq!(settings, EventSettings::default());
}

#[test]
fn materials_are_reported_for_each_side() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_event_settings(contacts());
    let ice = PhysicsMaterial::new(7).unwrap();
    let rubber = PhysicsMaterial::new(9).unwrap();
    let floor_shape = Shape::new_box_with_material(Vec3::new(5.0, 0.5, 5.0), 0.05, &ice).unwrap();
    let floor = world
        .create_body(
            &floor_shape,
            &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
        )
        .unwrap();
    let ball_shape = Shape::new_sphere_with_material(0.3, &rubber).unwrap();
    let ball = world
        .create_body(
            &ball_shape,
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 0.4, 0.0)),
        )
        .unwrap();
    drop((ice, rubber));
    step(&mut world, 30);
    let events = world.take_events();
    let ContactEvent::Added { manifold, .. } = &events.contacts[0] else {
        panic!("{events:?}");
    };
    assert_eq!((manifold.pair.body1, manifold.pair.body2), (floor, ball));
    assert_eq!(manifold.materials, [Some(7), Some(9)]);
}

#[test]
fn removing_a_body_reports_its_contact_removed_with_the_stale_id() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    world.set_event_settings(contacts());
    let floor = add_two_box_floor(&mut world);
    let cube = add_small_cube(&mut world, RVec3::new(-1.0, 0.3, 0.0));
    step(&mut world, 20);
    world.take_events();
    world.remove_body(cube).unwrap();
    step(&mut world, 1);
    let events = world.take_events();
    assert_eq!(events.contacts.len(), 1, "{events:?}");
    let pair = events.contacts[0].pair();
    assert!(matches!(events.contacts[0], ContactEvent::Removed(_)));
    assert_eq!((pair.body1, pair.body2), (floor, cube));
    assert!(world.body(cube).is_err(), "the id is stale");
}

#[test]
fn falling_asleep_removes_contacts_and_deactivates() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    world.set_event_settings(contacts().body_activation(true));
    add_two_box_floor(&mut world);
    let cube = add_small_cube(&mut world, RVec3::new(-1.0, 0.3, 0.0));
    let mut events = WorldEvents::default();
    for _ in 0..300 {
        step(&mut world, 1);
        let step_events = world.take_events();
        events.contacts.extend(step_events.contacts);
        events.activations.extend(step_events.activations);
        if !world.body(cube).unwrap().is_active() {
            break;
        }
    }
    assert!(!world.body(cube).unwrap().is_active(), "the cube sleeps");
    assert_eq!(
        events.activations,
        vec![
            ActivationEvent::Activated(cube),
            ActivationEvent::Deactivated(cube)
        ]
    );
    // The sleep step reports no contact, the next one the removal.
    step(&mut world, 1);
    let after = world.take_events();
    assert_eq!(kinds(&after), ['r'], "{after:?}");
}

#[test]
fn activation_follows_creation_wake_sleep_and_removal_but_not_restore() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    add_two_box_floor(&mut world);
    let cube = add_small_cube(&mut world, RVec3::new(-1.0, 0.3, 0.0));
    assert_eq!(
        world.take_events().activations,
        vec![ActivationEvent::Activated(cube)],
        "creation"
    );
    let saved_awake = world.save_state();
    for _ in 0..300 {
        step(&mut world, 1);
        if !world.body(cube).unwrap().is_active() {
            break;
        }
    }
    assert_eq!(
        world.take_events().activations,
        vec![ActivationEvent::Deactivated(cube)],
        "sleep"
    );
    // Restoring a state in which the cube was awake wakes it without an event.
    world.restore_state(&saved_awake).unwrap();
    assert!(world.body(cube).unwrap().is_active());
    assert!(world.take_events().activations.is_empty(), "restore");
    for _ in 0..300 {
        step(&mut world, 1);
        if !world.body(cube).unwrap().is_active() {
            break;
        }
    }
    world.take_events();
    world
        .body_mut(cube)
        .unwrap()
        .set_linear_velocity(Vec3::new(1.0, 0.0, 0.0))
        .unwrap();
    assert_eq!(
        world.take_events().activations,
        vec![ActivationEvent::Activated(cube)],
        "wake by velocity"
    );
    world.remove_body(cube).unwrap();
    assert_eq!(
        world.take_events().activations,
        vec![ActivationEvent::Deactivated(cube)],
        "removal while awake"
    );
}

#[test]
fn enabling_and_disabling_contacts_mid_run() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_two_box_floor(&mut world);
    let cube = add_small_cube(&mut world, RVec3::new(-1.0, 0.3, 0.0));
    step(&mut world, 20);
    world.set_event_settings(EventSettings::default().persisted_contacts(true));
    step(&mut world, 1);
    // Jolt reports against its contact cache, which already holds the contact.
    assert_eq!(kinds(&world.take_events()), ['p']);
    world.set_event_settings(EventSettings::default());
    lift(&mut world, cube, RVec3::new(-1.0, 5.0, 0.0));
    step(&mut world, 1);
    assert!(world.take_events().is_empty());
    // Events recorded before a change stay queued.
    // The cube lands after 59 steps and is still awake after 80.
    world.set_event_settings(contacts());
    step(&mut world, 80);
    world.set_event_settings(EventSettings::default());
    assert_eq!(kinds(&world.take_events()), ['a']);
}

#[test]
fn several_steps_queue_in_step_order() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    world.set_event_settings(contacts());
    add_two_box_floor(&mut world);
    let cube = add_small_cube(&mut world, RVec3::new(-1.0, 0.3, 0.0));
    step(&mut world, 20);
    lift(&mut world, cube, RVec3::new(-1.0, 3.0, 0.0));
    // It lands again after 45 steps and is still awake after 65.
    step(&mut world, 65);
    let kinds = kinds(&world.take_events());
    assert_eq!(kinds, ['a', 'r', 'a']);
}

#[test]
fn added_and_persisted_pairs_were_in_contact() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 4);
    world.set_event_settings(EventSettings::default().persisted_contacts(true));
    let ids = build_stacks(&mut world);
    let mut checked = 0;
    for _ in 0..120 {
        assert!(world.step(DT).unwrap().is_complete());
        for event in world.take_events().contacts {
            if let ContactEvent::Added { manifold, .. } | ContactEvent::Persisted { manifold, .. } =
                event
            {
                let pair = manifold.pair;
                assert!(world
                    .were_bodies_in_contact(pair.body1, pair.body2)
                    .unwrap());
                checked += 1;
            }
        }
    }
    assert!(checked > 100, "{checked}");
    assert!(ids.len() > 2);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn worlds_stepped_on_two_threads_see_only_their_own_events() {
    let run = |extra_bodies: usize| {
        std::thread::spawn(move || {
            let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
            world.set_event_settings(contacts().body_activation(true));
            add_two_box_floor(&mut world);
            for i in 0..extra_bodies {
                add_small_cube(&mut world, RVec3::new(20.0 + i as Real, 0.3, 0.0));
            }
            let cube = add_small_cube(&mut world, RVec3::new(-1.0, 0.3, 0.0));
            let mut events = WorldEvents::default();
            for _ in 0..60 {
                step(&mut world, 1);
                let step_events = world.take_events();
                events.contacts.extend(step_events.contacts);
                events.activations.extend(step_events.activations);
            }
            (cube.index(), extra_bodies, events)
        })
    };
    let handles = [run(0), run(3)];
    for handle in handles {
        let (cube, extra, events) = handle.join().unwrap();
        // Bodies 1..=extra are the extra cubes, extra + 1 the cube; nothing else.
        let expected_bodies: Vec<u32> = (1..=extra as u32 + 1).collect();
        assert_eq!(cube, extra as u32 + 1);
        let mut activated: Vec<u32> = events
            .activations
            .iter()
            .map(|e| e.body().index())
            .collect();
        activated.sort_unstable();
        activated.dedup();
        assert_eq!(activated, expected_bodies);
        assert_eq!(
            events
                .contacts
                .iter()
                .filter(|e| matches!(e, ContactEvent::Added { .. }))
                .count(),
            1,
            "only the cube lands; the extra cubes fall beside the floor"
        );
    }
}

#[test]
fn dropping_worlds_with_every_kind_of_object_and_events_on_is_clean() {
    for with_events in [false, true] {
        let (mut world, layers) = vehicle::car_world(Vec3::new(0.0, -9.81, 0.0), 2);
        if with_events {
            world.set_event_settings(every_event());
        }
        let ground = Shape::new_box(Vec3::new(50.0, 0.5, 50.0)).unwrap();
        world
            .create_body(
                &ground,
                &BodySettings::new_static()
                    .position(RVec3::new(0.0, -0.5, 0.0))
                    .object_layer(layers.ground),
            )
            .unwrap();
        vehicle::add_car(
            &mut world,
            &layers,
            RVec3::new(0.0, 1.0, 0.0),
            Quat::IDENTITY,
        );
        let a = add_small_cube(&mut world, RVec3::new(5.0, 2.0, 0.0));
        let b = add_small_cube(&mut world, RVec3::new(5.0, 3.0, 0.0));
        world
            .create_constraint(
                a,
                b,
                &PointConstraintSettings::new(RVec3::new(5.0, 2.5, 0.0)),
            )
            .unwrap();
        let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
        let inner = Shape::new_capsule(0.5, 0.3).unwrap();
        let settings = CharacterSettings::new(&capsule).inner_body(Some(InnerBody {
            shape: &inner,
            object_layer: ObjectLayer::MOVING,
        }));
        world
            .create_character(&settings, RVec3::new(-5.0, 1.0, 0.0), Quat::IDENTITY)
            .unwrap();
        step(&mut world, 10);
        drop(world);
    }
    let (mut world, layers) = ragdoll::ragdoll_world(2);
    world.set_event_settings(every_event());
    world
        .create_ragdoll(
            &ragdoll::humanoid_settings(layers.ragdoll),
            None,
            Activation::Activate,
        )
        .unwrap();
    step(&mut world, 10);
    assert!(!world.take_events().is_empty());
    drop(world);
}

// Soft bodies.

#[test]
fn a_cloth_on_a_table_reports_its_vertex_contacts() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    world.set_event_settings(every_event());
    let table = add_table(&mut world, 0.0);
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 0.3, 0.0), Quat::IDENTITY);
    let contacts = soft_contacts(&mut world, 60);
    let last = contacts.last().expect("the cloth lands");
    assert_eq!(last.soft_body, cloth);
    assert!(last.sensors.is_empty());
    assert_eq!(last.vertices.len(), 36, "every vertex lies on the table");
    assert!(last.vertices.windows(2).all(|w| w[0].vertex < w[1].vertex));
    for contact in &last.vertices {
        assert_eq!(contact.body, table);
        assert!(contact.position.y.abs() < 0.02, "{contact:?}");
        // Into the table: minus its upward surface normal.
        assert!(contact.normal.y < -0.99, "{contact:?}");
    }
    assert!(!world.were_bodies_in_contact(cloth, table).unwrap());
}

#[test]
fn rotated_cloth_contacts_are_converted_to_world_space() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_event_settings(EventSettings::default().soft_body_contacts(true));
    let table = add_table(&mut world, 0.0);
    // Half a turn about a horizontal diagonal: the cloth stays flat, its frame differs from the
    // world frame.
    let half = std::f32::consts::FRAC_1_SQRT_2;
    let rotation = quat_about(Vec3::new(half, 0.0, half), std::f32::consts::PI);
    let cloth = add_cloth(&mut world, RVec3::new(0.3, 0.3, -0.2), rotation);
    let contacts = soft_contacts(&mut world, 60);
    let last = contacts.last().expect("the cloth lands");
    let vertices = world.soft_body(cloth).unwrap().vertices();
    for contact in &last.vertices {
        assert_eq!(contact.body, table);
        assert!(contact.position.y.abs() < 0.02, "{contact:?}");
        assert!(contact.normal.y < -0.99, "{contact:?}");
        let vertex = vertices[contact.vertex as usize].position;
        let dx = (vertex.x - contact.position.x).abs();
        let dz = (vertex.z - contact.position.z).abs();
        assert!(dx < 0.05 && dz < 0.05, "{contact:?} vs {vertex:?}");
    }
}

#[test]
fn two_cloths_are_reported_in_id_order() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 4);
    world.set_event_settings(EventSettings::default().soft_body_contacts(true));
    add_table(&mut world, 0.0);
    add_table(&mut world, 10.0);
    let first = add_cloth(&mut world, RVec3::new(10.0, 0.3, 0.0), Quat::IDENTITY);
    let second = add_cloth(&mut world, RVec3::new(0.0, 0.3, 0.0), Quat::IDENTITY);
    for _ in 0..60 {
        step(&mut world, 1);
        let ids: Vec<BodyId> = world
            .take_events()
            .soft_body_contacts
            .iter()
            .map(|c| c.soft_body)
            .collect();
        if ids.len() == 2 {
            assert_eq!(ids, [first, second]);
            return;
        }
    }
    panic!("both cloths should land");
}

#[test]
fn a_bounding_box_overlap_is_validated_without_vertex_contact() {
    let mut world = world(Vec3::ZERO, 1);
    world.set_event_settings(
        EventSettings::default()
            .soft_body_contacts(true)
            .soft_body_validations(true),
    );
    // A thin post through the middle of a cell of a weightless cloth: the bounding boxes
    // overlap, and the nearest vertices stay 9 cm away from the post.
    let post_shape = Shape::new_box(Vec3::new(0.01, 1.0, 0.01)).unwrap();
    let post = world
        .create_body(&post_shape, &BodySettings::new_static())
        .unwrap();
    let cloth = add_cloth(&mut world, RVec3::ZERO, Quat::IDENTITY);
    step(&mut world, 5);
    let events = world.take_events();
    assert!(
        events
            .soft_body_validations
            .iter()
            .any(|v| v.soft_body == cloth && v.other == post),
        "{events:?}"
    );
    assert!(events
        .soft_body_contacts
        .iter()
        .all(|c| c.vertices.is_empty()));
}

/// Steps a scene of a falling cube pile and a cloth with or without events and returns every
/// body's state and the cloth's vertices, tick by tick.
fn observed_run(settings: EventSettings, listener: Option<Arc<dyn ContactListener>>) -> Vec<u8> {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    world.set_event_settings(settings);
    world.set_contact_listener(listener);
    let mut ids = build_stacks(&mut world);
    ids.push(add_table(&mut world, 10.0));
    let cloth = add_cloth(&mut world, RVec3::new(10.0, 0.3, 0.0), Quat::IDENTITY);
    ids.push(cloth);
    let mut digest = Vec::new();
    for _ in 0..120 {
        step(&mut world, 1);
        world.take_events();
        for &id in &ids {
            record_body(&world, id, &mut digest);
        }
        // `Debug` prints floats exactly (shortest round-trip form, `-0.0` apart from `0.0`).
        for vertex in world.soft_body(cloth).unwrap().vertices() {
            digest.extend_from_slice(format!("{vertex:?}").as_bytes());
        }
    }
    digest
}

#[test]
fn observation_leaves_the_simulation_bit_identical() {
    let silent = observed_run(EventSettings::default(), None);
    let observed = observed_run(every_event(), Some(Arc::new(NoOp)));
    assert!(silent == observed, "events changed the simulation");
}

// Replay and the other kinds of objects.

/// Per-tick events and body states over `ticks` steps.
fn record_ticks(world: &mut PhysicsWorld, ids: &[BodyId], ticks: usize) -> Vec<(String, Vec<u8>)> {
    (0..ticks)
        .map(|_| {
            step(world, 1);
            let events = format!("{:?}", world.take_events());
            let mut state = Vec::new();
            for &id in ids {
                record_body(world, id, &mut state);
            }
            (events, state)
        })
        .collect()
}

#[test]
fn a_replay_after_a_detour_reports_the_same_events() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    world.set_event_settings(every_event());
    world.set_contact_listener(Some(Arc::new(RoughContacts)));
    let ids = build_stacks(&mut world);
    step(&mut world, 30);
    world.take_events();
    let saved = world.save_state();
    let reference = record_ticks(&mut world, &ids, 90);

    // A detour without structural changes: other velocities, a teleport, other gravity.
    world.restore_state(&saved).unwrap();
    for &id in &ids[1..] {
        world
            .body_mut(id)
            .unwrap()
            .set_linear_velocity(Vec3::new(0.0, 4.0, 1.0))
            .unwrap();
    }
    world
        .body_mut(ids[3])
        .unwrap()
        .set_position(RVec3::new(-3.0, 4.0, 0.0), Activation::Activate)
        .unwrap();
    world.set_gravity(Vec3::new(1.0, -3.0, 0.0)).unwrap();
    step(&mut world, 40);
    world.take_events();

    world.restore_state(&saved).unwrap();
    assert!(world.take_events().is_empty(), "a restore reports nothing");
    let replay = record_ticks(&mut world, &ids, 90);
    for (tick, (a, b)) in reference.iter().zip(&replay).enumerate() {
        assert_eq!(a.0, b.0, "events differ at tick {tick}");
        assert!(a.1 == b.1, "state differs at tick {tick}");
    }
    assert!(reference
        .iter()
        .any(|(events, _)| events.contains("Persisted")));
}

#[test]
fn character_inner_bodies_and_vehicle_chassis_report_rigid_contacts() {
    let (mut world, layers) = vehicle::car_world(Vec3::new(0.0, -9.81, 0.0), 2);
    world.set_event_settings(contacts());
    let ground = Shape::new_box(Vec3::new(50.0, 0.5, 50.0)).unwrap();
    world
        .create_body(
            &ground,
            &BodySettings::new_static()
                .position(RVec3::new(0.0, -0.5, 0.0))
                .object_layer(layers.ground),
        )
        .unwrap();
    let crate_shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let obstacle = |world: &mut PhysicsWorld, x, z| {
        world
            .create_body(
                &crate_shape,
                &BodySettings::new_dynamic()
                    .position(RVec3::new(x, 0.5, z))
                    .object_layer(layers.moving),
            )
            .unwrap()
    };
    let (chassis, car) = vehicle::add_car(
        &mut world,
        &layers,
        RVec3::new(0.0, 1.0, 0.0),
        Quat::IDENTITY,
    );
    let crate_ahead = obstacle(&mut world, 0.0, 8.0);
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    // The inner body is 10 cm wider than the character, so it overlaps what the character
    // walks into instead of stopping at the character's padding.
    let wider = Shape::new_capsule(0.5, 0.4).unwrap();
    let settings = CharacterSettings::new(&capsule).inner_body(Some(InnerBody {
        shape: &wider,
        object_layer: layers.moving,
    }));
    let character = world
        .create_character(&settings, RVec3::new(-10.0, 0.8, 0.0), Quat::IDENTITY)
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    let crate_beside = obstacle(&mut world, -8.0, 0.0);
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    let mut pairs = Vec::new();
    for _ in 0..180 {
        let mut vehicle = world.vehicle_mut(car).unwrap();
        vehicle.set_gravity(gravity).unwrap();
        vehicle
            .set_driver_input(DriverInput {
                forward: 1.0,
                ..DriverInput::default()
            })
            .unwrap();
        world
            .character_mut(character)
            .unwrap()
            .set_linear_velocity(Vec3::new(2.0, 0.0, 0.0))
            .unwrap();
        world
            .update_character(
                character,
                DT,
                gravity,
                &ExtendedUpdateSettings::default(),
                &QueryFilter::new(),
            )
            .unwrap();
        step(&mut world, 1);
        for event in world.take_events().contacts {
            let pair = event.pair();
            pairs.push((pair.body1, pair.body2));
        }
    }
    let touched = |a: BodyId, b: BodyId| {
        pairs
            .iter()
            .any(|&(x, y)| (x, y) == (a, b) || (x, y) == (b, a))
    };
    assert!(touched(chassis, crate_ahead), "the car runs into the crate");
    assert!(
        touched(inner, crate_beside),
        "the character walks into the crate"
    );
}
