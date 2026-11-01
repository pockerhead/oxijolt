//! The sensor flag on live bodies (`BodyMut::set_is_sensor`): how lasting contacts continue,
//! which bodies wake, how a contact listener and rollback see the change, and the refusals.

mod common;

use std::sync::{Arc, Mutex};

use common::controls::*;
use common::events::add_cloth;
use common::ragdoll::{humanoid_settings, ragdoll_world};
use common::vehicle::{add_car, car_world};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// Contact events, persisted ones too.
fn every_contact() -> EventSettings {
    EventSettings::default()
        .contacts(true)
        .persisted_contacts(true)
}

/// Sets the sensor flag of `id`.
fn set_sensor(
    world: &mut PhysicsWorld,
    id: BodyId,
    sensor: bool,
    activation: Activation,
) -> Result<(), BodyError> {
    world
        .body_mut(id)
        .unwrap()
        .set_is_sensor(sensor, activation)
}

/// The penetration depth of the last persisted contact between `a` and `b` in `events`.
fn last_persisted_depth(events: &[ContactEvent], a: BodyId, b: BodyId) -> Option<f32> {
    events.iter().rev().find_map(|event| match event {
        ContactEvent::Persisted { manifold, .. } if is_between(event, a, b) => {
            Some(manifold.penetration_depth)
        }
        _ => None,
    })
}

#[test]
fn floor_turned_sensor_lets_a_resting_box_fall() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(every_contact());
    let floor = add_floor(&mut world);
    // Dropped deep enough to settle into the floor: the contact's shapes then touch, and the
    // pair's contact survives the toggle.
    let cube = add_cube(&mut world, RVec3::new(0.0, 1.0, 0.0));
    let events = contact_events(&mut world, 40);
    let depth = last_persisted_depth(&events, floor, cube).unwrap();
    assert!(depth > 1e-3, "{depth}");
    assert!(world.body(cube).unwrap().is_active());

    set_sensor(&mut world, floor, true, Activation::DontActivate).unwrap();
    assert!(world.body(floor).unwrap().is_sensor());
    let first = contact_events(&mut world, 1);
    let contacts = sensor_contacts(&first, floor, cube);
    assert_eq!(contacts.len(), 1, "{contacts:?}");
    assert!(matches!(contacts[0], ('p', Some(settings)) if settings.is_sensor()));

    let events = contact_events(&mut world, 60);
    let contacts = sensor_contacts(&events, floor, cube);
    // The cube falls through the 2 m floor, reported as a sensor contact while it overlaps.
    let kinds = contact_kinds(&events, floor, cube);
    assert!(kinds.ends_with("pr"), "{kinds}");
    assert!(!kinds.contains('a'), "{kinds}");
    assert!(contacts
        .iter()
        .filter_map(|(_, settings)| *settings)
        .all(|settings| settings.is_sensor()));
    assert!(world.body(cube).unwrap().position().y < -2.5);
}

#[test]
fn a_speculative_contact_ends_when_the_body_becomes_a_sensor() {
    let mut world = world(Vec3::ZERO, 1);
    world.set_event_settings(every_contact());
    let floor = add_floor(&mut world);
    // 1 cm above the floor, within Jolt's speculative contact distance.
    let cube = add_cube(&mut world, RVec3::new(0.0, 0.51, 0.0));
    let events = contact_events(&mut world, 10);
    let depth = last_persisted_depth(&events, floor, cube).unwrap();
    assert!(depth < 0.0, "{depth}");

    set_sensor(&mut world, floor, true, Activation::DontActivate).unwrap();
    let events = contact_events(&mut world, 10);
    assert_eq!(contact_kinds(&events, floor, cube), "r");
}

#[test]
fn sensor_turned_solid_blocks_and_pushes_out() {
    let mut world = world(Vec3::ZERO, 1);
    world.set_event_settings(every_contact());
    let zone = add_static_sensor(&mut world, 1.0, RVec3::ZERO);
    // The cube's lower 0.3 m are inside the zone.
    let cube = add_cube(&mut world, RVec3::new(0.0, 1.2, 0.0));
    let events = contact_events(&mut world, 3);
    assert_eq!(contact_kinds(&events, zone, cube), "app");

    set_sensor(&mut world, zone, false, Activation::DontActivate).unwrap();
    assert!(!world.body(zone).unwrap().is_sensor());
    let first = contact_events(&mut world, 1);
    let contacts = sensor_contacts(&first, zone, cube);
    assert_eq!(contacts.len(), 1, "{contacts:?}");
    assert!(matches!(contacts[0], ('p', Some(settings)) if !settings.is_sensor()));

    step(&mut world, 60);
    assert_finite(&world, cube);
    // Pushed out to within Jolt's penetration slop of the zone's top face.
    let bottom = world.body(cube).unwrap().position().y - 0.5;
    assert!(bottom > 0.97, "{bottom}");
}

#[test]
fn toggled_sensor_wakes_its_sleepers() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    let floor = add_floor(&mut world);
    let lower = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    let upper = add_cube(&mut world, RVec3::new(0.0, 1.5, 0.0));
    fall_asleep(&mut world, upper);
    fall_asleep(&mut world, lower);
    world.take_events();

    set_sensor(&mut world, floor, true, Activation::DontActivate).unwrap();
    // The cube touching the floor wakes at once; the one on top wakes through its contact.
    assert!(world.body(lower).unwrap().is_active());
    step(&mut world, 60);
    for cube in [lower, upper] {
        let body = world.body(cube).unwrap();
        assert!(body.position().y < -1.0, "{:?}", body.position());
    }
}

/// Tries to turn every sensor contact into an ordinary one, and keeps the setter's results.
#[derive(Default)]
struct Desensitizer {
    results: Mutex<Vec<Result<(), ContactSettingsError>>>,
}

impl ContactListener for Desensitizer {
    fn contact_added(&self, _: &ContactManifold, settings: &mut ContactSettings) {
        if settings.is_sensor() {
            self.results
                .lock()
                .unwrap()
                .push(settings.set_is_sensor(false));
        }
    }
}

#[test]
fn listener_cannot_solidify_a_runtime_sensor() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    let zone = world
        .create_body(
            &Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap(),
            &BodySettings::new_static(),
        )
        .unwrap();
    set_sensor(&mut world, zone, true, Activation::DontActivate).unwrap();
    let listener = Arc::new(Desensitizer::default());
    world.set_contact_listener(Some(listener.clone()));
    let cube = add_cube(&mut world, RVec3::new(0.0, 3.0, 0.0));
    let events = contact_events(&mut world, 90);
    let results = listener.results.lock().unwrap().clone();
    assert!(!results.is_empty());
    assert!(results
        .iter()
        .all(|result| *result == Err(ContactSettingsError::SensorBody)));
    assert!(sensor_contacts(&events, zone, cube)
        .iter()
        .filter_map(|(_, settings)| *settings)
        .all(|settings| settings.is_sensor()));
    assert!(world.body(cube).unwrap().position().y < -2.0);
}

#[test]
fn static_sensor_made_at_run_time_detects_only_awake_bodies() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    add_floor(&mut world);
    // A solid zone around a cube created asleep: Jolt never pairs the two, so nothing pushes
    // the cube out.
    let zone = world
        .create_body(
            &Shape::new_box(Vec3::new(2.0, 2.0, 2.0)).unwrap(),
            &BodySettings::new_static().position(RVec3::new(0.0, 1.0, 0.0)),
        )
        .unwrap();
    let cube = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.5, 0.0))
                .activation(Activation::DontActivate),
        )
        .unwrap();
    step(&mut world, 5);
    world.take_events();

    // The toggle wakes the cube, the sensor detects it, and loses it when the cube falls
    // asleep again.
    set_sensor(&mut world, zone, true, Activation::DontActivate).unwrap();
    assert!(world.body(cube).unwrap().is_active());
    let mut events = Vec::new();
    for _ in 0..600 {
        events.extend(contact_events(&mut world, 1));
        if world.body(cube).unwrap().is_sleeping() {
            break;
        }
    }
    assert!(world.body(cube).unwrap().is_sleeping());
    events.extend(contact_events(&mut world, 30));
    assert_eq!(contact_kinds(&events, zone, cube), "ar");
}

#[test]
fn a_runtime_sensor_stays_awake_until_it_is_solid_again() {
    let mut world = world(Vec3::ZERO, 1);
    let cube = add_cube(&mut world, RVec3::ZERO);
    set_sensor(&mut world, cube, true, Activation::Activate).unwrap();
    step(&mut world, 300);
    assert!(world.body(cube).unwrap().is_active());
    set_sensor(&mut world, cube, false, Activation::Activate).unwrap();
    fall_asleep(&mut world, cube);
}

#[test]
fn a_constraint_body_accepts_the_toggle() {
    let mut world = world(Vec3::ZERO, 1);
    let first = add_cube(&mut world, RVec3::ZERO);
    let second = add_cube(&mut world, RVec3::new(1.5, 0.0, 0.0));
    world
        .create_constraint(
            first,
            second,
            &DistanceConstraintSettings::new(RVec3::ZERO, RVec3::new(1.5, 0.0, 0.0)),
        )
        .unwrap();
    set_sensor(&mut world, second, true, Activation::Activate).unwrap();
    assert!(world.body(second).unwrap().is_sensor());
    step(&mut world, 10);
    set_sensor(&mut world, second, false, Activation::Activate).unwrap();
    assert!(!world.body(second).unwrap().is_sensor());
}

/// Asserts that `refused` is `expected`, and that it changed neither the flag of `id` nor the
/// sleep state of `sleeper`, nor kept `saved` from restoring.
fn assert_refused(
    world: &mut PhysicsWorld,
    id: BodyId,
    sleeper: BodyId,
    saved: &WorldState,
    refused: Result<(), BodyError>,
    expected: fn(&BodyError) -> bool,
) {
    let error = refused.unwrap_err();
    assert!(expected(&error), "{error:?}");
    assert!(!world.body(id).unwrap().is_sensor());
    assert!(world.body(sleeper).unwrap().is_sleeping());
    world.restore_state(saved).unwrap();
}

#[test]
fn refusals_change_nothing() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let sleeper = add_cube(&mut world, RVec3::new(0.0, 0.5, 0.0));
    fall_asleep(&mut world, sleeper);
    // Every refused body touches the sleeper, which an accepted toggle would wake.
    let near = |x: Real| RVec3::new(x, 0.5, 0.0);
    let cloth = add_cloth(&mut world, RVec3::new(-0.5, 1.0, -0.5), Quat::IDENTITY);
    let fast = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(near(1.0))
                .motion_quality(MotionQuality::LinearCast)
                .activation(Activation::DontActivate),
        )
        .unwrap();
    let may_move = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_static()
                .position(near(-1.0))
                .allow_dynamic_or_kinematic(true)
                .motion_quality(MotionQuality::LinearCast),
        )
        .unwrap();
    let terrain = world
        .create_body(&flat_height_field(), &BodySettings::new_static())
        .unwrap();
    let (vertices, triangles) = common::meshes::grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let ramp = world
        .create_body(
            &mesh,
            &BodySettings::new_kinematic()
                .mass(1.0)
                .position(RVec3::new(0.0, 0.0, 0.0))
                .activation(Activation::DontActivate),
        )
        .unwrap();
    let saved = world.save_state();

    let refused = set_sensor(&mut world, cloth, true, Activation::Activate);
    let soft = |e: &BodyError| matches!(e, BodyError::NotRigidBody(_));
    assert_refused(&mut world, cloth, sleeper, &saved, refused, soft);
    let value = |e: &BodyError| matches!(e, BodyError::InvalidValue(_));
    for id in [fast, may_move, terrain, ramp] {
        let refused = set_sensor(&mut world, id, true, Activation::Activate);
        assert_refused(&mut world, id, sleeper, &saved, refused, value);
    }

    // Owned bodies.
    let (mut world, layers) = car_world(GRAVITY, 1);
    let (chassis, _) = add_car(
        &mut world,
        &layers,
        RVec3::new(0.0, 2.0, 0.0),
        Quat::IDENTITY,
    );
    let saved = world.save_state();
    assert_eq!(
        set_sensor(&mut world, chassis, true, Activation::Activate),
        Err(BodyError::UsedByVehicle(chassis))
    );
    assert!(!world.body(chassis).unwrap().is_sensor());
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
    let saved = world.save_state();
    let ragdolled = |e: &BodyError| matches!(e, BodyError::OwnedByRagdoll(_));
    let refused = set_sensor(&mut world, part, true, Activation::Activate);
    assert_refused(&mut world, part, part, &saved, refused, ragdolled);
    assert_eq!(
        set_sensor(&mut world, inner, true, Activation::Activate),
        Err(BodyError::OwnedByCharacter(inner))
    );
    assert!(!world.body(inner).unwrap().is_sensor());
    world.restore_state(&saved).unwrap();
}

#[test]
fn a_runtime_sensor_refuses_linear_cast_and_static_only_shapes() {
    let mut world = world(GRAVITY, 1);
    let cube = add_cube(&mut world, RVec3::new(0.0, 2.0, 0.0));
    set_sensor(&mut world, cube, true, Activation::Activate).unwrap();
    let mut body = world.body_mut(cube).unwrap();
    assert!(invalid(body.set_motion_quality(MotionQuality::LinearCast)));
    assert_eq!(body.motion_quality(), Some(MotionQuality::Discrete));
    let (vertices, triangles) = common::meshes::grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let kinematic = world
        .create_body(
            &cube_shape(),
            &BodySettings::new_kinematic()
                .mass(1.0)
                .position(RVec3::new(4.0, 0.0, 0.0)),
        )
        .unwrap();
    set_sensor(&mut world, kinematic, true, Activation::Activate).unwrap();
    let mut body = world.body_mut(kinematic).unwrap();
    assert!(invalid(body.set_shape(
        &mesh,
        Some(1.0),
        Activation::Activate
    )));
    assert!(body.is_sensor());
    // Made solid again, both are accepted.
    set_sensor(&mut world, kinematic, false, Activation::Activate).unwrap();
    world
        .body_mut(kinematic)
        .unwrap()
        .set_shape(&mesh, Some(1.0), Activation::Activate)
        .unwrap();
    set_sensor(&mut world, cube, false, Activation::Activate).unwrap();
    world
        .body_mut(cube)
        .unwrap()
        .set_motion_quality(MotionQuality::LinearCast)
        .unwrap();
}
