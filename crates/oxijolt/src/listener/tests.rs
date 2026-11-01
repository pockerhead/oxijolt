//! Panic containment of the event callbacks: a test-only switch makes a chosen callback panic
//! on Jolt's worker threads or on the calling thread.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::Ordering;
use std::sync::Mutex;

use oxijolt_sys::{JPH_ContactSettings, JPH_SoftBodyContactSettings};

use super::contact::ContactFacts;
use super::*;
use crate::{
    Activation, BodyId, BodySettings, CompoundChild, ContactSettingsError, PhysicsWorld, Quat,
    RVec3, Shape, SoftBodyBendType, SoftBodySettings, SoftBodySharedSettings, SoftBodyVertex,
    SoftBodyVertexAttributes, Vec3, WorldSettings,
};

pub(super) const NO_PANIC: u8 = 0;
/// Every callback panics with a string payload.
const ALWAYS: u8 = 254;
/// Every callback panics with a payload whose `Drop` panics as well.
const ALWAYS_WITH_PANICKING_DROP: u8 = 255;

const DT: f32 = 1.0 / 60.0;

/// A panic payload whose drop panics again.
struct PanicsOnDrop;

impl Drop for PanicsOnDrop {
    fn drop(&mut self) {
        panic!("panic payload dropped");
    }
}

impl ListenerContext {
    /// Panics when the test switch names `callback` or every callback.
    pub(super) fn panic_if_asked(&self, callback: Callback) {
        match self.panic_in.load(Ordering::Relaxed) {
            ALWAYS_WITH_PANICKING_DROP => std::panic::panic_any(PanicsOnDrop),
            ALWAYS => panic!("listener test panic in every callback"),
            asked if asked == callback as u8 => panic!("listener test panic in {callback:?}"),
            _ => {}
        }
    }
}

fn set_panic(world: &PhysicsWorld, value: u8) {
    let context = world.listeners.context.as_ref().expect("events are on");
    context.panic_in.store(value, Ordering::Relaxed);
}

/// The bodies of the test scene.
struct Scene {
    cube: BodyId,
}

fn every_event() -> EventSettings {
    EventSettings::default()
        .persisted_contacts(true)
        .body_activation(true)
        .soft_body_contacts(true)
        .soft_body_validations(true)
}

/// Four workers; a cube that lands on a two-box compound floor, and a cloth that lands on a
/// table 10 m away.
fn scene() -> (PhysicsWorld, Scene) {
    let mut world = PhysicsWorld::new(WorldSettings::default().worker_threads(4)).unwrap();
    world.set_event_settings(every_event());
    let half = Shape::new_box(Vec3::new(1.0, 0.5, 1.0)).unwrap();
    let child = |x| CompoundChild {
        shape: &half,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data: 0,
    };
    let floor = Shape::new_compound(&[child(-1.0), child(1.0)]).unwrap();
    let at = |x, y| BodySettings::new_static().position(RVec3::new(x, y, 0.0));
    world.create_body(&floor, &at(0.0, -0.5)).unwrap();
    world.create_body(&half, &at(10.0, -0.5)).unwrap();
    let cube_shape = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    let cube = world
        .create_body(
            &cube_shape,
            &BodySettings::new_dynamic().position(RVec3::new(-1.0, 0.3, 0.0)),
        )
        .unwrap();
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    for z in 0..4 {
        for x in 0..4 {
            let position = Vec3::new(x as f32 * 0.25 - 0.375, 0.0, z as f32 * 0.25 - 0.375);
            vertices.push(SoftBodyVertex::new(position));
            if x < 3 && z < 3 {
                let v = z * 4 + x;
                faces.push([v, v + 4, v + 5]);
                faces.push([v, v + 5, v + 1]);
            }
        }
    }
    let cloth = SoftBodySharedSettings::builder(vertices, faces)
        .create_constraints(SoftBodyBendType::None, SoftBodyVertexAttributes::default())
        .build()
        .unwrap();
    world
        .create_soft_body(
            &cloth,
            &SoftBodySettings::default().position(RVec3::new(10.0, 0.3, 0.0)),
        )
        .unwrap();
    world.take_events();
    (world, Scene { cube })
}

fn message(payload: &(dyn Any + Send)) -> String {
    payload
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| payload.downcast_ref::<&str>().map(|s| (*s).to_owned()))
        .unwrap_or_default()
}

/// Steps until a step panics, lifting the cube after a second so that its contact is
/// removed; returns the payload.
fn step_until_panic(world: &mut PhysicsWorld, scene: &Scene) -> Box<dyn Any + Send> {
    for tick in 0..900 {
        if tick == 60 {
            let mut cube = world.body_mut(scene.cube).unwrap();
            cube.set_position(RVec3::new(-1.0, 3.0, 0.0), Activation::Activate)
                .unwrap();
        }
        if let Err(payload) = catch_unwind(AssertUnwindSafe(|| world.step(DT))) {
            return payload;
        }
    }
    panic!("no step panicked");
}

/// After a resumed panic the world steps, records and removes bodies as before.
fn assert_world_still_works(world: &mut PhysicsWorld, scene: &Scene) {
    set_panic(world, NO_PANIC);
    for _ in 0..5 {
        assert!(world.step(DT).unwrap().is_complete());
    }
    world.take_events();
    world.remove_body(scene.cube).unwrap();
    assert!(world.step(DT).unwrap().is_complete());
    let events = world.take_events();
    assert!(
        events
            .contacts
            .iter()
            .all(|event| event.pair().body1 != scene.cube
                || matches!(event, ContactEvent::Removed(_)))
    );
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn a_panic_in_any_callback_during_a_step_is_resumed_by_step() {
    for callback in [
        Callback::ContactValidate,
        Callback::ContactAdded,
        Callback::ContactPersisted,
        Callback::ContactRemoved,
        Callback::BodyDeactivated,
        Callback::SoftBodyValidate,
        Callback::SoftBodyAdded,
    ] {
        let (mut world, scene) = scene();
        set_panic(&world, callback as u8);
        let payload = step_until_panic(&mut world, &scene);
        assert_eq!(
            message(&*payload),
            format!("listener test panic in {callback:?}")
        );
        assert_world_still_works(&mut world, &scene);
    }
}

/// A validation that panics on every call accepts the hit, so the cube still lands on the floor,
/// and every step that validated resumes the panic.
#[test]
#[cfg(not(target_arch = "wasm32"))]
fn a_validate_panic_accepts_and_is_resumed_by_step() {
    let (mut world, scene) = scene();
    world.set_contact_listener(Some(Arc::new(RejectAll)));
    set_panic(&world, Callback::ContactValidate as u8);
    let mut panics = 0;
    for _ in 0..60 {
        if let Err(payload) = catch_unwind(AssertUnwindSafe(|| world.step(DT))) {
            assert_eq!(message(&*payload), "listener test panic in ContactValidate");
            panics += 1;
        }
    }
    assert!(panics > 0);
    let height = world.body(scene.cube).unwrap().position().y;
    assert!(height > 0.2, "the floor holds the cube: {height}");
    assert_world_still_works(&mut world, &scene);
}

/// Rejects every hit.
struct RejectAll;

impl ContactListener for RejectAll {
    fn contact_validate(&self, _: &ContactCandidate) -> ValidateResult {
        ValidateResult::RejectAllContactsForThisBodyPair
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn without_a_user_listener_validation_accepts_every_pair() {
    let (mut world, scene) = scene();
    assert!(world.listeners.context.as_ref().unwrap().listener.is_none());
    for _ in 0..60 {
        assert!(world.step(DT).unwrap().is_complete());
    }
    let height = world.body(scene.cube).unwrap().position().y;
    assert!(height > 0.2, "the floor holds the cube: {height}");
    world.set_contact_listener(Some(Arc::new(RejectAll)));
    world
        .body_mut(scene.cube)
        .unwrap()
        .set_position(RVec3::new(-1.0, 1.0, 0.0), Activation::Activate)
        .unwrap();
    for _ in 0..60 {
        assert!(world.step(DT).unwrap().is_complete());
    }
    let height = world.body(scene.cube).unwrap().position().y;
    assert!(height < -1.0, "a rejecting listener lets it fall: {height}");
}

/// A ball dropped on a sleeping cube wakes it inside a step, on a worker thread.
#[test]
#[cfg(not(target_arch = "wasm32"))]
fn a_panic_in_body_activation_during_a_step_is_resumed_by_step() {
    let mut world = PhysicsWorld::new(WorldSettings::default().worker_threads(4)).unwrap();
    world.set_event_settings(EventSettings::default().body_activation(true));
    let floor = Shape::new_box(Vec3::new(5.0, 0.5, 5.0)).unwrap();
    world
        .create_body(
            &floor,
            &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
        )
        .unwrap();
    let cube_shape = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    let cube = world
        .create_body(
            &cube_shape,
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 0.25, 0.0)),
        )
        .unwrap();
    let asleep = (0..300).any(|_| {
        assert!(world.step(DT).unwrap().is_complete());
        world
            .take_events()
            .activations
            .contains(&ActivationEvent::Deactivated(cube))
    });
    assert!(asleep, "the cube falls asleep");
    let ball_shape = Shape::new_sphere(0.2).unwrap();
    world
        .create_body(
            &ball_shape,
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 1.0, 0.0)),
        )
        .unwrap();
    world.take_events();

    set_panic(&world, Callback::BodyActivated as u8);
    let payload = (0..120)
        .find_map(|_| catch_unwind(AssertUnwindSafe(|| world.step(DT))).err())
        .expect("waking the cube panics");
    assert_eq!(message(&*payload), "listener test panic in BodyActivated");
    set_panic(&world, NO_PANIC);
    assert!(world.step(DT).unwrap().is_complete());
    world.take_events();
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn a_panic_outside_a_step_is_resumed_by_take_events() {
    let (mut world, scene) = scene();
    set_panic(&world, Callback::BodyActivated as u8);
    let shape = Shape::new_sphere(0.2).unwrap();
    let ball = world
        .create_body(
            &shape,
            &BodySettings::new_dynamic().position(RVec3::new(5.0, 2.0, 0.0)),
        )
        .unwrap();
    set_panic(&world, NO_PANIC);
    let payload = catch_unwind(AssertUnwindSafe(|| world.take_events())).unwrap_err();
    assert_eq!(message(&*payload), "listener test panic in BodyActivated");
    world.remove_body(ball).unwrap();
    assert_eq!(
        world.take_events().activations,
        vec![ActivationEvent::Deactivated(ball)]
    );
    assert_world_still_works(&mut world, &scene);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn an_unresumed_panic_between_steps_is_resumed_by_the_next_step() {
    let (mut world, _) = scene();
    set_panic(&world, Callback::BodyActivated as u8);
    let shape = Shape::new_sphere(0.2).unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_dynamic().position(RVec3::new(5.0, 2.0, 0.0)),
        )
        .unwrap();
    set_panic(&world, NO_PANIC);
    let payload = catch_unwind(AssertUnwindSafe(|| world.step(DT))).unwrap_err();
    assert_eq!(message(&*payload), "listener test panic in BodyActivated");
    assert!(world.step(DT).unwrap().is_complete());
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn always_panicking_callbacks_are_resumed_every_step() {
    let (mut world, scene) = scene();
    set_panic(&world, ALWAYS);
    // The first steps report nothing until the bodies come close.
    step_until_panic(&mut world, &scene);
    // A push keeps the cube awake, so its contact is reported (and panics) every step.
    let pushed_step = |world: &mut PhysicsWorld| {
        let mut cube = world.body_mut(scene.cube).unwrap();
        cube.set_linear_velocity(Vec3::new(0.2, 0.0, 0.0)).unwrap();
        catch_unwind(AssertUnwindSafe(|| world.step(DT))).unwrap_err()
    };
    for _ in 0..60 {
        let payload = pushed_step(&mut world);
        assert_eq!(message(&*payload), "listener test panic in every callback");
    }
    set_panic(&world, ALWAYS_WITH_PANICKING_DROP);
    for _ in 0..60 {
        let payload = pushed_step(&mut world);
        assert!(payload.is::<PanicsOnDrop>());
        // Dropping it would panic in this test.
        std::mem::forget(payload);
    }
    assert_world_still_works(&mut world, &scene);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn replacing_the_settings_keeps_events_and_panics() {
    let (mut world, _) = scene();
    set_panic(&world, Callback::BodyActivated as u8);
    let shape = Shape::new_sphere(0.2).unwrap();
    let ball = world
        .create_body(
            &shape,
            &BodySettings::new_dynamic().position(RVec3::new(5.0, 2.0, 0.0)),
        )
        .unwrap();
    world.remove_body(ball).unwrap();
    world.set_event_settings(EventSettings::default());
    assert!(world.listeners.context.is_none(), "nothing is installed");
    let payload = catch_unwind(AssertUnwindSafe(|| world.take_events())).unwrap_err();
    assert_eq!(message(&*payload), "listener test panic in BodyActivated");
    let events = world.take_events();
    assert_eq!(events.activations, vec![ActivationEvent::Deactivated(ball)]);
    assert!(world.step(DT).unwrap().is_complete());
    assert!(world.take_events().is_empty());
}

fn contact_settings(sensor_body: bool, lever_arm: f64) -> ContactSettings {
    let settings = JPH_ContactSettings {
        combinedFriction: 0.5,
        combinedRestitution: 0.0,
        invMassScale1: 1.0,
        invInertiaScale1: 1.0,
        invMassScale2: 1.0,
        invInertiaScale2: 1.0,
        isSensor: u32::from(sensor_body),
        relativeLinearSurfaceVelocity: Vec3::ZERO.to_jph(),
        relativeAngularSurfaceVelocity: Vec3::ZERO.to_jph(),
    };
    ContactSettings::new(
        &settings,
        ContactFacts {
            sensor_body,
            lever_arm,
        },
    )
}

/// The next `f32` above `value`.
fn next_up(value: f32) -> f32 {
    f32::from_bits(if value == 0.0 { 1 } else { value.to_bits() + 1 })
}

type Setter = fn(&mut ContactSettings, f32) -> Result<(), ContactSettingsError>;

/// Values beyond 1, which Jolt would take (2 halves a body's mass) and this API refuses.
fn beyond_one() -> Vec<f32> {
    vec![
        next_up(1.0),
        2.0,
        -f32::MIN_POSITIVE,
        -1.0,
        f32::NAN,
        f32::INFINITY,
    ]
}

/// Positive scales below `limits::MIN_CONTACT_SCALE`, down to ones that make Jolt's contact
/// impulse overflow.
fn below_the_scale_floor() -> Vec<f32> {
    let floor = crate::limits::MIN_CONTACT_SCALE;
    vec![
        floor.next_down(),
        1.0e-20,
        1.0e-36,
        f32::MIN_POSITIVE,
        1.0e-45,
    ]
}

#[test]
fn contact_settings_setters_accept_their_range_and_refuse_beyond() {
    let floor = crate::limits::MIN_CONTACT_SCALE;
    let scale_setters: [(Setter, ContactSettingsError); 4] = [
        (
            ContactSettings::set_inverse_mass_scale1,
            ContactSettingsError::InverseMassScale,
        ),
        (
            ContactSettings::set_inverse_mass_scale2,
            ContactSettingsError::InverseMassScale,
        ),
        (
            ContactSettings::set_inverse_inertia_scale1,
            ContactSettingsError::InverseInertiaScale,
        ),
        (
            ContactSettings::set_inverse_inertia_scale2,
            ContactSettingsError::InverseInertiaScale,
        ),
    ];
    let scales = scale_setters.into_iter().map(|(setter, error)| {
        let invalid = [beyond_one(), below_the_scale_floor()].concat();
        (setter, error, vec![0.0, -0.0, floor, 0.5, 1.0], invalid)
    });
    let restitution = (
        ContactSettings::set_combined_restitution as Setter,
        ContactSettingsError::Restitution,
        vec![0.0, 1.0e-36, 0.5, 1.0],
        beyond_one(),
    );
    for (setter, error, valid, invalid) in scales.chain([restitution]) {
        let mut settings = contact_settings(false, 0.0);
        for valid in valid {
            assert_eq!(setter(&mut settings, valid), Ok(()), "{valid}");
        }
        for invalid in invalid {
            let before = settings;
            assert_eq!(setter(&mut settings, invalid), Err(error), "{invalid}");
            assert_eq!(settings, before, "a refused value changes nothing");
        }
    }
    let mut settings = contact_settings(false, 0.0);
    let max = crate::limits::MAX_FRICTION;
    for valid in [0.0, 1.0, max] {
        assert_eq!(settings.set_combined_friction(valid), Ok(()));
    }
    for invalid in [next_up(max), -1.0, f32::NAN] {
        assert_eq!(
            settings.set_combined_friction(invalid),
            Err(ContactSettingsError::Friction)
        );
    }
    assert_eq!(settings.combined_friction(), max);
}

#[test]
fn a_contact_with_a_sensor_body_stays_a_sensor_contact() {
    let mut sensor = contact_settings(true, 0.0);
    assert!(sensor.is_sensor());
    assert_eq!(
        sensor.set_is_sensor(false),
        Err(ContactSettingsError::SensorBody)
    );
    assert_eq!(sensor.set_is_sensor(true), Ok(()));
    let mut ordinary = contact_settings(false, 0.0);
    assert_eq!(ordinary.set_is_sensor(true), Ok(()));
    assert_eq!(ordinary.set_is_sensor(false), Ok(()));
}

#[test]
fn surface_velocities_are_bounded_alone_and_together() {
    let max_linear = crate::limits::MAX_LINEAR_VELOCITY;
    let max_angular = crate::limits::MAX_ANGULAR_VELOCITY;
    let x = |value| Vec3::new(value, 0.0, 0.0);
    let mut settings = contact_settings(false, 0.0);
    assert_eq!(
        settings.set_relative_linear_surface_velocity(x(max_linear)),
        Ok(())
    );
    assert_eq!(
        settings.set_relative_linear_surface_velocity(x(next_up(max_linear))),
        Err(ContactSettingsError::SurfaceVelocity)
    );
    assert_eq!(
        settings.set_relative_linear_surface_velocity(x(f32::NAN)),
        Err(ContactSettingsError::SurfaceVelocity)
    );
    // With no lever, the angular velocity adds nothing at the contact.
    assert_eq!(
        settings.set_relative_angular_surface_velocity(x(max_angular)),
        Ok(())
    );
    assert_eq!(
        settings.set_relative_angular_surface_velocity(x(next_up(max_angular))),
        Err(ContactSettingsError::SurfaceVelocity)
    );

    // At the edge of a 2000 m shape, 0.25 rad/s move the surface by exactly 500 m/s.
    let lever = f64::from(crate::limits::MAX_SHAPE_EXTENT);
    let mut settings = contact_settings(false, lever);
    let refused = Err(ContactSettingsError::SurfaceVelocity);
    assert_eq!(
        settings.set_relative_angular_surface_velocity(x(0.25)),
        Ok(())
    );
    assert_eq!(
        settings.set_relative_linear_surface_velocity(x(0.001)),
        refused
    );
    assert_eq!(
        settings.set_relative_angular_surface_velocity(x(next_up(0.25))),
        refused
    );
    assert_eq!(
        settings.set_relative_angular_surface_velocity(x(0.125)),
        Ok(())
    );
    assert_eq!(
        settings.set_relative_linear_surface_velocity(x(250.0)),
        Ok(())
    );
    assert_eq!(
        settings.set_relative_linear_surface_velocity(x(next_up(250.0))),
        refused
    );
    assert_eq!(
        settings.set_relative_angular_surface_velocity(x(next_up(0.125))),
        refused
    );
    assert_eq!(settings.relative_angular_surface_velocity(), x(0.125));
    assert_eq!(settings.relative_linear_surface_velocity(), x(250.0));
}

#[test]
fn soft_body_contact_settings_setters_keep_scales_in_range() {
    let mut settings = SoftBodyContactSettings::from_jph(&JPH_SoftBodyContactSettings {
        invMassScale1: 1.0,
        invMassScale2: 1.0,
        invInertiaScale2: 1.0,
        isSensor: false,
    });
    type SoftSetter = fn(&mut SoftBodyContactSettings, f32) -> Result<(), ContactSettingsError>;
    let setters: [SoftSetter; 3] = [
        SoftBodyContactSettings::set_inverse_mass_scale1,
        SoftBodyContactSettings::set_inverse_mass_scale2,
        SoftBodyContactSettings::set_inverse_inertia_scale2,
    ];
    for setter in setters {
        for valid in [0.0, crate::limits::MIN_CONTACT_SCALE, 1.0] {
            assert_eq!(setter(&mut settings, valid), Ok(()));
        }
        for invalid in [beyond_one(), below_the_scale_floor()].concat() {
            assert!(setter(&mut settings, invalid).is_err(), "{invalid}");
        }
    }
    settings.set_is_sensor(true);
    assert!(settings.is_sensor());
}

#[test]
fn settings_are_checked_again_against_the_contact_that_takes_them() {
    let ordinary = ContactFacts {
        sensor_body: false,
        lever_arm: 0.5,
    };
    let sensor = ContactFacts {
        sensor_body: true,
        ..ordinary
    };
    let long_lever = ContactFacts {
        lever_arm: 60.0,
        ..ordinary
    };
    let mut kept = contact_settings(false, ordinary.lever_arm);
    kept.set_relative_angular_surface_velocity(Vec3::new(0.0, 20.0, 0.0))
        .unwrap();
    assert_eq!(kept.checked_for(ordinary), Ok(kept));
    assert_eq!(
        kept.checked_for(sensor),
        Err(ContactSettingsError::SensorBody)
    );
    assert_eq!(
        kept.checked_for(long_lever),
        Err(ContactSettingsError::SurfaceVelocity)
    );
    // An accepted value takes the facts of its new contact, so its setters check against them.
    let mut moved = contact_settings(false, 0.0)
        .checked_for(long_lever)
        .unwrap();
    assert_eq!(
        moved.set_relative_angular_surface_velocity(Vec3::new(0.0, 20.0, 0.0)),
        Err(ContactSettingsError::SurfaceVelocity)
    );
    let sensor_contact = contact_settings(true, 0.0).checked_for(sensor).unwrap();
    assert!(sensor_contact.checked_for(ordinary).is_ok());
}

#[test]
fn equality_and_debug_leave_out_the_contact_facts() {
    let near = contact_settings(false, 0.5);
    let far = contact_settings(false, 60.0);
    assert_eq!(near, far);
    assert_eq!(format!("{near:?}"), format!("{far:?}"));
    assert!(!format!("{near:?}").contains("lever"));
    let mut spun = near;
    spun.set_relative_angular_surface_velocity(Vec3::new(0.0, 1.0, 0.0))
        .unwrap();
    assert_ne!(near, spun);
}

/// A listener that changes nothing.
struct Unchanged;

impl ContactListener for Unchanged {}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn a_rejection_is_recorded_and_the_listener_is_still_called() {
    let mut world = PhysicsWorld::new(WorldSettings::default().worker_threads(1)).unwrap();
    let shape = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
    let body = world
        .create_body(&shape, &BodySettings::new_static())
        .unwrap();
    world.set_contact_listener(Some(Arc::new(Unchanged)));
    let context = world.listeners.context.clone().expect("a listener is set");
    let pair = SubShapeIdPair {
        body1: body,
        sub_shape1: crate::SubShapeId::new(0),
        body2: body,
        sub_shape2: crate::SubShapeId::new(0),
    };
    context.reject(pair, ContactSettingsError::SensorBody);
    assert_eq!(
        context.call_listener(|| 7),
        Some(7),
        "the listener is called"
    );
    assert!(world.step(DT).is_ok(), "a rejection is not a panic");
    let rejection = ContactSettingsRejection {
        pair,
        error: ContactSettingsError::SensorBody,
    };
    assert_eq!(
        world.take_events().rejected_contact_settings,
        vec![rejection]
    );
}

/// Keeps the settings of the donor's first contact, spun by `spin`, and assigns them over the
/// settings of every other contact in Added or in Persisted calls.
struct Transplant {
    donor: BodyId,
    spin: Vec3,
    in_persisted: bool,
    kept: Mutex<Option<ContactSettings>>,
}

impl Transplant {
    fn call(&self, persisted: bool, manifold: &ContactManifold, settings: &mut ContactSettings) {
        let pair = manifold.pair;
        let mut kept = self.kept.lock().unwrap();
        if pair.body1 == self.donor || pair.body2 == self.donor {
            if kept.is_none() {
                settings
                    .set_relative_angular_surface_velocity(self.spin)
                    .unwrap();
                *kept = Some(*settings);
            }
        } else if persisted == self.in_persisted {
            if let Some(kept) = *kept {
                *settings = kept;
            }
        }
    }
}

impl ContactListener for Transplant {
    fn contact_added(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        self.call(false, manifold, settings);
    }

    fn contact_persisted(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        self.call(true, manifold, settings);
    }
}

/// The settings of the contact between `a` and `b` recorded by Added (or Persisted) events.
fn recorded_settings(
    events: WorldEvents,
    persisted: bool,
    a: BodyId,
    b: BodyId,
) -> Option<ContactSettings> {
    events.contacts.into_iter().find_map(|event| {
        let (manifold, settings) = match event {
            ContactEvent::Added {
                manifold, settings, ..
            } if !persisted => (manifold, settings),
            ContactEvent::Persisted { manifold, settings } if persisted => (manifold, settings),
            _ => return None,
        };
        let pair = manifold.pair;
        let bodies = [pair.body1, pair.body2];
        (bodies == [a, b] || bodies == [b, a]).then_some(settings)
    })
}

/// A listener assigns settings kept from an ordinary contact near the floor's centre to a
/// contact with a sensor body, or to a contact 60 m from the floor's centre, whose lever turns
/// the kept 20 rad/s into 1 200 m/s. Jolt keeps its own settings, so an asserts build does not
/// reach Jolt's sensor assertion, and the step reports the refusal.
#[test]
#[cfg(not(target_arch = "wasm32"))]
fn settings_moved_to_a_contact_they_do_not_fit_are_rejected() {
    let cases = [
        (
            RVec3::new(5.0, 1.5, 0.0),
            Vec3::ZERO,
            ContactSettingsError::SensorBody,
        ),
        (
            RVec3::new(60.0, 0.3, 0.0),
            Vec3::new(0.0, 20.0, 0.0),
            ContactSettingsError::SurfaceVelocity,
        ),
    ];
    for (recipient_at, spin, error) in cases {
        for in_persisted in [false, true] {
            let mut world = PhysicsWorld::new(WorldSettings::default().worker_threads(1)).unwrap();
            world.set_event_settings(EventSettings::default().persisted_contacts(true));
            let at = |x, y| BodySettings::new_static().position(RVec3::new(x, y, 0.0));
            let floor_shape = Shape::new_box(Vec3::new(100.0, 0.5, 100.0)).unwrap();
            let floor = world.create_body(&floor_shape, &at(0.0, -0.5)).unwrap();
            let sensor_shape = Shape::new_box(Vec3::new(1.0, 1.0, 1.0)).unwrap();
            let sensor = world.create_body(&sensor_shape, &at(5.0, 1.0)).unwrap();
            // SAFETY: the body interface is the live world's, and no step runs.
            unsafe {
                oxijolt_sys::JPH_BodyInterface_SetIsSensor(
                    world.body_interface.as_ptr(),
                    sensor.to_raw(),
                    true,
                );
            }
            let cube = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
            let dynamic = |position| BodySettings::new_dynamic().position(position);
            let donor = world
                .create_body(&cube, &dynamic(RVec3::new(0.0, 0.3, 0.0)))
                .unwrap();
            let listener = Arc::new(Transplant {
                donor,
                spin,
                in_persisted,
                kept: Mutex::default(),
            });
            world.set_contact_listener(Some(listener.clone()));
            for _ in 0..30 {
                assert!(world.step(DT).unwrap().is_complete());
            }
            assert!(listener.kept.lock().unwrap().is_some());
            world.take_events();

            let recipient = world.create_body(&cube, &dynamic(recipient_at)).unwrap();
            let target = match error {
                ContactSettingsError::SensorBody => sensor,
                _ => floor,
            };
            let mut events = WorldEvents::default();
            let mut reported = 0;
            for _ in 0..10 {
                reported += world.step(DT).unwrap().rejected_contact_settings;
                events.append(world.take_events());
            }
            assert!(!events.rejected_contact_settings.is_empty());
            assert_eq!(
                reported as usize,
                events.rejected_contact_settings.len(),
                "the step reports count the events"
            );
            for rejection in &events.rejected_contact_settings {
                assert_eq!(rejection.error, error, "persisted: {in_persisted}");
                let bodies = [rejection.pair.body1, rejection.pair.body2];
                assert!(bodies.contains(&recipient) && bodies.contains(&target));
            }
            let recorded = recorded_settings(events, in_persisted, target, recipient)
                .expect("the contact is recorded");
            assert_eq!(
                recorded.is_sensor(),
                target == sensor,
                "Jolt's own settings"
            );
            assert_eq!(recorded.relative_angular_surface_velocity(), Vec3::ZERO);
        }
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn a_panic_while_copying_an_estimate_frees_joltcs_impulses() {
    use super::estimate::tests::{CREATED, FREED, PANIC_WHILE_COPYING};
    // The only test in this binary that turns estimates on, so the counters are its own.
    let (mut world, landing) = scene();
    world.set_event_settings(every_event().collision_estimates(true));
    PANIC_WHILE_COPYING.store(true, Ordering::SeqCst);
    let payload = step_until_panic(&mut world, &landing);
    PANIC_WHILE_COPYING.store(false, Ordering::SeqCst);
    assert_eq!(message(&*payload), "injected estimate copy panic");
    let created = CREATED.load(Ordering::SeqCst);
    assert!(created > 0);
    assert_eq!(FREED.load(Ordering::SeqCst), created);
    // Without the injection the cube's landing is estimated.
    let (mut world, _) = scene();
    world.set_event_settings(EventSettings::default().collision_estimates(true));
    let mut estimated = false;
    for _ in 0..60 {
        assert!(world.step(DT).unwrap().is_complete());
        for event in world.take_events().contacts {
            if let ContactEvent::Added { estimate, .. } = event {
                estimated |= estimate.is_some();
            }
        }
    }
    assert!(estimated);
    assert_eq!(FREED.load(Ordering::SeqCst), CREATED.load(Ordering::SeqCst));
}
