//! Contact listeners that reject contacts before Jolt resolves them
//! (`ContactListener::contact_validate`).

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use std::sync::{Arc, Mutex};

use common::events::*;
use common::*;
use oxijolt::*;

/// User data of a body every rigid contact ignores.
const GHOST: u64 = 1;
/// User data of a one-way platform.
const PLATFORM: u64 = 2;

/// Rejects every pair with a [`GHOST`].
struct GhostsPassThrough;

impl ContactListener for GhostsPassThrough {
    fn contact_validate(&self, contact: &ContactCandidate) -> ValidateResult {
        if contact.user_data.contains(&GHOST) {
            ValidateResult::RejectAllContactsForThisBodyPair
        } else {
            ValidateResult::AcceptAllContactsForThisBodyPair
        }
    }
}

fn add_box(world: &mut PhysicsWorld, half: Vec3, settings: BodySettings) -> BodyId {
    let shape = Shape::new_box(half).unwrap();
    world.create_body(&shape, &settings).unwrap()
}

fn cube_at(position: RVec3, user_data: u64) -> BodySettings {
    BodySettings::new_dynamic()
        .position(position)
        .user_data(user_data)
}

fn height(world: &PhysicsWorld, id: BodyId) -> Real {
    world.body(id).unwrap().position().y
}

#[test]
fn rejecting_a_pair_lets_a_body_fall_through_the_floor() {
    for threads in [1, 4] {
        let mut world = world(Vec3::new(0.0, -9.81, 0.0), threads);
        world.set_contact_listener(Some(Arc::new(GhostsPassThrough)));
        add_floor(&mut world);
        let half = Vec3::new(0.25, 0.25, 0.25);
        let solid = add_box(&mut world, half, cube_at(RVec3::new(0.0, 0.3, 0.0), 0));
        let ghost = add_box(&mut world, half, cube_at(RVec3::new(2.0, 0.3, 0.0), GHOST));
        step(&mut world, 60);
        assert!(height(&world, solid) > 0.2);
        assert!(height(&world, ghost) < -2.0);
    }
}

/// Lets bodies pass a [`PLATFORM`] from below and holds them from above: a hit is accepted only
/// when the other body is pushed out through the platform's top.
struct OneWayPlatform;

impl ContactListener for OneWayPlatform {
    fn contact_validate(&self, contact: &ContactCandidate) -> ValidateResult {
        // Body 2 moves out of body 1 along the penetration axis.
        let up_out_of_platform = match contact.user_data {
            [PLATFORM, _] => contact.penetration_axis.y > 0.0,
            [_, PLATFORM] => contact.penetration_axis.y < 0.0,
            _ => return ValidateResult::AcceptAllContactsForThisBodyPair,
        };
        if up_out_of_platform {
            ValidateResult::AcceptContact
        } else {
            ValidateResult::RejectContact
        }
    }
}

/// A cube fired up from the floor at a thin platform 2 m up; its final height.
fn fire_at_platform(listener: Option<Arc<dyn ContactListener>>) -> Real {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    world.set_contact_listener(listener);
    add_floor(&mut world);
    let platform = BodySettings::new_static()
        .position(RVec3::new(0.0, 2.0, 0.0))
        .user_data(PLATFORM);
    add_box(&mut world, Vec3::new(2.0, 0.05, 2.0), platform);
    let cube = cube_at(RVec3::new(0.0, 0.25, 0.0), 0).linear_velocity(Vec3::new(0.0, 8.0, 0.0));
    let cube = add_box(&mut world, Vec3::new(0.25, 0.25, 0.25), cube);
    step(&mut world, 180);
    height(&world, cube)
}

#[test]
fn a_one_way_platform_lets_a_body_through_from_below_and_holds_it_from_above() {
    let blocked = fire_at_platform(None);
    assert!(
        blocked < 0.3,
        "without the listener it bounces off: {blocked}"
    );
    let on_top = fire_at_platform(Some(Arc::new(OneWayPlatform)));
    assert!(
        (on_top - 2.3).abs() < 0.03,
        "it passes and rests on top: {on_top}"
    );
}

/// Rejects every pair of a projectile, whose user data is its shooter's raw id, with that
/// shooter.
struct OwnProjectiles;

impl ContactListener for OwnProjectiles {
    fn contact_validate(&self, contact: &ContactCandidate) -> ValidateResult {
        let [user1, user2] = contact.user_data;
        let own = user1 == u64::from(contact.body2.to_raw())
            || user2 == u64::from(contact.body1.to_raw());
        if own {
            ValidateResult::RejectAllContactsForThisBodyPair
        } else {
            ValidateResult::AcceptAllContactsForThisBodyPair
        }
    }
}

#[test]
fn own_projectiles_pass_through_their_shooter() {
    let mut world = world(Vec3::ZERO, 2);
    world.set_contact_listener(Some(Arc::new(OwnProjectiles)));
    world.set_event_settings(contacts());
    let shooter = add_box(
        &mut world,
        Vec3::new(0.5, 0.5, 0.5),
        cube_at(RVec3::ZERO, u64::MAX),
    );
    let wall = add_box(
        &mut world,
        Vec3::new(0.25, 2.0, 2.0),
        BodySettings::new_static().position(RVec3::new(5.0, 0.0, 0.0)),
    );
    let shooter_id = u64::from(shooter.to_raw());
    let projectile = BodySettings::new_dynamic()
        .position(RVec3::new(0.3, 0.0, 0.0))
        .linear_velocity(Vec3::new(10.0, 0.0, 0.0))
        .user_data(shooter_id);
    let projectile = add_box(&mut world, Vec3::new(0.1, 0.1, 0.1), projectile);
    let mut hit_wall = false;
    for _ in 0..60 {
        step(&mut world, 1);
        let events = world.take_events();
        hit_wall |= events
            .contacts
            .iter()
            .any(|event| [event.pair().body1, event.pair().body2] == sorted(projectile, wall));
        assert!(events.contacts.iter().all(|event| {
            let bodies = [event.pair().body1, event.pair().body2];
            !(bodies.contains(&shooter) && bodies.contains(&projectile))
        }));
    }
    assert!(hit_wall, "the projectile still hits the wall");
    assert_eq!(world.body(shooter).unwrap().linear_velocity(), Vec3::ZERO);
    let x = world.body(projectile).unwrap().position().x;
    assert!(x > 4.0 && x < 4.75, "stopped by the wall: {x}");
}

fn sorted(a: BodyId, b: BodyId) -> [BodyId; 2] {
    if a.to_raw() < b.to_raw() {
        [a, b]
    } else {
        [b, a]
    }
}

/// Records every candidate and accepts it.
#[derive(Default)]
struct Recorder {
    candidates: Mutex<Vec<ContactCandidate>>,
    soft_body_calls: Mutex<u32>,
}

impl Recorder {
    fn pairs(&self) -> Vec<[BodyId; 2]> {
        let candidates = self.candidates.lock().unwrap();
        candidates.iter().map(|c| [c.body1, c.body2]).collect()
    }
}

impl ContactListener for Recorder {
    fn contact_validate(&self, contact: &ContactCandidate) -> ValidateResult {
        self.candidates.lock().unwrap().push(*contact);
        ValidateResult::AcceptContact
    }

    fn soft_body_contact_validate(
        &self,
        _: BodyId,
        _: BodyId,
        _: &mut SoftBodyContactSettings,
    ) -> SoftBodyValidateResult {
        *self.soft_body_calls.lock().unwrap() += 1;
        SoftBodyValidateResult::AcceptContact
    }
}

#[test]
fn validation_sees_body1_with_the_higher_motion_type() {
    let mut world = world(Vec3::ZERO, 2);
    let recorder = Arc::new(Recorder::default());
    world.set_contact_listener(Some(recorder.clone()));
    let half = Vec3::new(0.25, 0.25, 0.25);
    let at = |x, y| RVec3::new(x, y, 0.0);
    // Created first, so each has the lower id of its pair.
    let kinematic = add_box(
        &mut world,
        half,
        BodySettings::new_kinematic()
            .position(at(0.0, 0.0))
            .linear_velocity(Vec3::new(1.0, 0.0, 0.0)),
    );
    let fixed = add_box(
        &mut world,
        half,
        BodySettings::new_static().position(at(0.0, 10.0)),
    );
    let first = add_box(&mut world, half, cube_at(at(0.0, 20.0), 0));
    let pushed = add_box(&mut world, half, cube_at(at(0.6, 0.0), 0));
    let resting = add_box(
        &mut world,
        half,
        cube_at(at(0.0, 10.55), 0).linear_velocity(Vec3::new(0.0, -1.0, 0.0)),
    );
    let second = add_box(
        &mut world,
        half,
        cube_at(at(0.6, 20.0), 0).linear_velocity(Vec3::new(-1.0, 0.0, 0.0)),
    );
    step(&mut world, 30);
    let pairs = recorder.pairs();
    assert!(
        pairs.contains(&[pushed, kinematic]),
        "dynamic over kinematic"
    );
    assert!(pairs.contains(&[resting, fixed]), "dynamic over static");
    assert!(pairs.contains(&[first, second]), "equal types: lower id");
    for [body1, body2] in [[kinematic, pushed], [fixed, resting], [second, first]] {
        assert!(!pairs.contains(&[body1, body2]));
    }

    // In the continuous stage the cast body is body 1, though it has the higher id.
    let target = add_box(&mut world, half, cube_at(at(10.0, 30.0), 0));
    let rod = BodySettings::new_dynamic()
        .position(at(0.0, 30.0))
        .linear_velocity(Vec3::new(200.0, 0.0, 0.0))
        .motion_quality(MotionQuality::LinearCast);
    let rod = add_box(&mut world, Vec3::new(0.05, 0.05, 0.05), rod);
    recorder.candidates.lock().unwrap().clear();
    step(&mut world, 5);
    assert!(target.to_raw() < rod.to_raw());
    assert!(recorder.pairs().contains(&[rod, target]));
}

/// A fast `LinearCast` cube fired at a thin wall; its final x.
fn fire_through_wall(listener: Option<Arc<dyn ContactListener>>) -> Real {
    let mut world = world(Vec3::ZERO, 2);
    world.set_contact_listener(listener);
    let wall = BodySettings::new_static()
        .position(RVec3::new(10.0, 0.0, 0.0))
        .user_data(GHOST);
    add_box(&mut world, Vec3::new(0.05, 2.0, 2.0), wall);
    let bullet = BodySettings::new_dynamic()
        .linear_velocity(Vec3::new(200.0, 0.0, 0.0))
        .motion_quality(MotionQuality::LinearCast);
    let bullet = add_box(&mut world, Vec3::new(0.1, 0.1, 0.1), bullet);
    step(&mut world, 10);
    world.body(bullet).unwrap().position().x
}

#[test]
fn continuous_collision_is_validated_too() {
    let stopped = fire_through_wall(None);
    assert!(stopped < 10.0, "continuous collision stops it: {stopped}");
    let passed = fire_through_wall(Some(Arc::new(GhostsPassThrough)));
    assert!(passed > 20.0, "a rejected wall lets it through: {passed}");
}

#[test]
fn soft_bodies_use_their_own_validator() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 2);
    let recorder = Arc::new(Recorder::default());
    world.set_contact_listener(Some(recorder.clone()));
    add_table(&mut world, 0.0);
    let cloth = add_cloth(&mut world, RVec3::new(0.0, 0.3, 0.0), Quat::IDENTITY);
    let cube = add_box(
        &mut world,
        Vec3::new(0.25, 0.25, 0.25),
        cube_at(RVec3::new(1.5, 0.3, 0.0), 0),
    );
    step(&mut world, 60);
    assert!(*recorder.soft_body_calls.lock().unwrap() > 0);
    let pairs = recorder.pairs();
    assert!(pairs.iter().any(|pair| pair.contains(&cube)));
    assert!(pairs.iter().all(|pair| !pair.contains(&cloth)));
}

/// Two cubes over a floor, one a ghost; the cubes' final positions.
fn ghost_scene(listener: Option<Arc<dyn ContactListener>>) -> Vec<RVec3> {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_contact_listener(listener);
    add_floor(&mut world);
    let half = Vec3::new(0.25, 0.25, 0.25);
    let cubes = [
        add_box(&mut world, half, cube_at(RVec3::new(0.0, 0.3, 0.0), 0)),
        add_box(&mut world, half, cube_at(RVec3::new(2.0, 0.3, 0.0), GHOST)),
    ];
    step(&mut world, 60);
    cubes
        .iter()
        .map(|&id| world.body(id).unwrap().position())
        .collect()
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn independent_worlds_with_different_validators_step_concurrently() {
    let sequential = [
        ghost_scene(Some(Arc::new(GhostsPassThrough))),
        ghost_scene(None),
    ];
    let concurrent = std::thread::scope(|scope| {
        let rejecting = scope.spawn(|| ghost_scene(Some(Arc::new(GhostsPassThrough))));
        let accepting = scope.spawn(|| ghost_scene(None));
        [rejecting.join().unwrap(), accepting.join().unwrap()]
    });
    assert_eq!(sequential, concurrent);
    assert!(sequential[0][1].y < -2.0 && sequential[1][1].y > 0.2);
}
