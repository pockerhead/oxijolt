//! Character contact listeners (`CharacterContactListener`): moving platforms, rejected
//! contacts, the push settings, and the added, persisted and removed contacts of an update.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use std::sync::{Arc, Mutex};
use std::thread::{self, ThreadId};

use common::*;
use oxijolt::*;

const RADIUS: f32 = 0.4;
const HALF_HEIGHT: f32 = 0.7;
/// Puts the capsule's bottom at the character position.
const FOOT_OFFSET: Vec3 = Vec3::new(0.0, HALF_HEIGHT + RADIUS, 0.0);
const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// User data of a conveyor body.
const CONVEYOR: u64 = 1;
/// The conveyor's surface velocity.
const BELT: Vec3 = Vec3::new(2.0, 0.0, 0.0);

/// What a listener call reported.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Event {
    Adjusted { body: BodyId, user_data: u64 },
    Added(CharacterId, CharacterContactKey),
    Persisted(CharacterId, CharacterContactKey),
    Removed(CharacterId, CharacterContactKey),
}

/// A configurable listener that records its calls and the threads they ran on.
#[derive(Default)]
struct Recorder {
    /// Reports [`BELT`] for [`CONVEYOR`] bodies.
    conveyor: bool,
    /// The body every contact with which is rejected.
    ghost: Mutex<Option<BodyId>>,
    /// Rejects every contact with another character.
    ignore_characters: bool,
    /// The settings applied to every added and persisted contact, when set.
    settings: Option<CharacterContactSettings>,
    events: Mutex<Vec<Event>>,
    threads: Mutex<Vec<ThreadId>>,
    /// The results of the velocity setter checks, filled by the first adjust call.
    setter_checks: Mutex<Option<Vec<bool>>>,
    /// Runs the velocity setter checks in the first adjust call.
    check_setters: bool,
}

impl Recorder {
    fn push(&self, event: Event) {
        self.events.lock().unwrap().push(event);
        self.threads.lock().unwrap().push(thread::current().id());
    }

    fn take(&self) -> Vec<Event> {
        std::mem::take(&mut *self.events.lock().unwrap())
    }
}

/// The key of a contact.
fn key(contact: &CharacterContact) -> CharacterContactKey {
    CharacterContactKey {
        body: contact.body,
        character: contact.character,
        sub_shape_id: contact.sub_shape_id,
    }
}

/// The next `f32` above `value`.
fn next_up(value: f32) -> f32 {
    f32::from_bits(value.to_bits() + 1)
}

/// Whether each setter accepts its boundary and refuses beyond it, NaN and infinity.
fn setter_checks(velocity: &mut BodyVelocity) -> Vec<bool> {
    let linear = limits::MAX_LINEAR_VELOCITY;
    let angular = limits::MAX_ANGULAR_VELOCITY;
    let along = |v: f32| Vec3::new(0.0, 0.0, v);
    let mut checks = Vec::new();
    for (value, accepted) in [
        (linear, true),
        (next_up(linear), false),
        (f32::NAN, false),
        (f32::INFINITY, false),
    ] {
        let mut copy = *velocity;
        checks.push(copy.set_linear_velocity(along(value)).is_ok() == accepted);
        checks.push(!accepted || copy.linear_velocity() == along(value));
    }
    for (value, accepted) in [
        (angular, true),
        (next_up(angular), false),
        (f32::NAN, false),
        (f32::NEG_INFINITY, false),
    ] {
        let mut copy = *velocity;
        checks.push(copy.set_angular_velocity(along(value)).is_ok() == accepted);
        checks.push(!accepted || copy.angular_velocity() == along(value));
    }
    checks
}

impl CharacterContactListener for Recorder {
    fn adjust_body_velocity(
        &self,
        _: CharacterId,
        body: BodyId,
        user_data: u64,
        velocity: &mut BodyVelocity,
    ) {
        self.push(Event::Adjusted { body, user_data });
        if self.check_setters {
            let mut checks = self.setter_checks.lock().unwrap();
            if checks.is_none() {
                *checks = Some(setter_checks(velocity));
            }
        }
        if self.conveyor && user_data == CONVEYOR {
            velocity.set_linear_velocity(BELT).unwrap();
        }
    }

    fn contact_validate(&self, _: CharacterId, contact: &CharacterContact) -> bool {
        self.threads.lock().unwrap().push(thread::current().id());
        let ghost = contact.body.is_some() && contact.body == *self.ghost.lock().unwrap();
        let character = self.ignore_characters && contact.character.is_some();
        !(ghost || character)
    }

    fn contact_added(
        &self,
        character: CharacterId,
        contact: &CharacterContact,
        settings: &mut CharacterContactSettings,
    ) {
        self.push(Event::Added(character, key(contact)));
        if let Some(chosen) = self.settings {
            *settings = chosen;
        }
    }

    fn contact_persisted(
        &self,
        character: CharacterId,
        contact: &CharacterContact,
        settings: &mut CharacterContactSettings,
    ) {
        self.push(Event::Persisted(character, key(contact)));
        if let Some(chosen) = self.settings {
            *settings = chosen;
        }
    }

    fn contact_removed(&self, character: CharacterId, contact: CharacterContactKey) {
        self.push(Event::Removed(character, contact));
    }
}

/// `value` as a [`Real`].
#[allow(clippy::useless_conversion)] // `Real` is `f32` without the `double-precision` feature.
fn real(value: f32) -> Real {
    Real::from(value)
}

fn capsule() -> Shape {
    Shape::new_capsule(HALF_HEIGHT, RADIUS).unwrap()
}

fn add_static(world: &mut PhysicsWorld, half: Vec3, position: RVec3, user_data: u64) -> BodyId {
    let shape = Shape::new_box(half).unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_static()
                .position(position)
                .user_data(user_data),
        )
        .unwrap()
}

/// A floor of `user_data` with its top at y = 0.
fn add_ground(world: &mut PhysicsWorld, user_data: u64) -> BodyId {
    add_static(
        world,
        Vec3::new(20.0, 0.5, 20.0),
        RVec3::new(0.0, -0.5, 0.0),
        user_data,
    )
}

fn add_character(world: &mut PhysicsWorld, position: RVec3, characters: bool) -> CharacterId {
    let shape = capsule();
    let settings = CharacterSettings::new(&shape)
        .shape_offset(FOOT_OFFSET)
        .collide_with_characters(characters);
    let id = world
        .create_character(&settings, position, Quat::IDENTITY)
        .unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    id
}

/// One update of `id` with `velocity` plus a push toward the ground.
fn walk(world: &mut PhysicsWorld, id: CharacterId, velocity: Vec3) {
    world
        .character_mut(id)
        .unwrap()
        .set_linear_velocity(Vec3::new(velocity.x, velocity.y - 1.0, velocity.z))
        .unwrap();
    world
        .update_character(
            id,
            DT,
            GRAVITY,
            &ExtendedUpdateSettings::default(),
            &QueryFilter::new(),
        )
        .unwrap();
}

fn position(world: &PhysicsWorld, id: CharacterId) -> RVec3 {
    world.character(id).unwrap().position()
}

fn world_with(listener: Option<Arc<Recorder>>) -> PhysicsWorld {
    let mut world = world(GRAVITY, 1);
    world.set_character_contact_listener(listener.map(|l| l as Arc<dyn CharacterContactListener>));
    world
}

/// A character standing on a conveyor for a second; it walks with the ground velocity.
fn ride_conveyor(listener: Option<Arc<Recorder>>) -> (RVec3, Vec3) {
    let mut world = world_with(listener);
    add_ground(&mut world, CONVEYOR);
    let id = add_character(&mut world, RVec3::ZERO, false);
    let mut ground = Vec3::ZERO;
    for _ in 0..60 {
        walk(&mut world, id, ground);
        ground = world.character(id).unwrap().ground_velocity();
    }
    (position(&world, id), ground)
}

#[test]
fn a_conveyor_moves_a_character_standing_on_a_static_body() {
    let listener = Arc::new(Recorder {
        conveyor: true,
        ..Recorder::default()
    });
    let (moved, ground) = ride_conveyor(Some(listener));
    assert_eq!(ground, BELT, "the adjusted velocity bit for bit");
    assert!(moved.x > 1.8, "{moved:?}");
    let (stayed, ground) = ride_conveyor(None);
    assert_eq!(ground, Vec3::ZERO);
    assert!(stayed.x.abs() < 1e-3, "{stayed:?}");
}

#[test]
fn adjust_receives_the_bodys_user_data() {
    let listener = Arc::new(Recorder::default());
    let mut world = world_with(Some(listener.clone()));
    let floor = add_ground(&mut world, 77);
    let id = add_character(&mut world, RVec3::ZERO, false);
    walk(&mut world, id, Vec3::ZERO);
    let adjusted: Vec<_> = listener
        .take()
        .into_iter()
        .filter(|event| matches!(event, Event::Adjusted { .. }))
        .collect();
    assert!(!adjusted.is_empty());
    for event in adjusted {
        assert_eq!(
            event,
            Event::Adjusted {
                body: floor,
                user_data: 77
            }
        );
    }
}

#[test]
fn adjusted_velocities_outside_the_limits_are_refused() {
    let listener = Arc::new(Recorder {
        check_setters: true,
        ..Recorder::default()
    });
    let mut world = world_with(Some(listener.clone()));
    add_ground(&mut world, 0);
    let id = add_character(&mut world, RVec3::ZERO, false);
    walk(&mut world, id, Vec3::ZERO);
    let checks = listener.setter_checks.lock().unwrap().clone().unwrap();
    assert!(checks.iter().all(|&ok| ok), "{checks:?}");
}

/// Reports the largest velocities a body may have for the box it stands on.
struct Spinner;

impl CharacterContactListener for Spinner {
    fn adjust_body_velocity(&self, _: CharacterId, _: BodyId, _: u64, v: &mut BodyVelocity) {
        v.set_linear_velocity(Vec3::new(limits::MAX_LINEAR_VELOCITY, 0.0, 0.0))
            .unwrap();
        v.set_angular_velocity(Vec3::new(0.0, limits::MAX_ANGULAR_VELOCITY, 0.0))
            .unwrap();
    }
}

/// A character at the far edge of a static box of the largest extent, told that the box moves
/// and spins as fast as a body may, next to a light dynamic cube it pushes.
#[test]
fn a_far_lever_adjusted_velocity_stays_finite() {
    let mut world = world(GRAVITY, 1);
    world.set_character_contact_listener(Some(Arc::new(Spinner)));
    let extent = limits::MAX_SHAPE_EXTENT;
    add_static(
        &mut world,
        Vec3::new(extent, 0.5, extent),
        RVec3::new(0.0, -0.5, 0.0),
        0,
    );
    let edge = real(extent) - 1.0;
    let cube = world
        .create_body(
            &Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(edge, 0.25, 0.6))
                .mass(1.0),
        )
        .unwrap();
    let id = add_character(&mut world, RVec3::new(edge, 0.0, 0.0), false);
    for _ in 0..120 {
        let ground = world.character(id).unwrap().ground_velocity();
        let velocity = if limits_ok(ground) {
            ground
        } else {
            Vec3::ZERO
        };
        walk(&mut world, id, Vec3::new(velocity.x, 0.0, velocity.z + 1.0));
        step(&mut world, 1);
        let character = world.character(id).unwrap();
        assert!(finite(character.position()) && finite_v(character.linear_velocity()));
        let body = world.body(cube).unwrap();
        assert!(finite(body.position()) && finite_v(body.linear_velocity()));
    }
}

/// Whether a ground velocity may be set on a character.
fn limits_ok(v: Vec3) -> bool {
    finite_v(v) && (v.x * v.x + v.y * v.y + v.z * v.z).sqrt() < limits::MAX_LINEAR_VELOCITY
}

fn finite(p: RVec3) -> bool {
    p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
}

fn finite_v(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

/// A character walking 3 m along x into a 1 m wall, which `listener` rejects; its final x.
fn walk_into_ghost(listener: Option<Arc<Recorder>>) -> Real {
    let mut world = world_with(listener.clone());
    add_ground(&mut world, 0);
    let wall = add_static(
        &mut world,
        Vec3::new(0.5, 2.0, 2.0),
        RVec3::new(2.0, 2.0, 0.0),
        0,
    );
    if let Some(listener) = listener {
        *listener.ghost.lock().unwrap() = Some(wall);
    }
    let id = add_character(&mut world, RVec3::ZERO, false);
    for _ in 0..90 {
        walk(&mut world, id, Vec3::new(2.0, 0.0, 0.0));
    }
    position(&world, id).x
}

#[test]
fn a_rejected_body_contact_lets_the_character_walk_through() {
    let blocked = walk_into_ghost(None);
    assert!(blocked < 1.2, "{blocked}");
    let through = walk_into_ghost(Some(Arc::new(Recorder::default())));
    assert!(through > 2.9, "{through}");
}

/// How close two blocking characters' centres come: both capsules and both paddings (Jolt's
/// default padding, 0.02 m).
const CLOSEST: f32 = 2.0 * (RADIUS + 0.02);

/// Where a walk into another character ended.
#[derive(Debug, PartialEq)]
struct Passage {
    walker: RVec3,
    other: RVec3,
    /// The smallest `other.x - walker.x` after any update.
    closest: Real,
    /// Whether an update of the walker collided with the other character.
    touched: bool,
}

/// Two characters that collide with characters, one walking 3 m along x into the other.
fn walk_into_character(listener: Option<Arc<Recorder>>) -> Passage {
    let mut world = world_with(listener);
    add_ground(&mut world, 0);
    let walker = add_character(&mut world, RVec3::ZERO, true);
    let other = add_character(&mut world, RVec3::new(2.0, 0.0, 0.0), true);
    let (mut closest, mut touched) = (Real::MAX, false);
    for _ in 0..90 {
        walk(&mut world, walker, Vec3::new(2.0, 0.0, 0.0));
        closest = closest.min(position(&world, other).x - position(&world, walker).x);
        touched |= world
            .character(walker)
            .unwrap()
            .active_contacts()
            .iter()
            .any(|contact| contact.character == Some(other) && contact.had_collision);
        walk(&mut world, other, Vec3::ZERO);
        closest = closest.min(position(&world, other).x - position(&world, walker).x);
    }
    Passage {
        walker: position(&world, walker),
        other: position(&world, other),
        closest,
        touched,
    }
}

/// Blocking is judged by the gap, not by where the walker stops: whether a character standing on
/// the floor is pushed along by one walking into it is a float tie with its floor contact
/// (pushed in double precision, not in single precision in this scene).
#[test]
fn a_rejected_character_contact_lets_two_characters_pass() {
    let blocked = walk_into_character(None);
    assert!(blocked.closest >= real(CLOSEST) - 1e-4, "{blocked:?}");
    assert!(blocked.touched, "{blocked:?}");
    let listened = walk_into_character(Some(Arc::new(Recorder::default())));
    assert_eq!(listened, blocked, "a listener that changes nothing");

    let listener = Arc::new(Recorder {
        ignore_characters: true,
        ..Recorder::default()
    });
    let through = walk_into_character(Some(listener));
    assert!(through.walker.x > 2.9, "{through:?}");
    assert_eq!(through.other.x, 2.0, "not pushed: {through:?}");
    assert!(!through.touched, "{through:?}");
}

/// A character walking into a 1 kg box for a second; how far the box moved along x.
fn push_box(settings: Option<CharacterContactSettings>) -> Real {
    let listener = Arc::new(Recorder {
        settings,
        ..Recorder::default()
    });
    let mut world = world_with(Some(listener));
    add_ground(&mut world, 0);
    let start = RVec3::new(1.0, 0.5, 0.0);
    let cube = world
        .create_body(
            &Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap(),
            &BodySettings::new_dynamic().position(start).mass(1.0),
        )
        .unwrap();
    let id = add_character(&mut world, RVec3::new(-0.5, 0.0, 0.0), false);
    for _ in 0..60 {
        walk(&mut world, id, Vec3::new(1.0, 0.0, 0.0));
        step(&mut world, 1);
    }
    world.body(cube).unwrap().position().x - start.x
}

#[test]
fn a_character_without_impulses_does_not_push_a_box_sideways() {
    let pushes = CharacterContactSettings {
        can_push_character: true,
        can_receive_impulses: true,
    };
    let pushed = push_box(Some(pushes));
    assert!(pushed > 0.3, "{pushed}");
    let still = push_box(Some(CharacterContactSettings {
        can_receive_impulses: false,
        ..pushes
    }));
    assert!(still.abs() < 1e-3, "{still}");
}

/// A kinematic wall moving at 1 m/s along x into a standing character for a second; how far
/// the character moved.
fn moving_wall(can_push_character: bool) -> Real {
    let listener = Arc::new(Recorder {
        settings: Some(CharacterContactSettings {
            can_push_character,
            can_receive_impulses: true,
        }),
        ..Recorder::default()
    });
    let mut world = world_with(Some(listener));
    add_ground(&mut world, 0);
    world
        .create_body(
            &Shape::new_box(Vec3::new(0.5, 1.0, 1.0)).unwrap(),
            &BodySettings::new_kinematic()
                .position(RVec3::new(-1.0, 1.05, 0.0))
                .linear_velocity(Vec3::new(1.0, 0.0, 0.0)),
        )
        .unwrap();
    let id = add_character(&mut world, RVec3::ZERO, false);
    for _ in 0..60 {
        step(&mut world, 1);
        walk(&mut world, id, Vec3::ZERO);
    }
    position(&world, id).x
}

#[test]
fn a_moving_wall_that_cannot_push_does_not_push_the_character() {
    let pushed = moving_wall(true);
    let resisted = moving_wall(false);
    assert!(pushed > 0.4, "{pushed}");
    assert!(resisted < pushed - 0.2, "{resisted} vs {pushed}");
}

/// A character standing 1 cm from another whose velocity is 3 m/s toward it, updated once (the
/// other is not updated); how far the standing one moved along x.
fn pushed_by_a_character(can_push_character: bool) -> Real {
    let listener = Arc::new(Recorder {
        settings: Some(CharacterContactSettings {
            can_push_character,
            can_receive_impulses: true,
        }),
        ..Recorder::default()
    });
    let mut world = world_with(Some(listener));
    add_ground(&mut world, 0);
    let standing = add_character(&mut world, RVec3::ZERO, true);
    let pusher = add_character(&mut world, RVec3::new(-0.81, 0.0, 0.0), true);
    world
        .character_mut(pusher)
        .unwrap()
        .set_linear_velocity(Vec3::new(3.0, 0.0, 0.0))
        .unwrap();
    walk(&mut world, standing, Vec3::ZERO);
    position(&world, standing).x
}

/// Jolt gives a character contact the other character's velocity; `can_push_character` decides
/// whether it moves this one.
#[test]
fn a_moving_character_that_cannot_push_does_not_push_the_character() {
    let pushed = pushed_by_a_character(true);
    let resisted = pushed_by_a_character(false);
    assert!(pushed > 0.05, "{pushed}");
    assert!(resisted.abs() < 1e-6, "{resisted}");
}

/// Keys of the contact events of `events` by kind.
fn keys(
    events: &[Event],
    kind: fn(&Event) -> Option<CharacterContactKey>,
) -> Vec<CharacterContactKey> {
    events.iter().filter_map(kind).collect()
}

fn added(event: &Event) -> Option<CharacterContactKey> {
    match *event {
        Event::Added(_, key) => Some(key),
        _ => None,
    }
}

fn persisted(event: &Event) -> Option<CharacterContactKey> {
    match *event {
        Event::Persisted(_, key) => Some(key),
        _ => None,
    }
}

fn removed(event: &Event) -> Option<CharacterContactKey> {
    match *event {
        Event::Removed(_, key) => Some(key),
        _ => None,
    }
}

/// The raw values of a key, which removals are sorted by; "none" is `u32::MAX`.
fn raw(key: &CharacterContactKey) -> (u32, u32, u32) {
    (
        key.body.map_or(u32::MAX, BodyId::to_raw),
        key.character.map_or(u32::MAX, CharacterId::to_raw),
        key.sub_shape_id.to_raw(),
    )
}

/// Whether `keys` are in delivery order: body, character, sub-shape by raw value, none last.
fn in_delivery_order(keys: &[CharacterContactKey]) -> bool {
    keys.windows(2).all(|pair| raw(&pair[0]) <= raw(&pair[1]))
}

/// The contact events of `character`.
fn of(character: CharacterId, events: &[Event]) -> Vec<Event> {
    events
        .iter()
        .copied()
        .filter(|event| match *event {
            Event::Added(c, _) | Event::Persisted(c, _) | Event::Removed(c, _) => c == character,
            Event::Adjusted { .. } => false,
        })
        .collect()
}

/// A character walks over the floor into the corner of a wall and another character, then is
/// moved away: every contact is added once before it persists, and all end in that one update,
/// delivered sorted.
#[test]
fn added_persisted_removed_follow_a_walk_across_two_boxes() {
    let listener = Arc::new(Recorder::default());
    let mut world = world_with(Some(listener.clone()));
    let floor = add_ground(&mut world, 0);
    let wall = add_static(
        &mut world,
        Vec3::new(2.0, 1.0, 0.5),
        RVec3::new(0.0, 1.0, -1.2),
        0,
    );
    let walker = add_character(&mut world, RVec3::ZERO, true);
    let neighbour = add_character(&mut world, RVec3::new(-1.0, 0.0, 0.0), true);
    for _ in 0..30 {
        walk(&mut world, walker, Vec3::new(-1.0, 0.0, -1.0));
        walk(&mut world, neighbour, Vec3::ZERO);
    }
    let events = of(walker, &listener.take());
    let touched = keys(&events, added);
    for body in [floor, wall] {
        assert!(
            touched.iter().any(|key| key.body == Some(body)),
            "{touched:?}"
        );
    }
    assert!(
        touched.iter().any(|key| key.character == Some(neighbour)),
        "{touched:?}"
    );
    for key in &touched {
        assert_eq!(
            touched.iter().filter(|k| *k == key).count(),
            1,
            "added once"
        );
        let first_added = events.iter().position(|e| added(e) == Some(*key));
        let first_persisted = events.iter().position(|e| persisted(e) == Some(*key));
        assert!(first_persisted.is_none_or(|p| first_added < Some(p)));
    }
    assert!(keys(&events, removed).is_empty());

    world
        .character_mut(walker)
        .unwrap()
        .set_position(RVec3::new(10.0, 3.0, 10.0))
        .unwrap();
    walk(&mut world, walker, Vec3::ZERO);
    let gone = keys(&of(walker, &listener.take()), removed);
    assert!(gone.iter().any(|key| key.body == Some(floor)), "{gone:?}");
    assert!(gone.iter().any(|key| key.body == Some(wall)), "{gone:?}");
    assert!(
        gone.iter().any(|key| key.character == Some(neighbour)),
        "{gone:?}"
    );
    assert!(in_delivery_order(&gone), "{gone:?}");
}

/// Records removals and panics on the second one.
#[derive(Default)]
struct PanicsOnSecondRemoval {
    removed: Mutex<Vec<CharacterContactKey>>,
}

impl CharacterContactListener for PanicsOnSecondRemoval {
    fn contact_removed(&self, _: CharacterId, contact: CharacterContactKey) {
        let mut removed = self.removed.lock().unwrap();
        removed.push(contact);
        if removed.len() == 2 {
            drop(removed);
            panic!("second removal");
        }
    }
}

/// A walker touching the floor, a wall and a neighbour, then moved away from all of them and
/// updated once; the world and the walker.
fn leave_three_contacts(
    listener: Arc<dyn CharacterContactListener>,
) -> (PhysicsWorld, CharacterId) {
    let mut world = world(GRAVITY, 1);
    world.set_character_contact_listener(Some(listener));
    add_ground(&mut world, 0);
    add_static(
        &mut world,
        Vec3::new(2.0, 1.0, 0.5),
        RVec3::new(0.0, 1.0, -1.2),
        0,
    );
    let walker = add_character(&mut world, RVec3::ZERO, true);
    let neighbour = add_character(&mut world, RVec3::new(-1.0, 0.0, 0.0), true);
    for _ in 0..30 {
        walk(&mut world, walker, Vec3::new(-1.0, 0.0, -1.0));
        walk(&mut world, neighbour, Vec3::ZERO);
    }
    world
        .character_mut(walker)
        .unwrap()
        .set_position(RVec3::new(10.0, 3.0, 10.0))
        .unwrap();
    (world, walker)
}

/// `contact_removed` runs after Jolt returned and is not caught: the removals before a panicking
/// one have been delivered, the rest of that update's are lost, and the world goes on.
#[test]
fn a_panic_in_contact_removed_keeps_the_delivered_prefix() {
    let recorder = Arc::new(Recorder::default());
    let (mut world, walker) = leave_three_contacts(recorder.clone());
    recorder.take();
    walk(&mut world, walker, Vec3::ZERO);
    let all = keys(&of(walker, &recorder.take()), removed);
    assert!(all.len() >= 3, "{all:?}");

    let panicking = Arc::new(PanicsOnSecondRemoval::default());
    let (mut world, walker) = leave_three_contacts(panicking.clone());
    panicking.removed.lock().unwrap().clear();
    let payload = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        walk(&mut world, walker, Vec3::ZERO)
    }))
    .unwrap_err();
    assert_eq!(payload.downcast_ref::<&str>(), Some(&"second removal"));
    let delivered: Vec<_> = panicking.removed.lock().unwrap().iter().map(raw).collect();
    let expected: Vec<_> = all[..2].iter().map(raw).collect();
    assert_eq!(delivered, expected, "two worlds: compared by raw ids");
    assert!(position(&world, walker).y < 3.0, "the update itself moved");

    walk(&mut world, walker, Vec3::ZERO);
    assert_eq!(
        panicking.removed.lock().unwrap().len(),
        2,
        "the rest is lost"
    );
}

/// `count` static spheres around a character standing at `at` with its centre line, at `height`,
/// each touching the capsule's padding.
fn add_ring(world: &mut PhysicsWorld, at: RVec3, height: f32, count: usize) {
    let shape = Shape::new_sphere(0.05).unwrap();
    let distance = RADIUS + 0.02 + 0.05;
    for i in 0..count {
        let (sin, cos) = (std::f32::consts::TAU * i as f32 / count as f32).sin_cos();
        let position = RVec3::new(
            at.x + real(distance * cos),
            at.y + real(height),
            at.z + real(distance * sin),
        );
        world
            .create_body(&shape, &BodySettings::new_static().position(position))
            .unwrap();
    }
}

/// Moves `id` to `at` and refreshes its contacts; the removals that reported.
fn refresh_at(
    world: &mut PhysicsWorld,
    listener: &Recorder,
    id: CharacterId,
    at: RVec3,
) -> Vec<CharacterContactKey> {
    listener.take();
    world.character_mut(id).unwrap().set_position(at).unwrap();
    world
        .refresh_character_contacts(id, &QueryFilter::new())
        .unwrap();
    keys(&listener.take(), removed)
}

/// Jolt reports removals in the order of a hash map that keeps the capacity of earlier updates;
/// delivered removals are sorted, also after a detour through more contacts grew that map.
#[test]
fn removals_arrive_sorted_after_the_contact_map_grew() {
    let listener = Arc::new(Recorder::default());
    let mut world = world_with(Some(listener.clone()));
    add_ground(&mut world, 0);
    let (small, large, open) = (
        RVec3::ZERO,
        RVec3::new(10.0, 0.0, 0.0),
        RVec3::new(5.0, 0.0, 0.0),
    );
    add_ring(&mut world, small, 1.1, 12);
    add_ring(&mut world, large, 0.8, 24);
    add_ring(&mut world, large, 1.4, 24);
    let id = add_character(&mut world, small, false);

    let first = refresh_at(&mut world, &listener, id, open);
    assert_eq!(first.len(), 12, "{first:?}");
    assert!(in_delivery_order(&first), "{first:?}");

    refresh_at(&mut world, &listener, id, large);
    let detour = refresh_at(&mut world, &listener, id, small);
    assert_eq!(detour.len(), 48, "{detour:?}");
    assert!(in_delivery_order(&detour), "{detour:?}");

    let after = refresh_at(&mut world, &listener, id, open);
    let raw_keys = |keys: &[CharacterContactKey]| keys.iter().map(raw).collect::<Vec<_>>();
    assert_eq!(
        raw_keys(&after),
        raw_keys(&first),
        "the same removals, in the same order"
    );
}

#[test]
fn removing_a_contacted_body_or_character_reports_its_removal() {
    for refresh in [false, true] {
        let listener = Arc::new(Recorder::default());
        let mut world = world_with(Some(listener.clone()));
        add_ground(&mut world, 0);
        let wall = add_static(
            &mut world,
            Vec3::new(0.5, 1.0, 2.0),
            RVec3::new(-0.9, 1.0, 0.0),
            0,
        );
        let walker = add_character(&mut world, RVec3::ZERO, true);
        let neighbour = add_character(&mut world, RVec3::new(0.8, 0.0, 0.0), true);
        for _ in 0..10 {
            walk(&mut world, walker, Vec3::new(-1.0, 0.0, 0.0));
        }
        listener.take();
        world.remove_body(wall).unwrap();
        world.remove_character(neighbour).unwrap();
        if refresh {
            world
                .refresh_character_contacts(walker, &QueryFilter::new())
                .unwrap();
        } else {
            walk(&mut world, walker, Vec3::ZERO);
        }
        let gone = keys(&listener.take(), removed);
        assert!(gone.iter().any(|key| key.body == Some(wall)), "{gone:?}");
        assert!(
            gone.iter().any(|key| key.character == Some(neighbour)),
            "{gone:?}"
        );
    }
}

#[test]
fn refresh_reports_added_for_a_character_created_on_the_ground() {
    let listener = Arc::new(Recorder::default());
    let mut world = world_with(Some(listener.clone()));
    let floor = add_ground(&mut world, 0);
    add_character(&mut world, RVec3::ZERO, false);
    let events = listener.take();
    assert!(keys(&events, added)
        .iter()
        .any(|key| key.body == Some(floor)));
}

/// Walking up and down stairs, stick to floor and walk stairs run inside one update: no contact
/// is reported both removed and added (or persisted) in the same update.
#[test]
fn one_extended_update_is_one_tracking_scope() {
    let listener = Arc::new(Recorder::default());
    let mut world = world_with(Some(listener.clone()));
    add_ground(&mut world, 0);
    for step_index in 0..4 {
        let height = 0.15 * (step_index + 1) as f32;
        add_static(
            &mut world,
            Vec3::new(0.25, height / 2.0, 2.0),
            RVec3::new(1.0 + 0.5 * real(step_index as f32), real(height / 2.0), 0.0),
            0,
        );
    }
    let id = add_character(&mut world, RVec3::ZERO, false);
    let extended = ExtendedUpdateSettings::default()
        .walk_stairs_step_up(Vec3::new(0.0, 0.4, 0.0))
        .stick_to_floor_step_down(Vec3::new(0.0, -0.5, 0.0));
    let mut stairs_seen = false;
    for tick in 0..240 {
        let along = if tick < 120 { 1.0 } else { -1.0 };
        world
            .character_mut(id)
            .unwrap()
            .set_linear_velocity(Vec3::new(along, -1.0, 0.0))
            .unwrap();
        world
            .update_character(id, DT, GRAVITY, &extended, &QueryFilter::new())
            .unwrap();
        let events = listener.take();
        let ended = keys(&events, removed);
        for key in keys(&events, added).iter().chain(&keys(&events, persisted)) {
            assert!(!ended.contains(key), "tick {tick}: {key:?} in {events:?}");
            stairs_seen |= key.body.is_some_and(|b| b.to_raw() > 0);
        }
    }
    assert!(stairs_seen);
    assert!(position(&world, id).y < 0.1, "back down");
}

#[test]
fn callbacks_run_on_the_calling_thread() {
    let listener = Arc::new(Recorder::default());
    let mut world = world(GRAVITY, 4);
    world.set_character_contact_listener(Some(listener.clone()));
    add_ground(&mut world, 0);
    let a = add_character(&mut world, RVec3::ZERO, true);
    let b = add_character(&mut world, RVec3::new(0.8, 0.0, 0.0), true);
    for _ in 0..30 {
        walk(&mut world, a, Vec3::new(1.0, 0.0, 0.0));
        walk(&mut world, b, Vec3::ZERO);
        step(&mut world, 1);
    }
    let threads = listener.threads.lock().unwrap();
    assert!(!threads.is_empty());
    assert!(threads.iter().all(|&t| t == thread::current().id()));
}

#[test]
fn replacing_or_removing_the_listener_takes_effect_at_the_next_update() {
    let first = Arc::new(Recorder::default());
    let second = Arc::new(Recorder::default());
    let mut world = world_with(Some(first.clone()));
    add_ground(&mut world, 0);
    let id = add_character(&mut world, RVec3::ZERO, false);
    walk(&mut world, id, Vec3::ZERO);
    assert!(!first.take().is_empty());
    world.set_character_contact_listener(Some(second.clone()));
    walk(&mut world, id, Vec3::ZERO);
    assert!(first.take().is_empty());
    let events = second.take();
    assert!(!events.is_empty());
    assert!(
        keys(&events, added).is_empty(),
        "the floor persists: {events:?}"
    );
    world.set_character_contact_listener(None);
    walk(&mut world, id, Vec3::ZERO);
    assert!(first.take().is_empty() && second.take().is_empty());
}

/// A conveyor ride with a listener that reports the conveyor or not; the final pose.
fn ride(conveyor: bool) -> (RVec3, Vec3) {
    let listener = Arc::new(Recorder {
        conveyor,
        ..Recorder::default()
    });
    ride_conveyor(Some(listener))
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn independent_worlds_with_different_character_listeners_update_concurrently() {
    let sequential = [ride(true), ride(false)];
    let concurrent = thread::scope(|scope| {
        let moving = scope.spawn(|| ride(true));
        let still = scope.spawn(|| ride(false));
        [moving.join().unwrap(), still.join().unwrap()]
    });
    assert_eq!(sequential, concurrent);
    assert_ne!(sequential[0], sequential[1]);
}
