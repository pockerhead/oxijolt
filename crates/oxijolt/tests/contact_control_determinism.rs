//! Same-machine determinism and rollback of contact control: rigid contact validation, the
//! character contact listener and collision groups.
//!
//! Three scenes, each recorded tick by tick: the canonical events and the ordered callback
//! sequences (shape), and every body, character and contact state (state).
//! - `validation`: 80 cubes of two kinds over a floor and a thin one-way platform, a pure
//!   validator (one kind passes the platform from below, the other passes it from either side),
//!   and a `LinearCast` rod fired every 40 ticks.
//! - `characters`: four characters that collide with each other walk over a conveyor, a pile of
//!   30 small boxes, a ghost wall and a moving kinematic platform; the listener adjusts the
//!   conveyor, rejects the ghost, lets even boxes receive no impulses and stops one character
//!   pair from pushing.
//! - `groups`: a 12-link chain whose neighbours do not collide, a car that ignores a driver body
//!   and 18 cubes in three groups.
//!
//! The gates compare 1 and 4 workers and the caller job systems across processes, and a replay
//! after a divergent detour (other inputs, other listeners, teleports, a contact-cache
//! invalidation right before the restore) with the first run, in one process and across two.
//! Validate and adjust calls are compared only through their outcomes: Jolt may make them any
//! number of times, in no fixed order. See `common::determinism` for the child processes.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use std::sync::{Arc, Mutex};

use common::determinism::*;
use common::jobs::{self, JobChoice};
use common::vehicle::{add_car_with, car_world, chassis_settings, GRAVITY as CAR_GRAVITY};
use common::*;
use oxijolt::*;

const CHILD: &str = "contact_control_child";
const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
/// Ticks before the save, ticks recorded after it, and ticks of the detour.
const WARM_UP: usize = 60;
const RECORDED: usize = 60;
const DETOUR: usize = 100;
const SCENES: [&str; 3] = ["validation", "characters", "groups"];

/// User data of a one-way platform.
const PLATFORM: u64 = 1;
/// User data of cubes that the one-way rule applies to.
const ONE_WAY: u64 = 2;
/// User data of cubes that pass the platform from either side.
const PASSING: u64 = 3;
/// User data of the conveyor.
const CONVEYOR: u64 = 4;
/// User data of the wall characters walk through.
const GHOST: u64 = 5;
/// User data of the first box of the pile; box `i` has `BOX + i`.
const BOX: u64 = 100;

/// The validation scene's rule.
struct OneWay {
    /// The detour's validator accepts everything.
    accept_all: bool,
}

impl ContactListener for OneWay {
    fn contact_validate(&self, contact: &ContactCandidate) -> ValidateResult {
        if self.accept_all {
            return ValidateResult::AcceptAllContactsForThisBodyPair;
        }
        let up_out_of_platform = match contact.user_data {
            [PLATFORM, PASSING] | [PASSING, PLATFORM] => {
                return ValidateResult::RejectAllContactsForThisBodyPair
            }
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

/// The characters scene's listener, recording the ordered contact callbacks.
#[derive(Default)]
struct CharacterRules {
    /// The detour's listener changes nothing.
    passive: bool,
    /// The bodies with their user data, fixed for the run.
    user_data: Vec<(BodyId, u64)>,
    /// The two characters that do not push each other.
    no_push: Option<(CharacterId, CharacterId)>,
    log: Mutex<Vec<String>>,
}

impl CharacterRules {
    fn user_data(&self, body: Option<BodyId>) -> Option<u64> {
        let body = body?;
        self.user_data
            .iter()
            .find_map(|&(id, data)| (id == body).then_some(data))
    }

    fn settings(
        &self,
        character: CharacterId,
        contact: &CharacterContact,
        settings: &mut CharacterContactSettings,
    ) {
        if self.passive {
            return;
        }
        if let Some(data) = self.user_data(contact.body) {
            if data >= BOX && (data - BOX).is_multiple_of(2) {
                settings.can_receive_impulses = false;
            }
        }
        if let (Some((a, b)), Some(other)) = (self.no_push, contact.character) {
            if (character, other) == (a, b) || (character, other) == (b, a) {
                settings.can_push_character = false;
            }
        }
    }

    fn take(&self) -> Vec<String> {
        std::mem::take(&mut *self.log.lock().unwrap())
    }
}

impl CharacterContactListener for CharacterRules {
    fn adjust_body_velocity(&self, _: CharacterId, _: BodyId, data: u64, v: &mut BodyVelocity) {
        if !self.passive && data == CONVEYOR {
            v.set_linear_velocity(Vec3::new(0.0, 0.0, 1.5)).unwrap();
        }
    }

    fn contact_validate(&self, _: CharacterId, contact: &CharacterContact) -> bool {
        self.passive || self.user_data(contact.body) != Some(GHOST)
    }

    fn contact_added(
        &self,
        character: CharacterId,
        contact: &CharacterContact,
        settings: &mut CharacterContactSettings,
    ) {
        self.settings(character, contact, settings);
        let line = format!("added {character:?} {contact:?} {settings:?}");
        self.log.lock().unwrap().push(line);
    }

    fn contact_persisted(
        &self,
        character: CharacterId,
        contact: &CharacterContact,
        settings: &mut CharacterContactSettings,
    ) {
        self.settings(character, contact, settings);
        let line = format!("persisted {character:?} {contact:?} {settings:?}");
        self.log.lock().unwrap().push(line);
    }

    fn contact_removed(&self, character: CharacterId, contact: CharacterContactKey) {
        let line = format!("removed {character:?} {contact:?}");
        self.log.lock().unwrap().push(line);
    }
}

/// One scene: its world, what it records and how it is driven.
struct Scene {
    kind: &'static str,
    world: PhysicsWorld,
    bodies: Vec<BodyId>,
    characters: Vec<CharacterId>,
    rod: Option<BodyId>,
    platform: Option<BodyId>,
    rules: Option<Arc<CharacterRules>>,
    vehicle: Option<VehicleId>,
}

impl Scene {
    fn new(kind: &str, threads: u32) -> Self {
        match kind {
            "validation" => validation_scene(threads),
            "characters" => characters_scene(threads),
            "groups" => groups_scene(threads),
            kind => panic!("unknown scene {kind}"),
        }
    }

    /// Installs the listeners of the run (`detour` false) or of the detour.
    fn install(&mut self, detour: bool) {
        match self.kind {
            "validation" => {
                self.world
                    .set_contact_listener(Some(Arc::new(OneWay { accept_all: detour })));
            }
            "characters" => {
                let rules = self.rules.as_ref().unwrap();
                let listener = CharacterRules {
                    passive: detour,
                    user_data: rules.user_data.clone(),
                    no_push: rules.no_push,
                    log: Mutex::default(),
                };
                let listener = Arc::new(listener);
                self.rules = Some(listener.clone());
                self.world.set_character_contact_listener(Some(listener));
            }
            _ => {}
        }
    }

    /// The inputs of `tick`; the detour's are different.
    fn drive(&mut self, tick: usize, detour: bool) {
        if let (Some(rod), true) = (self.rod, tick % 40 == 10) {
            fire(&mut self.world, rod, detour);
        }
        if let Some(platform) = self.platform {
            let along = if (tick / 60).is_multiple_of(2) {
                1.0
            } else {
                -1.0
            };
            let speed = if detour { -2.0 * along } else { along };
            let mut body = self.world.body_mut(platform).unwrap();
            body.set_linear_velocity(Vec3::new(speed, 0.0, 0.0))
                .unwrap();
        }
        if let Some(vehicle) = self.vehicle {
            let forward = if detour { -1.0 } else { 1.0 };
            let input = DriverInput {
                forward,
                right: if tick % 120 < 60 { 0.3 } else { -0.3 },
                ..DriverInput::default()
            };
            let mut vehicle = self.world.vehicle_mut(vehicle).unwrap();
            vehicle.set_driver_input(input).unwrap();
        }
        let ids = self.characters.clone();
        for (index, &id) in ids.iter().enumerate() {
            let angle = 0.02 * tick as f32 + index as f32 * std::f32::consts::FRAC_PI_2;
            let (sin, cos) = angle.sin_cos();
            let speed = if detour { 3.0 } else { 1.5 };
            let velocity = Vec3::new(speed * cos + 0.5, -1.0, speed * sin);
            self.world
                .character_mut(id)
                .unwrap()
                .set_linear_velocity(velocity)
                .unwrap();
            self.world
                .update_character(
                    id,
                    DT,
                    GRAVITY,
                    &ExtendedUpdateSettings::default(),
                    &QueryFilter::new(),
                )
                .unwrap();
        }
        let report = self.world.step(DT).unwrap();
        assert!(report.is_complete(), "tick {tick}: {report:?}");
    }

    /// Steps `ticks` ticks from `first` and records each.
    fn record(&mut self, first: usize, ticks: usize, digest: &mut Digest) {
        for tick in first..first + ticks {
            self.drive(tick, false);
            let record = digest.push();
            self.write_shape(&mut record.shape);
            self.write_state(&mut record.state);
        }
    }

    /// Steps `ticks` ticks from `first` without recording.
    fn advance(&mut self, first: usize, ticks: usize, detour: bool) {
        for tick in first..first + ticks {
            self.drive(tick, detour);
            self.world.take_events();
            if let Some(rules) = &self.rules {
                rules.take();
            }
        }
    }

    fn write_shape(&mut self, out: &mut Vec<u8>) {
        let events = self.world.take_events();
        for line in events.contacts.iter().map(|e| format!("{e:?}")) {
            assert!(!line.contains("NaN"), "{line}");
            out.extend_from_slice(line.as_bytes());
            out.push(b'\n');
        }
        if let Some(rules) = &self.rules {
            for line in rules.take() {
                out.extend_from_slice(line.as_bytes());
                out.push(b'\n');
            }
        }
    }

    fn write_state(&self, out: &mut Vec<u8>) {
        for &id in &self.bodies {
            record_body(&self.world, id, out);
        }
        for &id in &self.characters {
            let character = self.world.character(id).unwrap();
            assert!(!character.max_hits_exceeded(), "{id:?}");
            let line = format!(
                "{:?} {:?} {:?} {:?} {:?}\n",
                character.position(),
                character.rotation(),
                character.linear_velocity(),
                character.ground_state(),
                character.active_contacts()
            );
            out.extend_from_slice(line.as_bytes());
        }
        if self.kind == "validation" {
            // Outcome flags: which cubes went below the floor or rest on the platform.
            for &id in &self.bodies {
                let y = self.world.body(id).unwrap().position().y;
                out.push(u8::from(y < -1.0) | (u8::from(y > 1.9) << 1));
            }
        }
    }

    /// The detour: teleports, other velocities, other listeners.
    fn detour(&mut self) {
        self.install(true);
        if let Some(&last) = self.bodies.last() {
            let mut body = self.world.body_mut(last).unwrap();
            body.set_position(RVec3::new(0.5, 6.0, 0.5), Activation::Activate)
                .unwrap();
            body.set_linear_velocity(Vec3::new(0.0, -5.0, 3.0)).unwrap();
        }
        if let Some(&id) = self.characters.first() {
            // Into the box pile, so its tracked contacts grow.
            self.world
                .character_mut(id)
                .unwrap()
                .set_position(RVec3::new(6.0, 0.0, 0.0))
                .unwrap();
        }
        self.advance(WARM_UP + RECORDED, DETOUR, true);
        if self.kind == "validation" {
            // A pending invalidation right before the restore must not reach the replay.
            let resting = self.bodies[80];
            self.world
                .body_mut(resting)
                .unwrap()
                .invalidate_contact_cache();
        }
    }
}

fn add_box(world: &mut PhysicsWorld, half: Vec3, settings: &BodySettings) -> BodyId {
    world
        .create_body(&Shape::new_box(half).unwrap(), settings)
        .unwrap()
}

fn validation_scene(threads: u32) -> Scene {
    let mut world = world(GRAVITY, threads);
    world.set_event_settings(EventSettings::default().persisted_contacts(true));
    add_floor(&mut world);
    let platform = BodySettings::new_static()
        .position(RVec3::new(0.0, 1.5, 0.0))
        .user_data(PLATFORM);
    add_box(&mut world, Vec3::new(4.0, 0.05, 4.0), &platform);
    let mut bodies = Vec::new();
    for i in 0..80 {
        let (column, row) = ((i % 10) as Real, (i / 10) as Real);
        let kind = if i % 2 == 0 { ONE_WAY } else { PASSING };
        let below = i % 3 == 0;
        let y = if below {
            0.3 + 0.6 * row
        } else {
            2.5 + 0.6 * row
        };
        let mut settings = BodySettings::new_dynamic()
            .position(RVec3::new(-4.5 + column, y, -3.0 + 0.7 * row))
            .user_data(kind);
        if below {
            settings = settings.linear_velocity(Vec3::new(0.0, 7.0, 0.0));
        }
        bodies.push(add_box(&mut world, Vec3::new(0.2, 0.2, 0.2), &settings));
    }
    // Rests on the floor away from the others for the whole run, awake.
    let resting = BodySettings::new_dynamic()
        .position(RVec3::new(8.0, 0.2, 8.0))
        .user_data(ONE_WAY)
        .allow_sleeping(false);
    bodies.push(add_box(&mut world, Vec3::new(0.2, 0.2, 0.2), &resting));
    let rod = BodySettings::new_dynamic()
        .position(RVec3::new(-7.0, 0.05, -7.0))
        .motion_quality(MotionQuality::LinearCast);
    let rod = add_box(&mut world, Vec3::new(0.5, 0.05, 0.05), &rod);
    bodies.push(rod);
    let mut scene = Scene {
        kind: "validation",
        world,
        bodies,
        characters: Vec::new(),
        rod: Some(rod),
        platform: None,
        rules: None,
        vehicle: None,
    };
    scene.install(false);
    scene
}

/// Holds the rod tilted above the floor and throws it down, at 30 m/s in the run and 20 m/s in
/// the detour.
fn fire(world: &mut PhysicsWorld, rod: BodyId, detour: bool) {
    let tilt = 20f32.to_radians();
    let mut body = world.body_mut(rod).unwrap();
    body.set_position_and_rotation(
        RVec3::new(-7.0, 0.3, -7.0),
        quat_about(Vec3::new(0.0, 0.0, 1.0), tilt),
        Activation::Activate,
    )
    .unwrap();
    body.set_angular_velocity(Vec3::ZERO).unwrap();
    let speed = if detour { -20.0 } else { -30.0 };
    body.set_linear_velocity(Vec3::new(0.0, speed, 0.0))
        .unwrap();
}

fn characters_scene(threads: u32) -> Scene {
    let mut world = world(GRAVITY, threads);
    let mut user_data = Vec::new();
    let ground = BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0));
    user_data.push((add_box(&mut world, Vec3::new(30.0, 0.5, 30.0), &ground), 0));
    let conveyor = BodySettings::new_static()
        .position(RVec3::new(-4.0, 0.05, 0.0))
        .user_data(CONVEYOR);
    user_data.push((
        add_box(&mut world, Vec3::new(2.0, 0.05, 2.0), &conveyor),
        CONVEYOR,
    ));
    // Across the first character's path, clear of the others' starts.
    let ghost = BodySettings::new_static()
        .position(RVec3::new(-1.2, 1.0, 2.25))
        .user_data(GHOST);
    user_data.push((add_box(&mut world, Vec3::new(0.8, 1.0, 0.2), &ghost), GHOST));
    let platform = BodySettings::new_kinematic().position(RVec3::new(0.0, 0.1, -4.0));
    let platform = add_box(&mut world, Vec3::new(1.5, 0.1, 1.5), &platform);
    user_data.push((platform, 0));
    let mut bodies: Vec<BodyId> = user_data.iter().map(|&(id, _)| id).collect();
    for i in 0..30 {
        let (column, row) = ((i % 5) as Real, (i / 5) as Real);
        let data = BOX + i as u64;
        let settings = BodySettings::new_dynamic()
            .position(RVec3::new(
                3.0 + 0.4 * column,
                0.15 + 0.3 * row,
                -1.0 + 0.4 * column,
            ))
            .user_data(data)
            .mass(2.0);
        let id = add_box(&mut world, Vec3::new(0.15, 0.15, 0.15), &settings);
        user_data.push((id, data));
        bodies.push(id);
    }
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let settings = CharacterSettings::new(&capsule)
        .shape_offset(Vec3::new(0.0, 0.8, 0.0))
        .collide_with_characters(true);
    let characters: Vec<CharacterId> = [(-2.0, 0.0), (2.0, 0.0), (0.0, 2.0), (0.0, -2.0)]
        .iter()
        .map(|&(x, z)| {
            world
                .create_character(&settings, RVec3::new(x, 0.0, z), Quat::IDENTITY)
                .unwrap()
        })
        .collect();
    let rules = CharacterRules {
        user_data,
        no_push: Some((characters[0], characters[1])),
        ..CharacterRules::default()
    };
    let mut scene = Scene {
        kind: "characters",
        world,
        bodies,
        characters,
        rod: None,
        platform: Some(platform),
        rules: Some(Arc::new(rules)),
        vehicle: None,
    };
    scene.install(false);
    for id in scene.characters.clone() {
        scene
            .world
            .refresh_character_contacts(id, &QueryFilter::new())
            .unwrap();
    }
    scene
}

fn groups_scene(threads: u32) -> Scene {
    let (mut world, layers) = car_world(Vec3::ZERO, threads);
    world.set_gravity(GRAVITY).unwrap();
    world.set_event_settings(EventSettings::default().contacts(true));
    let ground = BodySettings::new_static()
        .position(RVec3::new(0.0, -1.0, 0.0))
        .object_layer(layers.ground);
    let mut bodies = vec![add_box(&mut world, Vec3::new(40.0, 1.0, 40.0), &ground)];

    let mut chain = GroupFilterTableBuilder::new(12).unwrap();
    for link in 0..11 {
        chain.disable_collision(link, link + 1).unwrap();
    }
    let chain = chain.build();
    for link in 0..12 {
        let settings = BodySettings::new_dynamic()
            .position(RVec3::new(
                -6.0 + 0.5 * link as Real,
                1.0 + 0.05 * link as Real,
                6.0,
            ))
            .object_layer(layers.moving)
            .collision_group(CollisionGroup::new(&chain, 1, link).unwrap());
        bodies.push(add_box(&mut world, Vec3::new(0.3, 0.1, 0.1), &settings));
    }

    let crew_table = GroupFilterTableBuilder::new(1).unwrap().build();
    let crew = CollisionGroup::new(&crew_table, 1, 0).unwrap();
    let chassis = chassis_settings(&layers, RVec3::new(0.0, 0.9, -6.0), Quat::IDENTITY)
        .collision_group(crew.clone());
    let (chassis, car) = add_car_with(
        &mut world,
        &chassis,
        VehicleCollisionTester::ray(layers.probe),
    );
    world
        .vehicle_mut(car)
        .unwrap()
        .set_gravity(CAR_GRAVITY)
        .unwrap();
    bodies.push(chassis);
    let driver = BodySettings::new_dynamic()
        .position(RVec3::new(0.0, 1.2, -6.0))
        .object_layer(layers.moving)
        .gravity_factor(0.0)
        .collision_group(crew);
    bodies.push(add_box(&mut world, Vec3::new(0.3, 0.3, 0.3), &driver));

    let cubes = GroupFilterTableBuilder::new(1).unwrap().build();
    for i in 0..18 {
        let group = CollisionGroup::new(&cubes, 1 + (i % 3) as u32, 0).unwrap();
        let (column, row) = ((i % 3) as Real, (i / 3) as Real);
        let settings = BodySettings::new_dynamic()
            .position(RVec3::new(
                6.0 + 0.3 * column,
                0.5 + 0.9 * row,
                0.2 * column,
            ))
            .object_layer(layers.moving)
            .collision_group(group);
        bodies.push(add_box(&mut world, Vec3::new(0.4, 0.4, 0.4), &settings));
    }
    Scene {
        kind: "groups",
        world,
        bodies,
        characters: Vec::new(),
        rod: None,
        platform: None,
        rules: None,
        vehicle: Some(car),
    }
}

/// `plain`: the warm-up and the recorded ticks. `replay`: the warm-up, then the recorded ticks
/// once, a detour and a replay from the save; the recorded ticks of the replay.
fn run(kind: &str, threads: u32, variant: &str) -> Digest {
    let mut scene = Scene::new(kind, threads);
    let mut digest = Digest::new();
    scene.record(0, WARM_UP, &mut digest);
    match variant {
        "plain" => scene.record(WARM_UP, RECORDED, &mut digest),
        "replay" => {
            let saved = scene.world.save_state();
            let mut first = Digest::new();
            scene.record(WARM_UP, RECORDED, &mut first);
            scene.world.restore_state(&saved).unwrap();
            scene.detour();
            scene.world.restore_state(&saved).unwrap();
            scene.install(false);
            scene.record(WARM_UP, RECORDED, &mut digest);
            let replayed = Digest {
                ticks: digest.ticks[WARM_UP..].to_vec(),
            };
            assert_same(&format!("{kind} replay in one process"), &first, &replayed);
        }
        variant => panic!("unknown variant {variant}"),
    }
    digest
}

#[test]
#[ignore = "child process of the contact control determinism gates"]
fn contact_control_child() {
    let Some((scene, threads, variant)) = child_request() else {
        return;
    };
    let job_choice = JobChoice::from_env();
    let digest = run(&scene, threads, &variant);
    match job_choice {
        JobChoice::Native | JobChoice::SingleThreaded => {
            assert_eq!(jobs::queued(), 0, "a caller job system was used")
        }
        _ => assert!(
            jobs::queued() > 0,
            "the {job_choice:?} job system was handed no job"
        ),
    }
    finish_child(&digest);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn contact_control_is_identical_with_1_and_4_workers() {
    for scene in SCENES {
        let one = digest_in_child(CHILD, scene, 1, "plain");
        let four = digest_in_child(CHILD, scene, 4, "plain");
        assert!(
            one.ticks.iter().any(|tick| !tick.shape.is_empty()),
            "{scene}"
        );
        assert_same(&format!("{scene}, 1 vs 4 workers"), &one, &four);
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn contact_control_is_identical_with_caller_job_systems() {
    for scene in SCENES {
        assert_caller_job_systems_agree(CHILD, scene, "plain");
    }
}

#[test]
fn contact_control_replays_after_a_detour_in_one_process() {
    for scene in SCENES {
        for threads in [1, 4] {
            run(scene, threads, "replay");
        }
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn contact_control_replays_after_a_detour_across_processes() {
    for scene in SCENES {
        let plain = digest_in_child(CHILD, scene, 1, "plain");
        let replayed = digest_in_child(CHILD, scene, 4, "replay");
        assert_same(
            &format!("{scene}, replay across processes"),
            &plain,
            &replayed,
        );
    }
}

/// The scenes do what they are meant to: cubes pass the platform, characters ride the conveyor
/// and walk through the ghost, and grouped bodies pass through each other.
#[test]
fn the_scenes_exercise_their_rules() {
    let mut validation = Scene::new("validation", 2);
    validation.advance(0, 180, false);
    let heights: Vec<Real> = validation.bodies[..80]
        .iter()
        .map(|&id| validation.world.body(id).unwrap().position().y)
        .collect();
    assert!(
        heights.iter().any(|&y| y > 1.5),
        "cubes rest on the platform"
    );
    assert!(
        heights.iter().any(|&y| y < 1.5),
        "cubes passed the platform"
    );

    let mut characters = Scene::new("characters", 2);
    let conveyor = characters.bodies[1];
    let mut log = Vec::new();
    let (mut rode, mut passed) = (false, false);
    for tick in 0..240 {
        characters.drive(tick, false);
        log.extend(characters.rules.as_ref().unwrap().take());
        for &id in &characters.characters {
            let character = characters.world.character(id).unwrap();
            rode |= character.ground_body() == Some(conveyor)
                && character.ground_velocity() == Vec3::new(0.0, 0.0, 1.5);
            let p = character.position();
            // The ghost wall spans x -2..-0.4 and z 2.05..2.45.
            passed |= (-2.0..-0.4).contains(&p.x) && (2.05..2.45).contains(&p.z);
        }
    }
    assert!(rode, "a character rides the conveyor");
    assert!(passed, "a character stands inside the ghost wall");
    assert!(log.iter().any(|line| line.starts_with("added")));
    assert!(log.iter().any(|line| line.starts_with("removed")));
    assert!(log
        .iter()
        .any(|line| line.contains("can_receive_impulses: false")));

    let mut groups = Scene::new("groups", 2);
    let bodies = groups.bodies.clone();
    let (links, chassis, driver, cubes) = (&bodies[1..13], bodies[13], bodies[14], &bodies[15..]);
    let parked = groups.world.body(driver).unwrap().position();
    let mut touching = Vec::new();
    for tick in 0..240 {
        groups.drive(tick, false);
        for event in groups.world.take_events().contacts {
            if !matches!(event, ContactEvent::Removed(_)) {
                let pair = event.pair();
                touching.push((pair.body1, pair.body2));
            }
        }
    }
    let touched = |a: BodyId, b: BodyId| touching.contains(&(a.min(b), a.max(b)));
    for pair in links.windows(2) {
        assert!(!touched(pair[0], pair[1]), "chain neighbours {pair:?}");
    }
    assert!(links.iter().any(|&link| touched(bodies[0], link)));
    assert!(!touched(chassis, driver));
    assert_eq!(
        groups.world.body(driver).unwrap().position(),
        parked,
        "the driver stays put"
    );
    let (mut same, mut different) = (false, false);
    for (i, &a) in cubes.iter().enumerate() {
        for (j, &b) in cubes.iter().enumerate().skip(i + 1) {
            if i % 3 == j % 3 {
                same |= touched(a, b);
            } else {
                different |= touched(a, b);
            }
        }
    }
    assert!(!same, "cubes of one group id never touch");
    assert!(different, "cubes of different group ids do");
}
