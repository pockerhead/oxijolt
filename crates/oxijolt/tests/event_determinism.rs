//! Same-machine determinism of the event stream: the events a world records each step, every
//! field bit for bit, must not depend on the worker thread count or the job system, and
//! recording events with a listener that changes nothing must not change the simulation.
//!
//! The scene drops 64 cubes of two materials on a two-box compound floor and a heightfield of
//! two materials, drapes a cloth over a sphere, throws a rod with continuous collision detection
//! onto the floor every 40 ticks, and removes one cube and creates another halfway. A pure
//! contact listener sets the friction by material, halves the other body's inverse mass for
//! the cloth's contacts with even-indexed bodies, and gives every fourth body's contacts a
//! settings value kept from a contact with a short lever, which their longer levers reject.
//! Every added contact carries Jolt's collision estimate.
//!
//! Each run happens in its own child process (this test binary, running the ignored
//! `event_determinism_child` test); see `common::determinism`.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use std::sync::{Arc, Mutex};

use common::determinism::*;
use common::jobs::{self, JobChoice};
use common::soft_body::Cloth;
use common::*;
use oxijolt::*;

const CHILD: &str = "event_determinism_child";
const TICKS: usize = 240;
const MATERIAL_A: u64 = 1;
const MATERIAL_B: u64 = 2;

/// Friction by material, a softer push from the cloth on even-indexed bodies, and `kept` for
/// the contacts of bodies 2 whose index is 1 modulo 4.
struct Policy {
    kept: ContactSettings,
}

impl ContactListener for Policy {
    fn contact_added(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        self.contact_persisted(manifold, settings);
    }

    fn contact_persisted(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        if manifold.pair.body2.index() % 4 == 1 {
            *settings = self.kept;
            return;
        }
        let friction = if manifold.materials.contains(&Some(MATERIAL_B)) {
            0.8
        } else if manifold.materials.contains(&Some(MATERIAL_A)) {
            0.2
        } else {
            return;
        };
        settings.set_combined_friction(friction).unwrap();
    }

    fn soft_body_contact_validate(
        &self,
        _: BodyId,
        other: BodyId,
        settings: &mut SoftBodyContactSettings,
    ) -> SoftBodyValidateResult {
        if other.index().is_multiple_of(2) {
            settings.set_inverse_mass_scale2(0.5).unwrap();
        }
        SoftBodyValidateResult::AcceptContact
    }
}

/// Settings read from a cube resting right above a floor's centre of mass, with the largest
/// angular surface velocity that lever allows: too fast for a contact farther from body 1's
/// centre of mass.
fn short_lever_settings() -> ContactSettings {
    struct Keep(Mutex<Option<ContactSettings>>);
    impl ContactListener for Keep {
        fn contact_added(&self, _: &ContactManifold, settings: &mut ContactSettings) {
            let mut kept = self.0.lock().unwrap();
            if kept.is_none() {
                let mut spin = limits::MAX_ANGULAR_VELOCITY;
                while settings
                    .set_relative_angular_surface_velocity(Vec3::new(0.0, spin, 0.0))
                    .is_err()
                {
                    spin *= 0.9;
                }
                *kept = Some(*settings);
            }
        }
    }
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    let keep = Arc::new(Keep(Mutex::default()));
    world.set_contact_listener(Some(keep.clone()));
    let floor = Shape::new_box(Vec3::new(10.0, 0.5, 10.0)).unwrap();
    let cube = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    let at = |y| RVec3::new(0.0, y, 0.0);
    world
        .create_body(&floor, &BodySettings::new_static().position(at(-0.5)))
        .unwrap();
    world
        .create_body(&cube, &BodySettings::new_dynamic().position(at(0.26)))
        .unwrap();
    for _ in 0..10 {
        assert!(world.step(DT).unwrap().is_complete());
    }
    let kept = *keep.0.lock().unwrap();
    kept.expect("the cube should touch the floor")
}

/// A listener that changes nothing.
struct NoOp;

impl ContactListener for NoOp {}

/// What a run installs: `events` (every event and [`Policy`]), `observed` (every event and
/// [`NoOp`]) or `silent` (nothing).
fn configure(world: &mut PhysicsWorld, variant: &str) {
    let every_event = EventSettings::default()
        .persisted_contacts(true)
        .collision_estimates(true)
        .body_activation(true)
        .soft_body_contacts(true)
        .soft_body_validations(true);
    match variant {
        "events" => {
            world.set_event_settings(every_event);
            let kept = short_lever_settings();
            world.set_contact_listener(Some(Arc::new(Policy { kept })));
        }
        "observed" => {
            world.set_event_settings(every_event);
            world.set_contact_listener(Some(Arc::new(NoOp)));
        }
        "silent" => {}
        variant => panic!("unknown variant {variant}"),
    }
}

/// The two materials, the compound floor (x and z in -10..10, top at y = 0), a heightfield east
/// of it (x in 12..28), and a static sphere on the floor; returns the static bodies.
fn build_ground(world: &mut PhysicsWorld, materials: &[PhysicsMaterial; 2]) -> Vec<BodyId> {
    let [a, b] = materials;
    let left = Shape::new_box_with_material(Vec3::new(5.0, 0.5, 10.0), 0.05, a).unwrap();
    let right = Shape::new_box_with_material(Vec3::new(5.0, 0.5, 10.0), 0.05, b).unwrap();
    let child = |shape, x| CompoundChild {
        shape,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data: 0,
    };
    let floor = Shape::new_compound(&[child(&left, -5.0), child(&right, 5.0)]).unwrap();
    let at = |x, y, z| BodySettings::new_static().position(RVec3::new(x, y, z));
    let mut ids = vec![world.create_body(&floor, &at(0.0, -0.5, 0.0)).unwrap()];

    let n = 17;
    let samples: Vec<f32> = (0..n * n)
        .map(|i| {
            let (x, z) = ((i % n) as f32, (i / n) as f32);
            0.2 * (0.5 * x).sin() * (0.5 * z).cos()
        })
        .collect();
    let cells = (n - 1) * (n - 1);
    let indices: Vec<u8> = (0..cells)
        .map(|i| ((i % (n - 1) + i / (n - 1)) % 2) as u8)
        .collect();
    let materials = [a, b];
    let settings = HeightFieldSettings::default()
        .offset(Vec3::new(12.0, 0.0, -8.0))
        .materials(&materials, &indices);
    let field = Shape::new_height_field(n as u32, &samples, &settings).unwrap();
    ids.push(world.create_body(&field, &at(0.0, 0.0, 0.0)).unwrap());

    let sphere = Shape::new_sphere(1.0).unwrap();
    ids.push(world.create_body(&sphere, &at(0.0, 0.5, -5.0)).unwrap());
    ids
}

/// A dynamic cube of half extent 0.25 made of `material`.
fn add_cube(world: &mut PhysicsWorld, material: &PhysicsMaterial, position: RVec3) -> BodyId {
    let shape = Shape::new_box_with_material(Vec3::new(0.25, 0.25, 0.25), 0.05, material).unwrap();
    world
        .create_body(&shape, &BodySettings::new_dynamic().position(position))
        .unwrap()
}

/// 32 cubes over the floor and 32 over the heightfield, alternating materials.
fn add_cubes(world: &mut PhysicsWorld, materials: &[PhysicsMaterial; 2]) -> Vec<BodyId> {
    let mut ids = Vec::new();
    for (field, x0) in [(0, -8.0), (1, 14.0)] {
        for i in 0..32 {
            let (column, row) = ((i % 8) as Real, (i / 8) as Real);
            let position = RVec3::new(x0 + 1.5 * column, 1.0 + 0.6 * row, 2.0 + 1.3 * row);
            ids.push(add_cube(world, &materials[(i + field) % 2], position));
        }
    }
    ids
}

/// Tilt of the rod when it is fired, radians.
const ROD_TILT: f32 = 20.0 * std::f32::consts::PI / 180.0;

/// A thin 1 m rod with continuous collision detection, lying on the floor's west end.
fn add_rod(world: &mut PhysicsWorld) -> BodyId {
    let shape = Shape::new_box(Vec3::new(0.5, 0.05, 0.05)).unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_dynamic()
                .position(RVec3::new(-7.0, 0.05, -8.0))
                .motion_quality(MotionQuality::LinearCast),
        )
        .unwrap()
}

/// Holds the rod tilted 1 mm above the floor and throws it down at 30 m/s. Its lower end
/// touches in the discrete stage, which stops that end but leaves the centre of mass fast
/// enough for the continuous cast, which then hits the floor again: one pair, two events.
fn fire(world: &mut PhysicsWorld, rod: BodyId) {
    let lowest = 0.5 * ROD_TILT.sin() + 0.05 * ROD_TILT.cos();
    let mut body = world.body_mut(rod).unwrap();
    body.set_position_and_rotation(
        RVec3::new(-7.0, Real::from(lowest) + 0.001, -8.0),
        quat_about(Vec3::new(0.0, 0.0, 1.0), ROD_TILT),
        Activation::Activate,
    )
    .unwrap();
    body.set_angular_velocity(Vec3::ZERO).unwrap();
    body.set_linear_velocity(Vec3::new(0.0, -30.0, 0.0))
        .unwrap();
}

fn add_cloth(world: &mut PhysicsWorld) -> BodyId {
    let settings = Cloth::new(8, 0.25)
        .builder()
        .create_constraints(SoftBodyBendType::None, SoftBodyVertexAttributes::default())
        .build()
        .unwrap();
    world
        .create_soft_body(
            &settings,
            &SoftBodySettings::default().position(RVec3::new(0.0, 2.0, -5.0)),
        )
        .unwrap()
}

/// Every event of a step, each written with `Debug`, which prints floats in their shortest
/// exact form (so equal text means equal bits, `-0.0` apart from `0.0`). `Debug` prints every
/// NaN alike, so no event may hold one, nor an infinity.
fn record_events(events: &WorldEvents, out: &mut Vec<u8>) {
    let lines = events
        .contacts
        .iter()
        .map(|e| format!("{e:?}"))
        .chain(events.activations.iter().map(|e| format!("{e:?}")))
        .chain(
            events
                .soft_body_validations
                .iter()
                .map(|e| format!("{e:?}")),
        )
        .chain(events.soft_body_contacts.iter().map(|e| format!("{e:?}")))
        .chain(
            events
                .rejected_contact_settings
                .iter()
                .map(|e| format!("{e:?}")),
        );
    for line in lines {
        assert!(!line.contains("NaN") && !line.contains("inf"), "{line}");
        out.extend_from_slice(line.as_bytes());
        out.push(b'\n');
    }
}

/// Whether one call of `collision_steps` collision steps reported a pair from a continuous
/// collision cast and a discrete contact in one collision step. A pair gets at most one Added
/// or Persisted per collision step from either alone, so this is more Added and Persisted
/// events for one pair than the call has collision steps.
fn has_repeated_pair(contacts: &[ContactEvent], collision_steps: u32) -> bool {
    let mut counts: Vec<(SubShapeIdPair, u32)> = Vec::new();
    for event in contacts {
        if matches!(event, ContactEvent::Removed(_)) {
            continue;
        }
        match counts.iter_mut().find(|(pair, _)| *pair == event.pair()) {
            Some((_, count)) => *count += 1,
            None => counts.push((event.pair(), 1)),
        }
    }
    counts.iter().any(|&(_, count)| count > collision_steps)
}

fn run_events(threads: u32, variant: &str, collision_steps: u32) -> Digest {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), threads);
    configure(&mut world, variant);
    let materials = [
        PhysicsMaterial::new(MATERIAL_A).unwrap(),
        PhysicsMaterial::new(MATERIAL_B).unwrap(),
    ];
    let mut ids = build_ground(&mut world, &materials);
    let cubes = add_cubes(&mut world, &materials);
    ids.extend(&cubes);
    let rod = add_rod(&mut world);
    ids.push(rod);
    let cloth = add_cloth(&mut world);
    ids.push(cloth);

    let mut digest = Digest::new();
    let mut repeated_pair = false;
    let mut rod_estimated = false;
    let mut rejections = 0;
    for tick in 0..TICKS {
        if tick % 40 == 10 {
            fire(&mut world, rod);
        }
        if tick == TICKS / 2 {
            world.remove_body(cubes[5]).unwrap();
            ids.retain(|&id| id != cubes[5]);
        }
        if tick == TICKS / 2 + 1 {
            ids.push(add_cube(
                &mut world,
                &materials[0],
                RVec3::new(-2.0, 3.0, 8.0),
            ));
        }
        let report = world
            .step_with_collision_steps(DT, collision_steps)
            .unwrap();
        assert!(report.is_complete(), "tick {tick}: {report:?}");
        let events = world.take_events();
        repeated_pair |= has_repeated_pair(&events.contacts, collision_steps);
        rod_estimated |= events.contacts.iter().any(|event| {
            matches!(
                event,
                ContactEvent::Added {
                    estimate: Some(_),
                    ..
                }
            ) && [event.pair().body1, event.pair().body2].contains(&rod)
        });
        rejections += events.rejected_contact_settings.len();
        let record = digest.push();
        record_events(&events, &mut record.shape);
        for &id in &ids {
            record_body(&world, id, &mut record.state);
        }
        for vertex in world.soft_body(cloth).unwrap().vertices() {
            record
                .state
                .extend_from_slice(format!("{vertex:?}").as_bytes());
        }
    }
    if variant != "silent" {
        assert!(
            repeated_pair,
            "the rod should touch the floor in one step both discretely and continuously"
        );
        assert!(rod_estimated, "the rod's impacts should be estimated");
    }
    if variant == "events" {
        assert!(
            rejections > 0,
            "the kept settings should be rejected somewhere"
        );
    }
    digest
}

#[test]
#[ignore = "child process of the event determinism gates"]
fn event_determinism_child() {
    let Some((scenario, threads, variant)) = child_request() else {
        return;
    };
    let job_choice = JobChoice::from_env();
    let digest = match scenario.as_str() {
        "events" => run_events(threads, &variant, 1),
        "sub_stepped_events" => run_events(threads, &variant, PhysicsWorld::MAX_COLLISION_STEPS),
        scenario => panic!("unknown scenario {scenario}"),
    };
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
fn events_are_identical_with_1_and_4_workers() {
    let one = digest_in_child(CHILD, "events", 1, "events");
    let four = digest_in_child(CHILD, "events", 4, "events");
    assert!(one.ticks.iter().any(|tick| !tick.shape.is_empty()));
    assert_same("events, 1 vs 4 workers", &one, &four);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn sub_stepped_events_are_identical_with_1_and_4_workers() {
    let one = digest_in_child(CHILD, "sub_stepped_events", 1, "events");
    let four = digest_in_child(CHILD, "sub_stepped_events", 4, "events");
    assert!(one.ticks.iter().any(|tick| !tick.shape.is_empty()));
    assert_same("sub-stepped events, 1 vs 4 workers", &one, &four);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn events_are_identical_with_caller_job_systems() {
    assert_caller_job_systems_agree(CHILD, "events", "events");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn events_are_identical_with_the_single_threaded_job_system() {
    for scenario in ["events", "sub_stepped_events"] {
        assert_single_threaded_agrees(CHILD, scenario, "events");
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn observing_every_event_leaves_the_simulation_unchanged() {
    let silent = digest_in_child(CHILD, "events", 2, "silent");
    let observed = digest_in_child(CHILD, "events", 2, "observed");
    assert_eq!(silent.ticks.len(), observed.ticks.len());
    assert!(silent.ticks.iter().all(|tick| tick.shape.is_empty()));
    assert!(observed.ticks.iter().any(|tick| !tick.shape.is_empty()));
    for (tick, (a, b)) in silent.ticks.iter().zip(&observed.ticks).enumerate() {
        assert!(a.state == b.state, "state differs at tick {tick}");
    }
}
