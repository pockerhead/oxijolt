//! Contact tuning per world: bounds, read-back, the penetration slop as a tolerance, speculative
//! contacts, and equal results with 1 and 4 workers.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::determinism::{
    assert_same, child_request, digest_in_child, finish_child, first_divergence, Digest,
};
use common::math::v3;
use common::DT;
use oxijolt::*;

type Setter = fn(WorldSettings, f32) -> WorldSettings;

const SETTERS: [(&str, Setter); 2] = [
    ("penetration_slop", WorldSettings::penetration_slop),
    (
        "speculative_contact_distance",
        WorldSettings::speculative_contact_distance,
    ),
];

#[test]
fn contact_distances_outside_their_range_are_refused() {
    for (name, set) in SETTERS {
        for value in [
            -0.001,
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            limits::MAX_SHAPE_EXTENT.next_up(),
        ] {
            assert!(
                matches!(
                    PhysicsWorld::new(set(WorldSettings::default(), value)),
                    Err(WorldError::InvalidValue(_))
                ),
                "{name}({value:?}) was accepted"
            );
        }
    }
}

#[test]
fn contact_distances_read_back_from_jolt() {
    let read = |settings: WorldSettings| {
        let world = PhysicsWorld::new(settings).unwrap();
        (
            world.penetration_slop().to_bits(),
            world.speculative_contact_distance().to_bits(),
        )
    };
    let bits = |slop: f32, distance: f32| (slop.to_bits(), distance.to_bits());
    assert_eq!(read(WorldSettings::default()), bits(0.02, 0.02));
    assert_eq!(
        read(WorldSettings::default().penetration_slop(0.005)),
        bits(0.005, 0.02)
    );
    assert_eq!(
        read(WorldSettings::default().speculative_contact_distance(0.05)),
        bits(0.02, 0.05)
    );
    assert_eq!(
        read(
            WorldSettings::default()
                .penetration_slop(0.005)
                .speculative_contact_distance(0.05)
        ),
        bits(0.005, 0.05)
    );
}

/// How far a dropped sphere sank into the floor.
struct Settled {
    /// The deepest penetration over the run, in metres.
    peak: f64,
    /// The penetration after the last step, in metres.
    rest: f64,
    /// Whether the sphere sleeps after the last step.
    sleeping: bool,
}

/// Drops a sphere of radius 0.5 without restitution from `height` (its centre) onto the floor in
/// a world with `slop` and follows it for 600 steps.
fn drop_sphere(slop: f32, height: Real) -> Settled {
    let mut world = PhysicsWorld::new(WorldSettings::default().penetration_slop(slop)).unwrap();
    common::add_floor(&mut world);
    let sphere = world
        .create_body(
            &Shape::new_sphere(0.5).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, height, 0.0))
                .restitution(0.0),
        )
        .unwrap();
    let mut peak = f64::NEG_INFINITY;
    let mut rest = f64::NAN;
    for _ in 0..600 {
        assert!(world.step(DT).unwrap().is_complete());
        rest = 0.5 - v3(world.body(sphere).unwrap().position())[1];
        peak = peak.max(rest);
    }
    let sleeping = world.body(sphere).unwrap().is_sleeping();
    Settled {
        peak,
        rest,
        sleeping,
    }
}

/// How far past the slop the solver may leave a resting sphere in these fixtures.
const TOLERANCE: f64 = 1e-4;

#[test]
fn a_deep_impact_is_corrected_down_to_the_slop() {
    for height in [3.0, 5.0] {
        let default = drop_sphere(0.02, height);
        let tuned = drop_sphere(0.005, height);
        let report = format!(
            "from {height} m: default peak {} rest {}, tuned peak {} rest {}",
            default.peak, default.rest, tuned.peak, tuned.rest
        );
        assert!(default.peak > 0.03 && tuned.peak > 0.03, "{report}");
        assert!(
            (0.019..=0.02 + TOLERANCE).contains(&default.rest),
            "{report}"
        );
        assert!(tuned.rest <= 0.005 + TOLERANCE, "{report}");
        assert!(tuned.rest < default.rest - 0.01, "{report}");
        assert!(default.sleeping && tuned.sleeping, "{report}");
    }
}

#[test]
fn a_gently_placed_sphere_rests_within_the_slop_not_at_it() {
    for slop in [0.02, 0.005] {
        let settled = drop_sphere(slop, 0.51);
        let report = format!("slop {slop}: peak {} rest {}", settled.peak, settled.rest);
        assert!(settled.rest <= f64::from(slop) + TOLERANCE, "{report}");
        assert!(settled.sleeping, "{report}");
        if slop == 0.02 {
            assert!(settled.rest < 0.005, "{report}");
        }
    }
}

/// The contact events of the first step between a floor and a still sphere whose bottom is
/// 0.05 m above it, without gravity, in a world with `distance`.
fn first_step_contacts(distance: f32) -> Vec<ContactEvent> {
    let settings = WorldSettings::default()
        .gravity(Vec3::ZERO)
        .speculative_contact_distance(distance);
    let mut world = PhysicsWorld::new(settings).unwrap();
    world.set_event_settings(EventSettings::default().contacts(true));
    let floor = common::add_floor(&mut world);
    let sphere = world
        .create_body(
            &Shape::new_sphere(0.5).unwrap(),
            &BodySettings::new_dynamic().position(RVec3::new(0.0, 0.55, 0.0)),
        )
        .unwrap();
    assert!(world.step(DT).unwrap().is_complete());
    world
        .take_events()
        .contacts
        .into_iter()
        .filter(|event| {
            let pair = event.pair();
            (pair.body1, pair.body2) == (floor, sphere)
                || (pair.body1, pair.body2) == (sphere, floor)
        })
        .collect()
}

#[test]
fn speculative_contacts_reach_bodies_within_the_distance() {
    let wide = first_step_contacts(0.1);
    assert_eq!(wide.len(), 1, "{wide:?}");
    match &wide[0] {
        ContactEvent::Added { manifold, .. } => assert!(
            (-0.06..-0.04).contains(&manifold.penetration_depth),
            "{manifold:?}"
        ),
        other => panic!("expected an added contact, got {other:?}"),
    }
    let narrow = first_step_contacts(0.02);
    assert!(narrow.is_empty(), "{narrow:?}");
}

/// Asserts that every component of the state of `id` is finite.
fn assert_finite(world: &PhysicsWorld, id: BodyId, what: &str) {
    let body = world.body(id).unwrap();
    let position = v3(body.position());
    let rotation: [f32; 4] = body.rotation().into();
    let linear: [f32; 3] = body.linear_velocity().into();
    let angular: [f32; 3] = body.angular_velocity().into();
    assert!(
        position.iter().all(|c| c.is_finite())
            && rotation
                .into_iter()
                .chain(linear)
                .chain(angular)
                .all(f32::is_finite),
        "{what}: body {id:?} at {position:?}, rotation {rotation:?}, velocities {linear:?} \
         {angular:?}"
    );
}

#[test]
fn contact_distances_at_their_bounds_keep_a_small_scene_finite() {
    const M: f32 = limits::MAX_SHAPE_EXTENT;
    for (slop, distance) in [(0.0, 0.0), (M, M), (0.0, M), (M, 0.0), (M, 0.02), (0.02, M)] {
        let settings = WorldSettings::default()
            .penetration_slop(slop)
            .speculative_contact_distance(distance);
        let mut world = PhysicsWorld::new(settings).unwrap();
        let mut ids = common::build_stacks(&mut world);
        ids.push(
            world
                .create_body(
                    &Shape::new_sphere(0.5).unwrap(),
                    &BodySettings::new_dynamic().position(RVec3::new(-3.0, 3.0, 0.0)),
                )
                .unwrap(),
        );
        common::step(&mut world, 120);
        let what = format!("slop {slop}, distance {distance}");
        for id in ids {
            assert_finite(&world, id, &what);
        }
    }
}

#[test]
fn far_bodies_within_the_speculative_distance_stay_finite() {
    let settings = WorldSettings::default()
        .gravity(Vec3::ZERO)
        .speculative_contact_distance(limits::MAX_SHAPE_EXTENT);
    let mut world = PhysicsWorld::new(settings).unwrap();
    world.set_event_settings(EventSettings::default().contacts(true));
    let ball = Shape::new_sphere(0.5).unwrap();
    let mut sphere = |x: Real, speed: f32| {
        world
            .create_body(
                &ball,
                &BodySettings::new_dynamic()
                    .position(RVec3::new(x, 0.0, 0.0))
                    .linear_velocity(Vec3::new(speed, 0.0, 0.0)),
            )
            .unwrap()
    };
    let left = sphere(-999.5, 10.0);
    let right = sphere(999.5, -10.0);
    assert!(world.step(DT).unwrap().is_complete());
    let events = world.take_events().contacts;
    let far_contact = events.iter().any(|event| match event {
        ContactEvent::Added { manifold, .. } => {
            let pair = (manifold.pair.body1, manifold.pair.body2);
            (pair == (left, right) || pair == (right, left)) && manifold.penetration_depth < -1990.0
        }
        _ => false,
    });
    assert!(far_contact, "{events:?}");
    for delta_time in [DT, 1.0e-3] {
        for _ in 0..30 {
            assert!(world.step(delta_time).unwrap().is_complete());
        }
        for id in [left, right] {
            assert_finite(&world, id, &format!("after steps of {delta_time} s"));
        }
    }
}

/// The two stacks with the default or the tuned contact distances, every body per tick.
fn run_stacks(threads: u32, variant: &str) -> Digest {
    let settings = match variant {
        "default" => WorldSettings::default(),
        "tuned" => WorldSettings::default()
            .penetration_slop(0.005)
            .speculative_contact_distance(0.05),
        other => panic!("unknown contact tuning variant {other:?}"),
    };
    let mut world = PhysicsWorld::new(common::jobs::with_threads(settings, threads)).unwrap();
    let ids = common::build_stacks(&mut world);
    let mut digest = Digest::new();
    for _ in 0..120 {
        assert!(world.step(DT).unwrap().is_complete());
        let tick = digest.push();
        for &id in &ids {
            common::record_body(&world, id, &mut tick.state);
        }
    }
    digest
}

#[test]
#[ignore = "child process of the contact tuning determinism gate"]
fn contact_tuning_child() {
    let Some((scenario, threads, variant)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "stacks");
    finish_child(&run_stacks(threads, &variant));
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn tuned_stacks_are_identical_with_1_and_4_workers() {
    let one = digest_in_child("contact_tuning_child", "stacks", 1, "tuned");
    let four = digest_in_child("contact_tuning_child", "stacks", 4, "tuned");
    let default = digest_in_child("contact_tuning_child", "stacks", 1, "default");
    assert_eq!(one.ticks.len(), 120);
    assert_ne!(one.ticks[0], one.ticks[119], "the stacks did not move");
    assert_same("tuned stacks, 1 vs 4 workers", &one, &four);
    assert!(
        first_divergence(&one, &default).is_some(),
        "the tuned run equals the default run"
    );
}
