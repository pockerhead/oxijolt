//! Determinism of batch body calls: a scene that creates detached bodies, adds them in batches
//! and removes bodies in batches runs bit for bit the same with 1 and 4 worker threads, in one
//! process and in two.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::*;
use oxijolt::*;

const TICKS: usize = 240;
const REMOVAL_TICK: usize = 200;

/// A floor and three waves of cubes, each created detached and added in one batch: a layer at
/// tick 1, a second layer dropped on it at tick 30 and a wave added asleep at tick 120. At tick
/// [`REMOVAL_TICK`], when the pile sleeps, a batch of the first layer is removed, which wakes
/// what rests on it.
struct Scene {
    world: PhysicsWorld,
    cubes: Vec<BodyId>,
}

impl Scene {
    fn new(threads: u32) -> Self {
        let mut world = world(Vec3::new(0.0, -9.81, 0.0), threads);
        world.set_event_settings(EventSettings::default().body_activation(true));
        add_floor(&mut world);
        Self {
            world,
            cubes: Vec::new(),
        }
    }

    /// Creates a 4 x 4 layer of cubes at `height`, offset by `shift`, and adds it in one batch,
    /// listed in reverse creation order.
    fn add_wave(&mut self, height: Real, shift: Real, activation: Activation) {
        let shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
        let mut wave = Vec::new();
        for x in 0..4 {
            for z in 0..4 {
                let position = RVec3::new(1.1 * x as Real + shift, height, 1.1 * z as Real);
                let settings = BodySettings::new_dynamic().position(position);
                wave.push(self.world.create_body_detached(&shape, &settings).unwrap());
            }
        }
        wave.reverse();
        self.world.add_bodies(&wave, activation).unwrap();
        self.cubes.extend(wave);
    }

    fn tick(&mut self, tick: usize) {
        match tick {
            1 => self.add_wave(0.6, 0.0, Activation::Activate),
            30 => self.add_wave(2.0, 0.4, Activation::Activate),
            REMOVAL_TICK => {
                let removed: Vec<BodyId> = self.cubes[..16].iter().step_by(3).copied().collect();
                self.world.remove_bodies(&removed).unwrap();
                self.cubes.retain(|id| !removed.contains(id));
            }
            120 => self.add_wave(0.6, 8.0, Activation::DontActivate),
            _ => {}
        }
        assert!(self.world.step(DT).unwrap().is_complete());
    }

    /// Every cube's state and the step's activation events.
    fn record(&mut self, digest: &mut Digest) {
        let tick = digest.push();
        for &id in &self.cubes {
            record_body(&self.world, id, &mut tick.state);
        }
        let events = self.world.take_events();
        tick.state
            .extend(format!("{:?}", events.activations).bytes());
    }
}

/// The scene run for [`TICKS`] ticks with `threads` workers.
fn batches(threads: u32) -> Digest {
    let mut scene = Scene::new(threads);
    let mut digest = Digest::new();
    for tick in 1..=TICKS {
        scene.tick(tick);
        scene.record(&mut digest);
    }
    digest
}

#[test]
#[ignore = "child process of the batch body determinism gate"]
fn batch_bodies_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "batches");
    finish_child(&batches(threads));
}

#[test]
fn batches_match_with_1_and_4_workers() {
    let one = batches(1);
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in one process", &one, &batches(4));
    // The scene moves, and the removal wakes cubes that had fallen asleep.
    assert_ne!(one.ticks[0].state, one.ticks[TICKS - 1].state);
    let text = String::from_utf8_lossy(&one.ticks[REMOVAL_TICK - 1].state).into_owned();
    assert!(
        text.contains("Activated("),
        "the removal woke nothing: {text}"
    );
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn batches_match_across_processes() {
    let one = digest_in_child("batch_bodies_child", "batches", 1, "");
    let four = digest_in_child("batch_bodies_child", "batches", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &batches(1), &one);
}
