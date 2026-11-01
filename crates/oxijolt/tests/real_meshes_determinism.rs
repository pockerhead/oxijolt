//! Determinism on real meshes: 20 spheres and boxes dropped on the track tile, the dungeon
//! corridor and (with `OXIJOLT_MODELS`, see `real_meshes_downloaded.rs`) the skull statue run
//! bit for bit the same with 1 and 4 worker threads, each run in its own process, and most of
//! them land on the mesh.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

#[path = "real_meshes_checks/mod.rs"]
mod checks;
mod common;

use checks::*;
use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::*;
use oxijolt::*;

const TICKS: usize = 300;
/// Bodies dropped on each mesh.
const DROPPED: usize = 20;
/// Of them, how many must touch the mesh.
const LANDED: usize = 15;

/// The indices (from 1) of the dropped bodies that start touching `mesh_body` in `events`,
/// sorted, one byte each.
fn mesh_contacts(events: &WorldEvents, mesh_body: BodyId, bodies: &[BodyId]) -> Vec<u8> {
    let mut touching: Vec<u8> = events
        .contacts
        .iter()
        .filter_map(|event| match event {
            ContactEvent::Added { manifold, .. } => Some(manifold.pair),
            _ => None,
        })
        .filter_map(|pair| match (pair.body1, pair.body2) {
            (mesh, other) | (other, mesh) if mesh == mesh_body => Some(other),
            _ => None,
        })
        .filter_map(|id| bodies.iter().position(|&body| body == id))
        .map(|index| index as u8)
        .collect();
    touching.sort_unstable();
    touching.dedup();
    touching
}

/// The bodies on `model`'s mesh, recorded every tick, and in each tick's shape record the
/// bodies that started touching the mesh.
fn run(model: &Model, threads: u32) -> Digest {
    let (mesh, _) = Shape::new_mesh(&model.vertices, &model.triangles).unwrap();
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), threads);
    world.set_event_settings(EventSettings::default().contacts(true));
    let mut bodies = vec![world
        .create_body(&mesh, &BodySettings::new_static())
        .unwrap()];
    let (min, max) = model.bounds();
    let sphere = Shape::new_sphere(0.1).unwrap();
    let block = Shape::new_box(Vec3::new(0.05, 0.05, 0.05)).unwrap();
    for i in 0..DROPPED {
        let (u, v) = ((i % 5) as f32 / 4.0, (i / 5) as f32 / 3.0);
        let x = min.x + (max.x - min.x) * (0.15 + 0.7 * u);
        let z = min.z + (max.z - min.z) * (0.15 + 0.7 * v);
        let position = RVec3::new(real(x), real(max.y + 0.5 + 0.3 * (i % 3) as f32), real(z));
        let shape = if i % 2 == 0 { &sphere } else { &block };
        bodies.push(
            world
                .create_body(shape, &BodySettings::new_dynamic().position(position))
                .unwrap(),
        );
    }
    let mut digest = Digest::new();
    for _ in 0..TICKS {
        step(&mut world, 1);
        let tick = digest.push();
        tick.shape = mesh_contacts(&world.take_events(), bodies[0], &bodies);
        for &id in &bodies {
            record_body(&world, id, &mut tick.state);
        }
    }
    digest
}

fn model(scenario: &str) -> Model {
    match scenario {
        "skull" => Model::load("ScatteringSkull").scaled(10.0),
        name => Model::load(name),
    }
}

#[test]
#[ignore = "child process of the real mesh determinism gates"]
fn real_meshes_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    finish_child(&run(&model(&scenario), threads));
}

fn assert_runs_agree(scenario: &str) {
    let one = digest_in_child("real_meshes_child", scenario, 1, "");
    let four = digest_in_child("real_meshes_child", scenario, 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same(&format!("{scenario}: 1 vs 4 workers"), &one, &four);
    assert_ne!(
        one.ticks[0].state,
        one.ticks[TICKS - 1].state,
        "{scenario}: the bodies move"
    );
    let mut landed: Vec<u8> = one
        .ticks
        .iter()
        .flat_map(|tick| tick.shape.clone())
        .collect();
    landed.sort_unstable();
    landed.dedup();
    println!(
        "{scenario}: {} of {DROPPED} bodies touched the mesh",
        landed.len()
    );
    assert!(
        landed.len() >= LANDED,
        "{scenario}: {} of {DROPPED} bodies touched the mesh",
        landed.len()
    );
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn track_tile_runs_identically_with_1_and_4_workers() {
    assert_runs_agree("track-straight");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn dungeon_corridor_runs_identically_with_1_and_4_workers() {
    assert_runs_agree("corridor-wide-corner");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
#[ignore = "needs scripts/fetch_models.py"]
fn skull_runs_identically_with_1_and_4_workers() {
    assert_runs_agree("skull");
}
