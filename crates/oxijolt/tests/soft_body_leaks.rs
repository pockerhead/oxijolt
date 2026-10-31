//! A leak gate for soft bodies: shared settings are built, a soft body is created from them,
//! stepped, read and written, skinned, removed and the settings dropped, over many rounds
//! against a budget of 200 bytes per round.
//!
//! It measures the private bytes of the process, because Jolt's soft bodies and their settings
//! are allocated by C++, which a Rust global allocator does not see. The file holds exactly one
//! test, so its binary runs alone and no parallel test disturbs the counter. The gate was
//! checked once against a build that never destroyed the soft body creation settings, which
//! exceeded the budget.
#![cfg(windows)]

mod common;

use common::memory::private_bytes;
use common::soft_body::Cloth;
use common::*;
use oxijolt::*;

const WARM_UP_ROUNDS: usize = 500;
const MEASURED_ROUNDS: usize = 5_000;
/// 200 bytes per measured round.
const MAX_GROWTH: usize = 200 * MEASURED_ROUNDS;

fn soft_body_round(world: &mut PhysicsWorld, cloth: &Cloth, round: usize) {
    let shared = cloth.settings();
    let id = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default().position(RVec3::new(0.0, 1.0, 0.0)),
        )
        .unwrap();
    drop(shared);
    let mut body = world.soft_body_mut(id).unwrap();
    let vertices = body.vertices();
    assert_eq!(vertices.len(), cloth.vertices.len());
    body.set_vertex_velocity(5, Vec3::new(0.0, 0.1 * (round % 7) as f32, 0.0))
        .unwrap();
    body.set_vertex_inverse_mass(5, 0.5).unwrap();
    let at = vertices[0].position;
    body.move_kinematic_vertex(0, RVec3::new(at.x, at.y + 0.01, at.z), DT)
        .unwrap();
    world
        .body_mut(id)
        .unwrap()
        .add_force(Vec3::new(1.0, 0.0, 0.0))
        .unwrap();
    assert!(world.step(DT).unwrap().is_complete());
    let mut readout = Vec::new();
    world.soft_body(id).unwrap().vertices_into(&mut readout);
    world.remove_body(id).unwrap();
    skinned_round(world, cloth, round);
}

/// A skinned body made from settings the caller dropped, skinned four times, stepped and
/// removed.
fn skinned_round(world: &mut PhysicsWorld, cloth: &Cloth, round: usize) {
    let shared = cloth.skinned(&[Some(0.0), Some(0.1)]).build().unwrap();
    let id = world
        .create_soft_body(
            &shared,
            &SoftBodySettings::default().position(RVec3::new(0.0, 1.0, 0.0)),
        )
        .unwrap();
    drop(shared);
    let mut body = world.soft_body_mut(id).unwrap();
    for skin in 0..4 {
        let pose = SkeletonPose {
            root_offset: RVec3::new(0.01 * (round % 5) as Real, 1.0, 0.01 * skin as Real),
            joints: vec![JointTransform {
                translation: Vec3::ZERO,
                rotation: Quat::IDENTITY,
            }],
        };
        if skin == 3 {
            body.hard_skin_vertices(&pose).unwrap();
        } else {
            body.skin_vertices(&pose).unwrap();
        }
    }
    assert!(world.step(DT).unwrap().is_complete());
    world.remove_body(id).unwrap();
}

#[test]
fn soft_bodies_do_not_leak() {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    add_floor(&mut world);
    let cloth = Cloth::new(4, 0.2).pin(&[0]);
    for round in 0..WARM_UP_ROUNDS {
        soft_body_round(&mut world, &cloth, round);
    }
    let before = private_bytes();
    for round in 0..MEASURED_ROUNDS {
        soft_body_round(&mut world, &cloth, round);
    }
    let after = private_bytes();
    let growth = after.saturating_sub(before);
    assert!(
        growth < MAX_GROWTH,
        "private bytes grew by {growth} over {MEASURED_ROUNDS} rounds"
    );
}
