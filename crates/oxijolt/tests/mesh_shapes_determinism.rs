//! Determinism of the hull, mesh, scaled and tapered shapes: a scene of them runs bit for bit
//! the same with 1 and 4 worker threads in separate processes, and a rollback after a divergent
//! detour replays the straight run bit for bit.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::meshes::*;
use common::*;
use oxijolt::*;

/// Ticks of the straight run.
const TICKS: usize = 300;
/// Tick after which the rollback saves.
const SAVE_TICK: usize = 100;
/// Ticks of the detour between the two restores.
const DETOUR_TICKS: usize = 50;

/// The bumpy ground, a kinematic platform on a velocity schedule and 30 dynamic bodies of the
/// new shape kinds, built in a fixed order.
struct Scene {
    world: PhysicsWorld,
    platform: BodyId,
    /// Every body in creation order.
    bodies: Vec<BodyId>,
}

/// The platform's velocity at `tick`: a slow circle, so it keeps moving under its riders.
fn platform_velocity(tick: usize, detour: bool) -> Vec3 {
    let angle = tick as f32 * 0.02;
    let speed = if detour { -0.8 } else { 0.5 };
    Vec3::new(speed * angle.cos(), 0.0, speed * angle.sin())
}

impl Scene {
    fn new(threads: u32) -> Self {
        let mut world = world(Vec3::new(0.0, -9.81, 0.0), threads);
        let materials = [
            PhysicsMaterial::new(1).unwrap(),
            PhysicsMaterial::new(2).unwrap(),
        ];
        let (vertices, triangles) = grid(24, 1.0, |x, z| 0.3 * (0.4 * x).sin() * (0.3 * z).cos());
        let indices: Vec<u8> = (0..triangles.len()).map(|i| (i / 2 % 2) as u8).collect();
        let list = [&materials[0], &materials[1]];
        let (ground_shape, _) = Shape::new_mesh_with_settings(
            &vertices,
            &triangles,
            &MeshSettings::default().materials(&list, &indices),
        )
        .unwrap();
        let mut bodies = vec![world
            .create_body(&ground_shape, &BodySettings::new_static())
            .unwrap()];
        let platform = world
            .create_body(
                &flat_grid(3, 1.0),
                &BodySettings::new_kinematic()
                    .mass(100.0)
                    .position(RVec3::new(6.0, 1.0, 6.0)),
            )
            .unwrap();
        bodies.push(platform);
        for (i, shape) in dynamic_shapes().iter().enumerate() {
            let (column, row) = ((i % 6) as Real, (i / 6) as Real);
            let position = RVec3::new(-7.5 + 3.0 * column, 2.5 + 0.5 * row, -6.0 + 3.0 * row);
            let rotation = quat_about(Vec3::new(0.6, 0.0, 0.8), 0.3 * i as f32);
            let settings = BodySettings::new_dynamic()
                .position(position)
                .rotation(rotation);
            bodies.push(world.create_body(shape, &settings).unwrap());
        }
        Self {
            world,
            platform,
            bodies,
        }
    }

    fn tick(&mut self, tick: usize, detour: bool) {
        self.world
            .body_mut(self.platform)
            .unwrap()
            .set_linear_velocity(platform_velocity(tick, detour))
            .unwrap();
        step(&mut self.world, 1);
    }

    fn record(&self, digest: &mut Digest) {
        let tick = digest.push();
        for &id in &self.bodies {
            record_body(&self.world, id, &mut tick.state);
        }
    }

    /// The shape facts the scene starts from: each moving body's mass and a down-ray's hit
    /// distance over each body, as bits.
    fn shape_facts(&self) -> Vec<u8> {
        let mut bytes = Vec::new();
        for &id in &self.bodies {
            let body = self.world.body(id).unwrap();
            bytes.extend(body.mass().unwrap_or(0.0).to_bits().to_le_bytes());
            let p = body.position();
            let ray = RayCast::new(RVec3::new(p.x, p.y + 5.0, p.z), Vec3::new(0.0, -10.0, 0.0));
            let hit = self.world.cast_ray(&ray, &QueryFilter::new()).unwrap();
            bytes.extend(hit.map_or(-1.0, |hit| hit.distance).to_bits().to_le_bytes());
        }
        bytes
    }
}

/// 30 shapes: irregular hulls, scaled hulls, scaled boxes, a compound with a scaled hull child,
/// tapered capsules and cones, five of each.
fn dynamic_shapes() -> Vec<Shape> {
    let hull = Shape::new_convex_hull_with_convex_radius(&irregular_points(), 0.05).unwrap();
    let block = Shape::new_box(Vec3::new(0.4, 0.4, 0.4)).unwrap();
    let mut shapes = Vec::new();
    for i in 0..5 {
        let k = i as f32;
        let stretch = Vec3::new(0.6 + 0.1 * k, 0.5, 0.8 - 0.05 * k);
        let scaled_hull = Shape::new_scaled(&hull, stretch).unwrap();
        let compound = Shape::new_compound(&[
            CompoundChild {
                shape: &scaled_hull,
                position: Vec3::ZERO,
                rotation: Quat::IDENTITY,
                user_data: 0,
            },
            CompoundChild {
                shape: &block,
                position: Vec3::new(0.0, 0.0, 0.9),
                rotation: Quat::IDENTITY,
                user_data: 1,
            },
        ])
        .unwrap();
        shapes.push(
            Shape::new_convex_hull_with_convex_radius(&irregular_points(), 0.02 * k).unwrap(),
        );
        shapes.push(Shape::new_scaled(&hull, stretch).unwrap());
        shapes.push(Shape::new_scaled(&block, Vec3::new(1.0, 0.5 + 0.2 * k, 1.5)).unwrap());
        shapes.push(compound);
        shapes.push(Shape::new_tapered_capsule(0.3, 0.1 + 0.02 * k, 0.25).unwrap());
        shapes.push(
            Shape::new_tapered_cylinder_with_convex_radius(0.4, 0.0, 0.3 + 0.02 * k, 0.02).unwrap(),
        );
    }
    shapes
}

/// The straight run: ticks `1..=TICKS`, recorded after every step, with the shape facts in
/// tick 1's shape section.
fn straight(threads: u32) -> Digest {
    let mut scene = Scene::new(threads);
    let facts = scene.shape_facts();
    let mut digest = Digest::new();
    for tick in 1..=TICKS {
        scene.tick(tick, false);
        scene.record(&mut digest);
    }
    digest.ticks[0].shape = facts;
    digest
}

/// The rollback: save after [`SAVE_TICK`], a detour (other platform velocities, two hulls
/// teleported, every body woken), a restore, and a replay of ticks `SAVE_TICK + 1..=TICKS`.
fn rollback(threads: u32) -> Digest {
    let mut scene = Scene::new(threads);
    for tick in 1..=SAVE_TICK {
        scene.tick(tick, false);
    }
    let saved = scene.world.save_state();
    for (n, &id) in scene.bodies[2..4].iter().enumerate() {
        scene
            .world
            .body_mut(id)
            .unwrap()
            .set_position(RVec3::new(n as Real, 6.0, 0.0), Activation::Activate)
            .unwrap();
    }
    for &id in &scene.bodies[2..] {
        scene
            .world
            .body_mut(id)
            .unwrap()
            .set_linear_velocity(Vec3::new(0.0, 1.0, 0.0))
            .unwrap();
    }
    for tick in 1..=DETOUR_TICKS {
        scene.tick(tick, true);
    }
    scene.world.restore_state(&saved).unwrap();
    let mut digest = Digest::new();
    for tick in SAVE_TICK + 1..=TICKS {
        scene.tick(tick, false);
        scene.record(&mut digest);
    }
    digest
}

#[test]
#[ignore = "child process of the mesh shape determinism gates"]
fn mesh_shapes_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    let digest = match scenario.as_str() {
        "straight" => straight(threads),
        "rollback" => rollback(threads),
        other => panic!("unknown scenario {other}"),
    };
    finish_child(&digest);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn new_shapes_run_identically_with_1_and_4_workers() {
    let one = digest_in_child("mesh_shapes_child", "straight", 1, "");
    let four = digest_in_child("mesh_shapes_child", "straight", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers", &one, &four);
    // The scene moves: the bodies fall and the platform turns.
    assert_ne!(one.ticks[0].state, one.ticks[TICKS - 1].state);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn rollback_after_a_detour_replays_the_straight_run() {
    let straight = digest_in_child("mesh_shapes_child", "straight", 1, "");
    let replay = digest_in_child("mesh_shapes_child", "rollback", 4, "");
    let tail = Digest {
        ticks: straight.ticks[SAVE_TICK..]
            .iter()
            .map(|tick| common::determinism::TickDigest {
                shape: Vec::new(),
                state: tick.state.clone(),
            })
            .collect(),
    };
    assert_eq!(replay.ticks.len(), TICKS - SAVE_TICK);
    assert_same("straight run vs replay after a detour", &tail, &replay);
}
