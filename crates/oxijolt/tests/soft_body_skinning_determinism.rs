//! Determinism of skinned soft bodies: a cloth skinned to two animated joints, with attached,
//! limited and back-stopped rows, falls onto a box and runs bit for bit the same with 1 and 4
//! worker threads, in one process and in several, with caller job systems and with Jolt's
//! single-threaded job system.

mod common;

use common::determinism::{
    assert_caller_job_systems_agree, assert_same, assert_single_threaded_agrees, child_request,
    digest_in_child, finish_child, Digest,
};
use common::math::{add, v3, V3};
use common::ragdoll::pose_from;
use common::soft_body::{cloth_attributes, Cloth};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const TICKS: usize = 240;
/// The tick of the one hard skin.
const HARD_SKIN_TICK: usize = 120;
const CHILD_TEST: &str = "soft_body_skinning_child";
const SCENARIO: &str = "skinning";

const SIDE: u32 = 24;
const SPACING: f32 = 0.1;
/// Where the cloth's vertex frame, and the two joints' frame, is in the world.
const CLOTH_POSITION: RVec3 = RVec3::new(0.0, 2.0, 0.0);
/// The box under the cloth: centre and half extent.
const BOX_CENTRE: RVec3 = RVec3::new(0.0, 0.9, 0.3);
const BOX_HALF_EXTENT: Vec3 = Vec3::new(0.4, 0.2, 0.4);

/// The bind position of joint `joint` (0 left, 1 right) in the cloth's frame: above both ends
/// of the attached row.
fn bind_position(joint: u32) -> Vec3 {
    let half = 0.5 * SPACING * (SIDE - 1) as f32;
    Vec3::new(if joint == 0 { -half } else { half }, 0.0, -half)
}

/// Row 0 attached, rows 1 and 2 within 0.3 m with back stops, the rest free, each skinned
/// vertex weighted between the two joints by its column.
fn shared_settings() -> SoftBodySharedSettings {
    let cloth = Cloth::new(SIDE, SPACING);
    let mut builder = cloth
        .builder()
        .create_constraints(SoftBodyBendType::Dihedral, cloth_attributes())
        .inv_binds(
            (0..2).map(|joint| SoftBodyInvBind::new(joint, bind_position(joint), Quat::IDENTITY)),
        );
    for z in 0..3 {
        for x in 0..SIDE {
            let right = x as f32 / (SIDE - 1) as f32;
            let mut weights = [SoftBodySkinWeight::default(); 4];
            weights[0] = SoftBodySkinWeight::new(0, 1.0 - right);
            weights[1] = SoftBodySkinWeight::new(1, right);
            let mut skinned = SoftBodySkinned::new(cloth.index(x, z), weights);
            if z == 0 {
                skinned.max_distance = Some(0.0);
            } else {
                skinned.max_distance = Some(0.3);
                skinned.back_stop = Some(SoftBodyBackStop {
                    distance: 0.05,
                    radius: 1.0,
                });
            }
            builder = builder.skinned(skinned);
        }
    }
    builder.build().unwrap()
}

/// The two joints at `tick`, swaying and turning; `phase` shifts the animation.
fn joints(tick: usize, phase: f64) -> SkeletonPose {
    let t = 2.0 * std::f64::consts::PI * tick as f64 / 90.0 + phase;
    let world: Vec<(V3, Quat)> = (0..2)
        .map(|joint| {
            let side = if joint == 0 { 1.0 } else { -1.0 };
            let bind = bind_position(joint);
            let at = add(
                v3(CLOTH_POSITION),
                [f64::from(bind.x), f64::from(bind.y), f64::from(bind.z)],
            );
            let sway = [0.2 * t.sin(), 0.1 * (2.0 * t).sin(), 0.15 * side * t.cos()];
            let turn = quat_about(Vec3::new(0.0, 1.0, 0.0), 0.2 * t.sin() as f32);
            (add(at, sway), turn)
        })
        .collect();
    pose_from(&world)
}

struct Scene {
    world: PhysicsWorld,
    cloth: BodyId,
    obstacle: BodyId,
    phase: f64,
}

impl Scene {
    fn new(threads: u32, phase: f64) -> Self {
        let mut world = world(GRAVITY, threads);
        let obstacle = world
            .create_body(
                &Shape::new_box(BOX_HALF_EXTENT).unwrap(),
                &BodySettings::new_static().position(BOX_CENTRE),
            )
            .unwrap();
        let cloth = world
            .create_soft_body(
                &shared_settings(),
                &SoftBodySettings::default().position(CLOTH_POSITION),
            )
            .unwrap();
        Self {
            world,
            cloth,
            obstacle,
            phase,
        }
    }

    fn tick(&mut self, tick: usize) {
        let pose = joints(tick, self.phase);
        let mut cloth = self.world.soft_body_mut(self.cloth).unwrap();
        if tick == HARD_SKIN_TICK {
            cloth.hard_skin_vertices(&pose).unwrap();
        } else {
            cloth.skin_vertices(&pose).unwrap();
        }
        step(&mut self.world, 1);
    }

    /// The cloth body and every vertex position and velocity, as bits.
    fn record(&self, digest: &mut Digest) {
        let tick = digest.push();
        for id in [self.obstacle, self.cloth] {
            record_body(&self.world, id, &mut tick.state);
        }
        for vertex in self.world.soft_body(self.cloth).unwrap().vertices() {
            let position: [Real; 3] = vertex.position.into();
            for value in position {
                tick.state.extend_from_slice(&value.to_bits().to_le_bytes());
            }
            let velocity: [f32; 3] = vertex.velocity.into();
            for value in velocity {
                tick.state.extend_from_slice(&value.to_bits().to_le_bytes());
            }
        }
    }

    /// The height above the box's top face of every vertex over that face, 5 cm in from its
    /// edges.
    fn heights_over_the_box(&self) -> Vec<f64> {
        let centre = v3(BOX_CENTRE);
        let inner = [BOX_HALF_EXTENT.x, BOX_HALF_EXTENT.z].map(|half| f64::from(half) - 0.05);
        let top = centre[1] + f64::from(BOX_HALF_EXTENT.y);
        self.world
            .soft_body(self.cloth)
            .unwrap()
            .vertices()
            .iter()
            .map(|v| v3(v.position))
            .filter(|p| (p[0] - centre[0]).abs() < inner[0] && (p[2] - centre[2]).abs() < inner[1])
            .map(|p| p[1] - top)
            .collect()
    }
}

/// The variant's animation phase: "forward" or "shifted".
fn phase(variant: &str) -> f64 {
    match variant {
        "forward" => 0.0,
        "shifted" => 0.7,
        other => panic!("unknown variant {other}"),
    }
}

/// The scene run for [`TICKS`] ticks with `threads` workers; whether a vertex landed on the
/// box's top face, and whether one was ever more than 5 cm inside the box.
fn skinning(threads: u32, variant: &str) -> (Digest, bool, bool) {
    let mut scene = Scene::new(threads, phase(variant));
    let mut digest = Digest::new();
    let (mut landed, mut inside) = (false, false);
    let depth = 2.0 * f64::from(BOX_HALF_EXTENT.y);
    for tick in 1..=TICKS {
        scene.tick(tick);
        scene.record(&mut digest);
        for height in scene.heights_over_the_box() {
            landed |= height.abs() < 0.05;
            inside |= height < -0.05 && height > 0.05 - depth;
        }
    }
    (digest, landed, inside)
}

#[test]
#[ignore = "child process of the soft body skinning determinism gate"]
fn soft_body_skinning_child() {
    let Some((scenario, threads, variant)) = child_request() else {
        return;
    };
    assert_eq!(scenario, SCENARIO);
    finish_child(&skinning(threads, &variant).0);
}

#[test]
fn skinning_matches_with_1_and_4_workers() {
    let (one, landed, inside) = skinning(1, "forward");
    assert_eq!(one.ticks.len(), TICKS);
    // The cloth hits the box and does not pass through it.
    assert!(landed && !inside, "landed {landed}, inside {inside}");
    assert_same(
        "1 vs 4 workers in one process",
        &one,
        &skinning(4, "forward").0,
    );
    // Another animation gives another run.
    assert_ne!(one, skinning(1, "shifted").0);
}

#[test]
fn skinning_matches_across_processes() {
    let one = digest_in_child(CHILD_TEST, SCENARIO, 1, "forward");
    let four = digest_in_child(CHILD_TEST, SCENARIO, 4, "forward");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &skinning(1, "forward").0, &one);
}

#[test]
fn skinning_matches_with_caller_job_systems() {
    assert_caller_job_systems_agree(CHILD_TEST, SCENARIO, "forward");
}

#[test]
fn skinning_matches_with_the_single_threaded_job_system() {
    assert_single_threaded_agrees(CHILD_TEST, SCENARIO, "forward");
}
