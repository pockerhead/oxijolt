//! Determinism of a ragdoll driven from an animation through the skeleton mapper: the humanoid
//! falls from 0.3 m onto a floor under the caller's radial gravity while its motors follow the
//! reverse-mapped pose of an animated rig every tick, and each tick also maps the simulated pose
//! back onto the rig. Bodies and both mapped poses match bit for bit with 1 and 4 worker threads,
//! in one process and in two.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::animation::Rig;
use common::determinism::{
    assert_same, child_request, digest_in_child, finish_child, first_divergence, Digest, Divergence,
};
use common::math::wide;
use common::ragdoll::*;
use common::*;
use oxijolt::*;

const TICKS: usize = 240;
/// The tick from which the changed variant turns one animation joint.
const CHANGE_TICK: usize = 60;

/// The scene: a floor, the humanoid above it and the mapper to the rig.
struct Scene {
    world: PhysicsWorld,
    floor: BodyId,
    ragdoll: RagdollId,
    rig: Rig,
    mapper: SkeletonMapper,
}

impl Scene {
    fn new(threads: u32) -> Self {
        let (mut world, layers) = ragdoll_world(threads);
        let floor = world
            .create_body(
                &Shape::new_box(Vec3::new(20.0, 1.0, 20.0)).unwrap(),
                &BodySettings::new_static()
                    .object_layer(layers.fixed)
                    .position(RVec3::new(0.0, -1.0, 0.0)),
            )
            .unwrap();
        let start = transformed_pose(
            &bind_pose(),
            Quat::IDENTITY,
            [0.0, f64::from(FEET) + 0.3, 0.0],
        );
        let ragdoll = world
            .create_ragdoll(
                &humanoid_settings(layers.ragdoll),
                Some(&start),
                Activation::Activate,
            )
            .unwrap();
        let rig = Rig::humanoid();
        let (ragdoll_skeleton, animation_skeleton) = (skeleton(), rig.skeleton());
        let mapper = SkeletonMapper::new(
            MappedSkeleton {
                skeleton: &ragdoll_skeleton,
                neutral_pose: &bind_pose(),
            },
            MappedSkeleton {
                skeleton: &animation_skeleton,
                neutral_pose: &rig.neutral_pose(),
            },
            TranslationLocks::All,
        )
        .unwrap();
        Self {
            world,
            floor,
            ragdoll,
            rig,
            mapper,
        }
    }

    /// The rig's local pose at `tick`; the changed variant turns the right upper arm 1e-3 rad
    /// further from [`CHANGE_TICK`] on.
    fn animation_local(&self, tick: usize, changed: bool) -> Vec<JointTransform> {
        let mut local = self.rig.animated_local(tick as f32 * DT * 2.0);
        if changed && tick >= CHANGE_TICK {
            let arm = &mut local[self.rig.index("upper_arm_r")];
            arm.rotation = mul(arm.rotation, quat_about(Vec3::new(0.0, 0.0, 1.0), 1.0e-3));
        }
        local
    }

    /// One tick: gravity, motors toward the reverse-mapped animation, a step, and the record of
    /// every body (the state section) and both mapped poses (the shape section, kept apart so
    /// that the control can tell what the simulation did from what the mapper returned).
    fn tick(&mut self, tick: usize, changed: bool, digest: &mut Digest) {
        let local = self.animation_local(tick, changed);
        let animation = self.rig.local_to_model(&local, RVec3::ZERO);
        let target = self.mapper.map_reverse(&animation).unwrap();
        apply_gravity(&mut self.world, self.ragdoll, PLANET_CENTRE);
        self.world
            .ragdoll_mut(self.ragdoll)
            .unwrap()
            .drive_to_pose_using_motors(&target)
            .unwrap();
        step(&mut self.world, 1);

        let record = digest.push();
        record_body(&self.world, self.floor, &mut record.state);
        for &part in self.world.ragdoll(self.ragdoll).unwrap().body_ids() {
            record_body(&self.world, part, &mut record.state);
        }
        let shown = self
            .mapper
            .map(&self.world.ragdoll(self.ragdoll).unwrap().pose(), &local)
            .unwrap();
        record_pose(&target, &mut record.shape);
        record_pose(&shown, &mut record.shape);
    }

    fn floor_contact(&self) -> bool {
        let parts = self
            .world
            .ragdoll(self.ragdoll)
            .unwrap()
            .body_ids()
            .to_vec();
        parts
            .iter()
            .any(|&part| self.world.were_bodies_in_contact(self.floor, part).unwrap())
    }
}

/// Appends every bit of `pose`.
fn record_pose(pose: &SkeletonPose, digest: &mut Vec<u8>) {
    let offset = [pose.root_offset.x, pose.root_offset.y, pose.root_offset.z];
    for value in offset {
        digest.extend_from_slice(&wide(value).to_bits().to_le_bytes());
    }
    for joint in &pose.joints {
        let t: [f32; 3] = joint.translation.into();
        let r: [f32; 4] = joint.rotation.into();
        for value in t.into_iter().chain(r) {
            digest.extend_from_slice(&value.to_bits().to_le_bytes());
        }
    }
}

/// The scene run for [`TICKS`] ticks with `threads` workers, checking that it plays as
/// described: clear of the floor after the first tick, on it later, motors on, and an
/// animation that moves.
fn run(threads: u32, changed: bool) -> Digest {
    let mut scene = Scene::new(threads);
    let mut digest = Digest::new();
    let mut touched = None;
    for tick in 0..TICKS {
        scene.tick(tick, changed, &mut digest);
        if tick == 0 {
            assert!(!scene.floor_contact(), "the humanoid starts on the floor");
        }
        if touched.is_none() && scene.floor_contact() {
            touched = Some(tick);
        }
    }
    assert!(touched.is_some(), "the humanoid never reached the floor");
    let ragdoll = scene.world.ragdoll(scene.ragdoll).unwrap();
    assert_eq!(ragdoll.joint_motors_on(HEAD as u32), Some(true));
    let first = scene.animation_local(0, changed);
    let last = scene.animation_local(TICKS - 1, changed);
    assert_ne!(first, last, "the animation does not move");
    digest
}

#[test]
#[ignore = "child process of the skeleton mapper determinism gate"]
fn skeleton_mapper_determinism_child() {
    let Some((scenario, threads, variant)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "mapper");
    finish_child(&run(threads, variant == "changed"));
}

#[test]
fn skeleton_mapper_match_with_1_and_4_workers() {
    let one = run(1, false);
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in one process", &one, &run(4, false));
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn skeleton_mapper_match_across_processes() {
    let one = digest_in_child("skeleton_mapper_determinism_child", "mapper", 1, "");
    let four = digest_in_child("skeleton_mapper_determinism_child", "mapper", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &run(1, false), &one);
}

#[test]
fn a_changed_mapped_input_changes_the_digest() {
    let (unchanged, changed) = (run(1, false), run(1, true));
    match first_divergence(&unchanged, &changed) {
        Some(Divergence::Tick { tick, .. }) => assert_eq!(tick, CHANGE_TICK),
        other => panic!("the changed animation left the digest {other:?}"),
    }
    // The bodies change in the same tick: the motors follow the changed mapped pose.
    let first_body_change = unchanged
        .ticks
        .iter()
        .zip(&changed.ticks)
        .position(|(a, b)| a.state != b.state);
    assert_eq!(first_body_change, Some(CHANGE_TICK));
}
