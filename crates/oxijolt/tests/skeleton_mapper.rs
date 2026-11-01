//! The skeleton mapper between the 12-part humanoid ragdoll and a 22-joint animation rig: both
//! directions against Jolt's stages (direct joints, chains, unmapped joints, translation locks),
//! validation of skeletons, poses and chains, frames, and mapped poses driving a ragdoll.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use std::thread;

use common::animation::{Rig, Rigid};
use common::math::{dot, norm, sub, wide};
use common::ragdoll::*;
use common::*;
use oxijolt::*;

/// A skewed unit axis.
const AXIS: Vec3 = Vec3::new(0.48, 0.6, 0.64);
/// A unit quaternion within Rust's tolerance and outside Jolt's (see the mapper's unit tests).
const EDGE_ROTATION: Quat = Quat::from_xyzw(0.641_428_5, -0.449_664_26, 0.383_378_86, -0.489_287_4);
/// Ragdoll joints without a chain or a lock: their animation joints are direct mappings.
const DIRECT_ONLY: [usize; 10] = [
    ABDOMEN,
    HEAD,
    UPPER_ARM_L,
    FOREARM_L,
    UPPER_ARM_R,
    FOREARM_R,
    THIGH_L,
    SHIN_L,
    THIGH_R,
    SHIN_R,
];

fn mapper_with(
    ragdoll_neutral: &SkeletonPose,
    rig: &Rig,
    animation_neutral: &SkeletonPose,
    locks: TranslationLocks<'_>,
) -> Result<SkeletonMapper, RagdollError> {
    let (ragdoll, animation) = (skeleton(), rig.skeleton());
    SkeletonMapper::new(
        MappedSkeleton {
            skeleton: &ragdoll,
            neutral_pose: ragdoll_neutral,
        },
        MappedSkeleton {
            skeleton: &animation,
            neutral_pose: animation_neutral,
        },
        locks,
    )
}

fn mapper_for(rig: &Rig, locks: TranslationLocks<'_>) -> SkeletonMapper {
    mapper_with(&bind_pose(), rig, &rig.neutral_pose(), locks).unwrap()
}

/// The bind pose with each part turned and shifted a little, then the whole pose turned and
/// moved.
fn posed_ragdoll() -> SkeletonPose {
    let mut pose = bind_pose();
    for (part, joint) in pose.joints.iter_mut().enumerate() {
        joint.rotation = mul(quat_about(AXIS, 0.05 * part as f32), joint.rotation);
        joint.translation.z += 0.01 * part as f32;
    }
    transformed_pose(&pose, quat_about(AXIS, 0.6), [1.0, 0.5, -2.0])
}

/// Where Jolt's direct mapping puts the animation joint `joint` of ragdoll `part`:
/// `pose[part] * inv(bind[part]) * neutral[joint]`.
fn direct(pose: &SkeletonPose, rig: &Rig, part: usize, joint: usize) -> Rigid {
    Rigid::at(pose, part)
        .then(Rigid::at(&bind_pose(), part).inverse())
        .then(Rigid::at(&rig.neutral_pose(), joint))
}

#[track_caller]
fn assert_near(actual: Rigid, expected: Rigid, tolerance: f64) {
    let (distance, angle) = actual.difference(expected);
    assert!(
        distance <= tolerance && angle <= tolerance,
        "{distance} m, {angle} rad apart: {actual:?} vs {expected:?}"
    );
}

/// The cosine of the angle between two directions.
fn cos(a: [f64; 3], b: [f64; 3]) -> f64 {
    dot(a, b) / (norm(a) * norm(b))
}

/// Every bit of a pose: root offset, translations and rotations.
fn bits(pose: &SkeletonPose) -> Vec<u64> {
    let offset = [pose.root_offset.x, pose.root_offset.y, pose.root_offset.z];
    let mut bits: Vec<u64> = offset.iter().map(|&c| wide(c).to_bits()).collect();
    for joint in &pose.joints {
        let t = joint.translation;
        let r = joint.rotation;
        bits.extend([t.x, t.y, t.z, r.x, r.y, r.z, r.w].map(|c| u64::from(c.to_bits())));
    }
    bits
}

#[track_caller]
fn assert_invalid<T>(result: Result<T, RagdollError>) {
    assert!(
        matches!(result, Err(RagdollError::InvalidValue(_))),
        "accepted"
    );
}

#[test]
fn neutral_poses_map_onto_each_other() {
    let rig = Rig::humanoid();
    let mapper = mapper_for(&rig, TranslationLocks::None);
    let neutral = rig.neutral_pose();
    let mapped = mapper.map(&bind_pose(), &rig.neutral_local()).unwrap();
    assert_eq!(mapped.joints.len(), rig.len());
    for joint in 0..rig.len() {
        assert_near(Rigid::at(&mapped, joint), Rigid::at(&neutral, joint), 1e-5);
    }
    let reversed = mapper.map_reverse(&neutral).unwrap();
    for part in 0..PART_COUNT {
        assert_near(
            Rigid::at(&reversed, part),
            Rigid::at(&bind_pose(), part),
            1e-5,
        );
    }
}

#[test]
fn direct_joints_follow_the_ragdoll() {
    let rig = Rig::humanoid();
    let mapper = mapper_for(&rig, TranslationLocks::None);
    let pose = posed_ragdoll();
    let mapped = mapper.map(&pose, &rig.animated_local(0.4)).unwrap();
    assert_eq!(mapped.root_offset, pose.root_offset);
    for (part, spec) in PARTS.iter().enumerate() {
        let joint = rig.index(spec.name);
        assert_eq!(mapper.mapped_joint(part as u32), Some(joint as u32));
    }
    for part in DIRECT_ONLY {
        let joint = rig.index(PARTS[part].name);
        assert_near(
            Rigid::at(&mapped, joint),
            direct(&pose, &rig, part, joint),
            1e-5,
        );
    }
}

#[test]
fn a_chain_start_turns_toward_the_next_ragdoll_joint() {
    let rig = Rig::humanoid();
    let mapper = mapper_for(&rig, TranslationLocks::None);
    let mut pose = posed_ragdoll();
    pose.joints[HEAD].translation.x += 0.1;
    pose.joints[HEAD].translation.z += 0.05;
    let local = rig.animated_local(0.9);
    let mapped = mapper.map(&pose, &local).unwrap();
    let (chest, neck, head) = (rig.index("chest"), rig.index("neck"), rig.index("head"));

    let start = Rigid::at(&mapped, chest);
    let direct_start = direct(&pose, &rig, CHEST, chest);
    assert!(
        start.difference(direct_start).0 < 1e-5,
        "the chain start moved"
    );
    assert!(start.difference(direct_start).1 > 1e-3, "no correction");
    let end = start
        .then(Rigid::of(local[neck], RVec3::ZERO))
        .then(Rigid::of(local[head], RVec3::ZERO));
    let actual = sub(end.translation, start.translation);
    let desired = sub(
        Rigid::at(&pose, HEAD).translation,
        Rigid::at(&pose, CHEST).translation,
    );
    assert!(
        cos(actual, desired) > 1.0 - 1e-6,
        "{actual:?} vs {desired:?}"
    );
    assert_near(
        Rigid::at(&mapped, neck),
        start.then(Rigid::of(local[neck], RVec3::ZERO)),
        1e-5,
    );
}

#[test]
fn the_longest_chain_wins_and_ties_go_to_the_first_joint() {
    // The neck and each clavicle are one joint between the chest and its children: the head,
    // the lowest ragdoll joint, gets the chain and the clavicles stay unmapped.
    let rig = Rig::humanoid();
    let mapper = mapper_for(&rig, TranslationLocks::None);
    let mut pose = posed_ragdoll();
    pose.joints[UPPER_ARM_L].translation.y += 0.05;
    let local = rig.animated_local(0.2);
    let mapped = mapper.map(&pose, &local).unwrap();
    let chest = Rigid::at(&mapped, rig.index("chest"));
    let clavicle = rig.index("clavicle_l");
    assert_near(
        Rigid::at(&mapped, clavicle),
        chest.then(Rigid::of(local[clavicle], RVec3::ZERO)),
        1e-5,
    );

    // A shoulder makes the arm path longer: the chest now turns toward the upper arm and the
    // neck is unmapped.
    let rig = Rig::with_long_left_arm();
    let mapper = mapper_for(&rig, TranslationLocks::None);
    let local = rig.animated_local(0.2);
    let mapped = mapper.map(&pose, &local).unwrap();
    let chest = Rigid::at(&mapped, rig.index("chest"));
    let end = ["clavicle_l", "shoulder_l", "upper_arm_l"]
        .iter()
        .fold(chest, |model, name| {
            model.then(Rigid::of(local[rig.index(name)], RVec3::ZERO))
        });
    let desired = sub(
        Rigid::at(&pose, UPPER_ARM_L).translation,
        Rigid::at(&pose, CHEST).translation,
    );
    assert!(cos(sub(end.translation, chest.translation), desired) > 1.0 - 1e-6);
    let neck = rig.index("neck");
    assert_near(
        Rigid::at(&mapped, neck),
        chest.then(Rigid::of(local[neck], RVec3::ZERO)),
        1e-5,
    );
    let head = rig.index("head");
    assert_near(
        Rigid::at(&mapped, head),
        direct(&pose, &rig, HEAD, head),
        1e-5,
    );
}

#[test]
fn unmapped_joints_follow_their_parents() {
    let rig = Rig::humanoid();
    let mapper = mapper_for(&rig, TranslationLocks::None);
    let pose = posed_ragdoll();
    let local = rig.animated_local(0.7);
    let mapped = mapper.map(&pose, &local).unwrap();
    for (joint, parent) in [
        ("hand_l", "forearm_l"),
        ("foot_r", "shin_r"),
        ("head_end", "head"),
        ("clavicle_r", "chest"),
    ] {
        let (joint, parent) = (rig.index(joint), rig.index(parent));
        assert_near(
            Rigid::at(&mapped, joint),
            Rigid::at(&mapped, parent).then(Rigid::of(local[joint], RVec3::ZERO)),
            1e-5,
        );
    }
    // The extra root is its local transform relative to the pose's root offset.
    let root = rig.index("root");
    assert_near(
        Rigid::at(&mapped, root),
        Rigid::of(local[root], pose.root_offset),
        1e-5,
    );
}

#[test]
fn reverse_mapping_uses_direct_joints_only() {
    let rig = Rig::humanoid();
    let mapper = mapper_for(&rig, TranslationLocks::None);
    let animation = rig.local_to_model(&rig.animated_local(0.3), RVec3::new(1.0, 0.0, 2.0));
    let reversed = mapper.map_reverse(&animation).unwrap();
    assert_eq!(reversed.root_offset, animation.root_offset);

    let change = |name: &str| {
        let mut changed = animation.clone();
        let joint = &mut changed.joints[rig.index(name)];
        joint.rotation = mul(quat_about(AXIS, 0.3), joint.rotation);
        joint.translation.x += 0.1;
        mapper.map_reverse(&changed).unwrap()
    };
    for name in ["spine", "neck", "clavicle_l", "hand_l", "root"] {
        assert_eq!(bits(&change(name)), bits(&reversed), "{name}");
    }
    let thigh = change("thigh_l");
    for part in 0..PART_COUNT {
        assert_eq!(
            thigh.joints[part] == reversed.joints[part],
            part != THIGH_L,
            "part {part}"
        );
    }
}

/// The bind pose with the left forearm 5 cm further out along the arm.
fn stretched_ragdoll() -> SkeletonPose {
    let mut pose = bind_pose();
    pose.joints[FOREARM_L].translation.x += 0.05;
    pose
}

#[test]
fn all_translation_locks_keep_neutral_offsets() {
    let rig = Rig::humanoid();
    let (pelvis, upper_arm, forearm) = (
        rig.index("pelvis"),
        rig.index("upper_arm_l"),
        rig.index("forearm_l"),
    );
    let neutral = rig.neutral_pose();
    let neutral_offset = |joint: usize, parent: usize| {
        Rigid::at(&neutral, parent)
            .inverse()
            .then(Rigid::at(&neutral, joint))
            .translation
    };
    let local = rig.neutral_local();
    let parents = rig.parents();

    let locked = mapper_for(&rig, TranslationLocks::All);
    for joint in 0..rig.len() {
        // Every descendant of the joint the ragdoll root maps to.
        let expected = joint > pelvis;
        assert_eq!(
            locked.is_translation_locked(joint as u32),
            expected,
            "{joint}"
        );
    }
    let mapped = locked.map(&stretched_ragdoll(), &local).unwrap();
    for (joint, parent) in parents.iter().enumerate().skip(pelvis + 1) {
        let parent = parent.unwrap() as usize;
        let parent_model = Rigid::at(&mapped, parent);
        let expected = common::math::add(
            parent_model.translation,
            parent_model.apply_vector(neutral_offset(joint, parent)),
        );
        let at = Rigid::at(&mapped, joint).translation;
        assert!(norm(sub(at, expected)) < 1e-5, "joint {joint}");
    }

    // Without locks the stretch shows.
    let free = mapper_for(&rig, TranslationLocks::None);
    let mapped = free.map(&stretched_ragdoll(), &local).unwrap();
    let length = |pose: &SkeletonPose| {
        norm(sub(
            Rigid::at(pose, forearm).translation,
            Rigid::at(pose, upper_arm).translation,
        ))
    };
    assert!((length(&mapped) - length(&neutral) - 0.05).abs() < 1e-5);
}

#[test]
fn selected_translation_locks() {
    let rig = Rig::humanoid();
    let (forearm, hand) = (rig.index("forearm_l"), rig.index("hand_l"));
    let locks = [forearm as u32];
    let mapper = mapper_for(&rig, TranslationLocks::Joints(&locks));
    for joint in 0..rig.len() {
        assert_eq!(mapper.is_translation_locked(joint as u32), joint == forearm);
    }
    let pose = stretched_ragdoll();
    let local = rig.neutral_local();
    let mapped = mapper.map(&pose, &local).unwrap();
    let neutral = rig.neutral_pose();
    let upper_arm = rig.index("upper_arm_l");
    let offset = Rigid::at(&neutral, upper_arm)
        .inverse()
        .then(Rigid::at(&neutral, forearm));
    assert!(
        Rigid::at(&mapped, forearm)
            .difference(Rigid::at(&mapped, upper_arm).then(offset))
            .0
            < 1e-5
    );
    // Locks apply after unmapped joints: the hand follows the forearm as mapped, before its
    // lock.
    assert_near(
        Rigid::at(&mapped, hand),
        direct(&pose, &rig, FOREARM_L, forearm).then(Rigid::of(local[hand], RVec3::ZERO)),
        1e-5,
    );

    let root = [rig.index("root") as u32];
    let mapped_root = [rig.index("pelvis") as u32];
    let missing = [rig.len() as u32];
    for refused in [&root[..], &mapped_root, &missing] {
        assert_invalid(mapper_with(
            &bind_pose(),
            &rig,
            &neutral,
            TranslationLocks::Joints(refused),
        ));
    }
}

#[test]
fn mappings_are_validated() {
    let rig = Rig::humanoid();
    let renamed = |from: &str, to: &str| {
        let mut rig = rig.clone();
        for joint in &mut rig.joints {
            if joint.name == from {
                joint.name = to.to_owned();
            }
            if joint.parent.as_deref() == Some(from) {
                joint.parent = Some(to.to_owned());
            }
        }
        rig
    };
    let build = |rig: &Rig| {
        mapper_with(
            &bind_pose(),
            rig,
            &rig.neutral_pose(),
            TranslationLocks::None,
        )
        .err()
    };
    assert_eq!(build(&rig), None);
    assert_eq!(
        build(&renamed("head", "skull")),
        Some(RagdollError::UnmappedJoint(HEAD as u32))
    );
    let mut arm_on_hips = rig.clone();
    arm_on_hips.joints[rig.index("upper_arm_l")].parent = Some("pelvis".to_owned());
    assert_eq!(
        build(&arm_on_hips),
        Some(RagdollError::HierarchyMismatch(UPPER_ARM_L as u32))
    );
    // The extra root renamed to "abdomen" (and the abdomen to "belly"): the pelvis is below a
    // mapped joint.
    let upside_down = {
        let mut rig = renamed("abdomen", "belly");
        rig.joints[0].name = "abdomen".to_owned();
        let pelvis = rig.index("pelvis");
        rig.joints[pelvis].parent = Some("abdomen".to_owned());
        rig
    };
    assert_eq!(
        build(&upside_down),
        Some(RagdollError::HierarchyMismatch(PELVIS as u32))
    );
    // An animation skeleton with fewer joints than the ragdoll's.
    let mut small = rig.clone();
    small.joints.truncate(4);
    assert_eq!(
        build(&small),
        Some(RagdollError::UnmappedJoint(CHEST as u32))
    );

    // Neutral poses: counts, NaN, non-unit rotations, positions outside the frame.
    let neutral = rig.neutral_pose();
    let with = |ragdoll: &SkeletonPose, animation: &SkeletonPose| {
        mapper_with(ragdoll, &rig, animation, TranslationLocks::None)
    };
    let mut short = bind_pose();
    short.joints.pop();
    assert_invalid(with(&short, &neutral));
    let mut short = neutral.clone();
    short.joints.pop();
    assert_invalid(with(&bind_pose(), &short));
    let mut nan = neutral.clone();
    nan.joints[5].translation.y = f32::NAN;
    assert_invalid(with(&bind_pose(), &nan));
    let mut not_unit = bind_pose();
    not_unit.joints[HEAD].rotation = Quat::from_xyzw(0.0, 0.1, 0.0, 1.0);
    assert_invalid(with(&not_unit, &neutral));
    let mut far = neutral.clone();
    far.root_offset.x = limits::MAX_POSITION;
    assert_invalid(with(&bind_pose(), &far));
}

#[test]
fn poses_are_validated_per_call() {
    let rig = Rig::humanoid();
    let mapper = mapper_for(&rig, TranslationLocks::None);
    let local = rig.neutral_local();
    let mut short = bind_pose();
    short.joints.pop();
    assert_invalid(mapper.map(&short, &local));
    assert_invalid(mapper.map(&bind_pose(), &local[1..]));
    let mut nan = bind_pose();
    nan.joints[THIGH_R].translation.z = f32::NAN;
    assert_invalid(mapper.map(&nan, &local));
    let mut not_unit = local.clone();
    not_unit[rig.index("spine")].rotation = Quat::from_xyzw(0.0, 0.0, 0.0, 0.9);
    assert_invalid(mapper.map(&bind_pose(), &not_unit));
    let beyond = wide(limits::MAX_POSITION) as f32 * 1.01;
    let mut far = local.clone();
    far[rig.index("hand_l")].translation.x = beyond;
    assert_invalid(mapper.map(&bind_pose(), &far));

    let neutral = rig.neutral_pose();
    let mut short = neutral.clone();
    short.joints.pop();
    assert_invalid(mapper.map_reverse(&short));
    let mut nan = neutral.clone();
    nan.joints[rig.index("head")].rotation.w = f32::NAN;
    assert_invalid(mapper.map_reverse(&nan));
}

#[test]
fn rotations_at_the_tolerance_edge_are_normalised() {
    let rig = Rig::humanoid();
    let mut ragdoll_neutral = bind_pose();
    ragdoll_neutral.joints[CHEST].rotation = EDGE_ROTATION;
    let mut animation_neutral = rig.neutral_pose();
    animation_neutral.joints[rig.index("chest")].rotation = EDGE_ROTATION;
    animation_neutral.joints[rig.index("neck")].rotation = EDGE_ROTATION;
    let mapper = mapper_with(
        &ragdoll_neutral,
        &rig,
        &animation_neutral,
        TranslationLocks::All,
    )
    .unwrap();
    let mut pose = posed_ragdoll();
    pose.joints[HEAD].rotation = EDGE_ROTATION;
    let mut local = rig.animated_local(0.3);
    local[rig.index("neck")].rotation = EDGE_ROTATION;
    local[rig.index("hand_r")].rotation = EDGE_ROTATION;
    assert!(mapper.map(&pose, &local).is_ok());
    let mut animation = rig.local_to_model(&local, RVec3::ZERO);
    animation.joints[rig.index("thigh_r")].rotation = EDGE_ROTATION;
    assert!(mapper.map_reverse(&animation).is_ok());
}

/// The bind pose with its root offset at the chest, so that the chest's translation is exactly
/// zero and the chain bound near it is tight.
fn chest_centred() -> SkeletonPose {
    let mut pose = bind_pose();
    let chest = pose.joints[CHEST].translation;
    pose.root_offset = RVec3::new(
        Real::from(chest.x),
        Real::from(chest.y),
        Real::from(chest.z),
    );
    for joint in &mut pose.joints {
        joint.translation = Vec3::new(
            joint.translation.x - chest.x,
            joint.translation.y - chest.y,
            joint.translation.z - chest.z,
        );
    }
    assert_eq!(pose.joints[CHEST].translation, Vec3::ZERO);
    pose
}

/// The neutral local pose with the chest-neck-head chain two steps of `step` along +Y.
fn head_chain(rig: &Rig, step: f32) -> Vec<JointTransform> {
    let mut local = rig.neutral_local();
    for name in ["neck", "head"] {
        local[rig.index(name)].translation = Vec3::new(0.0, step, 0.0);
    }
    local
}

#[test]
fn degenerate_chains_are_refused() {
    let rig = Rig::humanoid();
    let mapper = mapper_for(&rig, TranslationLocks::None);
    let refused = Err(RagdollError::DegenerateChain(HEAD as u32));
    let map = |pose: &SkeletonPose, local: &[JointTransform]| mapper.map(pose, local).map(|_| ());

    // A tiny ragdoll direction from chest to head.
    let mut pose = bind_pose();
    pose.joints[CHEST].translation = Vec3::ZERO;
    pose.joints[HEAD].translation = Vec3::new(1.0e-30, 0.0, 0.0);
    assert_ne!(
        pose.joints[HEAD].translation.x - pose.joints[CHEST].translation.x,
        0.0
    );
    assert_eq!(map(&pose, &rig.neutral_local()), refused);
    // Exactly together: Jolt turns nothing.
    pose.joints[HEAD].translation = Vec3::ZERO;
    assert_eq!(map(&pose, &rig.neutral_local()), Ok(()));

    // An animation chain that cancels: up 0.1 m, down 0.1 m.
    let mut cancelling = rig.neutral_local();
    cancelling[rig.index("neck")].translation = Vec3::new(0.0, 0.1, 0.0);
    cancelling[rig.index("head")].translation = Vec3::new(0.0, -0.1, 0.0);
    assert_eq!(map(&bind_pose(), &cancelling), refused);
    assert_eq!(map(&bind_pose(), &head_chain(&rig, 1.0e-30)), refused);

    // At the floor, for both directions, along and against each other.
    let floor = limits::MIN_MAPPED_CHAIN_LENGTH;
    let (below, above) = (floor * 0.999, floor * 1.001);
    for sign in [1.0, -1.0] {
        let with_head = |length: f32| {
            let mut pose = chest_centred();
            pose.joints[HEAD].translation = Vec3::new(1.0e-3 * length, sign * length, 0.0);
            pose
        };
        let chain = |length: f32| head_chain(&rig, length / 2.0);
        assert_eq!(map(&with_head(below), &chain(0.5)), refused);
        assert_eq!(map(&with_head(0.5), &chain(below)), refused);
        assert_eq!(map(&with_head(above), &chain(0.5)), Ok(()));
        assert_eq!(map(&with_head(0.5), &chain(above)), Ok(()));
        assert_eq!(map(&with_head(above), &chain(above)), Ok(()));
    }

    // A chain of 33 links turns like a short one.
    let long = Rig::with_long_spine(32);
    let long_mapper = mapper_for(&long, TranslationLocks::None);
    let pose = posed_ragdoll();
    let local = long.animated_local(0.5);
    let mapped = long_mapper.map(&pose, &local).unwrap();
    let pelvis = Rigid::at(&mapped, long.index("pelvis"));
    let end = (0..32)
        .map(|i| format!("spine_{i}"))
        .chain(["abdomen".to_owned()])
        .fold(pelvis, |model, name| {
            model.then(Rigid::of(local[long.index(&name)], RVec3::ZERO))
        });
    let desired = sub(
        Rigid::at(&pose, ABDOMEN).translation,
        Rigid::at(&pose, PELVIS).translation,
    );
    assert!(cos(sub(end.translation, pelvis.translation), desired) > 1.0 - 1e-6);

    // Model coordinates 4 km from the root offset round to millimetres: a 6 mm chain is
    // refused there, a 50 cm one is not.
    let mut far = bind_pose();
    far.root_offset = RVec3::new(-4000.0, 0.0, 0.0);
    for joint in &mut far.joints {
        joint.translation.x += 4000.0;
    }
    assert_eq!(map(&far, &head_chain(&rig, 0.003)), refused);
    assert_eq!(map(&far, &head_chain(&rig, 0.25)), Ok(()));
    assert_eq!(map(&bind_pose(), &head_chain(&rig, 0.003)), Ok(()));
}

/// `rig` with every position on a multiple of 2^-10 m.
fn on_grid(rig: &Rig) -> Rig {
    let mut rig = rig.clone();
    for joint in &mut rig.joints {
        joint.position = joint.position.map(|c| (c * 1024.0).round() / 1024.0);
    }
    rig
}

#[test]
fn neutral_root_offsets_are_compared_in_one_frame() {
    let rig = on_grid(&Rig::humanoid());
    let pose = posed_ragdoll();
    let local = rig.animated_local(0.6);
    let animation = rig.local_to_model(&local, RVec3::new(0.5, 0.0, 0.0));
    let reference = mapper_for(&rig, TranslationLocks::All);

    // The same neutral 4 m along X, as root offset plus translations: every value is exact.
    let mut shifted = rig.neutral_pose();
    shifted.root_offset = RVec3::new(4.0, 0.0, 0.0);
    for joint in &mut shifted.joints {
        assert!((joint.translation.x - 4.0).abs() < 8.0);
        joint.translation.x -= 4.0;
    }
    let moved = mapper_with(&bind_pose(), &rig, &shifted, TranslationLocks::All).unwrap();
    assert_eq!(
        bits(&moved.map(&pose, &local).unwrap()),
        bits(&reference.map(&pose, &local).unwrap())
    );
    assert_eq!(
        bits(&moved.map_reverse(&animation).unwrap()),
        bits(&reference.map_reverse(&animation).unwrap())
    );

    // Both neutrals turned and moved together describe the same mapping.
    let turn = quat_about(AXIS, 1.1);
    let turned = mapper_with(
        &transformed_pose(&bind_pose(), turn, [3.0, -1.0, 2.0]),
        &rig,
        &transformed_pose(&rig.neutral_pose(), turn, [3.0, -1.0, 2.0]),
        TranslationLocks::All,
    )
    .unwrap();
    let (a, b) = (
        turned.map(&pose, &local).unwrap(),
        reference.map(&pose, &local).unwrap(),
    );
    for joint in 0..rig.len() {
        assert_near(Rigid::at(&a, joint), Rigid::at(&b, joint), 1e-6);
    }
    let (a, b) = (
        turned.map_reverse(&animation).unwrap(),
        reference.map_reverse(&animation).unwrap(),
    );
    for part in 0..PART_COUNT {
        assert_near(Rigid::at(&a, part), Rigid::at(&b, part), 1e-6);
    }
}

/// A two-joint ragdoll (`a`, `b` at 1 m along X) and a three-joint animation rig (`a`, `b`,
/// `c`) whose `b` sits `b_shift` further along X and `c` 2 m past `b`.
fn line_mapper(b_shift: f32, locks: TranslationLocks<'_>) -> SkeletonMapper {
    let joint = |name, parent| SkeletonJoint { name, parent };
    let ragdoll = Skeleton::new(&[joint("a", None), joint("b", Some(0))]).unwrap();
    let animation =
        Skeleton::new(&[joint("a", None), joint("b", Some(0)), joint("c", Some(1))]).unwrap();
    let pose = |xs: &[f32]| SkeletonPose {
        root_offset: RVec3::ZERO,
        joints: xs
            .iter()
            .map(|&x| JointTransform {
                translation: Vec3::new(x, 0.0, 0.0),
                rotation: Quat::IDENTITY,
            })
            .collect(),
    };
    SkeletonMapper::new(
        MappedSkeleton {
            skeleton: &ragdoll,
            neutral_pose: &pose(&[0.0, 1.0]),
        },
        MappedSkeleton {
            skeleton: &animation,
            neutral_pose: &pose(&[0.0, 1.0 + b_shift, 3.0 + b_shift]),
        },
        locks,
    )
    .unwrap()
}

fn line_pose(root_offset: Real, xs: &[f32]) -> SkeletonPose {
    SkeletonPose {
        root_offset: RVec3::new(root_offset, 0.0, 0.0),
        joints: xs
            .iter()
            .map(|&x| JointTransform {
                translation: Vec3::new(x, 0.0, 0.0),
                rotation: Quat::IDENTITY,
            })
            .collect(),
    }
}

fn line_local(c: f32) -> Vec<JointTransform> {
    line_pose(0.0, &[0.0, 1.0, c]).joints
}

#[test]
fn outputs_are_validated() {
    let edge = limits::MAX_POSITION;
    let refused = Err(RagdollError::InvalidValue(
        "the mapped pose leaves limits::MAX_POSITION",
    ));
    // `b` at 0.5 m inside the edge.
    let ragdoll = line_pose(edge - 1.5, &[0.0, 1.0]);

    // A direct mapping 1 m further out.
    let shifted = line_mapper(1.0, TranslationLocks::None);
    assert_eq!(shifted.map(&ragdoll, &line_local(0.1)).map(|_| ()), refused);
    // An unmapped joint 1 m past `b`, and 0.4 m.
    let plain = line_mapper(0.0, TranslationLocks::None);
    assert_eq!(plain.map(&ragdoll, &line_local(1.0)).map(|_| ()), refused);
    assert!(plain.map(&ragdoll, &line_local(0.4)).is_ok());
    // A lock that puts `c` back at its neutral 2 m from `b`.
    let locked = line_mapper(0.0, TranslationLocks::All);
    assert!(locked.is_translation_locked(2));
    assert_eq!(locked.map(&ragdoll, &line_local(0.4)).map(|_| ()), refused);

    // The reverse direction: `b` 1 m further in on the ragdoll, past the opposite edge.
    let animation = line_pose(-edge + 0.5, &[0.0, 0.2, 0.3]);
    assert_eq!(shifted.map_reverse(&animation).map(|_| ()), refused);
    assert!(plain.map_reverse(&animation).is_ok());
}

#[test]
fn the_mapper_owns_its_state() {
    let rig = Rig::humanoid();
    let locks = [rig.index("hand_l") as u32];
    let mapper = {
        let (ragdoll, animation) = (skeleton(), rig.skeleton());
        let (ragdoll_neutral, animation_neutral) = (bind_pose(), rig.neutral_pose());
        SkeletonMapper::new(
            MappedSkeleton {
                skeleton: &ragdoll,
                neutral_pose: &ragdoll_neutral,
            },
            MappedSkeleton {
                skeleton: &animation,
                neutral_pose: &animation_neutral,
            },
            TranslationLocks::Joints(&locks.clone()),
        )
        .unwrap()
    };
    let pose = posed_ragdoll();
    let local = rig.animated_local(0.1);
    let mapped = mapper.map(&pose, &local).unwrap();
    let again = mapper_for(&rig, TranslationLocks::Joints(&locks));
    assert_eq!(bits(&mapped), bits(&again.map(&pose, &local).unwrap()));
    assert_eq!(
        bits(&mapper.map_reverse(&mapped).unwrap()),
        bits(&again.map_reverse(&mapped).unwrap())
    );
    assert!(mapper.is_translation_locked(locks[0]));
    assert_eq!(
        (mapper.ragdoll_joint_count(), mapper.animation_joint_count()),
        (PART_COUNT as u32, rig.len() as u32)
    );
}

/// The rotation of `part` relative to its parent part, in the parent's frame.
fn relative_rotation(world: &PhysicsWorld, ragdoll: RagdollId, part: usize) -> Quat {
    let ids = world.ragdoll(ragdoll).unwrap().body_ids().to_vec();
    let parent = PARTS[part].parent.unwrap() as usize;
    let rotation = |id| world.body(id).unwrap().rotation();
    mul(conj(rotation(ids[parent])), rotation(ids[part]))
}

/// An animation pose: the rig bent over time, its left leg flexed forward at the hip.
fn animation_target(rig: &Rig, root_offset: RVec3) -> SkeletonPose {
    let mut pose = rig.local_to_model(&rig.animated_local(0.5), root_offset);
    let flex = quat_about(neg(X), 0.6);
    for name in ["thigh_l", "shin_l", "foot_l"] {
        let joint = &mut pose.joints[rig.index(name)];
        joint.rotation = mul(flex, joint.rotation);
    }
    pose
}

#[test]
fn mapped_poses_drive_a_ragdoll() {
    let rig = Rig::humanoid();
    let mapper = mapper_for(&rig, TranslationLocks::None);
    let (mut world, layers) = ragdoll_world(1);
    let settings = humanoid_settings(layers.ragdoll);

    // A mapped pose set directly reads back.
    let ragdoll = world
        .create_ragdoll(&settings, None, Activation::DontActivate)
        .unwrap();
    let target = mapper
        .map_reverse(&animation_target(&rig, RVec3::new(0.0, 0.3, 0.0)))
        .unwrap();
    world
        .ragdoll_mut(ragdoll)
        .unwrap()
        .set_pose(&target)
        .unwrap();
    let read = world.ragdoll(ragdoll).unwrap().pose();
    for part in 0..PART_COUNT {
        assert_near(Rigid::at(&read, part), Rigid::at(&target, part), 1e-4);
    }
    world.remove_ragdoll(ragdoll).unwrap();

    // Motors from a turned start reach the mapped joint rotations of a swing-twist (neck) and a
    // six-DOF (hip) joint.
    let turned = transformed_pose(&bind_pose(), quat_about(AXIS, 0.9), [0.0; 3]);
    let ragdoll = world
        .create_ragdoll(&settings, Some(&turned), Activation::Activate)
        .unwrap();
    for _ in 0..180 {
        world
            .ragdoll_mut(ragdoll)
            .unwrap()
            .drive_to_pose_using_motors(&target)
            .unwrap();
        step(&mut world, 1);
    }
    for part in [HEAD, THIGH_L] {
        let parent = PARTS[part].parent.unwrap() as usize;
        let wanted = mul(
            conj(target.joints[parent].rotation),
            target.joints[part].rotation,
        );
        let start = mul(
            conj(turned.joints[parent].rotation),
            turned.joints[part].rotation,
        );
        assert!(
            angle_between(start, wanted) > 0.2,
            "part {part} starts at its target"
        );
        let reached = angle_between(relative_rotation(&world, ragdoll, part), wanted);
        assert!(reached < 0.05, "part {part}: {reached} rad from the target");
    }
    world.remove_ragdoll(ragdoll).unwrap();

    // Kinematic parts driven to a mapped pose of a small move reach it in one step.
    let ragdoll = world
        .create_ragdoll(&settings, None, Activation::Activate)
        .unwrap();
    world
        .ragdoll_mut(ragdoll)
        .unwrap()
        .set_motion_type(MotionType::Kinematic, Activation::Activate)
        .unwrap();
    let moved = transformed_pose(&rig.neutral_pose(), quat_about(Y, 0.1), [0.2, 0.1, -0.1]);
    let kinematic_target = mapper.map_reverse(&moved).unwrap();
    world
        .ragdoll_mut(ragdoll)
        .unwrap()
        .drive_to_pose_using_kinematics(&kinematic_target, DT)
        .unwrap();
    step(&mut world, 1);
    let read = world.ragdoll(ragdoll).unwrap().pose();
    for part in 0..PART_COUNT {
        assert_near(
            Rigid::at(&read, part),
            Rigid::at(&kinematic_target, part),
            1e-3,
        );
    }

    // The simulated pose maps back onto the animation skeleton.
    let local = rig.animated_local(0.5);
    let shown = mapper.map(&read, &local).unwrap();
    for part in DIRECT_ONLY {
        let joint = rig.index(PARTS[part].name);
        assert_near(
            Rigid::at(&shown, joint),
            direct(&read, &rig, part, joint),
            1e-5,
        );
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn mapping_is_a_pure_function() {
    let rig = Rig::humanoid();
    let mapper = mapper_for(&rig, TranslationLocks::All);
    let inputs: Vec<(SkeletonPose, Vec<JointTransform>)> = (0..4)
        .map(|i| {
            let pose = transformed_pose(
                &posed_ragdoll(),
                quat_about(AXIS, 0.2 * i as f32),
                [0.1 * i as f64, 0.0, 0.0],
            );
            (pose, rig.animated_local(0.3 * i as f32))
        })
        .collect();
    let expected: Vec<(Vec<u64>, Vec<u64>)> = inputs
        .iter()
        .map(|(pose, local)| {
            let mapped = mapper.map(pose, local).unwrap();
            let reversed = mapper.map_reverse(&mapped).unwrap();
            (bits(&mapped), bits(&reversed))
        })
        .collect();
    let run = |offset: usize| {
        for round in 0..50 {
            let i = (round + offset) % inputs.len();
            let (pose, local) = &inputs[i];
            let mapped = mapper.map(pose, local).unwrap();
            assert_eq!(bits(&mapped), expected[i].0);
            assert_eq!(bits(&mapper.map_reverse(&mapped).unwrap()), expected[i].1);
        }
    };
    thread::scope(|scope| {
        scope.spawn(|| run(0));
        scope.spawn(|| run(1));
        run(2);
    });
}
