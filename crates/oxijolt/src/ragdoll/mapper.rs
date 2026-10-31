//! Jolt's `SkeletonMapper` between a ragdoll skeleton and a detailed animation skeleton.

use std::collections::BTreeMap;

use oxijolt_sys::*;

use super::settings::{validate_local, JointTransform, Skeleton, SkeletonPose};
use crate::math::{matrix, zero_matrix};
use crate::owned::{JoltObject, Owned};
use crate::{Quat, RVec3, RagdollError, Real, Vec3};

/// A skeleton mapper, owned with the one reference `JPH_SkeletonMapper_Create` returns, which
/// `JPH_SkeletonMapper_Destroy` releases.
impl JoltObject for JPH_SkeletonMapper {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), which this releases.
        unsafe { JPH_SkeletonMapper_Destroy(ptr) };
    }
}

const LOCK_RULE: &str =
    "locked joints exist and are neither the root nor the joint the ragdoll root maps to";
const MAPPED_POSITION_RULE: &str = "the mapped pose leaves limits::MAX_POSITION";
const MAPPED_ROTATION_RULE: &str = "the mapped pose has a rotation that is not a unit quaternion";

/// One skeleton of a [`SkeletonMapper`] with its neutral pose.
#[derive(Clone, Copy, Debug)]
pub struct MappedSkeleton<'a> {
    /// The skeleton.
    pub skeleton: &'a Skeleton,
    /// The skeleton in its neutral pose, in model space. The two neutral poses describe the
    /// character standing in one place, so that joints of the same name are where they belong
    /// to each other. [`SkeletonMapper::new`] does not check that they are close: neutral poses
    /// far apart put long translations into the transforms between the skeletons, and
    /// [`SkeletonMapper::map`] then refuses chains it can no longer turn reliably
    /// ([`RagdollError::DegenerateChain`]).
    ///
    /// See [docs/limits.md#skeleton-mapper-neutral-poses].
    ///
    /// [docs/limits.md#skeleton-mapper-neutral-poses]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#skeleton-mapper-neutral-poses
    pub neutral_pose: &'a SkeletonPose,
}

/// Which animation joints [`SkeletonMapper::map`] keeps at their neutral offset from their
/// parent (Jolt `LockTranslations`). Joint constraints stretch a little under load; a lock hides
/// that stretch in the animation pose, which then differs from the simulated bodies.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TranslationLocks<'a> {
    /// No joint is locked.
    #[default]
    None,
    /// Every descendant of the animation joint the ragdoll root maps to (Jolt
    /// `LockAllTranslations`).
    All,
    /// These animation joints. The animation root (joint 0) and the joint the ragdoll root maps
    /// to are refused: their position comes from the simulation.
    Joints(&'a [u32]),
}

/// Jolt's `SkeletonMapper`: maps poses between a ragdoll skeleton and a more detailed animation
/// skeleton of the same character.
///
/// Joints are matched by name. Every ragdoll joint needs an animation joint of its name, and
/// the animation skeleton keeps the ragdoll's hierarchy: the animation joint of a ragdoll joint
/// sits below the animation joint of its parent, with only unmatched joints in between. The
/// animation skeleton may also have extra joints above the ragdoll root and below its leaves.
///
/// - [`map`](Self::map) turns a ragdoll pose into an animation pose (Jolt `Map`), to show the
///   simulated ragdoll on the detailed skeleton. Matched joints follow their ragdoll joint. An
///   unmatched joint between a ragdoll joint and one of its children is a chain: the chain's
///   start turns so that the chain, laid out by the local animation pose, points at the child's
///   ragdoll joint. Jolt builds one chain per start joint, from the child with the longest path
///   (the lowest ragdoll joint index on a tie); unmatched joints below the start's other children
///   keep their local transforms, as do all other unmatched joints, extra roots included (relative
///   to the pose's root offset). Translation locks apply last.
/// - [`map_reverse`](Self::map_reverse) turns an animation pose into a ragdoll pose (Jolt
///   `MapReverse`) from the matched joints only, as a target for
///   [`RagdollMut::set_pose`](crate::RagdollMut::set_pose) or the drives.
///
/// The mapper copies what it needs and keeps no reference to the skeletons or poses. It never
/// changes after [`new`](Self::new); both mappings are pure functions of their inputs.
pub struct SkeletonMapper {
    mapper: Owned<JPH_SkeletonMapper>,
    ragdoll_joint_count: u32,
    animation_joint_count: u32,
}

impl std::fmt::Debug for SkeletonMapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SkeletonMapper")
            .field("ragdoll_joint_count", &self.ragdoll_joint_count)
            .field("animation_joint_count", &self.animation_joint_count)
            .finish_non_exhaustive()
    }
}

// SAFETY: the Jolt mapper is never changed after `new`; `Map`, `MapReverse`, `GetMappedJointIdx`
// and `IsJointTranslationLocked` only read its members (SkeletonMapper.cpp:165-235), the
// extension's temporary arrays come from Jolt's thread-safe allocator, and `RefTarget` counts
// references atomically (Reference.h:57-86).
unsafe impl Send for SkeletonMapper {}
// SAFETY: as for `Send`; `&SkeletonMapper` only maps poses and reads the mapping.
unsafe impl Sync for SkeletonMapper {}

impl SkeletonMapper {
    /// A mapper from `ragdoll` (Jolt's skeleton 1) to `animation` (skeleton 2) with `locks`.
    ///
    /// The animation neutral pose is re-expressed relative to the ragdoll neutral pose's root
    /// offset. Fails with [`RagdollError::UnmappedJoint`] for a ragdoll joint without an
    /// animation joint of its name, [`RagdollError::HierarchyMismatch`] when the hierarchies do
    /// not correspond, and [`RagdollError::InvalidValue`] when a neutral pose does not fit its
    /// skeleton ([`RagdollMut::set_pose`](crate::RagdollMut::set_pose)'s rules) or a lock names
    /// a missing or refused joint.
    pub fn new(
        ragdoll: MappedSkeleton<'_>,
        animation: MappedSkeleton<'_>,
        locks: TranslationLocks<'_>,
    ) -> Result<Self, RagdollError> {
        let ragdoll_count = ragdoll.skeleton.names().len();
        let animation_count = animation.skeleton.names().len();
        ragdoll.neutral_pose.validate(ragdoll_count)?;
        animation.neutral_pose.validate(animation_count)?;
        let mapped = match_joints(ragdoll.skeleton, animation.skeleton)?;
        let mask = match locks {
            TranslationLocks::Joints(joints) => lock_mask(joints, mapped[0], animation_count)?,
            TranslationLocks::None | TranslationLocks::All => Vec::new(),
        };
        let neutral1 = ragdoll
            .neutral_pose
            .joints
            .iter()
            .map(|joint| matrix(joint.translation, joint.rotation))
            .collect::<Vec<_>>();
        let neutral2 = reexpress(animation.neutral_pose, ragdoll.neutral_pose.root_offset)
            .iter()
            .map(|joint| matrix(joint.translation, joint.rotation))
            .collect::<Vec<_>>();

        // SAFETY: Jolt is initialised: a `Skeleton` exists, and `Skeleton::new` ran
        // `ensure_initialized`. The handle takes over the one reference joltc returns.
        let mapper = unsafe { Owned::from_raw(JPH_SkeletonMapper_Create()) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the mapper"));
        let (count1, count2) = (ragdoll_count as u32, animation_count as u32);
        // SAFETY: the mapper and both skeletons are live; each array holds one matrix per joint
        // (the poses were validated against the counts) and, for joint locks, the mask one flag
        // per animation joint. Both skeletons have at most `Skeleton::MAX_JOINTS` joints, so the stack array
        // `LockAllTranslations` allocates is small.
        let done = unsafe {
            JPH_SkeletonMapper_Initialize2(
                mapper.as_ptr(),
                ragdoll.skeleton.as_ptr(),
                neutral1.as_ptr(),
                count1,
                animation.skeleton.as_ptr(),
                neutral2.as_ptr(),
                count2,
            ) && match locks {
                TranslationLocks::None => true,
                TranslationLocks::All => JPH_SkeletonMapper_LockAllTranslations2(
                    mapper.as_ptr(),
                    animation.skeleton.as_ptr(),
                    neutral2.as_ptr(),
                    count2,
                ),
                TranslationLocks::Joints(_) => JPH_SkeletonMapper_LockTranslations2(
                    mapper.as_ptr(),
                    animation.skeleton.as_ptr(),
                    mask.as_ptr(),
                    neutral2.as_ptr(),
                    count2,
                ),
            }
        };
        if !done {
            unreachable!("the mapper's inputs were checked against every refusal of joltc_ext");
        }
        Ok(Self {
            mapper,
            ragdoll_joint_count: count1,
            animation_joint_count: count2,
        })
    }

    /// Ragdoll pose to animation pose (Jolt `Map`): `ragdoll_pose` in model space and
    /// `animation_local`, one transform per animation joint relative to its parent (the root's
    /// relative to the root offset), give the animation pose in model space with the ragdoll
    /// pose's root offset. `animation_local` gives the unmatched joints; matched joints come
    /// from the ragdoll pose.
    ///
    /// Fails with [`RagdollError::InvalidValue`] when `ragdoll_pose` does not fit the ragdoll
    /// skeleton ([`RagdollMut::set_pose`](crate::RagdollMut::set_pose)'s rules),
    /// `animation_local` has the wrong count, a non-unit rotation or a translation component
    /// above [`limits::MAX_POSITION`](crate::limits::MAX_POSITION), or the result leaves `MAX_POSITION`; with
    /// [`RagdollError::DegenerateChain`] when a chain is too short to turn (see
    /// [`limits::MIN_MAPPED_CHAIN_LENGTH`](crate::limits::MIN_MAPPED_CHAIN_LENGTH)).
    pub fn map(
        &self,
        ragdoll_pose: &SkeletonPose,
        animation_local: &[JointTransform],
    ) -> Result<SkeletonPose, RagdollError> {
        ragdoll_pose.validate(self.ragdoll_joint_count as usize)?;
        validate_local(animation_local, self.animation_joint_count as usize)?;
        let pose1 = matrices(&ragdoll_pose.joints);
        let local2 = matrices(animation_local);
        let mut out = vec![zero_matrix(); animation_local.len()];
        let mut degenerate = -1;
        // SAFETY: the mapper is live and only read; `pose1` and `out` hold one matrix per joint
        // of their skeleton and `local2` one per animation joint, as the counts say.
        let done = unsafe {
            JPH_SkeletonMapper_Map2(
                self.mapper.as_ptr(),
                pose1.as_ptr(),
                self.ragdoll_joint_count,
                local2.as_ptr(),
                self.animation_joint_count,
                out.as_mut_ptr(),
                &mut degenerate,
            )
        };
        if !done {
            return match u32::try_from(degenerate) {
                Ok(joint) => Err(RagdollError::DegenerateChain(joint)),
                Err(_) => unreachable!("the poses were checked against every other refusal"),
            };
        }
        mapped_pose(ragdoll_pose.root_offset, &out)
    }

    /// Animation pose to ragdoll pose (Jolt `MapReverse`): each ragdoll joint from its matched
    /// animation joint in `animation_pose` (model space), with the animation pose's root offset.
    /// Chains, unmatched joints and locks play no part.
    ///
    /// Fails with [`RagdollError::InvalidValue`] when `animation_pose` does not fit the
    /// animation skeleton ([`RagdollMut::set_pose`](crate::RagdollMut::set_pose)'s rules) or
    /// the result leaves [`limits::MAX_POSITION`](crate::limits::MAX_POSITION).
    pub fn map_reverse(&self, animation_pose: &SkeletonPose) -> Result<SkeletonPose, RagdollError> {
        animation_pose.validate(self.animation_joint_count as usize)?;
        let pose2 = matrices(&animation_pose.joints);
        let mut out = vec![zero_matrix(); self.ragdoll_joint_count as usize];
        // SAFETY: the mapper is live and only read; `pose2` holds one matrix per animation joint
        // and `out` one per ragdoll joint, as the counts say.
        let done = unsafe {
            JPH_SkeletonMapper_MapReverse2(
                self.mapper.as_ptr(),
                pose2.as_ptr(),
                self.animation_joint_count,
                out.as_mut_ptr(),
                self.ragdoll_joint_count,
            )
        };
        if !done {
            unreachable!("the pose was checked against every refusal of joltc_ext");
        }
        mapped_pose(animation_pose.root_offset, &out)
    }

    /// The animation joint matched to `ragdoll_joint`; `None` when there is no such ragdoll
    /// joint.
    pub fn mapped_joint(&self, ragdoll_joint: u32) -> Option<u32> {
        if ragdoll_joint >= self.ragdoll_joint_count {
            return None;
        }
        // SAFETY: the mapper is live; the getter only reads it. The index is below
        // `Skeleton::MAX_JOINTS`, so it fits `int`.
        let joint = unsafe {
            JPH_SkeletonMapper_GetMappedJointIndex(self.mapper.as_ptr(), ragdoll_joint as i32)
        };
        u32::try_from(joint).ok()
    }

    /// Whether [`map`](Self::map) keeps `animation_joint` at its neutral offset from its parent;
    /// `false` when there is no such joint.
    pub fn is_translation_locked(&self, animation_joint: u32) -> bool {
        animation_joint < self.animation_joint_count
            // SAFETY: the mapper is live; the getter only reads it. The index is below
            // `Skeleton::MAX_JOINTS`, so it fits `int`.
            && unsafe {
                JPH_SkeletonMapper_IsJointTranslationLocked(
                    self.mapper.as_ptr(),
                    animation_joint as i32,
                )
            }
    }

    /// Number of ragdoll skeleton joints.
    pub fn ragdoll_joint_count(&self) -> u32 {
        self.ragdoll_joint_count
    }

    /// Number of animation skeleton joints.
    pub fn animation_joint_count(&self) -> u32 {
        self.animation_joint_count
    }
}

/// The animation joint of every ragdoll joint, matched by name, after checking that the
/// animation skeleton keeps the ragdoll's hierarchy.
fn match_joints(ragdoll: &Skeleton, animation: &Skeleton) -> Result<Vec<u32>, RagdollError> {
    let by_name: BTreeMap<&str, u32> = animation
        .names()
        .iter()
        .enumerate()
        .map(|(index, name)| (name.as_str(), index as u32))
        .collect();
    let mapped = ragdoll
        .names()
        .iter()
        .enumerate()
        .map(|(joint, name)| {
            by_name
                .get(name.as_str())
                .copied()
                .ok_or(RagdollError::UnmappedJoint(joint as u32))
        })
        .collect::<Result<Vec<u32>, _>>()?;
    let mut is_mapped = vec![false; animation.names().len()];
    for &joint in &mapped {
        is_mapped[joint as usize] = true;
    }
    for (joint, &animation_joint) in mapped.iter().enumerate() {
        let expected = ragdoll.parents()[joint].map(|parent| mapped[parent as usize]);
        if mapped_ancestor(animation.parents(), &is_mapped, animation_joint) != expected {
            return Err(RagdollError::HierarchyMismatch(joint as u32));
        }
    }
    Ok(mapped)
}

/// The nearest ancestor of `joint` that is mapped.
fn mapped_ancestor(parents: &[Option<u32>], is_mapped: &[bool], joint: u32) -> Option<u32> {
    let mut current = parents[joint as usize];
    while let Some(ancestor) = current {
        if is_mapped[ancestor as usize] {
            return Some(ancestor);
        }
        current = parents[ancestor as usize];
    }
    None
}

/// One flag per animation joint, set for `joints`. `mapped_root` is the joint the ragdoll root
/// maps to.
fn lock_mask(joints: &[u32], mapped_root: u32, count: usize) -> Result<Vec<bool>, RagdollError> {
    let mut mask = vec![false; count];
    for &joint in joints {
        if joint == 0 || joint == mapped_root || joint as usize >= count {
            return Err(RagdollError::InvalidValue(LOCK_RULE));
        }
        mask[joint as usize] = true;
    }
    Ok(mask)
}

/// `pose`'s joints relative to `root_offset` instead of its own root offset. Both poses were
/// validated, so every component is a difference of two positions in the frame plus rounding:
/// at most `2 * limits::MAX_POSITION * (1 + 2^-22)`, finite as `f32` (docs/limits.md, "Skeleton
/// mapper neutral poses").
fn reexpress(pose: &SkeletonPose, root_offset: RVec3) -> Vec<JointTransform> {
    let o = pose.root_offset;
    // `Real` is `f32` without the `double-precision` feature, so the cast is a no-op there.
    #[allow(clippy::unnecessary_cast)]
    let shift = |own: Real, other: Real, t: f32| (own - other + Real::from(t)) as f32;
    pose.joints
        .iter()
        .map(|joint| {
            let t = joint.translation;
            JointTransform {
                translation: Vec3::new(
                    shift(o.x, root_offset.x, t.x),
                    shift(o.y, root_offset.y, t.y),
                    shift(o.z, root_offset.z, t.z),
                ),
                rotation: joint.rotation,
            }
        })
        .collect()
}

fn matrices(joints: &[JointTransform]) -> Vec<JPH_Mat4> {
    joints
        .iter()
        .map(|joint| matrix(joint.translation, joint.rotation))
        .collect()
}

/// The pose of `matrices` at `root_offset`, if it is a valid pose.
fn mapped_pose(root_offset: RVec3, matrices: &[JPH_Mat4]) -> Result<SkeletonPose, RagdollError> {
    let joints = matrices
        .iter()
        .map(|matrix| {
            let mut translation = Vec3::ZERO.to_jph();
            let mut rotation = Quat::IDENTITY.to_jph();
            // SAFETY: joltc copies the matrix and writes the two live locals.
            unsafe {
                JPH_Mat4_GetTranslation(matrix, &mut translation);
                JPH_Mat4_GetQuaternion(matrix, &mut rotation);
            }
            JointTransform {
                translation: Vec3::from_jph(translation),
                rotation: Quat::from_jph(rotation).normalized(),
            }
        })
        .collect();
    let pose = SkeletonPose {
        root_offset,
        joints,
    };
    match pose.validate(matrices.len()) {
        Ok(()) => Ok(pose),
        Err(_) if pose.joints.iter().any(|j| !j.rotation.is_valid_rotation()) => {
            Err(RagdollError::InvalidValue(MAPPED_ROTATION_RULE))
        }
        Err(_) => Err(RagdollError::InvalidValue(MAPPED_POSITION_RULE)),
    }
}

#[cfg(test)]
mod tests;
