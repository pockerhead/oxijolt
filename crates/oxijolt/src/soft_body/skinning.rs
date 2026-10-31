//! Skinning a soft body in the world to joint transforms, and its skinned max distance
//! multiplier.

use oxijolt_sys::*;

use super::handle::{require_body, SoftBodyMut, SoftBodyRef};
use crate::body::{with_locked_body, with_read_locked_body};
use crate::limits::{self, is_in_frame, is_local_offset};
use crate::math::{matrix, ROTATION_RULE};
use crate::{BodyError, Quat, RVec3, SkeletonPose, Vec3};

const NOT_SKINNED: &str = "the soft body has no skinned constraints";
const POSE_POSITION_RULE: &str =
    "skin pose positions must be finite and within limits::MAX_POSITION";
const SHORT_POSE_RULE: &str = "a skin pose needs a transform for every bound joint";
const JOINT_DISTANCE_RULE: &str =
    "skin joints must be within limits::MAX_SHAPE_EXTENT of the soft body's centre of mass";
const MULTIPLIER_RULE: &str =
    "the skinned max distance multiplier must be finite and within 0..=limits::MAX_RATIO";

impl SoftBodyRef<'_> {
    /// The factor Jolt multiplies every skinned constraint's max distance with; 1 unless
    /// [`SoftBodyMut::set_skinned_max_distance_multiplier`] changed it.
    pub fn skinned_max_distance_multiplier(&self) -> f32 {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure; the getter
            // only reads it.
            unsafe { JPH_Body_GetSoftBodySkinnedMaxDistanceMultiplier(body.as_ptr()) }
        })
        .unwrap_or(1.0)
    }
}

impl SoftBodyMut<'_> {
    /// Moves the skinned positions of the body's skinned constraints to the joints of `pose`
    /// (Jolt `SoftBodyMotionProperties::SkinVertices`). The next step pulls each skinned vertex
    /// toward its new skinned position within its max distance and back stop.
    ///
    /// `pose` is a world pose, as [`RagdollRef::pose`](crate::RagdollRef::pose) returns it:
    /// joint `j` of the inverse binds is `pose.joints[j]`. The pose needs a transform for every
    /// bound joint (more are ignored), each finite with a unit rotation, and every joint up to
    /// the largest bound one, bound or not, within [`limits::MAX_POSITION`] and within
    /// [`limits::MAX_SHAPE_EXTENT`] (per axis) of the body's centre of mass. The conversion to the body's frame is done in double precision with
    /// the `double-precision` feature. Fails with [`BodyError::InvalidValue`] otherwise, or
    /// when the body has no skinned constraints; then nothing changes.
    ///
    /// Call it once before each step: during the step the skinned positions move from those
    /// of the previous call to these. A step without a call before it holds the last skinned
    /// positions. With several collision steps per step the first one moves the skinned
    /// positions and the later ones hold them. The body's collision bounds follow the
    /// vertices in the next step. Each call wakes the body.
    ///
    /// See [docs/soft-bodies.md].
    ///
    /// [docs/soft-bodies.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/soft-bodies.md
    pub fn skin_vertices(&mut self, pose: &SkeletonPose) -> Result<(), BodyError> {
        self.skin(pose, false)
    }

    /// Like [`skin_vertices`](Self::skin_vertices), and also moves every skinned vertex to its
    /// skinned position at once and sets its velocity to zero, whatever its max distance: for
    /// a teleport, or to start an animation without the cloth swinging in from the old pose.
    pub fn hard_skin_vertices(&mut self, pose: &SkeletonPose) -> Result<(), BodyError> {
        self.skin(pose, true)
    }

    /// Sets the factor Jolt multiplies every skinned constraint's max distance with, finite
    /// and within `0..=`[`limits::MAX_RATIO`]: 0 attaches every skinned vertex to its skinned
    /// position, also those without a max distance, and values above 1 loosen the cloth. Jolt
    /// applies a back stop only while its distance is below the scaled max distance, so a
    /// multiplier below 1 can switch back stops off. The body needs skinned constraints. Fails with [`BodyError::InvalidValue`] otherwise; then
    /// nothing changes. Wakes the body.
    ///
    /// The multiplier is configuration that a [`WorldState`](crate::WorldState) does not save.
    pub fn set_skinned_max_distance_multiplier(
        &mut self,
        multiplier: f32,
    ) -> Result<(), BodyError> {
        require_body(
            (0.0..=limits::MAX_RATIO).contains(&multiplier),
            MULTIPLIER_RULE,
        )?;
        with_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure; the getter
            // only reads it.
            let joint_count = unsafe { JPH_Body_GetSoftBodySkinJointCount(body.as_ptr()) };
            require_body(joint_count > 0, NOT_SKINNED)?;
            // SAFETY: as above; the multiplier is finite and not negative. The call refuses only
            // a body that is not a soft body, and this view was made for a soft body.
            unsafe { JPH_Body_SetSoftBodySkinnedMaxDistanceMultiplier(body.as_ptr(), multiplier) };
            Ok(())
        })
        .ok_or(BodyError::NotFound(self.id))??;
        self.activate();
        Ok(())
    }

    fn skin(&mut self, pose: &SkeletonPose, hard: bool) -> Result<(), BodyError> {
        validate_pose(pose)?;
        with_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure, and the view
            // borrows the world mutably, so no step runs. The getters only read the body and
            // write the live locals.
            let (joint_count, center_of_mass, rotation) = unsafe {
                let mut center_of_mass = RVec3::ZERO.to_jph();
                let mut rotation = Quat::IDENTITY.to_jph();
                JPH_Body_GetCenterOfMassPosition(body.as_ptr(), &mut center_of_mass);
                JPH_Body_GetRotation(body.as_ptr(), &mut rotation);
                (
                    JPH_Body_GetSoftBodySkinJointCount(body.as_ptr()),
                    RVec3::from_jph(center_of_mass),
                    Quat::from_jph(rotation),
                )
            };
            require_body(joint_count > 0, NOT_SKINNED)?;
            require_body(pose.joints.len() >= joint_count as usize, SHORT_POSE_RULE)?;
            let joints = joint_matrices(pose, joint_count as usize, center_of_mass, rotation)?;
            // SAFETY: as above, the body is write-locked and no step runs. `joints` holds
            // `joint_count` matrices, one per joint up to the largest bound one, and lives for
            // the call; joltc copies them into aligned storage.
            let skinned = unsafe {
                JPH_Body_SkinSoftBodyVertices(body.as_ptr(), joints.as_ptr(), joint_count, hard)
            };
            // The body has skinned constraints and `joints` covers every bound joint, the only
            // refusals of the native call.
            require_body(skinned, NOT_SKINNED)
        })
        .ok_or(BodyError::NotFound(self.id))??;
        self.activate();
        Ok(())
    }
}

/// Checks every joint of a world pose: finite, a unit rotation and within the world.
fn validate_pose(pose: &SkeletonPose) -> Result<(), BodyError> {
    require_body(is_in_frame(pose.root_offset), POSE_POSITION_RULE)?;
    for (index, joint) in pose.joints.iter().enumerate() {
        require_body(joint.translation.is_finite(), POSE_POSITION_RULE)?;
        require_body(joint.rotation.is_valid_rotation(), ROTATION_RULE)?;
        require_body(is_in_frame(pose.position(index)), POSE_POSITION_RULE)?;
    }
    Ok(())
}

/// The first `count` joints of `pose` relative to the centre of mass transform
/// (`center_of_mass`, `rotation`), as Jolt's `SkinVertices` takes them.
fn joint_matrices(
    pose: &SkeletonPose,
    count: usize,
    center_of_mass: RVec3,
    rotation: Quat,
) -> Result<Vec<JPH_Mat4>, BodyError> {
    let inverse = rotation.normalized().conjugated();
    (0..count)
        .map(|index| {
            let world = pose.position(index);
            let local = inverse.rotate_real(RVec3::new(
                world.x - center_of_mass.x,
                world.y - center_of_mass.y,
                world.z - center_of_mass.z,
            ));
            // `Real` is `f32` without the `double-precision` feature, so the casts are no-ops
            // there.
            #[allow(clippy::unnecessary_cast)]
            let local = Vec3::new(local.x as f32, local.y as f32, local.z as f32);
            require_body(is_local_offset(local), JOINT_DISTANCE_RULE)?;
            let joint_rotation = inverse.product(pose.joints[index].rotation).normalized();
            Ok(matrix(local, joint_rotation))
        })
        .collect()
}
