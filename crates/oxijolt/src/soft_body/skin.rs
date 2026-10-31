//! Skinning inputs of soft body shared settings: inverse binds, skin weights and skinned
//! constraints, checked before the settings are built.

use std::collections::BTreeSet;

use oxijolt_sys::*;

use super::shared::require;
use crate::limits::{self, is_in_frame, is_local_distance, is_local_offset, MAX_SKIN_JOINTS};
use crate::math::{matrix, ROTATION_RULE};
use crate::{Quat, RVec3, SkeletonPose, SoftBodyError, Vec3};

const JOINT_RULE: &str = "an inverse bind joint must be below limits::MAX_SKIN_JOINTS";
const UNIQUE_JOINT_RULE: &str = "a joint has at most one inverse bind";
const BIND_POSITION_RULE: &str =
    "an inverse bind position must be finite and within limits::MAX_SHAPE_EXTENT";
const PAIRING_RULE: &str = "inverse binds and skinned constraints go together";
const VERTEX_RULE: &str = "a skinned constraint must name a vertex";
const UNIQUE_VERTEX_RULE: &str = "a vertex has at most one skinned constraint";
const WEIGHT_RULE: &str = "skin weights must be finite and not negative";
const POSITIVE_WEIGHT_RULE: &str = "a skinned constraint needs a positive weight";
const WEIGHT_INDEX_RULE: &str = "a positive skin weight must name an inverse bind";
const NORMALIZED_WEIGHT_RULE: &str = "a positive skin weight must stay above 0 once normalised";
const MAX_DISTANCE_RULE: &str =
    "a max distance must be finite and within 0..=limits::MAX_SHAPE_EXTENT";
const BACK_STOP_DISTANCE_RULE: &str =
    "a back stop distance must be finite and within 0..=limits::MAX_SHAPE_EXTENT";
const BACK_STOP_RADIUS_RULE: &str =
    "a back stop radius must be above 0 and at most limits::MAX_SHAPE_EXTENT";
const BACK_STOP_ORDER_RULE: &str = "a back stop distance must be below the max distance";
const BACK_STOP_FACE_RULE: &str = "a back stop needs a face whose three vertices are skinned";
const FACES_PER_VERTEX_RULE: &str =
    "a skinned vertex must touch fewer than 256 faces whose vertices are all skinned";
const FACE_TOTAL_RULE: &str =
    "the skinned vertices must touch fewer than 2^24 all-skinned faces in total";
const POSE_RULE: &str =
    "a bind pose must be finite with unit rotations and within limits::MAX_POSITION";

/// Jolt's default back stop radius, passed when a constraint has no back stop.
const DEFAULT_BACK_STOP_RADIUS: f32 = 40.0;

/// The bind pose of one joint (Jolt `SoftBodySharedSettings::InvBind`): where the joint is in
/// the frame of the vertex positions ([`SoftBodyVertex::position`](super::SoftBodyVertex::position))
/// while the vertices are at those positions. The settings store its inverse, which takes a
/// vertex from its bind position into the joint's local space.
///
/// Only rigid binds are taken (no scale or shear), and each joint has at most one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyInvBind {
    /// The joint, an index into the joint transforms of every skinning call, below
    /// [`limits::MAX_SKIN_JOINTS`].
    pub joint: u32,
    /// The joint's position in the settings frame, in metres, every component at most
    /// [`limits::MAX_SHAPE_EXTENT`] in absolute value.
    pub position: Vec3,
    /// The joint's rotation in the settings frame, a unit quaternion.
    pub rotation: Quat,
}

impl SoftBodyInvBind {
    /// The bind pose of `joint` at `position` with `rotation`, in the settings frame.
    pub fn new(joint: u32, position: Vec3, rotation: Quat) -> Self {
        Self {
            joint,
            position,
            rotation,
        }
    }

    /// One bind per joint of `pose`, in joint order, for settings whose frame is at the world
    /// `position` with `rotation` while the joints are in `pose`: for a cape made at a
    /// character's shoulders, the pose the character has and the place the cape's vertex
    /// frame has at that moment. `pose` is a world pose as
    /// [`RagdollRef::pose`](crate::RagdollRef::pose) returns it.
    ///
    /// The positions are converted in double precision with the `double-precision` feature.
    /// Fails with [`SoftBodyError::InvalidValue`] when `pose` or the frame is not finite, has a
    /// rotation that is not a unit quaternion or a position beyond [`limits::MAX_POSITION`], or
    /// a joint is further than [`limits::MAX_SHAPE_EXTENT`] (per axis) from `position`.
    pub fn from_pose(
        pose: &SkeletonPose,
        position: RVec3,
        rotation: Quat,
    ) -> Result<Vec<Self>, SoftBodyError> {
        require(is_in_frame(position), limits::POSITION_RULE)?;
        require(rotation.is_valid_rotation(), ROTATION_RULE)?;
        require(is_in_frame(pose.root_offset), POSE_RULE)?;
        let inverse = rotation.normalized().conjugated();
        pose.joints
            .iter()
            .enumerate()
            .map(|(index, transform)| {
                require(
                    transform.translation.is_finite() && transform.rotation.is_valid_rotation(),
                    POSE_RULE,
                )?;
                let world = pose.position(index);
                require(is_in_frame(world), POSE_RULE)?;
                let local = inverse.rotate_real(RVec3::new(
                    world.x - position.x,
                    world.y - position.y,
                    world.z - position.z,
                ));
                // `Real` is `f32` without the `double-precision` feature, so the casts are
                // no-ops there.
                #[allow(clippy::unnecessary_cast)]
                let local = Vec3::new(local.x as f32, local.y as f32, local.z as f32);
                require(is_local_offset(local), BIND_POSITION_RULE)?;
                let joint =
                    u32::try_from(index).map_err(|_| SoftBodyError::InvalidValue(JOINT_RULE))?;
                Ok(Self {
                    joint,
                    position: local,
                    rotation: inverse.product(transform.rotation).normalized(),
                })
            })
            .collect()
    }
}

/// One joint's share of a skinned vertex (Jolt `SoftBodySharedSettings::SkinWeight`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SoftBodySkinWeight {
    /// Index of the inverse bind, in the order the binds were added to the builder.
    pub inv_bind: u32,
    /// Weight, finite and not negative; 0 ignores this entry.
    pub weight: f32,
}

impl SoftBodySkinWeight {
    /// The weight `weight` of inverse bind `inv_bind`.
    pub fn new(inv_bind: u32, weight: f32) -> Self {
        Self { inv_bind, weight }
    }
}

/// A sphere behind a skinned vertex that the simulated vertex is pushed out of, standing in
/// for the body under a cape.
///
/// The sphere's surface is `distance` behind the skinned position, against the average normal
/// of the faces around the vertex whose three vertices are all skinned; its centre is
/// `distance + radius` behind it. A large radius approximates a plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyBackStop {
    /// Distance behind the skinned position, in metres, finite and within
    /// `0..=`[`limits::MAX_SHAPE_EXTENT`], below the constraint's max distance.
    pub distance: f32,
    /// Radius in metres, above 0 and at most [`limits::MAX_SHAPE_EXTENT`]. Jolt's default is 40.
    pub radius: f32,
}

/// A constraint that skins one vertex to joints (Jolt `SoftBodySharedSettings::Skinned`):
/// each skinning call moves the vertex's skinned position with the joints, and the simulated
/// vertex is kept within `max_distance` of it and outside its back stop.
///
/// The vertex's bind position is its [`SoftBodyVertex::position`](super::SoftBodyVertex::position).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodySkinned {
    /// The vertex.
    pub vertex: u32,
    /// Up to four joint weights. Zero weights are ignored wherever they are; the positive
    /// ones are normalised to add up to 1 and keep their order.
    pub weights: [SoftBodySkinWeight; 4],
    /// How far the simulated vertex may move from its skinned position, in metres, finite and
    /// within `0..=`[`limits::MAX_SHAPE_EXTENT`]: `Some(0.0)` attaches it to the skin, `None`
    /// does not limit it.
    pub max_distance: Option<f32>,
    /// The back stop, if any.
    pub back_stop: Option<SoftBodyBackStop>,
}

impl SoftBodySkinned {
    /// Skins `vertex` with `weights`, without a max distance or a back stop.
    pub fn new(vertex: u32, weights: [SoftBodySkinWeight; 4]) -> Self {
        Self {
            vertex,
            weights,
            max_distance: None,
            back_stop: None,
        }
    }
}

/// The validated inverse binds of skinned settings, kept to put a new body's skin at its bind
/// pose.
#[derive(Clone, Debug)]
pub(super) struct SkinBinding {
    /// Joint, position and rotation of each bind, in bind order.
    pub(super) binds: Vec<(u32, Vec3, Quat)>,
    /// 1 + the largest bound joint.
    pub(super) joint_count: u32,
}

impl SkinBinding {
    /// The joint matrices of the bind pose relative to a new body's centre of mass, which is
    /// the settings frame, turned by `baked_rotation` when the body bakes its rotation into the
    /// vertices; identity for joints without a bind.
    pub(super) fn bind_pose_matrices(&self, baked_rotation: Option<Quat>) -> Vec<JPH_Mat4> {
        let mut joints = vec![matrix(Vec3::ZERO, Quat::IDENTITY); self.joint_count as usize];
        for &(joint, position, rotation) in &self.binds {
            joints[joint as usize] = match baked_rotation {
                Some(baked) => {
                    let baked = baked.normalized();
                    matrix(baked.rotate(position), baked.product(rotation))
                }
                None => matrix(position, rotation),
            };
        }
        joints
    }
}

/// Skinning inputs ready for Jolt.
pub(super) struct ValidatedSkin {
    pub(super) binding: SkinBinding,
    pub(super) constraints: Vec<JPH_SoftBodySkinnedConstraint>,
}

/// Checks the skinning inputs of settings with `vertex_count` vertices and `faces`, and
/// converts the constraints with normalised weights. `None` without skinning.
pub(super) fn validate_skin(
    vertex_count: usize,
    faces: &[[u32; 3]],
    inv_binds: &[SoftBodyInvBind],
    skinned: &[SoftBodySkinned],
) -> Result<Option<ValidatedSkin>, SoftBodyError> {
    require(inv_binds.is_empty() == skinned.is_empty(), PAIRING_RULE)?;
    if inv_binds.is_empty() {
        return Ok(None);
    }
    let binding = validate_inv_binds(inv_binds)?;
    let mut is_skinned = vec![false; vertex_count];
    for constraint in skinned {
        let vertex = constraint.vertex as usize;
        require(vertex < vertex_count, VERTEX_RULE)?;
        require(!is_skinned[vertex], UNIQUE_VERTEX_RULE)?;
        is_skinned[vertex] = true;
    }
    let skinned_faces = skinned_face_counts(&is_skinned, faces);
    let mut total_faces = 0_u64;
    let constraints = skinned
        .iter()
        .map(|constraint| {
            let faces = skinned_faces[constraint.vertex as usize];
            require(faces < 256, FACES_PER_VERTEX_RULE)?;
            total_faces += u64::from(faces);
            to_jph(constraint, inv_binds.len(), faces > 0)
        })
        .collect::<Result<Vec<_>, _>>()?;
    require(total_faces < 1 << 24, FACE_TOTAL_RULE)?;
    Ok(Some(ValidatedSkin {
        binding,
        constraints,
    }))
}

fn validate_inv_binds(inv_binds: &[SoftBodyInvBind]) -> Result<SkinBinding, SoftBodyError> {
    let mut joints = BTreeSet::new();
    for bind in inv_binds {
        require(bind.joint < MAX_SKIN_JOINTS, JOINT_RULE)?;
        require(joints.insert(bind.joint), UNIQUE_JOINT_RULE)?;
        require(is_local_offset(bind.position), BIND_POSITION_RULE)?;
        require(bind.rotation.is_valid_rotation(), ROTATION_RULE)?;
    }
    Ok(SkinBinding {
        binds: inv_binds
            .iter()
            .map(|bind| (bind.joint, bind.position, bind.rotation))
            .collect(),
        joint_count: joints.last().map_or(0, |&joint| joint + 1),
    })
}

/// For each vertex, the number of faces whose three vertices are all skinned that touch it, as
/// Jolt's `CalculateSkinnedConstraintNormals` collects them.
fn skinned_face_counts(is_skinned: &[bool], faces: &[[u32; 3]]) -> Vec<u32> {
    let mut counts = vec![0_u32; is_skinned.len()];
    for face in faces {
        if face.iter().all(|&vertex| is_skinned[vertex as usize]) {
            for &vertex in face {
                counts[vertex as usize] += 1;
            }
        }
    }
    counts
}

/// Checks one constraint and converts it; `has_skinned_face` says whether the vertex has a
/// normal for a back stop.
fn to_jph(
    constraint: &SoftBodySkinned,
    inv_bind_count: usize,
    has_skinned_face: bool,
) -> Result<JPH_SoftBodySkinnedConstraint, SoftBodyError> {
    let weights = normalized_weights(&constraint.weights, inv_bind_count)?;
    if let Some(max_distance) = constraint.max_distance {
        require(is_local_distance(max_distance), MAX_DISTANCE_RULE)?;
    }
    let (back_stop_distance, back_stop_radius) = match constraint.back_stop {
        Some(back_stop) => {
            require(
                is_local_distance(back_stop.distance),
                BACK_STOP_DISTANCE_RULE,
            )?;
            require(
                back_stop.radius > 0.0 && back_stop.radius <= limits::MAX_SHAPE_EXTENT,
                BACK_STOP_RADIUS_RULE,
            )?;
            // Jolt ignores a back stop that starts at or beyond the max distance.
            require(
                constraint
                    .max_distance
                    .is_none_or(|max_distance| back_stop.distance < max_distance),
                BACK_STOP_ORDER_RULE,
            )?;
            // Without such a face the normal is zero and the sphere sits on the skin position.
            require(has_skinned_face, BACK_STOP_FACE_RULE)?;
            (back_stop.distance, back_stop.radius)
        }
        // Jolt's defaults: `FLT_MAX` disables the back stop.
        None => (f32::MAX, DEFAULT_BACK_STOP_RADIUS),
    };
    Ok(JPH_SoftBodySkinnedConstraint {
        vertex: constraint.vertex,
        weights,
        maxDistance: constraint.max_distance.unwrap_or(f32::MAX),
        backStopDistance: back_stop_distance,
        backStopRadius: back_stop_radius,
    })
}

/// The positive weights divided by their sum in `f64`, in their order, followed by zeros: Jolt
/// stops at the first zero weight and expects the weights to add up to 1, and its own `f32`
/// sum would overflow for large weights.
fn normalized_weights(
    weights: &[SoftBodySkinWeight; 4],
    inv_bind_count: usize,
) -> Result<[JPH_SoftBodySkinWeight; 4], SoftBodyError> {
    require(
        weights
            .iter()
            .all(|w| w.weight.is_finite() && w.weight >= 0.0),
        WEIGHT_RULE,
    )?;
    let positive: Vec<SoftBodySkinWeight> =
        weights.iter().copied().filter(|w| w.weight > 0.0).collect();
    require(!positive.is_empty(), POSITIVE_WEIGHT_RULE)?;
    require(
        positive
            .iter()
            .all(|w| (w.inv_bind as usize) < inv_bind_count),
        WEIGHT_INDEX_RULE,
    )?;
    let total: f64 = positive.iter().map(|w| f64::from(w.weight)).sum();
    let mut result = [JPH_SoftBodySkinWeight {
        invBindIndex: 0,
        weight: 0.0,
    }; 4];
    for (slot, w) in result.iter_mut().zip(&positive) {
        let weight = (f64::from(w.weight) / total) as f32;
        require(weight > 0.0, NORMALIZED_WEIGHT_RULE)?;
        *slot = JPH_SoftBodySkinWeight {
            invBindIndex: w.inv_bind,
            weight,
        };
    }
    Ok(result)
}

#[cfg(test)]
mod tests;
