//! Soft bodies: shared particle settings, creation, and read and write access to vertices.
//!
//! A soft body is an ordinary body of its world: [`PhysicsWorld::create_soft_body`] returns a
//! [`BodyId`], and the body counts against the world's body limit, is removed with
//! [`PhysicsWorld::remove_body`], saved by [`PhysicsWorld::save_state`], moved by
//! [`PhysicsWorld::rebase`] and found by queries. Its particles are described by
//! [`SoftBodySharedSettings`], which one or many soft bodies in any number of worlds share.
//!
//! [`PhysicsWorld::create_soft_body`]: crate::PhysicsWorld::create_soft_body
//! [`BodyId`]: crate::BodyId
//! [`PhysicsWorld::remove_body`]: crate::PhysicsWorld::remove_body
//! [`PhysicsWorld::save_state`]: crate::PhysicsWorld::save_state
//! [`PhysicsWorld::rebase`]: crate::PhysicsWorld::rebase

use oxijolt_sys::*;

use crate::limits::{self, is_compliance};
use crate::{SoftBodyError, Vec3};

mod handle;
mod settings;
mod shared;
mod skin;

pub use handle::{SoftBodyMut, SoftBodyRef, SoftBodyVertexState};
pub use settings::SoftBodySettings;
use shared::{require, COMPLIANCE_RULE};
pub use shared::{
    SoftBodyDihedralBend, SoftBodyEdge, SoftBodySharedSettings, SoftBodySharedSettingsBuilder,
    SoftBodyVolume,
};
pub use skin::{SoftBodyBackStop, SoftBodyInvBind, SoftBodySkinWeight, SoftBodySkinned};

/// One particle of a soft body as [`SoftBodySharedSettings`] describe it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyVertex {
    /// Position relative to the body origin, in metres, every component at most
    /// [`limits::MAX_SHAPE_EXTENT`] in absolute value. The vertices of a body without a
    /// kinematic vertex belong around the origin (see [`PhysicsWorld::create_soft_body`]).
    ///
    /// [`PhysicsWorld::create_soft_body`]: crate::PhysicsWorld::create_soft_body
    pub position: Vec3,
    /// Initial velocity relative to the body, in m/s, at most
    /// [`limits::MAX_LINEAR_VELOCITY`] long.
    pub velocity: Vec3,
    /// Inverse mass in 1/kg: 0 for a kinematic vertex, which only moves by its velocity, or the
    /// inverse of a mass within [`limits::MIN_MASS`]`..=`[`limits::MAX_MASS`], at most
    /// [`limits::MAX_VERTEX_INVERSE_MASS`] (1000).
    ///
    /// The 1 g floor applies per vertex, so a light, finely divided body is refused at its real
    /// masses: a 1 m² cotton cloth of 0.15 kg at 21 × 21 vertices (0.34 g each), a 2 m flag of
    /// 0.4 kg at 30 × 30, or a 57 g ball of 162 vertices. Use fewer vertices or give each at
    /// least 1 g. Gravity and damping move a vertex the same at any mass; pressure and added
    /// forces move a heavier vertex less, and a heavier body pushes the rigid bodies it
    /// touches harder.
    pub inverse_mass: f32,
}

impl SoftBodyVertex {
    /// A vertex of 1 kg at rest at `position`, Jolt's default vertex.
    pub fn new(position: Vec3) -> Self {
        Self {
            position,
            velocity: Vec3::ZERO,
            inverse_mass: 1.0,
        }
    }

    /// A kinematic vertex (inverse mass 0) at rest at `position`: it keeps its place unless
    /// it is given a velocity.
    pub fn kinematic(position: Vec3) -> Self {
        Self {
            inverse_mass: 0.0,
            ..Self::new(position)
        }
    }

    fn to_jph(self) -> JPH_SoftVertex {
        JPH_SoftVertex {
            position: self.position.to_jph(),
            velocity: self.velocity.to_jph(),
            invMass: self.inverse_mass,
        }
    }
}

/// Which bend constraints [`SoftBodySharedSettingsBuilder::create_constraints`] creates between
/// two faces that share an edge (Jolt `SoftBodySharedSettings::EBendType`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SoftBodyBendType {
    /// No bend constraints.
    None,
    /// An edge between the two vertices opposite the shared edge, the cheapest.
    Distance,
    /// A dihedral angle constraint, which also keeps a fold between faces that start out of
    /// one plane; the most expensive.
    Dihedral,
}

impl SoftBodyBendType {
    fn to_jph(self) -> JPH_SoftBodyBendType {
        match self {
            Self::None => JPH_SoftBodyBendType_None,
            Self::Distance => JPH_SoftBodyBendType_Distance,
            Self::Dihedral => JPH_SoftBodyBendType_Dihedral,
        }
    }
}

/// Which long range attachment (LRA) constraint ties a movable vertex to the closest kinematic
/// vertex (Jolt `SoftBodySharedSettings::ELRAType`). An LRA constraint keeps the vertex within
/// its rest distance from that kinematic vertex times a multiplier, so a hanging cloth does not
/// stretch under its own weight. The anchors are chosen when the settings are built.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LongRangeAttachment {
    /// No LRA constraint.
    None,
    /// The rest distance is the straight-line distance to the closest kinematic vertex.
    EuclideanDistance,
    /// The rest distance is measured along the edges to the closest kinematic vertex.
    GeodesicDistance,
}

impl LongRangeAttachment {
    fn to_jph(self) -> JPH_SoftBodyLRAType {
        match self {
            Self::None => JPH_SoftBodyLRAType_None,
            Self::EuclideanDistance => JPH_SoftBodyLRAType_EuclideanDistance,
            Self::GeodesicDistance => JPH_SoftBodyLRAType_GeodesicDistance,
        }
    }
}

/// How [`SoftBodySharedSettingsBuilder::create_constraints`] builds the constraints at a vertex
/// (Jolt `SoftBodySharedSettings::VertexAttributes`).
///
/// Compliances are inverse stiffnesses in the units of each constraint's own equation (an edge:
/// metres per newton; bend and volume constraints differ); 0 is rigid. Each must be finite and
/// within `0..=`[`limits::MAX_COMPLIANCE`]. An edge or shear edge uses the average of its two
/// vertices' compliances, a bend constraint the average over the shared edge.
///
/// The defaults are Jolt's: rigid edges and shear edges, no bend constraints, no LRA
/// constraint and an LRA multiplier of 1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyVertexAttributes {
    compliance: f32,
    shear_compliance: f32,
    bend_compliance: Option<f32>,
    long_range_attachment: LongRangeAttachment,
    lra_multiplier: f32,
}

impl Default for SoftBodyVertexAttributes {
    fn default() -> Self {
        Self {
            compliance: 0.0,
            shear_compliance: 0.0,
            bend_compliance: None,
            long_range_attachment: LongRangeAttachment::None,
            lra_multiplier: 1.0,
        }
    }
}

impl SoftBodyVertexAttributes {
    /// Compliance of the edges along the faces. Default 0.
    #[must_use]
    pub fn compliance(mut self, value: f32) -> Self {
        self.compliance = value;
        self
    }

    /// Compliance of the shear edges, the diagonals of two faces that form a quad. Default 0.
    #[must_use]
    pub fn shear_compliance(mut self, value: f32) -> Self {
        self.shear_compliance = value;
        self
    }

    /// Compliance of the bend constraints; `None` creates none for edges at this vertex (Jolt's
    /// `FLT_MAX`). Default `None`.
    #[must_use]
    pub fn bend_compliance(mut self, value: Option<f32>) -> Self {
        self.bend_compliance = value;
        self
    }

    /// The LRA constraint of this vertex and its multiplier of the rest distance, finite and
    /// within `1..=`[`limits::MAX_RATIO`] (1.01 lets the vertex move 1 % further away).
    /// Default [`LongRangeAttachment::None`] with multiplier 1.
    ///
    /// The bound keeps Jolt's square of the stretched rest distance finite
    /// ([docs/limits.md#soft-body-long-range-attachments]).
    ///
    /// [docs/limits.md#soft-body-long-range-attachments]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-long-range-attachments
    #[must_use]
    pub fn long_range_attachment(mut self, kind: LongRangeAttachment, multiplier: f32) -> Self {
        self.long_range_attachment = kind;
        self.lra_multiplier = multiplier;
        self
    }

    fn validate(&self) -> Result<(), SoftBodyError> {
        let compliances = [self.compliance, self.shear_compliance];
        require(
            compliances
                .into_iter()
                .chain(self.bend_compliance)
                .all(is_compliance),
            COMPLIANCE_RULE,
        )?;
        require(
            (1.0..=limits::MAX_RATIO).contains(&self.lra_multiplier),
            "an LRA multiplier must be finite and within 1..=limits::MAX_RATIO",
        )
    }

    fn to_jph(self) -> JPH_SoftBodyVertexAttributes {
        JPH_SoftBodyVertexAttributes {
            compliance: self.compliance,
            shearCompliance: self.shear_compliance,
            bendCompliance: self.bend_compliance.unwrap_or(f32::MAX),
            lraType: self.long_range_attachment.to_jph(),
            lraMaxDistanceMultiplier: self.lra_multiplier,
        }
    }
}

#[cfg(test)]
mod tests;
