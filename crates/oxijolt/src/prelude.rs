//! The types most programs use, for `use oxijolt::prelude::*`.
//!
//! Without another prelude, import the math types from [`math`] next to it:
//!
//! ```
//! use oxijolt::prelude::math::*;
//! use oxijolt::prelude::*;
//!
//! # fn main() -> oxijolt::error::Result<()> {
//! let mut world = PhysicsWorld::new(WorldSettings::default())?;
//! let ball = world.create_body(
//!     &Shape::new_sphere(0.5)?,
//!     &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
//! )?;
//! assert!(world.step(1.0 / 60.0)?.is_complete());
//! assert!(world.body(ball)?.position().y < 2.0);
//! # Ok(())
//! # }
//! ```
//!
//! The prelude names no math type and no `Result`, so it can be glob-imported next to
//! `bevy::prelude::*`: `Vec3`, `Quat` and `Result` stay Bevy's. With the glam feature of
//! Bevy's glam version (`glam033` for Bevy 0.20), `.into()` converts poses to Bevy's types; `?`
//! in a system that returns Bevy's `Result` converts any oxijolt error.
//!
//! ```ignore
//! use bevy::prelude::*;
//! use oxijolt::prelude::*;
//!
//! #[derive(Resource)]
//! struct Physics(PhysicsWorld);
//!
//! #[derive(Resource, Default)]
//! struct BodyEntities(std::collections::HashMap<BodyId, Entity>);
//!
//! fn sync_transforms(
//!     physics: Res<Physics>,
//!     bodies: Res<BodyEntities>,
//!     mut transforms: Query<&mut Transform>,
//! ) {
//!     for pose in physics.0.active_body_poses() {
//!         let entity = bodies.0.get(&pose.id);
//!         if let Some(mut transform) = entity.and_then(|&e| transforms.get_mut(e).ok()) {
//!             transform.translation = pose.position.into();
//!             transform.rotation = pose.rotation.into();
//!         }
//!     }
//! }
//!
//! fn step(mut physics: ResMut<Physics>, time: Res<Time<Fixed>>) -> Result {
//!     let report = physics.0.step(time.delta_secs())?;
//!     assert!(report.is_complete());
//!     Ok(())
//! }
//! ```
//!
//! `pose.position.into()` gives a `glam::Vec3` in single precision; with `double-precision` it
//! gives a `glam::DVec3`.
//!
//! `active_body_poses` returns the bodies awake when it is called; a body that fell asleep at
//! the end of the step is not among them. [`PhysicsWorld::active_body_poses`] says how a game
//! syncs such bodies.
//!
//! [`Error`](crate::error::Error) and [`Result`](crate::error::Result) are written by path
//! (`oxijolt::error::Result`). The crate root, `use oxijolt::*`, brings in everything else, the
//! math types included.

pub use crate::MutableCompound;
pub use crate::{
    Activation, AllowedDofs, BodyId, BodyMut, BodyPose, BodyRef, BodySettings, Inertia,
    MotionQuality, MotionType,
};
pub use crate::{ActivationEvent, ContactEvent, ContactListener, EventSettings, WorldEvents};
pub use crate::{AnyConstraintId, ConstraintId};
pub use crate::{AnyVehicleId, DriverInput, VehicleId, WheelSettings, WheeledVehicleSettings};
pub use crate::{
    BinaryStateError, BodyError, CharacterError, CollisionGroupError, ConstraintError,
    ContactSettingsError, ConvexHullError, MeshError, QueryError, RagdollError, ShapeError,
    SoftBodyError, StateError, StepError, ThinTrianglesError, VehicleError, WorldError,
};
pub use crate::{BodySelection, PhysicsWorld, StepReport, WorldSettings, WorldState};
pub use crate::{BroadPhaseLayer, CollisionLayers, ObjectLayer};
pub use crate::{CharacterId, CharacterSettings, ExtendedUpdateSettings};
pub use crate::{
    CollidePointHit, CollideShape, CollideShapeHit, QueryFilter, RayCast, RayCastHit, ShapeCast,
};
pub use crate::{CompoundChild, HeightFieldSettings, PhysicsMaterial, Shape, SubShapeId};
pub use crate::{DroppedTriangles, MeshBuildQuality, MeshSettings};
pub use crate::{RagdollId, RagdollSettings, Skeleton, SkeletonMapper};
pub use crate::{ShapeCastHit, SoftBodySettings, SoftBodySharedSettings};

/// The math types: [`Vec3`](crate::Vec3), [`RVec3`](crate::RVec3), [`Quat`](crate::Quat) and
/// [`Real`](crate::Real).
pub mod math {
    pub use crate::{Quat, RVec3, Real, Vec3};
}
