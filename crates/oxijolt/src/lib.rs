//! Safe Rust API for [Jolt Physics](https://github.com/jrouwe/JoltPhysics) 5.6 over the
//! [joltc](https://github.com/amerkoleci/joltc) raw layer in
//! [`oxijolt-sys`](https://github.com/pockerhead/oxijolt/tree/main/crates/oxijolt-sys).
//!
//! A [`PhysicsWorld`] owns a Jolt physics system with its collision layers
//! ([`CollisionLayers`]), its job system and temp allocator. Bodies are created from a [`Shape`]
//! and [`BodySettings`] and named by a [`BodyId`]; one shape may serve many bodies in many worlds.
//!
//! ```
//! use oxijolt::prelude::math::*;
//! use oxijolt::prelude::*;
//!
//! # fn main() -> oxijolt::error::Result<()> {
//! let mut world = PhysicsWorld::new(WorldSettings::default())?;
//!
//! let floor_shape = Shape::new_box(Vec3::new(100.0, 1.0, 100.0))?;
//! let floor = BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0));
//! world.create_body(&floor_shape, &floor)?;
//!
//! let ball_shape = Shape::new_sphere(0.5)?;
//! let ball = world.create_body(
//!     &ball_shape,
//!     &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
//! )?;
//!
//! for _ in 0..60 {
//!     let report = world.step(1.0 / 60.0)?;
//!     assert!(report.is_complete());
//! }
//! assert!(world.body(ball)?.position().y < 2.0);
//! # Ok(())
//! # }
//! ```
//!
//! # What a world holds
//! - Rigid bodies of box, sphere, cylinder, capsule, tapered capsule and cylinder, convex hull,
//!   triangle mesh, heightfield, compound and scaled [`Shape`]s, with [`PhysicsMaterial`]s that
//!   carry the caller's user data. [`BodyMut`] adds impulses, kinematic moves, waking and
//!   sleeping, and shape and motion type changes; [`BodySettings`] makes sensors and sets user
//!   data and locked axes ([body controls guide]).
//! - Scene queries on `&PhysicsWorld`: [`PhysicsWorld::cast_ray`], [`PhysicsWorld::cast_shape`]
//!   and [`PhysicsWorld::collide_shape`], filtered by [`QueryFilter`].
//! - Virtual characters: [`PhysicsWorld::create_character`], with a standing humanoid from
//!   [`CharacterSettings::humanoid`].
//! - Wheeled and tracked vehicles and motorcycles on a chassis body:
//!   [`PhysicsWorld::create_wheeled_vehicle`], [`PhysicsWorld::create_tracked_vehicle`] and
//!   [`PhysicsWorld::create_motorcycle`], with ready settings from [`WheeledVehicleSettings::car`]
//!   and [`MotorcycleSettings::bike`] and wheel poses for drawing from
//!   [`VehicleRef::wheel_world_transform`].
//! - Ragdolls: [`PhysicsWorld::create_ragdoll`].
//! - Constraints of twelve kinds between two bodies: [`PhysicsWorld::create_constraint`]
//!   ([constraints guide]).
//! - Soft bodies: [`PhysicsWorld::create_soft_body`] ([soft body guide]).
//! - Contact, activation and soft body contact events and a [`ContactListener`]:
//!   [`PhysicsWorld::set_event_settings`], [`PhysicsWorld::take_events`] ([events guide]).
//! - Snapshots for rollback: [`PhysicsWorld::save_state`], [`PhysicsWorld::restore_state`]
//!   ([state guide]).
//! - A floating origin: [`PhysicsWorld::rebase`].
//! - With the `debug-renderer` feature, the colliders' wireframe as line data
//!   (`PhysicsWorld::debug_lines`); nothing is drawn.
//!
//! The [guide] builds a game scene with terrain, queries, a character, a vehicle and a ragdoll.
//!
//! # Contract
//! - **Errors.** A call that returns `Err` was refused and changed nothing, unless its
//!   documentation says otherwise. A step that ran but dropped work returns `Ok`, and its
//!   [`StepReport`] says what was dropped. Code that calls several areas can return
//!   [`error::Result`]; `?` converts every area error into [`error::Error`].
//! - **Magnitudes.** Positions, extents, velocities, masses and the other inputs [`limits`]
//!   bounds are checked against it before they reach Jolt. Some inputs, such as damping and ray
//!   directions, are only checked to be finite. Some bounds are derived from Jolt's arithmetic
//!   ([limits doc]), some are only tested at the bound, and some cases no bound excludes
//!   ([coverage]).
//! - **Threads.** Changing a world, [`PhysicsWorld::step`] included, takes `&mut PhysicsWorld`;
//!   reading it takes `&PhysicsWorld`. `PhysicsWorld` is `Send` and `Sync`: many threads may read
//!   one world while nobody steps it, and different worlds may step on different threads at the
//!   same time. A world runs Jolt's jobs on Jolt's thread pool
//!   ([`WorldSettings::worker_threads`]) or on the caller's pool through a [`JobSystem`]
//!   ([job system guide]).
//! - **Determinism.** The order of calls, including the order of body creation and removal, is
//!   part of the state. The tests check bit-identical results with 1 and 4 worker threads and
//!   with caller job systems on one machine; results agree across platforms and compilers only
//!   with the `cross-platform-deterministic` feature ([determinism]).
//!
//! # Global state
//! oxijolt calls `JPH_Init` once per process and never calls `JPH_Shutdown`. It installs
//! joltc's process-global procs once and owns them: the object-layer, body and shape filters,
//! the contact, body activation and soft body contact listeners (the first time a world records
//! events), the debug renderer (with `debug-renderer`, which also serializes debug drawing) and
//! Jolt's assertion handler (before `JPH_Init`). Code that also uses `oxijolt-sys` directly must
//! leave them alone; see the notes on the raw API in `oxijolt-sys`.
//!
//! With the `asserts` feature, which compiles Jolt with its debug assertions, a failed assertion
//! prints its expression, message, file and line to stderr and aborts the process. The only
//! exception is the physics-update-error assertion, whose condition [`PhysicsWorld::step`]
//! returns in its [`StepReport`].
//!
//! # Features
//! Every feature adds to the API except `double-precision`, which changes [`Real`] in every
//! signature: only the final application enables it, and libraries built on oxijolt forward it.
//! `bindgen` is a build switch outside the semver promise.
//!
//! - `double-precision`: world positions ([`Real`], [`RVec3`]) use `f64`.
//! - `cross-platform-deterministic`: Jolt's cross-platform deterministic floating point settings.
//! - `debug-renderer`: collider wireframes as line data, see above.
//! - `asserts`: Jolt's debug assertions, reported as above.
//! - `bindgen`: generates the raw bindings with libclang at build time.
//! - `glam032`: `From` conversions both ways between [`Vec3`], [`Quat`], [`RVec3`] and glam
//!   0.32's `Vec3`, `Quat` and vector of [`Real`] (`DVec3` in double precision).
//! - `glam033`: the same conversions for glam 0.33, the version Bevy 0.20 uses. Both glam
//!   features can be enabled together.
//! - `mint`: `From` conversions both ways between [`Vec3`], [`Quat`], [`RVec3`] and mint's
//!   `Vector3<f32>`, `Quaternion<f32>` and `Vector3<Real>`.
//!
//! The conversions copy the fields: a value converted there and back has the same bits, and
//! nothing is normalized or checked until a call takes the value.
//!
//! ```
//! # #[cfg(feature = "glam033")]
//! # use glam033 as glam;
//! # #[cfg(any(feature = "glam032", feature = "glam033"))]
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! use oxijolt::{BodySettings, PhysicsWorld, Shape, WorldSettings};
//!
//! let mut world = PhysicsWorld::new(WorldSettings::default())?;
//! let ball = world.create_body(
//!     &Shape::new_sphere(0.5)?,
//!     &BodySettings::new_dynamic()
//!         .rotation(glam::Quat::from_rotation_y(1.0).into())
//!         .linear_velocity(glam::Vec3::new(1.0, 0.0, 0.0).into()),
//! )?;
//! let velocity: glam::Vec3 = world.body(ball)?.linear_velocity().into();
//! assert_eq!(velocity, glam::Vec3::X);
//! # Ok(())
//! # }
//! # #[cfg(not(any(feature = "glam032", feature = "glam033")))]
//! # fn main() {}
//! ```
//!
//! [guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/guide.md
//! [body controls guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/bodies.md
//! [constraints guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/constraints.md
//! [soft body guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/soft-bodies.md
//! [events guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/events.md
//! [state guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md
//! [job system guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/job-system.md
//! [determinism]: https://github.com/pockerhead/oxijolt/blob/main/docs/determinism.md
//! [limits doc]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md
//! [coverage]: https://github.com/pockerhead/oxijolt/blob/main/docs/coverage.md#not-covered
#![warn(
    missing_docs,
    missing_debug_implementations,
    unsafe_op_in_unsafe_fn,
    clippy::undocumented_unsafe_blocks,
    clippy::missing_safety_doc
)]
// Examples, including the README and the guides, are code users copy: they must build clean.
#![doc(test(attr(deny(warnings))))]

mod body;
mod character;
mod collision_group;
mod constraint;
#[cfg(feature = "debug-renderer")]
mod debug;
pub mod error;
mod filter;
#[cfg(any(feature = "glam032", feature = "glam033"))]
mod glam_interop;
mod job_system;
mod jolt_assert;
mod layers;
pub mod limits;
mod listener;
mod material;
mod math;
#[cfg(feature = "mint")]
mod mint_interop;
mod owned;
mod panic_slot;
pub mod prelude;
mod query;
mod ragdoll;
mod shape;
mod soft_body;
mod state;
mod vehicle;
mod world;

pub use body::{
    Activation, AllowedDofs, BodyId, BodyMut, BodyPose, BodyRef, BodySettings, BuoyancySettings,
    Inertia, MotionQuality, MotionType,
};
pub use character::{
    BodyVelocity, CharacterContact, CharacterContactKey, CharacterContactListener,
    CharacterContactSettings, CharacterId, CharacterMut, CharacterRef, CharacterSettings,
    CharacterState, ExtendedUpdateSettings, GroundState, InnerBody,
};
pub use collision_group::{CollisionGroup, GroupFilterTable, GroupFilterTableBuilder};
pub use constraint::{
    AnyConstraintId, ConeConstraint, ConeConstraintSettings, ConstraintId, ConstraintKind,
    ConstraintMut, ConstraintRef, ConstraintSettings, ConstraintSpace, ConstraintType,
    DistanceConstraint, DistanceConstraintSettings, DistanceRange, FixedConstraint,
    FixedConstraintSettings, GearConstraint, GearConstraintSettings, HermitePath, HermitePathPoint,
    HingeConstraint, HingeConstraintSettings, MotorSettings, MotorState, PathConstraint,
    PathConstraintSettings, PathRotationConstraint, PointConstraint, PointConstraintSettings,
    PulleyConstraint, PulleyConstraintSettings, PulleyLength, RackAndPinionConstraint,
    RackAndPinionConstraintSettings, SixDofAxis, SixDofConstraint, SixDofConstraintAxis,
    SixDofConstraintSettings, SliderConstraint, SliderConstraintSettings, SpringSettings,
    SwingTwistConstraint, SwingTwistConstraintSettings, SwingType,
};
#[cfg(feature = "debug-renderer")]
pub use debug::{DebugLine, DebugLineSettings, DebugLines};
pub use error::{
    BinaryStateError, BodyError, CharacterError, CollisionGroupError, ConstraintError,
    ContactSettingsError, ConvexHullError, MeshError, QueryError, RagdollError, ShapeError,
    SoftBodyError, StateError, StepError, ThinTrianglesError, VehicleError, WorldError,
};
pub use filter::QueryFilter;
pub use job_system::{Job, JobSystem};
pub use layers::{BroadPhaseLayer, CollisionLayers, ObjectLayer};
pub use listener::{
    ActivationEvent, CollisionEstimate, ContactCandidate, ContactEvent, ContactListener,
    ContactManifold, ContactPoint, ContactSettings, ContactSettingsRejection, EventSettings,
    SoftBodyContactSettings, SoftBodyContacts, SoftBodyValidateResult, SoftBodyValidation,
    SoftBodyVertexContact, SubShapeIdPair, ValidateResult, WorldEvents,
};
pub use material::PhysicsMaterial;
pub use math::{Quat, RVec3, Real, Vec3};
pub use query::{
    CollidePointHit, CollideShape, CollideShapeHit, RayCast, RayCastHit, ShapeCast, ShapeCastHit,
};
pub use ragdoll::{
    JointReading, JointTransform, MappedSkeleton, RagdollId, RagdollJoint, RagdollMut, RagdollPart,
    RagdollRef, RagdollSettings, SettleDetector, Skeleton, SkeletonJoint, SkeletonMapper,
    SkeletonPose, TranslationLocks,
};
pub use shape::{
    CompoundChild, CompoundSubShape, DroppedTriangles, HeightFieldSettings, MeshBuildQuality,
    MeshSettings, MutableCompound, Shape, SubShapeId,
};
pub use soft_body::{
    LongRangeAttachment, SoftBodyBendType, SoftBodyDihedralBend, SoftBodyEdge, SoftBodyMut,
    SoftBodyRef, SoftBodySettings, SoftBodySharedSettings, SoftBodySharedSettingsBuilder,
    SoftBodyVertex, SoftBodyVertexAttributes, SoftBodyVertexState, SoftBodyVolume,
};
pub use state::{BodySelection, WorldState};
pub use vehicle::{
    AnyVehicleId, DriverInput, Motorcycle, MotorcycleLean, MotorcycleSettings, TireFriction,
    TireMaxImpulseInput, TrackSide, TrackState, TrackedDriverInput, TrackedVehicle,
    TrackedVehicleSettings, TrackedWheelSettings, VehicleAntiRollBar, VehicleCollisionTester,
    VehicleDifferentialSettings, VehicleEngineSettings, VehicleId, VehicleKind, VehicleMut,
    VehicleRef, VehicleTrackSettings, VehicleTransmissionSettings, VehicleType, WheelContact,
    WheelSettings, WheelState, WheeledControllerKind, WheeledVehicle, WheeledVehicleSettings,
};
pub use world::{PhysicsWorld, StepReport, WorldSettings};

/// The repository's guide, `docs/guide.md`, whose examples run as doctests.
#[cfg(all(doctest, repository_docs))]
#[doc = include_str!("../../../docs/guide.md")]
pub struct Guide;

/// The repository's README, whose example runs as a doctest.
#[cfg(all(doctest, repository_docs))]
#[doc = include_str!("../../../README.md")]
pub struct Readme;

/// The constraints guide, `docs/constraints.md`.
#[cfg(all(doctest, repository_docs))]
#[doc = include_str!("../../../docs/constraints.md")]
pub struct ConstraintsGuide;

/// The soft body guide, `docs/soft-bodies.md`.
#[cfg(all(doctest, repository_docs))]
#[doc = include_str!("../../../docs/soft-bodies.md")]
pub struct SoftBodiesGuide;

/// The events guide, `docs/events.md`.
#[cfg(all(doctest, repository_docs))]
#[doc = include_str!("../../../docs/events.md")]
pub struct EventsGuide;

/// The body controls guide, `docs/bodies.md`.
#[cfg(all(doctest, repository_docs))]
#[doc = include_str!("../../../docs/bodies.md")]
pub struct BodiesGuide;

/// The save and restore guide, `docs/state.md`.
#[cfg(all(doctest, repository_docs))]
#[doc = include_str!("../../../docs/state.md")]
pub struct StateGuide;

/// The job system guide, `docs/job-system.md`.
#[cfg(all(doctest, repository_docs))]
#[doc = include_str!("../../../docs/job-system.md")]
pub struct JobSystemGuide;

/// The shape cooking guide, `docs/shape-cooking.md`.
#[cfg(all(doctest, repository_docs))]
#[doc = include_str!("../../../docs/shape-cooking.md")]
pub struct ShapeCookingGuide;

/// The vehicles guide, `docs/vehicles.md`.
#[cfg(all(doctest, repository_docs))]
#[doc = include_str!("../../../docs/vehicles.md")]
pub struct VehiclesGuide;
