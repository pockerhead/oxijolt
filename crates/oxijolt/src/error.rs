//! Error types. Every value Jolt only checks with debug assertions is validated before it
//! reaches Jolt and reported through these; magnitudes follow [`crate::limits`], which says
//! which assertion paths are derived and which are covered by tests.
//!
//! Each area has its own error, also exported at the crate root. [`Error`] wraps any of them and
//! [`Result`] defaults to it; both are named by this module's path (`oxijolt::error::Result`, as
//! `std::io::Result`), so `use oxijolt::*` does not bring in an `Error` or `Result` that would
//! clash with another glob import.
//!
//! Every error is `Copy`: payloads are static strings, ids, numbers or a fixed-size
//! [`JoltMessage`]. That is part of the API from 1.0 on.

use std::fmt;

use crate::{AnyConstraintId, AnyVehicleId, BodyId, CharacterId, ObjectLayer, RagdollId, Vec3};

/// Why a [`PhysicsWorld`](crate::PhysicsWorld) could not be created or changed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum WorldError {
    /// Jolt's one-time global initialisation failed.
    InitFailed,
    /// A world setting or the gravity is out of range; the payload names it.
    InvalidValue(&'static str),
    /// The collision layer tables are inconsistent; the payload says how.
    InvalidLayers(&'static str),
    /// Jolt or joltc returned null when creating the named object.
    AllocationFailed(&'static str),
}

impl fmt::Display for WorldError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InitFailed => f.write_str("Jolt initialisation failed"),
            Self::InvalidValue(what) => write!(f, "invalid world value: {what}"),
            Self::InvalidLayers(what) => write!(f, "invalid collision layers: {what}"),
            Self::AllocationFailed(what) => write!(f, "could not create the {what}"),
        }
    }
}

impl std::error::Error for WorldError {}

/// Why a [`Shape`](crate::Shape) could not be created.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ShapeError {
    /// Jolt's one-time global initialisation failed.
    InitFailed,
    /// A dimension, position or setting is out of range; the payload names it.
    InvalidValue(&'static str),
    /// The points of a convex hull do not span a volume.
    ConvexHull(ConvexHullError),
    /// A triangle mesh has nothing Jolt can build.
    Mesh(MeshError),
    /// A scale leaves mesh or heightfield triangles too thin to collide with.
    ThinTriangles(ThinTrianglesError),
    /// Jolt refused the shape settings; the payload is Jolt's message.
    Rejected(JoltMessage),
    /// joltc returned null.
    AllocationFailed,
    /// A compound edit named a child index the compound does not have.
    NoSubShape {
        /// The index given.
        index: u32,
        /// The number of children the compound has.
        count: u32,
    },
    /// A compound would hold more than
    /// [`limits::MAX_EXPANDED_SUB_SHAPES`](crate::limits::MAX_EXPANDED_SUB_SHAPES) shapes,
    /// counting a child shared by several parents at every use.
    TooManySubShapes {
        /// The shapes it would hold, at most `u32::MAX`.
        expanded: u32,
    },
    /// Shape binary state could not be saved or restored
    /// ([`Shape::save_binary_state`](crate::Shape::save_binary_state),
    /// [`Shape::restore_binary_state`](crate::Shape::restore_binary_state)).
    BinaryState(BinaryStateError),
}

impl fmt::Display for ShapeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InitFailed => f.write_str("Jolt initialisation failed"),
            Self::InvalidValue(what) => write!(f, "invalid shape value: {what}"),
            Self::ConvexHull(error) => write!(f, "invalid convex hull: {error}"),
            Self::Mesh(error) => write!(f, "invalid triangle mesh: {error}"),
            Self::ThinTriangles(error) => write!(f, "invalid scale: {error}"),
            Self::Rejected(message) => write!(f, "Jolt rejected the shape settings: {message}"),
            Self::AllocationFailed => f.write_str("could not create the shape"),
            Self::NoSubShape { index, count } => {
                write!(f, "compound has no sub-shape {index} (it has {count})")
            }
            Self::TooManySubShapes { expanded } => write!(
                f,
                "compound expands to {expanded} shapes, above limits::MAX_EXPANDED_SUB_SHAPES"
            ),
            Self::BinaryState(error) => write!(f, "invalid shape binary state: {error}"),
        }
    }
}

impl std::error::Error for ShapeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::BinaryState(error) => Some(error),
            Self::ConvexHull(error) => Some(error),
            Self::Mesh(error) => Some(error),
            Self::ThinTriangles(error) => Some(error),
            _ => None,
        }
    }
}

/// Why [`Shape::new_convex_hull`](crate::Shape::new_convex_hull) refused its points.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ConvexHullError {
    /// Fewer than 4 points.
    TooFewPoints,
    /// The points lie in one spot, on a line, or so close to a line that Jolt's hull builder
    /// cannot build a reliable hull of them.
    Degenerate,
    /// The points lie in one plane, or so close to one that the hull has next to no volume.
    Coplanar,
}

impl fmt::Display for ConvexHullError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::TooFewPoints => "a convex hull needs at least 4 points",
            Self::Degenerate => "the points lie on or close to a line",
            Self::Coplanar => {
                "the points lie on or close to a plane; thicken the cloud or centre it on the shape origin"
            }
        })
    }
}

impl std::error::Error for ConvexHullError {}

/// Why [`Shape::new_mesh`](crate::Shape::new_mesh) built nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum MeshError {
    /// Every triangle was too small, too thin or degenerate to collide with.
    NoTriangles,
}

impl fmt::Display for MeshError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::NoTriangles => {
                "no triangle is left after dropping small, thin and degenerate ones"
            }
        })
    }
}

impl std::error::Error for MeshError {}

/// Why [`Shape::new_scaled`](crate::Shape::new_scaled) refused a scale: a stored triangle of a mesh
/// or heightfield inside the shape, scaled, would be too thin for Jolt to collide with convex
/// shapes up to `max_convex_extent`.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct ThinTrianglesError {
    /// The scale that was refused.
    pub scale: Vec3,
    /// The [`MeshSettings::max_convex_extent`](crate::MeshSettings::max_convex_extent) the mesh
    /// was built with, or its default for a heightfield, metres.
    pub max_convex_extent: f32,
}

/// Equal when every value has the same bits, so that the error is `Eq`.
impl PartialEq for ThinTrianglesError {
    fn eq(&self, other: &Self) -> bool {
        let bits = |error: &Self| {
            [
                error.scale.x,
                error.scale.y,
                error.scale.z,
                error.max_convex_extent,
            ]
            .map(f32::to_bits)
        };
        bits(self) == bits(other)
    }
}

impl Eq for ThinTrianglesError {}

impl fmt::Display for ThinTrianglesError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let Vec3 { x, y, z } = self.scale;
        write!(
            f,
            "mesh or heightfield triangles scaled by ({x}, {y}, {z}) are too thin for convex shapes up to {} m",
            self.max_convex_extent
        )
    }
}

impl std::error::Error for ThinTrianglesError {}

/// Why [`Shape::save_binary_state`](crate::Shape::save_binary_state) or
/// [`Shape::restore_binary_state`](crate::Shape::restore_binary_state) failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum BinaryStateError {
    /// The bytes do not start with the shape binary state header.
    NotBinaryState,
    /// The bytes were saved by another build: another format version, Jolt or joltc commit,
    /// extension revision, precision, determinism mode or byte order. Save the shape again with
    /// this build.
    OtherBuild,
    /// The bytes end before the header or before the payload the header announces.
    Truncated,
    /// The checksum does not match: the bytes changed after they were saved.
    Corrupt,
    /// The bytes break a structural rule the checksum cannot see; the payload names it.
    Malformed(&'static str),
    /// joltc refused to save the shape or to restore its records; the payload is its message.
    Rejected(JoltMessage),
}

impl BinaryStateError {
    /// The version of the byte layout this build writes and reads. Bytes of another version are
    /// refused with [`OtherBuild`](Self::OtherBuild).
    pub const FORMAT_VERSION: u32 = 1;
}

impl fmt::Display for BinaryStateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotBinaryState => f.write_str("the bytes are not shape binary state"),
            Self::OtherBuild => f.write_str("the bytes were saved by another build"),
            Self::Truncated => f.write_str("the bytes end early"),
            Self::Corrupt => f.write_str("the checksum does not match"),
            Self::Malformed(rule) => write!(f, "malformed bytes: {rule}"),
            Self::Rejected(message) => write!(f, "joltc refused the bytes: {message}"),
        }
    }
}

impl std::error::Error for BinaryStateError {}

/// Jolt's diagnostic text for a refused shape, at most [`JoltMessage::CAPACITY`] bytes.
///
/// The wording is Jolt's and not a stable format: match on the typed [`ShapeError`] variants
/// instead of parsing it.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct JoltMessage {
    len: u8,
    bytes: [u8; JoltMessage::CAPACITY],
}

impl JoltMessage {
    /// Most bytes a message keeps. A longer message keeps the words that fit before a
    /// closing `...`.
    pub const CAPACITY: usize = 79;
    /// What ends a message that was cut.
    const CUT: &'static str = "...";

    /// The message read from a C buffer: the bytes up to the first NUL (all of them when there
    /// is none), up to their longest valid UTF-8 prefix. A text longer than
    /// [`CAPACITY`](Self::CAPACITY) is cut after the last whole word that leaves room for
    /// `...`, or inside the word when there is no space.
    pub(crate) fn from_c_buffer(buffer: &[u8]) -> Self {
        let end = buffer
            .iter()
            .position(|&byte| byte == 0)
            .unwrap_or(buffer.len());
        let text = &buffer[..end];
        let (kept, cut) = if text.len() <= Self::CAPACITY {
            (valid_prefix(text), "")
        } else {
            let room = valid_prefix(&text[..Self::CAPACITY - Self::CUT.len()]);
            let words = room
                .rfind(' ')
                .map_or(room, |space| room[..space].trim_end());
            (words, Self::CUT)
        };
        let mut bytes = [0; Self::CAPACITY];
        bytes[..kept.len()].copy_from_slice(kept.as_bytes());
        bytes[kept.len()..kept.len() + cut.len()].copy_from_slice(cut.as_bytes());
        Self {
            len: (kept.len() + cut.len()) as u8,
            bytes,
        }
    }

    /// The message text.
    pub fn as_str(&self) -> &str {
        // `from_c_buffer`, the only constructor, keeps a valid UTF-8 prefix.
        std::str::from_utf8(&self.bytes[..usize::from(self.len)]).unwrap_or_default()
    }
}

/// The longest prefix of `bytes` that is valid UTF-8.
fn valid_prefix(bytes: &[u8]) -> &str {
    let valid = match std::str::from_utf8(bytes) {
        Ok(_) => bytes.len(),
        Err(error) => error.valid_up_to(),
    };
    std::str::from_utf8(&bytes[..valid]).unwrap_or_default()
}

impl fmt::Display for JoltMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl fmt::Debug for JoltMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self.as_str(), f)
    }
}

/// Why a [`ContactSettings`](crate::ContactSettings) or
/// [`SoftBodyContactSettings`](crate::SoftBodyContactSettings) setter refused a value, or why
/// the settings a [`ContactListener`](crate::ContactListener) returned do not fit their contact.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ContactSettingsError {
    /// Friction outside `0..=limits::MAX_FRICTION`.
    Friction,
    /// Restitution outside `0..=1`.
    Restitution,
    /// An inverse mass scale neither 0 nor within `limits::MIN_CONTACT_SCALE..=1`.
    InverseMassScale,
    /// An inverse inertia scale neither 0 nor within `limits::MIN_CONTACT_SCALE..=1`.
    InverseInertiaScale,
    /// A contact with a sensor body must stay a sensor contact.
    SensorBody,
    /// A surface velocity beyond its bound.
    SurfaceVelocity,
}

impl fmt::Display for ContactSettingsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Friction => "contact friction out of range",
            Self::Restitution => "contact restitution out of range",
            Self::InverseMassScale => "inverse mass scale out of range",
            Self::InverseInertiaScale => "inverse inertia scale out of range",
            Self::SensorBody => "a contact with a sensor body stays a sensor contact",
            Self::SurfaceVelocity => "surface velocity out of range",
        })
    }
}

impl std::error::Error for ContactSettingsError {}

/// Why a [`GroupFilterTableBuilder`](crate::GroupFilterTableBuilder) or a
/// [`CollisionGroup`](crate::CollisionGroup) refused a value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum CollisionGroupError {
    /// Jolt's one-time global initialisation failed.
    InitFailed,
    /// A table needs at least one sub group.
    NoSubGroups,
    /// More sub groups than [`GroupFilterTable::MAX_SUB_GROUPS`](crate::GroupFilterTable::MAX_SUB_GROUPS).
    TooManySubGroups(u32),
    /// A sub group at or beyond the table's sub-group count.
    SubGroupOutOfRange(u32),
    /// A pair of sub groups that is one sub group, which never collides with itself.
    SameSubGroup(u32),
    /// A group id above [`CollisionGroup::MAX_GROUP_ID`](crate::CollisionGroup::MAX_GROUP_ID).
    GroupIdOutOfRange(u32),
}

impl fmt::Display for CollisionGroupError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InitFailed => f.write_str("Jolt initialisation failed"),
            Self::NoSubGroups => f.write_str("a group filter table needs a sub group"),
            Self::TooManySubGroups(count) => write!(f, "too many sub groups: {count}"),
            Self::SubGroupOutOfRange(id) => write!(f, "sub group {id} is not in the table"),
            Self::SameSubGroup(id) => write!(f, "sub group {id} paired with itself"),
            Self::GroupIdOutOfRange(id) => write!(f, "group id {id} is out of range"),
        }
    }
}

impl std::error::Error for CollisionGroupError {}

/// Why a scene query could not run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum QueryError {
    /// A query input is out of range; the payload names it.
    InvalidValue(&'static str),
    /// A body the filter excludes belongs to another world.
    WrongWorld(BodyId),
    /// An object layer the filter selects does not exist in this world's collision layers.
    UnknownObjectLayer(ObjectLayer),
    /// joltc returned null for a native object the query creates or reads.
    AllocationFailed,
}

impl fmt::Display for QueryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidValue(what) => write!(f, "invalid query value: {what}"),
            Self::WrongWorld(id) => write!(f, "body {id:?} of the filter belongs to another world"),
            Self::UnknownObjectLayer(layer) => write!(
                f,
                "object layer {} of the filter does not exist in this world",
                layer.get()
            ),
            Self::AllocationFailed => f.write_str("could not create a native query object"),
        }
    }
}

impl std::error::Error for QueryError {}

/// Why [`PhysicsWorld::step`](crate::PhysicsWorld::step) or
/// [`PhysicsWorld::step_with_collision_steps`](crate::PhysicsWorld::step_with_collision_steps)
/// rejected the call. The world did not advance. A step that ran but dropped work is reported in
/// its [`StepReport`](crate::StepReport) instead.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum StepError {
    /// The total time step is not finite, below
    /// [`PhysicsWorld::MIN_DELTA_TIME`](crate::PhysicsWorld::MIN_DELTA_TIME) or above
    /// [`PhysicsWorld::MAX_DELTA_TIME`](crate::PhysicsWorld::MAX_DELTA_TIME), or one collision
    /// step (`delta_time / collision_steps`, divided in `f32` as Jolt does) is shorter than
    /// `MIN_DELTA_TIME`.
    InvalidDeltaTime,
    /// A value other than the time step is out of range; the payload names it.
    InvalidValue(&'static str),
}

impl fmt::Display for StepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDeltaTime => write!(
                f,
                "the time step must be finite and between {} and {} s, and each collision step at least {} s",
                crate::PhysicsWorld::MIN_DELTA_TIME,
                crate::PhysicsWorld::MAX_DELTA_TIME,
                crate::PhysicsWorld::MIN_DELTA_TIME
            ),
            Self::InvalidValue(what) => write!(f, "invalid step value: {what}"),
        }
    }
}

impl std::error::Error for StepError {}

/// Why a body operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum BodyError {
    /// The id names no body in this world: it was removed, or its index was reused.
    NotFound(BodyId),
    /// The id belongs to another world.
    WrongWorld(BodyId),
    /// The object layer does not exist in this world's collision layers.
    UnknownObjectLayer(ObjectLayer),
    /// A value is out of range; the payload names it.
    InvalidValue(&'static str),
    /// The world already holds its maximum number of bodies.
    TooManyBodies,
    /// joltc returned null when creating the body settings.
    AllocationFailed,
    /// The body is the inner body of a character; remove the character instead.
    OwnedByCharacter(BodyId),
    /// The body is the chassis of a vehicle; remove the vehicle first.
    UsedByVehicle(BodyId),
    /// The body is a part of a ragdoll; remove the ragdoll instead.
    OwnedByRagdoll(BodyId),
    /// The body is used by a constraint; remove the constraint first.
    UsedByConstraint(BodyId),
    /// The operation needs a rigid body and the body is a soft body; Jolt ignores it on soft
    /// bodies or cannot attach it to one.
    NotRigidBody(BodyId),
    /// The operation needs a soft body and the body is a rigid body.
    NotSoftBody(BodyId),
    /// The operation needs a body with all six degrees of freedom and the body was created with
    /// fewer ([`BodySettings::allowed_dofs`](crate::BodySettings::allowed_dofs)).
    RestrictedDofs(BodyId),
    /// The operation needs a kinematic body and the body is static or dynamic.
    NotKinematic(BodyId),
    /// The body was created static without
    /// [`BodySettings::allow_dynamic_or_kinematic`](crate::BodySettings::allow_dynamic_or_kinematic),
    /// so Jolt gave it no motion properties and it cannot become kinematic or dynamic.
    CannotMove(BodyId),
}

impl fmt::Display for BodyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFound(id) => write!(f, "no body {id:?} in this world"),
            Self::WrongWorld(id) => write!(f, "body {id:?} belongs to another world"),
            Self::UnknownObjectLayer(layer) => {
                write!(
                    f,
                    "object layer {} does not exist in this world",
                    layer.get()
                )
            }
            Self::InvalidValue(what) => write!(f, "invalid body value: {what}"),
            Self::TooManyBodies => f.write_str("the world is full"),
            Self::AllocationFailed => f.write_str("could not create the body settings"),
            Self::OwnedByCharacter(id) => {
                write!(f, "body {id:?} is the inner body of a character")
            }
            Self::UsedByVehicle(id) => write!(f, "body {id:?} is the chassis of a vehicle"),
            Self::OwnedByRagdoll(id) => write!(f, "body {id:?} is a part of a ragdoll"),
            Self::UsedByConstraint(id) => write!(
                f,
                "body {id:?} is used by a constraint; remove the constraint first"
            ),
            Self::NotRigidBody(id) => {
                write!(
                    f,
                    "body {id:?} is a soft body; this operation needs a rigid body"
                )
            }
            Self::NotSoftBody(id) => write!(f, "body {id:?} is not a soft body"),
            Self::RestrictedDofs(id) => {
                write!(f, "body {id:?} does not have all six degrees of freedom")
            }
            Self::NotKinematic(id) => write!(f, "body {id:?} is not kinematic"),
            Self::CannotMove(id) => write!(f, "body {id:?} was created static and cannot move"),
        }
    }
}

impl std::error::Error for BodyError {}

/// Why a character operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum CharacterError {
    /// The id names no character in this world: it was removed.
    NotFound(CharacterId),
    /// The id belongs to another world.
    WrongWorld(CharacterId),
    /// A value is out of range; the payload names it.
    InvalidValue(&'static str),
    /// The character asked for an inner body and the world already holds its maximum number of
    /// bodies.
    TooManyBodies,
    /// The world has given out every character id.
    TooManyCharacters,
    /// The query filter of an update or refresh is not valid for this world, or joltc could
    /// not create it.
    Query(QueryError),
    /// Jolt could not read a saved [`CharacterState`](crate::CharacterState). A state saved by
    /// this build should never cause it; if it does, the character may be partly restored.
    RestoreFailed,
    /// The character has no inner body.
    NoInnerBody(CharacterId),
}

impl fmt::Display for CharacterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFound(id) => write!(f, "no character {id:?} in this world"),
            Self::WrongWorld(id) => write!(f, "character {id:?} belongs to another world"),
            Self::InvalidValue(what) => write!(f, "invalid character value: {what}"),
            Self::TooManyBodies => f.write_str("the world is full; no room for the inner body"),
            Self::TooManyCharacters => f.write_str("the world has no character ids left"),
            Self::Query(error) => write!(f, "unusable character query filter: {error}"),
            Self::RestoreFailed => f.write_str("the character state could not be restored"),
            Self::NoInnerBody(id) => write!(f, "character {id:?} has no inner body"),
        }
    }
}

impl std::error::Error for CharacterError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Query(error) => Some(error),
            _ => None,
        }
    }
}

/// Why a vehicle operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum VehicleError {
    /// The id names no vehicle in this world: it was removed.
    NotFound(AnyVehicleId),
    /// The id belongs to another world.
    WrongWorld(AnyVehicleId),
    /// A setting or input is out of range; the payload names it.
    InvalidValue(&'static str),
    /// The chassis body is not usable: not in this world, the inner body of a character or a
    /// part of a ragdoll.
    Body(BodyError),
    /// The chassis body is not dynamic.
    NotDynamic(BodyId),
    /// The chassis body already carries a vehicle.
    AlreadyHasVehicle(BodyId),
    /// The world has given out every vehicle id.
    TooManyVehicles,
    /// A motorcycle's lean spring integration coefficient is not 0. Jolt's state does not hold
    /// the integrated lean angle, so a restored [`WorldState`](crate::WorldState) could replay
    /// different steps; see
    /// [`MotorcycleSettings::lean_spring_integration_coefficient`](crate::MotorcycleSettings::lean_spring_integration_coefficient).
    LeanSpringIntegrationNotSaved,
    /// A setting that a vehicle's saved state depends on; it can only be set at creation. The
    /// payload names it, for example the gear counts and the clutch release time of
    /// [`VehicleMut::set_transmission`](crate::VehicleMut::set_transmission).
    FixedAtCreation(&'static str),
}

impl fmt::Display for VehicleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFound(id) => write!(f, "no vehicle {id:?} in this world"),
            Self::WrongWorld(id) => write!(f, "vehicle {id:?} belongs to another world"),
            Self::InvalidValue(what) => write!(f, "invalid vehicle value: {what}"),
            Self::Body(error) => write!(f, "unusable chassis body: {error}"),
            Self::NotDynamic(id) => write!(f, "chassis body {id:?} is not dynamic"),
            Self::AlreadyHasVehicle(id) => write!(f, "body {id:?} already carries a vehicle"),
            Self::TooManyVehicles => f.write_str("the world has no vehicle ids left"),
            Self::LeanSpringIntegrationNotSaved => f.write_str(
                "a motorcycle's lean spring integration coefficient must be 0: Jolt does not save the integrated lean angle",
            ),
            Self::FixedAtCreation(what) => {
                write!(f, "{what} cannot change after the vehicle is created")
            }
        }
    }
}

impl std::error::Error for VehicleError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Body(error) => Some(error),
            _ => None,
        }
    }
}

/// Why a skeleton, ragdoll settings or ragdoll operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum RagdollError {
    /// Jolt's one-time global initialisation failed.
    InitFailed,
    /// The id names no ragdoll in this world: it was removed.
    NotFound(RagdollId),
    /// The id belongs to another world.
    WrongWorld(RagdollId),
    /// A setting, pose or input is out of range; the payload names it.
    InvalidValue(&'static str),
    /// A part's object layer does not exist in this world's collision layers.
    UnknownObjectLayer(ObjectLayer),
    /// A part's body settings could not be turned into Jolt's.
    Body(BodyError),
    /// The world has no room for every part of the ragdoll.
    TooManyBodies,
    /// The world has given out every ragdoll id.
    TooManyRagdolls,
    /// A skeleton mapper's animation skeleton has no joint with the name of this ragdoll joint.
    UnmappedJoint(u32),
    /// The animation joint of this ragdoll joint does not sit below the animation joint of its
    /// parent with only unmapped joints between them; for joint 0, it sits below a mapped joint.
    HierarchyMismatch(u32),
    /// The skeleton mapper cannot turn the animation chain that ends at this ragdoll joint: one
    /// of its directions is shorter than
    /// [`limits::MIN_MAPPED_CHAIN_LENGTH`](crate::limits::MIN_MAPPED_CHAIN_LENGTH), the animation
    /// one after an allowance for rounding that grows with the chain's distance from the pose's
    /// root offset and with its number of links.
    DegenerateChain(u32),
}

impl fmt::Display for RagdollError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InitFailed => f.write_str("Jolt initialisation failed"),
            Self::NotFound(id) => write!(f, "no ragdoll {id:?} in this world"),
            Self::WrongWorld(id) => write!(f, "ragdoll {id:?} belongs to another world"),
            Self::InvalidValue(what) => write!(f, "invalid ragdoll value: {what}"),
            Self::UnknownObjectLayer(layer) => write!(
                f,
                "object layer {} of a ragdoll part does not exist in this world",
                layer.get()
            ),
            Self::Body(error) => write!(f, "unusable ragdoll part: {error}"),
            Self::TooManyBodies => f.write_str("the world has no room for every ragdoll part"),
            Self::TooManyRagdolls => f.write_str("the world has no ragdoll ids left"),
            Self::UnmappedJoint(joint) => {
                write!(
                    f,
                    "ragdoll joint {joint} has no animation joint of its name"
                )
            }
            Self::HierarchyMismatch(joint) => write!(
                f,
                "the animation joint of ragdoll joint {joint} is not below its parent's"
            ),
            Self::DegenerateChain(joint) => write!(
                f,
                "the mapped chain ending at ragdoll joint {joint} is too short to turn"
            ),
        }
    }
}

impl std::error::Error for RagdollError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Body(error) => Some(error),
            _ => None,
        }
    }
}

/// Why a constraint operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ConstraintError {
    /// The id names no constraint in this world: it was removed.
    NotFound(AnyConstraintId),
    /// The id belongs to another world.
    WrongWorld(AnyConstraintId),
    /// A setting or input is out of range; the payload names it.
    InvalidValue(&'static str),
    /// A body of the constraint is not usable: not in this world, the inner body of a character
    /// or a part of a ragdoll.
    Body(BodyError),
    /// A gear, rack and pinion or pulley names a static or kinematic body; Jolt's solver parts
    /// for these constraints only work between dynamic bodies.
    NotDynamic(BodyId),
    /// The constraint is referenced by another one (a gear or a rack and pinion); remove that
    /// one first.
    UsedByConstraint(AnyConstraintId),
    /// The world has given out every constraint id.
    TooManyConstraints,
}

impl fmt::Display for ConstraintError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFound(id) => write!(f, "no constraint {id:?} in this world"),
            Self::WrongWorld(id) => write!(f, "constraint {id:?} belongs to another world"),
            Self::InvalidValue(what) => write!(f, "invalid constraint value: {what}"),
            Self::Body(error) => write!(f, "unusable constraint body: {error}"),
            Self::NotDynamic(id) => write!(f, "constraint body {id:?} is not dynamic"),
            Self::UsedByConstraint(id) => write!(
                f,
                "constraint {id:?} is referenced by another constraint; remove that one first"
            ),
            Self::TooManyConstraints => f.write_str("the world has no constraint ids left"),
        }
    }
}

impl std::error::Error for ConstraintError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Body(error) => Some(error),
            _ => None,
        }
    }
}

/// Why [`SoftBodySharedSettings`](crate::SoftBodySharedSettings) could not be built.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum SoftBodyError {
    /// Jolt's one-time global initialisation failed.
    InitFailed,
    /// A vertex, face, constraint or attribute is out of range; the payload names the rule.
    InvalidValue(&'static str),
}

impl fmt::Display for SoftBodyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InitFailed => f.write_str("Jolt initialisation failed"),
            Self::InvalidValue(what) => write!(f, "invalid soft body value: {what}"),
        }
    }
}

impl std::error::Error for SoftBodyError {}

/// Why saving or restoring a [`WorldState`](crate::WorldState) refused or failed. On every error
/// but [`RestoreFailed`](Self::RestoreFailed) the world and the state are unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum StateError {
    /// The state was saved by another world. This is the crate's policy: Jolt itself can share
    /// state between worlds built with identical calls, but this API cannot yet check that a
    /// world is such a peer.
    WrongWorld,
    /// The world's structure has changed since the state was saved: a body, character, vehicle,
    /// ragdoll or constraint was created or removed, a body's or ragdoll's motion type was
    /// changed, a body's shape was set, or the world was rebased (a rebase that changes nothing
    /// does not count). Jolt saves neither
    /// which objects exist nor its body id allocator, so even a create followed by a remove
    /// leaves earlier states unrestorable.
    WorldChanged,
    /// Jolt or a character could not read the state. A state the world accepted should never
    /// cause it; if it does, the world may be partly restored and should be discarded.
    RestoreFailed,
    /// A [`BodySelection::Only`](crate::BodySelection::Only) names a body of another world
    /// ([`BodyError::WrongWorld`]) or a removed body ([`BodyError::NotFound`]).
    Body(BodyError),
}

impl fmt::Display for StateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongWorld => f.write_str("the state was saved by another world"),
            Self::WorldChanged => {
                f.write_str("the world's objects have changed since the state was saved")
            }
            Self::RestoreFailed => f.write_str("the state could not be restored"),
            Self::Body(error) => write!(f, "invalid body in the selection: {error}"),
        }
    }
}

impl std::error::Error for StateError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Body(error) => Some(error),
            _ => None,
        }
    }
}

/// Any error of this crate: one variant per area, wrapping that area's error.
///
/// Every fallible call still returns its own area's error; `?` converts each of them into this
/// type, so code that calls several areas can return [`Result<T>`](Result). The enum is
/// `#[non_exhaustive]`: a `match` on it needs a `_` arm. `Display` and `source` are those of
/// the wrapped error: an area error that wraps another one reports it as its source
/// (`VehicleError::Body` its `BodyError`, `ShapeError::Mesh` its `MeshError`), the others have
/// none.
///
/// ```
/// use oxijolt::prelude::*;
///
/// fn drop_ball(world: &mut PhysicsWorld) -> oxijolt::error::Result<BodyId> {
///     let shape = Shape::new_sphere(0.5)?; // ShapeError
///     let ball = world.create_body(&shape, &BodySettings::new_dynamic())?; // BodyError
///     let report = world.step(1.0 / 60.0)?; // StepError
///     assert!(report.is_complete());
///     Ok(ball)
/// }
///
/// # fn main() -> oxijolt::error::Result<()> {
/// let mut world = PhysicsWorld::new(WorldSettings::default())?; // WorldError
/// drop_ball(&mut world)?;
/// let error: oxijolt::error::Error = world.step(0.0).unwrap_err().into();
/// assert!(matches!(
///     error,
///     oxijolt::error::Error::Step(StepError::InvalidDeltaTime)
/// ));
/// # Ok(())
/// # }
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// A [`WorldError`].
    World(WorldError),
    /// A [`ShapeError`].
    Shape(ShapeError),
    /// A [`BodyError`].
    Body(BodyError),
    /// A [`StepError`].
    Step(StepError),
    /// A [`QueryError`].
    Query(QueryError),
    /// A [`ContactSettingsError`].
    ContactSettings(ContactSettingsError),
    /// A [`CharacterError`].
    Character(CharacterError),
    /// A [`VehicleError`].
    Vehicle(VehicleError),
    /// A [`RagdollError`].
    Ragdoll(RagdollError),
    /// A [`ConstraintError`].
    Constraint(ConstraintError),
    /// A [`SoftBodyError`].
    SoftBody(SoftBodyError),
    /// A [`StateError`].
    State(StateError),
    /// A [`CollisionGroupError`].
    CollisionGroup(CollisionGroupError),
}

/// `Result<T>` is `Result<T, oxijolt::error::Error>`; the second parameter keeps `Result<T, E>`
/// working in code that glob-imports this module.
pub type Result<T, E = Error> = std::result::Result<T, E>;

impl From<WorldError> for Error {
    fn from(error: WorldError) -> Self {
        Self::World(error)
    }
}

impl From<ShapeError> for Error {
    fn from(error: ShapeError) -> Self {
        Self::Shape(error)
    }
}

impl From<BodyError> for Error {
    fn from(error: BodyError) -> Self {
        Self::Body(error)
    }
}

impl From<StepError> for Error {
    fn from(error: StepError) -> Self {
        Self::Step(error)
    }
}

impl From<QueryError> for Error {
    fn from(error: QueryError) -> Self {
        Self::Query(error)
    }
}

impl From<ContactSettingsError> for Error {
    fn from(error: ContactSettingsError) -> Self {
        Self::ContactSettings(error)
    }
}

impl From<CharacterError> for Error {
    fn from(error: CharacterError) -> Self {
        Self::Character(error)
    }
}

impl From<VehicleError> for Error {
    fn from(error: VehicleError) -> Self {
        Self::Vehicle(error)
    }
}

impl From<RagdollError> for Error {
    fn from(error: RagdollError) -> Self {
        Self::Ragdoll(error)
    }
}

impl From<ConstraintError> for Error {
    fn from(error: ConstraintError) -> Self {
        Self::Constraint(error)
    }
}

impl From<SoftBodyError> for Error {
    fn from(error: SoftBodyError) -> Self {
        Self::SoftBody(error)
    }
}

impl From<StateError> for Error {
    fn from(error: StateError) -> Self {
        Self::State(error)
    }
}

impl From<CollisionGroupError> for Error {
    fn from(error: CollisionGroupError) -> Self {
        Self::CollisionGroup(error)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::World(error) => fmt::Display::fmt(error, f),
            Self::Shape(error) => fmt::Display::fmt(error, f),
            Self::Body(error) => fmt::Display::fmt(error, f),
            Self::Step(error) => fmt::Display::fmt(error, f),
            Self::Query(error) => fmt::Display::fmt(error, f),
            Self::ContactSettings(error) => fmt::Display::fmt(error, f),
            Self::Character(error) => fmt::Display::fmt(error, f),
            Self::Vehicle(error) => fmt::Display::fmt(error, f),
            Self::Ragdoll(error) => fmt::Display::fmt(error, f),
            Self::Constraint(error) => fmt::Display::fmt(error, f),
            Self::SoftBody(error) => fmt::Display::fmt(error, f),
            Self::State(error) => fmt::Display::fmt(error, f),
            Self::CollisionGroup(error) => fmt::Display::fmt(error, f),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::World(error) => error.source(),
            Self::Shape(error) => error.source(),
            Self::Body(error) => error.source(),
            Self::Step(error) => error.source(),
            Self::Query(error) => error.source(),
            Self::ContactSettings(error) => error.source(),
            Self::Character(error) => error.source(),
            Self::Vehicle(error) => error.source(),
            Self::Ragdoll(error) => error.source(),
            Self::Constraint(error) => error.source(),
            Self::SoftBody(error) => error.source(),
            Self::State(error) => error.source(),
            Self::CollisionGroup(error) => error.source(),
        }
    }
}

#[cfg(test)]
mod tests;
