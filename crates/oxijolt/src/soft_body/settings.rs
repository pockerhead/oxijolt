//! Creation settings of one soft body.

use oxijolt_sys::*;

use super::SoftBodySharedSettings;
use crate::body::{is_restitution, Activation, LINEAR_DAMPING_RULE, RESTITUTION_RULE};
use crate::limits::{self, is_friction, is_gravity_factor, is_in_frame, is_local_distance};
use crate::math::{is_finite_non_negative, ROTATION_RULE};
use crate::owned::{JoltObject, Owned};
use crate::{BodyError, CollisionGroup, ObjectLayer, Quat, RVec3};

/// How to create a soft body from [`SoftBodySharedSettings`].
///
/// The defaults are those of Jolt's `SoftBodyCreationSettings`, except the object layer: Jolt
/// uses layer 0, oxijolt [`ObjectLayer::MOVING`].
#[derive(Clone, Debug, PartialEq)]
pub struct SoftBodySettings {
    pub(super) position: RVec3,
    pub(super) rotation: Quat,
    pub(super) object_layer: ObjectLayer,
    pub(super) num_iterations: u32,
    pub(super) linear_damping: f32,
    pub(super) max_linear_velocity: f32,
    pub(super) restitution: f32,
    pub(super) friction: f32,
    pub(super) pressure: f32,
    pub(super) gravity_factor: f32,
    pub(super) vertex_radius: f32,
    pub(super) update_position: bool,
    pub(super) make_rotation_identity: bool,
    pub(super) allow_sleeping: bool,
    pub(super) faces_double_sided: bool,
    pub(super) activation: Activation,
    pub(super) collision_group: Option<CollisionGroup>,
}

impl Default for SoftBodySettings {
    fn default() -> Self {
        Self {
            position: RVec3::ZERO,
            rotation: Quat::IDENTITY,
            object_layer: ObjectLayer::MOVING,
            num_iterations: 5,
            linear_damping: 0.1,
            max_linear_velocity: 500.0,
            restitution: 0.0,
            friction: 0.2,
            pressure: 0.0,
            gravity_factor: 1.0,
            vertex_radius: 0.0,
            update_position: true,
            make_rotation_identity: true,
            allow_sleeping: true,
            faces_double_sided: false,
            activation: Activation::Activate,
            collision_group: None,
        }
    }
}

impl SoftBodySettings {
    /// Largest number of solver iterations per step.
    ///
    /// Jolt divides the step into this many sub-steps; the bound fixes the smallest sub-step
    /// that [`limits::MAX_COMPLIANCE`] is derived for.
    pub const MAX_ITERATIONS: u32 = 100;

    /// Initial position of the body origin in metres, every component at most
    /// [`limits::MAX_POSITION`] in absolute value. Default the origin.
    #[must_use]
    pub fn position(mut self, value: RVec3) -> Self {
        self.position = value;
        self
    }

    /// Initial rotation, a unit quaternion. Default identity.
    #[must_use]
    pub fn rotation(mut self, value: Quat) -> Self {
        self.rotation = value;
        self
    }

    /// The object layer, which must exist in the world's
    /// [`CollisionLayers`](crate::CollisionLayers). Default [`ObjectLayer::MOVING`].
    #[must_use]
    pub fn object_layer(mut self, value: ObjectLayer) -> Self {
        self.object_layer = value;
        self
    }

    /// Solver iterations per step, `1..=`[`MAX_ITERATIONS`](Self::MAX_ITERATIONS). Default 5.
    #[must_use]
    pub fn num_iterations(mut self, value: u32) -> Self {
        self.num_iterations = value;
        self
    }

    /// Linear damping of every vertex, finite and at least 0: Jolt scales the vertex velocity by
    /// `max(0, 1 - c * dt)` every sub-step. Default 0.1. Change it later with
    /// [`BodyMut::set_linear_damping`](crate::BodyMut::set_linear_damping).
    #[must_use]
    pub fn linear_damping(mut self, value: f32) -> Self {
        self.linear_damping = value;
        self
    }

    /// Largest speed of a vertex in m/s, above 0 and at most [`limits::MAX_LINEAR_VELOCITY`].
    /// Default 500.
    #[must_use]
    pub fn max_linear_velocity(mut self, value: f32) -> Self {
        self.max_linear_velocity = value;
        self
    }

    /// Restitution (bounciness) of collisions, between 0 and 1. Default 0. Change it later with
    /// [`BodyMut::set_restitution`](crate::BodyMut::set_restitution).
    #[must_use]
    pub fn restitution(mut self, value: f32) -> Self {
        self.restitution = value;
        self
    }

    /// Friction coefficient of collisions, between 0 and [`limits::MAX_FRICTION`]. Default 0.2.
    /// Change it later with [`BodyMut::set_friction`](crate::BodyMut::set_friction).
    #[must_use]
    pub fn friction(mut self, value: f32) -> Self {
        self.friction = value;
        self
    }

    /// Pressure coefficient of a closed body (`n · R · T`), finite and within
    /// `0..=`[`limits::MAX_SOFT_BODY_PRESSURE`]; 0 applies no pressure. Jolt pushes the faces
    /// outwards with it divided by the enclosed volume. Default 0.
    ///
    /// Above 0, [`PhysicsWorld::create_soft_body`] also requires faces wound counter-clockwise
    /// seen from outside that enclose, about the body origin, a volume large enough that the
    /// pressure at the start geometry gives a vertex of [`limits::MIN_MASS`] at most
    /// [`limits::MAX_ACCELERATION`]. An open mesh passes when its signed volume does. Keep the
    /// vertices of a pressurised body around its origin: far from it the body is refused
    /// ([docs/limits.md#soft-body-pressure]).
    ///
    /// [`PhysicsWorld::create_soft_body`]: crate::PhysicsWorld::create_soft_body
    ///
    /// [docs/limits.md#soft-body-pressure]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-pressure
    #[must_use]
    pub fn pressure(mut self, value: f32) -> Self {
        self.pressure = value;
        self
    }

    /// Multiplier for the world's gravity on every vertex, at most
    /// [`limits::MAX_GRAVITY_FACTOR`] in absolute value. Default 1. Change it later with
    /// [`BodyMut::set_gravity_factor`](crate::BodyMut::set_gravity_factor).
    #[must_use]
    pub fn gravity_factor(mut self, value: f32) -> Self {
        self.gravity_factor = value;
        self
    }

    /// Radius of every particle in metres, `0..=`[`limits::MAX_SHAPE_EXTENT`]: vertices keep
    /// this distance from the surfaces they collide with. Default 0.
    #[must_use]
    pub fn vertex_radius(mut self, value: f32) -> Self {
        self.vertex_radius = value;
        self
    }

    /// Whether Jolt moves the body origin to the centre of the vertices' bounds every step.
    /// Default true; false suits a body attached to the static world.
    #[must_use]
    pub fn update_position(mut self, value: bool) -> Self {
        self.update_position = value;
        self
    }

    /// Whether the initial rotation is baked into the vertices, leaving the body rotation at
    /// identity (Jolt simulates slightly more accurately that way). Default true.
    #[must_use]
    pub fn make_rotation_identity(mut self, value: bool) -> Self {
        self.make_rotation_identity = value;
        self
    }

    /// Whether the body may fall asleep when it comes to rest. Default true.
    #[must_use]
    pub fn allow_sleeping(mut self, value: bool) -> Self {
        self.allow_sleeping = value;
        self
    }

    /// Whether queries (ray casts, shape casts and collisions) hit the faces from both sides.
    /// Default false: only from the side their counter-clockwise winding faces.
    #[must_use]
    pub fn faces_double_sided(mut self, value: bool) -> Self {
        self.faces_double_sided = value;
        self
    }

    /// Whether the body starts awake. Default [`Activation::Activate`].
    #[must_use]
    pub fn activation(mut self, value: Activation) -> Self {
        self.activation = value;
        self
    }

    /// The soft body's collision group, fixed for its life, as for a rigid body
    /// ([`BodySettings::collision_group`](crate::BodySettings::collision_group)): Jolt checks
    /// it against each rigid body the soft body touches. Default none.
    #[must_use]
    pub fn collision_group(mut self, value: CollisionGroup) -> Self {
        self.collision_group = Some(value);
        self
    }

    pub(super) fn validate(&self, object_layer_count: u32) -> Result<(), BodyError> {
        if self.object_layer.get() >= object_layer_count {
            return Err(BodyError::UnknownObjectLayer(self.object_layer));
        }
        let check = |valid: bool, what| {
            if valid {
                Ok(())
            } else {
                Err(BodyError::InvalidValue(what))
            }
        };
        check(is_in_frame(self.position), limits::POSITION_RULE)?;
        check(self.rotation.is_valid_rotation(), ROTATION_RULE)?;
        check(
            (1..=Self::MAX_ITERATIONS).contains(&self.num_iterations),
            "iterations must be within 1..=SoftBodySettings::MAX_ITERATIONS",
        )?;
        check(
            is_finite_non_negative(self.linear_damping),
            LINEAR_DAMPING_RULE,
        )?;
        check(
            self.max_linear_velocity > 0.0
                && self.max_linear_velocity <= limits::MAX_LINEAR_VELOCITY,
            "max linear velocity must be above 0 and at most limits::MAX_LINEAR_VELOCITY",
        )?;
        check(is_restitution(self.restitution), RESTITUTION_RULE)?;
        check(is_friction(self.friction), limits::FRICTION_RULE)?;
        check(
            (0.0..=limits::MAX_SOFT_BODY_PRESSURE).contains(&self.pressure),
            "pressure must be finite and within 0..=limits::MAX_SOFT_BODY_PRESSURE",
        )?;
        check(
            is_gravity_factor(self.gravity_factor),
            limits::GRAVITY_FACTOR_RULE,
        )?;
        check(
            is_local_distance(self.vertex_radius),
            "vertex radius must be finite and within 0..=limits::MAX_SHAPE_EXTENT",
        )
    }
}

/// Soft body creation settings, owned whole by their owner.
impl JoltObject for JPH_SoftBodyCreationSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the settings (trait contract), which joltc deletes; their
        // `RefConst` releases the reference to the shared settings, and bodies created from
        // them keep their own.
        unsafe { JPH_SoftBodyCreationSettings_Destroy(ptr) };
    }
}

/// Jolt's creation settings for a body of `shared` made from `settings`, which the caller
/// validated.
pub(super) fn creation_settings(
    shared: &SoftBodySharedSettings,
    settings: &SoftBodySettings,
) -> Result<Owned<JPH_SoftBodyCreationSettings>, BodyError> {
    let position = settings.position.to_jph();
    let rotation = settings.rotation.to_jph();
    // SAFETY: `shared` is live for the call and the creation settings take their own reference
    // to it; `position` and `rotation` are live locals. The handle takes over the result.
    let creation = unsafe {
        Owned::from_raw(JPH_SoftBodyCreationSettings_Create2(
            shared.as_ptr(),
            &position,
            &rotation,
            settings.object_layer.get(),
        ))
    }
    .ok_or(BodyError::AllocationFailed)?;
    let ptr = creation.as_ptr();
    // SAFETY: `ptr` is the live settings object owned by `creation`; every input is a value.
    unsafe {
        JPH_SoftBodyCreationSettings_SetNumIterations(ptr, settings.num_iterations);
        JPH_SoftBodyCreationSettings_SetLinearDamping(ptr, settings.linear_damping);
        JPH_SoftBodyCreationSettings_SetMaxLinearVelocity(ptr, settings.max_linear_velocity);
        JPH_SoftBodyCreationSettings_SetRestitution(ptr, settings.restitution);
        JPH_SoftBodyCreationSettings_SetFriction(ptr, settings.friction);
        JPH_SoftBodyCreationSettings_SetPressure(ptr, settings.pressure);
        JPH_SoftBodyCreationSettings_SetGravityFactor(ptr, settings.gravity_factor);
        JPH_SoftBodyCreationSettings_SetVertexRadius(ptr, settings.vertex_radius);
        JPH_SoftBodyCreationSettings_SetUpdatePosition(ptr, settings.update_position);
        JPH_SoftBodyCreationSettings_SetMakeRotationIdentity(ptr, settings.make_rotation_identity);
        JPH_SoftBodyCreationSettings_SetAllowSleeping(ptr, settings.allow_sleeping);
        JPH_SoftBodyCreationSettings_SetFacesDoubleSided(ptr, settings.faces_double_sided);
    }
    if let Some(group) = &settings.collision_group {
        let group = group.to_jph();
        // SAFETY: `ptr` is live; the table behind `group` is live for the call, and the settings
        // take their own reference to it (Jolt's `RefConst<GroupFilter>`).
        unsafe { JPH_SoftBodyCreationSettings_SetCollisionGroup(ptr, &group) };
    }
    Ok(creation)
}
