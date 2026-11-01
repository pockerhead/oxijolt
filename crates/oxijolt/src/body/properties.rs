//! Body properties Jolt can change on a live body: materials, gravity, damping and motion
//! quality.
//!
//! Jolt saves none of them in a [`WorldState`](crate::WorldState), and rereads them in every
//! step: a restore keeps the values set last, so a caller that rolls back sets the old values
//! back and replays ([docs/state.md#what-is-not-saved]).
//!
//! [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved

use oxijolt_sys::*;

use super::load::require;
use super::{
    is_restitution, with_locked_body, with_read_locked_body, BodyMut, BodyRef, MotionQuality,
    ANGULAR_DAMPING_RULE, LINEAR_DAMPING_RULE, RESTITUTION_RULE, SENSOR_QUALITY_RULE,
};
use crate::limits::{self, is_friction, is_gravity_factor};
use crate::math::is_finite_non_negative;
use crate::BodyError;

/// Which bodies a motion property applies to.
#[derive(Clone, Copy, PartialEq)]
enum Bodies {
    /// Every body with motion properties, soft bodies too.
    Moving,
    /// Rigid bodies with motion properties: Jolt's soft body solver uses no angular damping and
    /// no motion quality.
    MovingRigid,
}

impl BodyRef<'_> {
    /// The friction coefficient, `0..=`[`limits::MAX_FRICTION`]
    /// ([`BodySettings::friction`](crate::BodySettings::friction),
    /// [`SoftBodySettings::friction`](crate::SoftBodySettings::friction)).
    pub fn friction(&self) -> f32 {
        // SAFETY: the body interface belongs to the world this view borrows, which holds the
        // body and cannot change while the borrow lasts.
        unsafe { JPH_BodyInterface_GetFriction(self.interface(), self.id.raw) }
    }

    /// The restitution, `0..=1` ([`BodySettings::restitution`](crate::BodySettings::restitution),
    /// [`SoftBodySettings::restitution`](crate::SoftBodySettings::restitution)).
    pub fn restitution(&self) -> f32 {
        // SAFETY: as in `friction`.
        unsafe { JPH_BodyInterface_GetRestitution(self.interface(), self.id.raw) }
    }

    /// The multiplier for the world's gravity
    /// ([`BodySettings::gravity_factor`](crate::BodySettings::gravity_factor)); `None` for a body
    /// without motion properties, a static body that cannot move.
    pub fn gravity_factor(&self) -> Option<f32> {
        self.can_be_kinematic_or_dynamic().then(|| {
            // SAFETY: as in `friction`; no body lock is held on this thread.
            unsafe { JPH_BodyInterface_GetGravityFactor(self.interface(), self.id.raw) }
        })
    }

    /// The linear damping, in 1/s
    /// ([`BodySettings::linear_damping`](crate::BodySettings::linear_damping),
    /// [`SoftBodySettings::linear_damping`](crate::SoftBodySettings::linear_damping)); `None`
    /// for a body without motion properties.
    pub fn linear_damping(&self) -> Option<f32> {
        self.read_motion_properties(Bodies::Moving, |properties| {
            // SAFETY: `properties` belongs to the body locked for reading around this call; the
            // getter only reads.
            unsafe { JPH_MotionProperties_GetLinearDamping(properties) }
        })
    }

    /// The angular damping, in 1/s
    /// ([`BodySettings::angular_damping`](crate::BodySettings::angular_damping)); `None` for a
    /// body without motion properties and for a soft body, whose solver uses none.
    pub fn angular_damping(&self) -> Option<f32> {
        self.read_motion_properties(Bodies::MovingRigid, |properties| {
            // SAFETY: as in `linear_damping`.
            unsafe { JPH_MotionProperties_GetAngularDamping(properties) }
        })
    }

    /// How the body is integrated
    /// ([`BodySettings::motion_quality`](crate::BodySettings::motion_quality)); `None` for a body
    /// without motion properties and for a soft body. A static body created with
    /// [`BodySettings::allow_dynamic_or_kinematic`](crate::BodySettings::allow_dynamic_or_kinematic)
    /// reports the quality it will move with.
    pub fn motion_quality(&self) -> Option<MotionQuality> {
        self.read_motion_properties(Bodies::MovingRigid, |properties| {
            // SAFETY: as in `linear_damping`.
            MotionQuality::from_jph(unsafe { JPH_MotionProperties_GetMotionQuality(properties) })
        })
    }

    /// Runs `read` on the motion properties of the body, locked for reading, when it is one of
    /// `bodies`; `None` otherwise.
    fn read_motion_properties<R>(
        &self,
        bodies: Bodies,
        read: impl FnOnce(*mut JPH_MotionProperties) -> R,
    ) -> Option<R> {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            let body = body.as_ptr();
            // SAFETY: `body` is locked for reading for the duration of the closure. A body that
            // can be kinematic or dynamic has motion properties, so the unchecked getter returns
            // a live member, without the assertion of the checked getter on a static body that
            // may move; joltc takes a mutable pointer but only reads it, and `read` only reads.
            unsafe {
                let rigid_only = bodies == Bodies::MovingRigid;
                if !JPH_Body_CanBeKinematicOrDynamic(body)
                    || (rigid_only && JPH_Body_IsSoftBody(body))
                {
                    return None;
                }
                Some(read(JPH_Body_GetMotionPropertiesUnchecked(body)))
            }
        })
        .flatten()
    }
}

impl BodyMut<'_> {
    /// Sets the friction coefficient, `0..=`[`limits::MAX_FRICTION`], on any body, rigid or soft,
    /// static too (Jolt `Body::SetFriction`).
    ///
    /// Jolt combines the friction of a contact again in every step from both bodies' values, so
    /// the new value acts from the next step on in every contact of an awake pair. It wakes
    /// nothing: a sleeping body uses it once something wakes it ([`activate`](Self::activate)).
    /// The value is not part of a [`WorldState`](crate::WorldState): a restore keeps it
    /// ([docs/state.md#what-is-not-saved]).
    ///
    /// Fails with [`BodyError::InvalidValue`] for a value outside the range, changing nothing.
    ///
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_friction(&mut self, friction: f32) -> Result<(), BodyError> {
        require(is_friction(friction), limits::FRICTION_RULE)?;
        // SAFETY: the world is borrowed mutably through this view and holds the body; this
        // thread holds no body lock.
        unsafe { JPH_BodyInterface_SetFriction(self.interface(), self.id.raw, friction) };
        Ok(())
    }

    /// Sets the restitution, `0..=1`, on any body, rigid or soft, static too (Jolt
    /// `Body::SetRestitution`).
    ///
    /// Jolt combines it in every step like the friction ([`set_friction`](Self::set_friction)),
    /// and as there it wakes nothing and a restore keeps it ([docs/state.md#what-is-not-saved]).
    ///
    /// Fails with [`BodyError::InvalidValue`] for a value outside the range, changing nothing.
    ///
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_restitution(&mut self, restitution: f32) -> Result<(), BodyError> {
        require(is_restitution(restitution), RESTITUTION_RULE)?;
        // SAFETY: as in `set_friction`.
        unsafe { JPH_BodyInterface_SetRestitution(self.interface(), self.id.raw, restitution) };
        Ok(())
    }

    /// Sets the multiplier for the world's gravity, at most [`limits::MAX_GRAVITY_FACTOR`] in
    /// absolute value, on a body with motion properties, soft bodies too (Jolt
    /// `MotionProperties::SetGravityFactor`). A kinematic body keeps it for when it becomes
    /// dynamic.
    ///
    /// It acts from the next step on and wakes nothing: a sleeping body keeps resting until
    /// something wakes it ([`activate`](Self::activate)). A restore keeps it
    /// ([docs/state.md#what-is-not-saved]). A vehicle whose gravity is set with
    /// [`VehicleMut::set_gravity`](crate::VehicleMut::set_gravity) writes 0 into its chassis'
    /// factor in every step in which the chassis is awake.
    ///
    /// Fails, changing nothing, with [`BodyError::CannotMove`] for a body created static without
    /// [`BodySettings::allow_dynamic_or_kinematic`](crate::BodySettings::allow_dynamic_or_kinematic),
    /// and with [`BodyError::InvalidValue`] for a value outside the range.
    ///
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_gravity_factor(&mut self, factor: f32) -> Result<(), BodyError> {
        self.require_motion_properties(Bodies::Moving)?;
        require(is_gravity_factor(factor), limits::GRAVITY_FACTOR_RULE)?;
        // SAFETY: as in `set_friction`. The body has motion properties (checked above), which
        // Jolt's setter checks again.
        unsafe { JPH_BodyInterface_SetGravityFactor(self.interface(), self.id.raw, factor) };
        Ok(())
    }

    /// Sets the linear damping in 1/s, finite and at least 0, on a body with motion properties,
    /// soft bodies too: Jolt scales the linear velocity (of a soft body, every vertex velocity
    /// in every sub-step) by `max(0, 1 - c * dt)`.
    ///
    /// It acts from the next step on and wakes nothing ([`activate`](Self::activate) wakes the
    /// body). A restore keeps it ([docs/state.md#what-is-not-saved]).
    ///
    /// Fails, changing nothing, with [`BodyError::CannotMove`] for a body created static without
    /// [`BodySettings::allow_dynamic_or_kinematic`](crate::BodySettings::allow_dynamic_or_kinematic),
    /// and with [`BodyError::InvalidValue`] for a value outside the range.
    ///
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_linear_damping(&mut self, damping: f32) -> Result<(), BodyError> {
        self.require_motion_properties(Bodies::Moving)?;
        require(is_finite_non_negative(damping), LINEAR_DAMPING_RULE)?;
        self.write_motion_properties(|properties| {
            // SAFETY: `properties` belongs to the body locked for writing around this call.
            unsafe { JPH_MotionProperties_SetLinearDamping(properties, damping) }
        });
        Ok(())
    }

    /// Sets the angular damping in 1/s, finite and at least 0, on a rigid body with motion
    /// properties: Jolt scales the angular velocity by `max(0, 1 - c * dt)` every step.
    ///
    /// It acts from the next step on and wakes nothing ([`activate`](Self::activate) wakes the
    /// body). A restore keeps it ([docs/state.md#what-is-not-saved]).
    ///
    /// Fails, changing nothing, with [`BodyError::NotRigidBody`] for a soft body, whose solver
    /// uses no angular damping, [`BodyError::CannotMove`] for a body created static without
    /// [`BodySettings::allow_dynamic_or_kinematic`](crate::BodySettings::allow_dynamic_or_kinematic),
    /// and [`BodyError::InvalidValue`] for a value outside the range.
    ///
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_angular_damping(&mut self, damping: f32) -> Result<(), BodyError> {
        self.require_motion_properties(Bodies::MovingRigid)?;
        require(is_finite_non_negative(damping), ANGULAR_DAMPING_RULE)?;
        self.write_motion_properties(|properties| {
            // SAFETY: as in `set_linear_damping`.
            unsafe { JPH_MotionProperties_SetAngularDamping(properties, damping) }
        });
        Ok(())
    }

    /// Sets how the body is integrated, on a rigid body with motion properties (Jolt
    /// `BodyInterface::SetMotionQuality`, which keeps Jolt's count of active continuous bodies
    /// right for awake and sleeping bodies alike). A static body that may move keeps it for when
    /// it moves.
    ///
    /// It acts from the next step on and wakes nothing ([`activate`](Self::activate) wakes the
    /// body). A restore keeps it ([docs/state.md#what-is-not-saved]).
    ///
    /// Fails, changing nothing, with [`BodyError::NotRigidBody`] for a soft body,
    /// [`BodyError::CannotMove`] for a body created static without
    /// [`BodySettings::allow_dynamic_or_kinematic`](crate::BodySettings::allow_dynamic_or_kinematic),
    /// and [`BodyError::InvalidValue`] for [`MotionQuality::LinearCast`] on a sensor, which Jolt
    /// integrates only discretely.
    ///
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_motion_quality(&mut self, quality: MotionQuality) -> Result<(), BodyError> {
        self.require_motion_properties(Bodies::MovingRigid)?;
        require(
            !(quality == MotionQuality::LinearCast && self.is_sensor()),
            SENSOR_QUALITY_RULE,
        )?;
        // SAFETY: as in `set_friction`. The body is rigid and has motion properties (checked
        // above); no step runs while the world is borrowed mutably, so Jolt's active body list
        // is not locked, as its setter asserts.
        unsafe {
            JPH_BodyInterface_SetMotionQuality(self.interface(), self.id.raw, quality.to_jph())
        };
        Ok(())
    }

    /// Fails with [`BodyError::NotRigidBody`] for a soft body when `bodies` asks for a rigid
    /// one, then with [`BodyError::CannotMove`] for a body without motion properties.
    fn require_motion_properties(&self, bodies: Bodies) -> Result<(), BodyError> {
        if bodies == Bodies::MovingRigid {
            self.reject_soft_body()?;
        }
        if !self.can_be_kinematic_or_dynamic() {
            return Err(BodyError::CannotMove(self.id));
        }
        Ok(())
    }

    /// Runs `write` on the motion properties of the body, locked for writing. The caller checked
    /// that the body has motion properties.
    fn write_motion_properties(&mut self, write: impl FnOnce(*mut JPH_MotionProperties)) {
        with_locked_body(self.inner.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure, and it has
            // motion properties (the caller checked `CanBeKinematicOrDynamic`, which the
            // mutable borrow of the world keeps true), so the unchecked getter returns a live
            // member.
            write(unsafe { JPH_Body_GetMotionPropertiesUnchecked(body.as_ptr()) })
        })
        .unwrap_or_else(|| unreachable!("`&mut` keeps the body in the world"));
    }
}
