//! Momentary inputs to bodies: impulses, kinematic moves and activation.

use std::ptr::NonNull;

use oxijolt_sys::*;

use super::load::{point_torque, read_load, require, Load, LoadState};
use super::lock::for_each_locked_body;
use super::{kinematic_velocities, with_locked_body, with_read_locked_body, MotionType};
use crate::limits::{
    is_angular_velocity, is_angular_velocity_change, is_in_frame, is_linear_velocity,
    is_velocity_change, ANGULAR_VELOCITY_CHANGE_RULE, POSITION_RULE, VELOCITY_CHANGE_RULE,
};
use crate::math::ROTATION_RULE;
use crate::world::DELTA_TIME_RULE;
use crate::{BodyError, BodyId, BodyMut, PhysicsWorld, Quat, RVec3, Vec3};

/// What [`BodyMut::move_kinematic`] must satisfy: the velocities Jolt derives are within the
/// speed bounds.
const KINEMATIC_MOVE_RULE: &str =
    "a kinematic move must imply velocities within the limits velocity bounds";

/// `v` in `f64`.
fn wide(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z].map(f64::from)
}

/// The largest component of `v`.
fn largest(v: Vec3) -> f32 {
    v.x.max(v.y).max(v.z)
}

/// Zeroes the linear and angular velocity of `body` unless it is static. Jolt zeroes the
/// velocities of an awake body it deactivates and leaves those of a sleeping one, which a body
/// created asleep may still have; the clamped setters used here never wake the body.
///
/// # Safety
/// `body` is locked for writing for the duration of the call.
unsafe fn stop_unless_static(body: NonNull<JPH_Body>) {
    let zero = Vec3::ZERO.to_jph();
    // SAFETY: `body` is locked for writing (function contract), and the clamped setters run only
    // on a body that is not static, as they assert. `zero` is a live local.
    unsafe {
        if !JPH_Body_IsStatic(body.as_ptr()) {
            JPH_Body_SetLinearVelocityClamped(body.as_ptr(), &zero);
            JPH_Body_SetAngularVelocityClamped(body.as_ptr(), &zero);
        }
    }
}

impl PhysicsWorld {
    /// Puts the bodies `ids` to sleep now, in the order given, and zeroes the linear and angular
    /// velocity of each, also of those asleep already (Jolt `BodyInterface::DeactivateBodies`):
    /// the same result, bit for bit, as [`BodyMut::deactivate`] called for each id in the same
    /// order, with the body locks taken once for the whole slice instead of for each body.
    ///
    /// Every id is checked before anything changes: an id of another world
    /// ([`BodyError::WrongWorld`]), of no body ([`BodyError::NotFound`]), of a detached body
    /// ([`BodyError::Detached`]), of a soft body ([`BodyError::NotRigidBody`]) or listed twice
    /// ([`BodyError::DuplicateId`]) fails the call and puts nothing to sleep. Static bodies are
    /// ignored, as `deactivate` ignores them. An empty slice does nothing.
    ///
    /// The order of `ids` is part of the simulation state: Jolt takes each body out of its list of
    /// awake bodies by moving the last one into its place, so the same bodies in another order
    /// can leave that list, and the steps after it, different. `ids` is not reordered. Each body that
    /// was awake records [`ActivationEvent::Deactivated`], in the order of `ids`.
    ///
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    /// let mut pile = Vec::new();
    /// for y in 0..3 {
    ///     let at = RVec3::new(0.0, 2.0 * y as Real, 0.0);
    ///     pile.push(world.create_body(&cube, &BodySettings::new_dynamic().position(at))?);
    /// }
    /// world.deactivate_bodies(&pile)?;
    /// assert!(world.body(pile[0])?.is_sleeping());
    /// assert_eq!(world.body(pile[2])?.linear_velocity(), Vec3::ZERO);
    /// # Ok(())
    /// # }
    /// ```
    ///
    /// [`ActivationEvent::Deactivated`]: crate::ActivationEvent::Deactivated
    pub fn deactivate_bodies(&mut self, ids: &[BodyId]) -> Result<(), BodyError> {
        for &id in ids {
            if self.body(id)?.is_soft_body() {
                return Err(BodyError::NotRigidBody(id));
            }
        }
        self.sorted_unique(ids)?;
        if ids.is_empty() {
            return Ok(());
        }
        let raw: Vec<JPH_BodyID> = ids.iter().map(|id| id.raw).collect();
        // SAFETY: the body interface belongs to this live world, borrowed mutably; `raw` holds
        // `raw.len()` ids, which joltc copies. The checks above confirmed that they are distinct
        // rigid bodies in the world, as Jolt asserts, so the count fits `u32` (a world holds at
        // most `1 << 23` bodies). This thread holds no body lock.
        unsafe {
            JPH_BodyInterface_DeactivateBodies(
                self.body_interface.as_ptr(),
                raw.as_ptr(),
                raw.len() as u32,
            )
        };
        let stopped = for_each_locked_body(self.body_lock_interface, &raw, |body| {
            // SAFETY: `for_each_locked_body` passes a body locked for writing for this call.
            unsafe { stop_unless_static(body) }
        });
        assert!(stopped, "checked bodies were not found");
        Ok(())
    }
}

impl BodyMut<'_> {
    /// Adds an impulse in N·s at the centre of mass: the linear velocity changes at once by
    /// `impulse / mass`, and the body wakes.
    ///
    /// The impulse must be finite, and on a dynamic body the velocity change `|impulse| / mass`
    /// at most [`limits::MAX_VELOCITY_CHANGE`]; otherwise [`BodyError::InvalidValue`] is
    /// returned and nothing changes. Jolt then clamps the new velocity to
    /// [`limits::MAX_LINEAR_VELOCITY`] and zeroes its locked axes. Static and kinematic bodies
    /// ignore impulses, as they ignore forces. Fails with [`BodyError::NotRigidBody`] for a soft
    /// body.
    ///
    /// [`limits::MAX_VELOCITY_CHANGE`]: crate::limits::MAX_VELOCITY_CHANGE
    /// [`limits::MAX_LINEAR_VELOCITY`]: crate::limits::MAX_LINEAR_VELOCITY
    pub fn add_impulse(&mut self, impulse: Vec3) -> Result<(), BodyError> {
        self.reject_soft_body()?;
        require(impulse.is_finite(), "impulse must be finite")?;
        if let Some(state) = self.dynamic_state()? {
            require(
                is_velocity_change(wide(impulse), state.inverse_mass),
                VELOCITY_CHANGE_RULE,
            )?;
        }
        let mut impulse = impulse.to_jph();
        // SAFETY: the world is borrowed mutably through this view and holds the body; this
        // thread holds no body lock, and joltc only reads `impulse`, a live local.
        unsafe { JPH_BodyInterface_AddImpulse(self.interface(), self.id.raw, &mut impulse) };
        Ok(())
    }

    /// Adds an angular impulse in N·m·s: the angular velocity changes at once by the world
    /// inverse inertia times `angular_impulse`, and the body wakes.
    ///
    /// The impulse must be finite, and on a dynamic body `|angular_impulse|` times the largest
    /// principal inverse inertia at most [`limits::MAX_ANGULAR_VELOCITY_CHANGE`]; otherwise
    /// [`BodyError::InvalidValue`] is returned and nothing changes. Jolt clamps the new angular
    /// velocity to [`limits::MAX_ANGULAR_VELOCITY`]. Static and kinematic bodies ignore it;
    /// fails with [`BodyError::NotRigidBody`] for a soft body.
    ///
    /// [`limits::MAX_ANGULAR_VELOCITY_CHANGE`]: crate::limits::MAX_ANGULAR_VELOCITY_CHANGE
    /// [`limits::MAX_ANGULAR_VELOCITY`]: crate::limits::MAX_ANGULAR_VELOCITY
    pub fn add_angular_impulse(&mut self, angular_impulse: Vec3) -> Result<(), BodyError> {
        self.reject_soft_body()?;
        require(
            angular_impulse.is_finite(),
            "angular impulse must be finite",
        )?;
        if let Some(state) = self.dynamic_state()? {
            require(
                is_angular_velocity_change(wide(angular_impulse), largest(state.inverse_inertia)),
                ANGULAR_VELOCITY_CHANGE_RULE,
            )?;
        }
        let mut angular_impulse = angular_impulse.to_jph();
        // SAFETY: as in `add_impulse`.
        unsafe {
            JPH_BodyInterface_AddAngularImpulse(self.interface(), self.id.raw, &mut angular_impulse)
        };
        Ok(())
    }

    /// Adds an impulse in N·s at a world-space `point`, which also adds the angular impulse
    /// `(point - centre_of_mass) × impulse`, and wakes the body.
    ///
    /// The impulse must be finite and the point within [`limits::MAX_POSITION`]. On a dynamic
    /// body both rules of [`add_impulse`](Self::add_impulse) and
    /// [`add_angular_impulse`](Self::add_angular_impulse) apply, the angular rule to the angular
    /// impulse Jolt's `f32` cross product can produce, rounding included; otherwise
    /// [`BodyError::InvalidValue`] is returned and nothing changes. Static and kinematic bodies
    /// ignore it; fails with [`BodyError::NotRigidBody`] for a soft body. [docs/limits.md#impulses]
    /// has the rounding bound.
    ///
    /// [`limits::MAX_POSITION`]: crate::limits::MAX_POSITION
    /// [docs/limits.md#impulses]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#impulses
    pub fn add_impulse_at_point(&mut self, impulse: Vec3, point: RVec3) -> Result<(), BodyError> {
        self.reject_soft_body()?;
        require(impulse.is_finite(), "impulse must be finite")?;
        require(
            is_in_frame(point),
            "point must be finite and within limits::MAX_POSITION",
        )?;
        if let Some(state) = self.dynamic_state()? {
            require(
                is_velocity_change(wide(impulse), state.inverse_mass),
                VELOCITY_CHANGE_RULE,
            )?;
            let angular_impulse = point_torque(impulse, point, state.center_of_mass)?;
            require(
                is_angular_velocity_change(
                    angular_impulse.largest_with([0.0; 3]),
                    largest(state.inverse_inertia),
                ),
                ANGULAR_VELOCITY_CHANGE_RULE,
            )?;
        }
        let mut impulse = impulse.to_jph();
        let mut point = point.to_jph();
        // SAFETY: as in `add_impulse`; joltc only reads the two live locals.
        unsafe {
            JPH_BodyInterface_AddImpulse2(self.interface(), self.id.raw, &mut impulse, &mut point)
        };
        Ok(())
    }

    /// Sets the velocities of a kinematic body so that it moves toward `position` (of the body
    /// origin, metres) and `rotation` over the next step of `delta_time` seconds (Jolt
    /// `BodyInterface::MoveKinematic`), and wakes it when they are not near zero.
    ///
    /// Jolt aims the centre of mass at where it is in the target pose, turns with a small-angle
    /// approximation and zeroes the locked axes, so the body approaches the pose rather than
    /// meeting it exactly. The velocity
    /// stays after the step: call this every step the body should move, and set a zero velocity
    /// to stop it. A sleeping body whose new velocities have a squared length of at most 1e-12
    /// is not woken and does not move in the next step.
    ///
    /// `position` must be within [`limits::MAX_POSITION`], `rotation` a unit quaternion and
    /// `delta_time` within the bounds of [`PhysicsWorld::step`], and the velocities Jolt derives
    /// (before it zeroes the locked axes) at most [`limits::MAX_LINEAR_VELOCITY`] and
    /// [`limits::MAX_ANGULAR_VELOCITY`] long, as Jolt computes them; otherwise
    /// [`BodyError::InvalidValue`] is returned and nothing changes. Fails with
    /// [`BodyError::NotKinematic`] for a static or dynamic body and with
    /// [`BodyError::NotRigidBody`] for a soft body. [docs/limits.md#kinematic-drive] has the
    /// derivation.
    ///
    /// [`limits::MAX_POSITION`]: crate::limits::MAX_POSITION
    /// [`limits::MAX_LINEAR_VELOCITY`]: crate::limits::MAX_LINEAR_VELOCITY
    /// [`limits::MAX_ANGULAR_VELOCITY`]: crate::limits::MAX_ANGULAR_VELOCITY
    /// [docs/limits.md#kinematic-drive]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#kinematic-drive
    pub fn move_kinematic(
        &mut self,
        position: RVec3,
        rotation: Quat,
        delta_time: f32,
    ) -> Result<(), BodyError> {
        self.reject_soft_body()?;
        if self.motion_type() != MotionType::Kinematic {
            return Err(BodyError::NotKinematic(self.id));
        }
        require(is_in_frame(position), POSITION_RULE)?;
        require(rotation.is_valid_rotation(), ROTATION_RULE)?;
        require(
            PhysicsWorld::is_valid_delta_time(delta_time),
            DELTA_TIME_RULE,
        )?;
        // SAFETY: the world is borrowed mutably through this view and holds the body, a rigid
        // body; this thread holds no body lock.
        let velocities = unsafe {
            kinematic_velocities(
                self.inner.body_interface,
                self.id,
                position,
                rotation,
                delta_time,
            )
        };
        let within_limits = velocities.is_some_and(|(linear, angular)| {
            is_linear_velocity(linear) && is_angular_velocity(angular)
        });
        require(within_limits, KINEMATIC_MOVE_RULE)?;
        let mut position = position.to_jph();
        let mut rotation = rotation.to_jph();
        // SAFETY: as in `add_impulse`. The body is kinematic, as Jolt's `Body::MoveKinematic`
        // asserts (not static, rigid), and the inputs were validated above.
        unsafe {
            JPH_BodyInterface_MoveKinematic(
                self.interface(),
                self.id.raw,
                &mut position,
                &mut rotation,
                delta_time,
            )
        };
        Ok(())
    }

    /// Wakes the body (Jolt `BodyInterface::ActivateBody`); an awake body restarts its sleep
    /// timer. Records [`ActivationEvent::Activated`] for a body that was asleep. Static bodies
    /// ignore it.
    ///
    /// [`ActivationEvent::Activated`]: crate::ActivationEvent::Activated
    pub fn activate(&mut self) {
        // SAFETY: the world is borrowed mutably through this view and holds the body; this
        // thread holds no body lock.
        unsafe { JPH_BodyInterface_ActivateBody(self.interface(), self.id.raw) };
    }

    /// Makes Jolt collide the pairs of this body afresh in the next step that simulates, instead
    /// of reusing their cached contacts (Jolt `BodyInterface::InvalidateContactCache`), so a
    /// [`ContactListener::contact_validate`] whose answer changed is asked again. Wakes the body,
    /// like [`activate`](Self::activate) (a static body stays asleep; its pairs are processed
    /// only while the other body is awake).
    ///
    /// The request waits for the next step in which some body is awake or a vehicle exists, the
    /// steps in which Jolt runs its collision passes; repeated requests before it are one. Until
    /// then it is part of the world's state: a [`WorldState`] saved meanwhile holds it, and
    /// [`PhysicsWorld::restore_state`] replaces the pending requests with the saved ones.
    ///
    /// [`ContactListener::contact_validate`]: crate::ContactListener::contact_validate
    /// [`WorldState`]: crate::WorldState
    /// [`PhysicsWorld::restore_state`]: crate::PhysicsWorld::restore_state
    pub fn invalidate_contact_cache(&mut self) {
        self.world.pending_cache_invalidations.insert(self.id.raw);
        self.activate();
    }

    /// Puts the body to sleep now (Jolt `BodyInterface::DeactivateBody`) and zeroes its linear
    /// and angular velocity, also when it was asleep already.
    ///
    /// Only a body that was awake records [`ActivationEvent::Deactivated`]. Forces added since the
    /// last step stay until the body wakes and steps. The body stays asleep until something wakes
    /// it (a contact with an awake body, a setter that activates, [`activate`](Self::activate)),
    /// also when it may not fall asleep on its own. A sleeping sensor no longer looks for contacts
    /// itself, but awake bodies still produce sensor contacts with it, so this does not turn a
    /// sensor off. [`PhysicsWorld::restore_state`] puts bodies back to sleep or wakes them without
    /// activation events. Static bodies ignore it; fails with [`BodyError::NotRigidBody`] for a
    /// soft body.
    ///
    /// [`ActivationEvent::Deactivated`]: crate::ActivationEvent::Deactivated
    /// [`PhysicsWorld::restore_state`]: crate::PhysicsWorld::restore_state
    pub fn deactivate(&mut self) -> Result<(), BodyError> {
        self.reject_soft_body()?;
        // SAFETY: as in `activate`.
        unsafe { JPH_BodyInterface_DeactivateBody(self.interface(), self.id.raw) };
        with_locked_body(self.inner.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure.
            unsafe { stop_unless_static(body) }
        })
        .ok_or(BodyError::NotFound(self.id))
    }

    /// What bounds an impulse on this body when it is a dynamic rigid body; `None` for static
    /// and kinematic bodies, which Jolt's impulse calls skip. Reads under a read lock that is
    /// released before the caller calls the locking body interface.
    fn dynamic_state(&self) -> Result<Option<LoadState>, BodyError> {
        let load = with_read_locked_body(self.inner.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure.
            unsafe { read_load(body) }
        })
        .ok_or(BodyError::NotFound(self.id))?;
        Ok(match load {
            Load::Rigid(state) => Some(state),
            Load::Ignored | Load::Soft(_) => None,
        })
    }
}
