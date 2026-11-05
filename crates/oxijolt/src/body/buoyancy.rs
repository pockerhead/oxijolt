//! Buoyancy and drag of a fluid on a rigid body (Jolt `Body::ApplyBuoyancyImpulse`).

use std::ptr::NonNull;

use oxijolt_sys::*;

use super::load::require;
use super::{with_locked_body, MotionType};
use crate::limits::buoyancy::{check, BuoyancyInputs};
use crate::limits::{
    is_acceleration, is_in_frame, is_linear_velocity, GRAVITY_RULE, LINEAR_VELOCITY_RULE,
    POSITION_RULE,
};
use crate::math::is_unit;
use crate::shape::local_bounds_of;
use crate::world::DELTA_TIME_RULE;
use crate::{BodyError, BodyMut, PhysicsWorld, RVec3, Vec3};

/// A fluid for [`BodyMut::apply_buoyancy_impulse`]: its surface, how strongly it lifts and how
/// it drags. The defaults are Jolt's suggestions (`Body.h`): surface through the origin with
/// normal +Y, buoyancy 1, linear drag 0.5, angular drag 0.01, still water.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BuoyancySettings {
    surface_position: RVec3,
    surface_normal: Vec3,
    buoyancy: f32,
    linear_drag: f32,
    angular_drag: f32,
    fluid_velocity: Vec3,
}

impl Default for BuoyancySettings {
    fn default() -> Self {
        Self {
            surface_position: RVec3::ZERO,
            surface_normal: Vec3::new(0.0, 1.0, 0.0),
            buoyancy: 1.0,
            linear_drag: 0.5,
            angular_drag: 0.01,
            fluid_velocity: Vec3::ZERO,
        }
    }
}

impl BuoyancySettings {
    /// The fluid's surface: a point on it in world space (metres, within
    /// [`limits::MAX_POSITION`]) and its unit normal, pointing up out of the fluid.
    ///
    /// [`limits::MAX_POSITION`]: crate::limits::MAX_POSITION
    #[must_use]
    pub fn surface(mut self, position: RVec3, normal: Vec3) -> Self {
        self.surface_position = position;
        self.surface_normal = normal;
        self
    }

    /// Jolt's buoyancy factor: the fluid's density divided by the body's, where the body's
    /// density is its mass over Jolt's total volume of its shape. 1 floats neutrally, more
    /// floats, less sinks. Finite and not negative. Jolt takes the volume of a box, capsule,
    /// cylinder or tapered shape from its bounding box; spheres and convex hulls use their own.
    #[must_use]
    pub fn buoyancy(mut self, factor: f32) -> Self {
        self.buoyancy = factor;
        self
    }

    /// Linear drag coefficient: a quadratic drag over the area the shape's bounding box turns
    /// toward the flow. Finite and not negative.
    #[must_use]
    pub fn linear_drag(mut self, coefficient: f32) -> Self {
        self.linear_drag = coefficient;
        self
    }

    /// Angular drag coefficient, damping the spin in proportion to the submerged fraction.
    /// Finite and not negative.
    #[must_use]
    pub fn angular_drag(mut self, coefficient: f32) -> Self {
        self.angular_drag = coefficient;
        self
    }

    /// Velocity of the fluid in world space, m/s, at most [`limits::MAX_LINEAR_VELOCITY`]
    /// long.
    ///
    /// [`limits::MAX_LINEAR_VELOCITY`]: crate::limits::MAX_LINEAR_VELOCITY
    #[must_use]
    pub fn fluid_velocity(mut self, velocity: Vec3) -> Self {
        self.fluid_velocity = velocity;
        self
    }

    /// Checks the settings' own values.
    fn validate(&self) -> Result<(), BodyError> {
        require(is_in_frame(self.surface_position), POSITION_RULE)?;
        require(
            is_unit(self.surface_normal),
            "surface normal must be a finite unit vector",
        )?;
        require(
            [self.buoyancy, self.linear_drag, self.angular_drag]
                .iter()
                .all(|value| (0.0..=f32::MAX).contains(value)),
            "buoyancy and drag must be finite and not negative",
        )?;
        require(
            is_linear_velocity(self.fluid_velocity),
            LINEAR_VELOCITY_RULE,
        )
    }
}

impl BodyMut<'_> {
    /// Applies one step of buoyancy and drag of the fluid in `settings` to this body (Jolt
    /// `Body::ApplyBuoyancyImpulse`): a lift at the centre of the submerged volume, quadratic
    /// drag against the flow and angular drag. Call it once per step, before
    /// [`PhysicsWorld::step`], with the step's `delta_time` in seconds and the `gravity` the
    /// body feels (m/s²; for radial gravity, the vector at the body).
    ///
    /// Returns `Ok(true)` when part of the body is below the surface. The velocities change at
    /// once, are then clamped to [`limits::MAX_LINEAR_VELOCITY`] and
    /// [`limits::MAX_ANGULAR_VELOCITY`] with the locked axes zeroed, as an impulse's are, and the
    /// body wakes (also when the factors make the impulse zero). Returns `Ok(false)` and changes
    /// nothing for a body above the surface, and for static and kinematic bodies.
    ///
    /// The lift scales with the body's gravity factor, so a body with gravity factor 0 gets
    /// none. In one call linear drag changes the velocity by at most the body's own speed and
    /// angular drag the spin by at most the spin, so a current takes a few steps to catch a
    /// slow body; only a body exactly at rest, as in zero gravity, stays put.
    ///
    /// Fails with [`BodyError::NotRigidBody`] for a soft body and with [`BodyError::InvalidValue`],
    /// changing nothing, when `settings`, `gravity` (at most [`limits::MAX_ACCELERATION`] long)
    /// or `delta_time` (within the bounds of [`PhysicsWorld::step`]) is out of range, when the
    /// buoyant velocity change `buoyancy · submerged / total volume · |gravity factor| ·
    /// |gravity| · delta_time` would exceed [`limits::MAX_VELOCITY_CHANGE`], or when a product
    /// in Jolt's arithmetic could overflow ([docs/limits.md#buoyancy]).
    ///
    /// ```
    /// # use oxijolt::*;
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let crate_shape = Shape::new_box(Vec3::new(0.5, 0.25, 0.5))?;
    /// let id = world.create_body(&crate_shape, &BodySettings::new_dynamic())?;
    /// // Water up to y = 0, four times as dense as the crate.
    /// let water = BuoyancySettings::default().buoyancy(4.0);
    /// let gravity = world.gravity();
    /// let dt = 1.0 / 60.0;
    /// let submerged = world
    ///     .body_mut(id)?
    ///     .apply_buoyancy_impulse(&water, gravity, dt)?;
    /// assert!(submerged);
    /// assert!(world.step(dt)?.is_complete());
    /// # Ok::<(), oxijolt::error::Error>(())
    /// ```
    ///
    /// [`limits::MAX_LINEAR_VELOCITY`]: crate::limits::MAX_LINEAR_VELOCITY
    /// [`limits::MAX_ANGULAR_VELOCITY`]: crate::limits::MAX_ANGULAR_VELOCITY
    /// [`limits::MAX_ACCELERATION`]: crate::limits::MAX_ACCELERATION
    /// [`limits::MAX_VELOCITY_CHANGE`]: crate::limits::MAX_VELOCITY_CHANGE
    /// [docs/limits.md#buoyancy]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#buoyancy
    pub fn apply_buoyancy_impulse(
        &mut self,
        settings: &BuoyancySettings,
        gravity: Vec3,
        delta_time: f32,
    ) -> Result<bool, BodyError> {
        self.reject_soft_body()?;
        settings.validate()?;
        require(is_acceleration(gravity), GRAVITY_RULE)?;
        require(
            PhysicsWorld::is_valid_delta_time(delta_time),
            DELTA_TIME_RULE,
        )?;
        if self.motion_type() != MotionType::Dynamic {
            return Ok(false);
        }
        // joltc reads the gravity factor only through the locking body interface, so it is read
        // before the body is locked; the world is borrowed mutably, so nothing changes it.
        // SAFETY: the world is borrowed mutably through this view and holds the body; this
        // thread holds no body lock.
        let gravity_factor =
            unsafe { JPH_BodyInterface_GetGravityFactor(self.interface(), self.id.raw) };
        let applied = with_locked_body(self.inner.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure, and it is a
            // dynamic rigid body (checked above, under the same `&mut` world borrow).
            unsafe { apply_locked(body, settings, gravity, gravity_factor, delta_time) }
        })
        .ok_or(BodyError::NotFound(self.id))??;
        if applied {
            self.activate();
        }
        Ok(applied)
    }
}

/// The checks and the impulse of [`BodyMut::apply_buoyancy_impulse`] on the locked body.
///
/// # Safety
/// `body` is a dynamic rigid body, locked for writing for the duration of the call.
unsafe fn apply_locked(
    body: NonNull<JPH_Body>,
    settings: &BuoyancySettings,
    gravity: Vec3,
    gravity_factor: f32,
    delta_time: f32,
) -> Result<bool, BodyError> {
    let body = body.as_ptr();
    let surface = settings.surface_position.to_jph();
    let normal = settings.surface_normal.to_jph();
    let (mut total_volume, mut submerged_volume) = (0.0, 0.0);
    let mut center = Vec3::ZERO.to_jph();
    // SAFETY: `body` is locked (contract) and a rigid body that is not static-only (dynamic
    // bodies hold no static-only shape), so Jolt computes its volume; every argument is a live
    // local.
    let found = unsafe {
        JPH_Body_GetSubmergedVolume(
            body,
            &surface,
            &normal,
            &mut total_volume,
            &mut submerged_volume,
            &mut center,
        )
    };
    let center_of_buoyancy = Vec3::from_jph(center);
    require(
        found
            && total_volume.is_finite()
            && submerged_volume.is_finite()
            && center_of_buoyancy.is_finite(),
        "the body's submerged volume is not finite",
    )?;
    if submerged_volume <= 0.0 {
        return Ok(false);
    }
    require(total_volume > 0.0, "the body's shape has no volume")?;
    // SAFETY: as above; the body is dynamic, so it has motion properties.
    let state = unsafe { read_body(body) };
    let inputs = BuoyancyInputs {
        buoyancy: settings.buoyancy,
        linear_drag: settings.linear_drag,
        angular_drag: settings.angular_drag,
        fluid_velocity: settings.fluid_velocity,
        gravity,
        gravity_factor,
        delta_time,
        total_volume,
        submerged_volume,
        center_of_buoyancy,
        inverse_mass: state.inverse_mass,
        largest_inverse_inertia: state.largest_inverse_inertia,
        linear_velocity: state.linear_velocity,
        angular_velocity: state.angular_velocity,
        bounds_size: state.bounds_size,
    };
    check(&inputs).map_err(|rule| BodyError::InvalidValue(rule.message()))?;
    let (fluid_velocity, gravity) = (inputs.fluid_velocity.to_jph(), gravity.to_jph());
    // SAFETY: `body` is a locked dynamic rigid body with a positive inverse mass (it is dynamic,
    // `MIN_MASS..=MAX_MASS`) and `total_volume > 0`; `check` bounds every product of Jolt's
    // arithmetic for exactly these values. The vectors are live locals.
    let applied = unsafe {
        JPH_Body_ApplyBuoyancyImpulse2(
            body,
            total_volume,
            submerged_volume,
            &center,
            settings.buoyancy,
            settings.linear_drag,
            settings.angular_drag,
            &fluid_velocity,
            &gravity,
            delta_time,
        )
    };
    if applied {
        // SAFETY: as above; the clamped setters run on a dynamic body and read live locals.
        unsafe { clamp_velocities(body) };
    }
    Ok(applied)
}

/// Clamps both velocities of `body` to its maxima and zeroes its locked axes, as Jolt does after
/// an impulse; buoyancy itself does neither (`MotionProperties::AddLinearVelocityStep`).
///
/// # Safety
/// `body` is a dynamic body locked for writing.
unsafe fn clamp_velocities(body: *mut JPH_Body) {
    let (mut linear, mut angular) = (Vec3::ZERO.to_jph(), Vec3::ZERO.to_jph());
    // SAFETY: `body` is locked and dynamic (contract); the outputs and inputs are live locals.
    unsafe {
        JPH_Body_GetLinearVelocity(body, &mut linear);
        JPH_Body_GetAngularVelocity(body, &mut angular);
        JPH_Body_SetLinearVelocityClamped(body, &linear);
        JPH_Body_SetAngularVelocityClamped(body, &angular);
    }
}

/// What [`apply_locked`] reads of the body itself.
struct BodyState {
    inverse_mass: f32,
    largest_inverse_inertia: f32,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    bounds_size: Vec3,
}

/// Reads what Jolt's buoyancy impulse uses of `body`.
///
/// # Safety
/// `body` is a dynamic rigid body, locked for the duration of the call.
unsafe fn read_body(body: *mut JPH_Body) -> BodyState {
    let mut inverse_inertia = Vec3::ZERO.to_jph();
    let (mut linear, mut angular) = (Vec3::ZERO.to_jph(), Vec3::ZERO.to_jph());
    // SAFETY: `body` is locked and dynamic (contract), so it has motion properties and a shape,
    // which the body keeps alive; the getters only read, into live locals.
    let (inverse_mass, (min, max)) = unsafe {
        let motion = JPH_Body_GetMotionProperties(body);
        JPH_MotionProperties_GetInverseInertiaDiagonal(motion, &mut inverse_inertia);
        JPH_Body_GetLinearVelocity(body, &mut linear);
        JPH_Body_GetAngularVelocity(body, &mut angular);
        (
            JPH_MotionProperties_GetInverseMassUnchecked(motion),
            local_bounds_of(JPH_Body_GetShape(body)),
        )
    };
    let inverse_inertia = Vec3::from_jph(inverse_inertia);
    BodyState {
        inverse_mass,
        largest_inverse_inertia: inverse_inertia
            .x
            .max(inverse_inertia.y)
            .max(inverse_inertia.z),
        linear_velocity: Vec3::from_jph(linear),
        angular_velocity: Vec3::from_jph(angular),
        bounds_size: Vec3::new(max.x - min.x, max.y - min.y, max.z - min.z),
    }
}
