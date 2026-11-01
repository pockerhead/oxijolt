//! The slider constraint: body 2 moves along an axis of body 1.

use std::ptr::NonNull;

use oxijolt_sys::*;

use super::lever::check_spring;
use super::rotational::{check_friction, check_motor};
use super::world::{sealed, ConstraintSettings};
use super::{
    non_negative, point_anchors, solver::SolverOverrides, validate_frame, within, ConstraintSpace,
    MotorSettings, SpringSettings, FRICTION_RULE, TARGET_VELOCITY_RULE,
};
use crate::limits;
use crate::{
    ConstraintError, ConstraintMut, ConstraintRef, MotorState, RVec3, SliderConstraint, Vec3,
};

/// What slider limits must satisfy: `min <= 0 <= max` within [`limits::MAX_SHAPE_EXTENT`], as
/// Jolt's `SliderConstraint::SetLimits` asserts the signs; `min == max` needs a soft spring, as
/// its constructor asserts.
fn validate_slider_limits(
    min: f32,
    max: f32,
    limits_spring: SpringSettings,
) -> Result<(), &'static str> {
    let extent = limits::MAX_SHAPE_EXTENT;
    if !(within(min, -extent, 0.0) && within(max, 0.0, extent)) {
        return Err(
            "slider limits must be min in [-MAX_SHAPE_EXTENT, 0] and max in [0, MAX_SHAPE_EXTENT]",
        );
    }
    if min == max && !limits_spring.is_soft() {
        return Err("slider limits with min == max need a soft limits spring");
    }
    Ok(())
}

/// Jolt's "no limits" for a slider.
const NO_SLIDER_LIMITS: (f32, f32) = (-f32::MAX, f32::MAX);

/// A slider (Jolt `SliderConstraintSettings`): body 2 moves along the slider axis of body 1 and
/// keeps its rotation relative to body 1. The position along the axis is 0 where both frame
/// points coincide.
///
/// The default is Jolt's: both frames at the origin with slider axis +X and normal axis +Y,
/// world space, no limits, no friction and a default motor.
#[derive(Clone, Debug, PartialEq)]
pub struct SliderConstraintSettings {
    space: ConstraintSpace,
    auto_detect_point: bool,
    point1: RVec3,
    slider_axis1: Vec3,
    normal_axis1: Vec3,
    point2: RVec3,
    slider_axis2: Vec3,
    normal_axis2: Vec3,
    limits: Option<(f32, f32)>,
    limits_spring: SpringSettings,
    max_friction_force: f32,
    motor: MotorSettings,
    pub(super) solver: SolverOverrides,
}

impl Default for SliderConstraintSettings {
    fn default() -> Self {
        Self::new(
            RVec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
    }
}

impl SliderConstraintSettings {
    /// A slider at `point` with the same frame on both bodies, in world space: `slider_axis` and
    /// the perpendicular `normal_axis` are unit vectors. No limits until set.
    pub fn new(point: RVec3, slider_axis: Vec3, normal_axis: Vec3) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            auto_detect_point: false,
            point1: point,
            slider_axis1: slider_axis,
            normal_axis1: normal_axis,
            point2: point,
            slider_axis2: slider_axis,
            normal_axis2: normal_axis,
            limits: None,
            limits_spring: SpringSettings::default(),
            max_friction_force: 0.0,
            motor: MotorSettings::default(),
            solver: SolverOverrides::default(),
        }
    }

    /// The space the frames are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The frame on body 1: a point and perpendicular unit slider and normal axes.
    #[must_use]
    pub fn frame1(mut self, point: RVec3, slider_axis: Vec3, normal_axis: Vec3) -> Self {
        self.point1 = point;
        self.slider_axis1 = slider_axis;
        self.normal_axis1 = normal_axis;
        self
    }

    /// The frame on body 2: a point and perpendicular unit slider and normal axes.
    #[must_use]
    pub fn frame2(mut self, point: RVec3, slider_axis: Vec3, normal_axis: Vec3) -> Self {
        self.point2 = point;
        self.slider_axis2 = slider_axis;
        self.normal_axis2 = normal_axis;
        self
    }

    /// Whether Jolt places the frame points between the bodies as they are when the constraint
    /// is created, instead of the given points. Only in [`ConstraintSpace::WorldSpace`]. Default
    /// false.
    #[must_use]
    pub fn auto_detect_point(mut self, value: bool) -> Self {
        self.auto_detect_point = value;
        self
    }

    /// The position limits in metres: `min` in `[-MAX_SHAPE_EXTENT, 0]`, `max` in
    /// `[0, MAX_SHAPE_EXTENT]` ([`limits::MAX_SHAPE_EXTENT`]); `min == max` needs a soft
    /// [`limits_spring`](Self::limits_spring). Default no limits.
    #[must_use]
    pub fn limits(mut self, min: f32, max: f32) -> Self {
        self.limits = Some((min, max));
        self
    }

    /// Makes the limits soft. Default rigid.
    #[must_use]
    pub fn limits_spring(mut self, value: SpringSettings) -> Self {
        self.limits_spring = value;
        self
    }

    /// Force in N that friction applies while the motor is off, at least 0. Default 0.
    #[must_use]
    pub fn max_friction_force(mut self, value: f32) -> Self {
        self.max_friction_force = value;
        self
    }

    /// The motor.
    #[must_use]
    pub fn motor(mut self, value: MotorSettings) -> Self {
        self.motor = value;
        self
    }
}

impl sealed::Settings for SliderConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        self.solver.validate()?;
        validate_frame(self.point1, self.slider_axis1, self.normal_axis1)?;
        validate_frame(self.point2, self.slider_axis2, self.normal_axis2)?;
        if self.auto_detect_point && self.space != ConstraintSpace::WorldSpace {
            return Err("an automatic point needs ConstraintSpace::WorldSpace");
        }
        self.limits_spring.validate()?;
        if let Some((min, max)) = self.limits {
            validate_slider_limits(min, max, self.limits_spring)?;
        }
        if !non_negative(self.max_friction_force) {
            return Err(FRICTION_RULE);
        }
        self.motor.validate()
    }

    fn springs(&self) -> Vec<SpringSettings> {
        vec![self.limits_spring, self.motor.spring]
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(if self.auto_detect_point {
            [sealed::Anchor::BetweenCentersOfMass; 2]
        } else {
            point_anchors(self.space, self.point1, self.point2)
        })
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let (min, max) = self.limits.unwrap_or(NO_SLIDER_LIMITS);
        let settings = JPH_SliderConstraintSettings {
            base: self.solver.to_jph(),
            space: self.space.to_jph(),
            autoDetectPoint: self.auto_detect_point,
            point1: self.point1.to_jph(),
            sliderAxis1: self.slider_axis1.to_jph(),
            normalAxis1: self.normal_axis1.to_jph(),
            point2: self.point2.to_jph(),
            sliderAxis2: self.slider_axis2.to_jph(),
            normalAxis2: self.normal_axis2.to_jph(),
            limitsMin: min,
            limitsMax: max,
            limitsSpringSettings: self.limits_spring.to_jph(),
            maxFrictionForce: self.max_friction_force,
            motorSettings: self.motor.to_jph(),
        };
        // SAFETY: as in `FixedConstraintSettings::create`.
        unsafe { JPH_SliderConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for SliderConstraintSettings {
    type Kind = SliderConstraint;
}

impl ConstraintRef<'_, SliderConstraint> {
    /// The position of body 2 along the slider axis, metres from where both frame points
    /// coincide.
    pub fn current_position(&self) -> f32 {
        // SAFETY: the world borrowed here owns the constraint. Jolt's `GetCurrentPosition` is
        // const and reads the constraint's frames and the bodies' transforms, which change only
        // behind `&mut PhysicsWorld`; joltc only declares the handle mutable.
        unsafe { JPH_SliderConstraint_GetCurrentPosition(self.ptr()) }
    }

    /// The motor's state.
    pub fn motor_state(&self) -> MotorState {
        // SAFETY: as in `current_position`; the getter reads a member.
        MotorState::from_jph(unsafe { JPH_SliderConstraint_GetMotorState(self.ptr()) })
    }

    /// The position a position motor drives to, metres.
    pub fn target_position(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_SliderConstraint_GetTargetPosition(self.ptr()) }
    }

    /// The velocity a velocity motor drives to, m/s.
    pub fn target_velocity(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_SliderConstraint_GetTargetVelocity(self.ptr()) }
    }

    /// The position limits in metres, `None` without limits.
    pub fn limits(&self) -> Option<(f32, f32)> {
        // SAFETY: as in `motor_state`.
        unsafe {
            JPH_SliderConstraint_HasLimits(self.ptr()).then(|| {
                (
                    JPH_SliderConstraint_GetLimitsMin(self.ptr()),
                    JPH_SliderConstraint_GetLimitsMax(self.ptr()),
                )
            })
        }
    }

    /// The friction force in N applied while the motor is off.
    pub fn max_friction_force(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_SliderConstraint_GetMaxFrictionForce(self.ptr()) }
    }

    /// The impulses in N·s that kept body 2 on the slider axis in the last collision step, along
    /// the two constraint axes perpendicular to it.
    pub fn total_lambda_position(&self) -> [f32; 2] {
        let mut value = [0.0; 2];
        // SAFETY: as in `motor_state`; `value` has room for the two values joltc writes.
        unsafe { JPH_SliderConstraint_GetTotalLambdaPosition(self.ptr(), value.as_mut_ptr()) };
        value
    }

    /// The impulse in N·s the motor, or the friction while the motor is off, applied in the last
    /// collision step.
    pub fn total_lambda_motor(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_SliderConstraint_GetTotalLambdaMotor(self.ptr()) }
    }

    /// The impulse in N·s the limits applied in the last collision step.
    pub fn total_lambda_position_limits(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_SliderConstraint_GetTotalLambdaPositionLimits(self.ptr()) }
    }

    /// The angular impulse in N·m·s that kept the rotation fixed in the last collision step.
    pub fn total_lambda_rotation(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `motor_state`; `value` is a live local.
        unsafe { JPH_SliderConstraint_GetTotalLambdaRotation(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }
}

impl ConstraintMut<'_, SliderConstraint> {
    /// Switches the motor and wakes the constraint's bodies.
    pub fn set_motor_state(&mut self, state: MotorState) {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. Jolt asserts valid motor settings for a running motor: every setting stored
        // was validated.
        unsafe { JPH_SliderConstraint_SetMotorState(self.ptr(), state.to_jph()) };
        self.wake_bodies();
    }

    /// Sets the position a position motor drives to, metres within
    /// [`limits::MAX_SHAPE_EXTENT`]; Jolt clamps it to the limits. Wakes the constraint's
    /// bodies.
    pub fn set_target_position(&mut self, position: f32) -> Result<(), ConstraintError> {
        let extent = limits::MAX_SHAPE_EXTENT;
        if !within(position, -extent, extent) {
            return Err(ConstraintError::InvalidValue(
                "target position must be finite and within limits::MAX_SHAPE_EXTENT",
            ));
        }
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_SliderConstraint_SetTargetPosition(self.ptr(), position) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the velocity a velocity motor drives to, m/s, at most
    /// [`limits::MAX_LINEAR_VELOCITY`] in magnitude. Wakes the constraint's bodies.
    pub fn set_target_velocity(&mut self, velocity: f32) -> Result<(), ConstraintError> {
        let speed = limits::MAX_LINEAR_VELOCITY;
        if !within(velocity, -speed, speed) {
            return Err(ConstraintError::InvalidValue(TARGET_VELOCITY_RULE));
        }
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_SliderConstraint_SetTargetVelocity(self.ptr(), velocity) };
        self.wake_bodies();
        Ok(())
    }

    /// Replaces the motor settings, checked as at creation and bounded through the bodies'
    /// effective mass. Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_motor_settings(&mut self, motor: MotorSettings) -> Result<(), ConstraintError> {
        check_motor(motor, self.effective_mass_bound())?;
        let mut motor = motor.to_jph();
        // SAFETY: as in `set_motor_state`; `motor` is a live local that joltc copies.
        unsafe { JPH_SliderConstraint_SetMotorSettings(self.ptr(), &mut motor) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the position limits, with the rules of [`SliderConstraintSettings::limits`]. Wakes
    /// the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_limits(&mut self, min: f32, max: f32) -> Result<(), ConstraintError> {
        validate_slider_limits(min, max, self.limits_spring())
            .map_err(ConstraintError::InvalidValue)?;
        self.write_limits(min, max);
        Ok(())
    }

    /// Removes the position limits (Jolt's `-FLT_MAX..FLT_MAX`). Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn remove_limits(&mut self) {
        let (min, max) = NO_SLIDER_LIMITS;
        self.write_limits(min, max);
    }

    /// Hands checked limits to Jolt and wakes the bodies.
    fn write_limits(&mut self, min: f32, max: f32) {
        // SAFETY: as in `set_motor_state`; Jolt asserts `min <= 0 <= max`, which the callers'
        // limits satisfy.
        unsafe { JPH_SliderConstraint_SetLimits(self.ptr(), min, max) };
        self.wake_bodies();
    }

    /// Replaces the spring that makes the limits soft, bounded through the bodies' effective
    /// mass. Limits with `min == max` keep needing a soft spring. Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_limits_spring(&mut self, spring: SpringSettings) -> Result<(), ConstraintError> {
        check_spring(spring, self.effective_mass_bound())?;
        // SAFETY: as in `set_motor_state`; the getters read members.
        let (has_limits, min, max) = unsafe {
            (
                JPH_SliderConstraint_HasLimits(self.ptr()),
                JPH_SliderConstraint_GetLimitsMin(self.ptr()),
                JPH_SliderConstraint_GetLimitsMax(self.ptr()),
            )
        };
        if has_limits {
            validate_slider_limits(min, max, spring).map_err(ConstraintError::InvalidValue)?;
        }
        let mut spring = spring.to_jph();
        // SAFETY: as in `set_motor_state`; `spring` is a live local that joltc copies.
        unsafe { JPH_SliderConstraint_SetLimitsSpringSettings(self.ptr(), &mut spring) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the friction force in N applied while the motor is off, finite and at least 0.
    /// Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_max_friction_force(&mut self, force: f32) -> Result<(), ConstraintError> {
        check_friction(force)?;
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_SliderConstraint_SetMaxFrictionForce(self.ptr(), force) };
        self.wake_bodies();
        Ok(())
    }

    /// The current limits spring.
    fn limits_spring(&self) -> SpringSettings {
        // SAFETY: an all-zero `JPH_SpringSettings` is valid; the getter fills it from a member.
        let mut spring: JPH_SpringSettings = unsafe { std::mem::zeroed() };
        // SAFETY: as in `set_motor_state`; `spring` is a live local.
        unsafe { JPH_SliderConstraint_GetLimitsSpringSettings(self.ptr(), &mut spring) };
        SpringSettings::from_jph(spring)
    }
}
