//! World constraints that limit or drive a rotation: hinge, cone, swing-twist and six degrees of
//! freedom.

use std::f32::consts::PI;
use std::ptr::NonNull;

use oxijolt_sys::*;

use super::lever::check_spring;
use super::world::{sealed, ConstraintSettings};
use super::{
    non_negative, point_anchors, solver::SolverOverrides, validate_hinge_limits, validate_point,
    within, ConstraintSpace, MotorSettings, SpringSettings, FRAME_AXES_RULE, FRICTION_RULE,
};
use crate::limits;
use crate::math::is_unit;
use crate::{
    ConeConstraint, ConstraintError, ConstraintMut, ConstraintRef, HingeConstraint,
    HingeConstraintSettings, MotorState, Quat, RVec3, SixDofConstraint, SixDofConstraintAxis,
    SixDofConstraintSettings, SwingTwistConstraint, SwingTwistConstraintSettings, Vec3,
};

/// `Err(InvalidValue)` unless `motor` is valid and its spring fits `bound`.
pub(crate) fn check_motor(motor: MotorSettings, bound: f64) -> Result<(), ConstraintError> {
    motor.validate().map_err(ConstraintError::InvalidValue)?;
    check_spring(motor.spring, bound)
}

/// `Err(InvalidValue)` unless `value` is a finite friction force or torque, at least 0.
pub(crate) fn check_friction(value: f32) -> Result<(), ConstraintError> {
    if non_negative(value) {
        Ok(())
    } else {
        Err(ConstraintError::InvalidValue(FRICTION_RULE))
    }
}

/// `Err(InvalidValue)` unless `value` is an angular velocity Jolt accepts, in rad/s.
pub(crate) fn check_angular_speed(value: f32) -> Result<(), ConstraintError> {
    if within(
        value,
        -limits::MAX_ANGULAR_VELOCITY,
        limits::MAX_ANGULAR_VELOCITY,
    ) {
        Ok(())
    } else {
        Err(ConstraintError::InvalidValue(
            "target angular velocity must be finite and at most limits::MAX_ANGULAR_VELOCITY",
        ))
    }
}

impl sealed::Settings for HingeConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        HingeConstraintSettings::validate(self)
    }

    fn springs(&self) -> Vec<SpringSettings> {
        HingeConstraintSettings::springs(self).collect()
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(point_anchors(self.space, self.point1, self.point2))
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = self.to_jph();
        // SAFETY: the caller locks both live bodies (trait contract); `settings` is a live,
        // validated local that joltc converts.
        unsafe { JPH_HingeConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for HingeConstraintSettings {
    type Kind = HingeConstraint;
}

impl ConstraintRef<'_, HingeConstraint> {
    /// The angle of body 2 about the hinge axis, radians in `[-π, π]`, measured from the pose
    /// where both normal axes coincide.
    pub fn current_angle(&self) -> f32 {
        // SAFETY: the world borrowed here owns the constraint. Jolt's `GetCurrentAngle` is const
        // and reads the constraint's axes and the bodies' rotations, which change only behind
        // `&mut PhysicsWorld`; joltc only declares the handle mutable.
        unsafe { JPH_HingeConstraint_GetCurrentAngle(self.ptr()) }
    }

    /// The motor's state.
    pub fn motor_state(&self) -> MotorState {
        // SAFETY: as in `current_angle`; the getter reads a member.
        MotorState::from_jph(unsafe { JPH_HingeConstraint_GetMotorState(self.ptr()) })
    }

    /// The angle a position motor drives to, radians.
    pub fn target_angle(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_HingeConstraint_GetTargetAngle(self.ptr()) }
    }

    /// The angular velocity a velocity motor drives to, rad/s.
    pub fn target_angular_velocity(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_HingeConstraint_GetTargetAngularVelocity(self.ptr()) }
    }

    /// The angle limits, radians: `(min, max)`; `(-π, π)` means no limits.
    pub fn limits(&self) -> (f32, f32) {
        // SAFETY: as in `motor_state`.
        unsafe {
            (
                JPH_HingeConstraint_GetLimitsMin(self.ptr()),
                JPH_HingeConstraint_GetLimitsMax(self.ptr()),
            )
        }
    }

    /// The friction torque in N·m applied while the motor is off.
    pub fn max_friction_torque(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_HingeConstraint_GetMaxFrictionTorque(self.ptr()) }
    }

    /// The impulse in N·s that kept the hinge points together in the last collision step.
    pub fn total_lambda_position(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `motor_state`; `value` is a live local.
        unsafe { JPH_HingeConstraint_GetTotalLambdaPosition(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The angular impulses in N·m·s that kept body 2 turning only about the hinge axis in the
    /// last collision step, about the two constraint axes perpendicular to it.
    pub fn total_lambda_rotation(&self) -> [f32; 2] {
        let mut value = [0.0; 2];
        // SAFETY: as in `motor_state`; `value` has room for the two values joltc writes.
        unsafe { JPH_HingeConstraint_GetTotalLambdaRotation(self.ptr(), value.as_mut_ptr()) };
        value
    }

    /// The angular impulse in N·m·s the motor, or the friction while the motor is off, applied
    /// in the last collision step.
    pub fn total_lambda_motor(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_HingeConstraint_GetTotalLambdaMotor(self.ptr()) }
    }

    /// The angular impulse in N·m·s the limits applied in the last collision step.
    pub fn total_lambda_rotation_limits(&self) -> f32 {
        // SAFETY: as in `motor_state`.
        unsafe { JPH_HingeConstraint_GetTotalLambdaRotationLimits(self.ptr()) }
    }
}

impl ConstraintMut<'_, HingeConstraint> {
    /// Switches the motor and wakes the constraint's bodies.
    pub fn set_motor_state(&mut self, state: MotorState) {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. Jolt asserts valid motor settings for a running motor: every setting stored
        // was validated.
        unsafe { JPH_HingeConstraint_SetMotorState(self.ptr(), state.to_jph()) };
        self.wake_bodies();
    }

    /// Sets the angle a position motor drives to, radians in `[-π, π]`; Jolt clamps it to the
    /// limits. Wakes the constraint's bodies.
    pub fn set_target_angle(&mut self, angle: f32) -> Result<(), ConstraintError> {
        if !within(angle, -PI, PI) {
            return Err(ConstraintError::InvalidValue(
                "target angle must be between -pi and pi",
            ));
        }
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_HingeConstraint_SetTargetAngle(self.ptr(), angle) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the angular velocity a velocity motor drives to, rad/s, at most
    /// [`limits::MAX_ANGULAR_VELOCITY`] in magnitude. Wakes the constraint's bodies.
    pub fn set_target_angular_velocity(&mut self, velocity: f32) -> Result<(), ConstraintError> {
        check_angular_speed(velocity)?;
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_HingeConstraint_SetTargetAngularVelocity(self.ptr(), velocity) };
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
        unsafe { JPH_HingeConstraint_SetMotorSettings(self.ptr(), &mut motor) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the angle limits: `min` in `[-π, 0]`, `max` in `[0, π]`; `min == max` needs a soft
    /// limits spring. `(-π, π)` turns the limits off. Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_limits(&mut self, min: f32, max: f32) -> Result<(), ConstraintError> {
        validate_hinge_limits(min, max, self.limits_spring())
            .map_err(ConstraintError::InvalidValue)?;
        // SAFETY: as in `set_motor_state`; Jolt asserts the ranges checked above.
        unsafe { JPH_HingeConstraint_SetLimits(self.ptr(), min, max) };
        self.wake_bodies();
        Ok(())
    }

    /// Replaces the spring that makes the limits soft, bounded through the bodies' effective
    /// mass. Limits with `min == max` keep needing a soft spring. Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_limits_spring(&mut self, spring: SpringSettings) -> Result<(), ConstraintError> {
        check_spring(spring, self.effective_mass_bound())?;
        // SAFETY: as in `set_motor_state`.
        let (min, max) = unsafe {
            (
                JPH_HingeConstraint_GetLimitsMin(self.ptr()),
                JPH_HingeConstraint_GetLimitsMax(self.ptr()),
            )
        };
        validate_hinge_limits(min, max, spring).map_err(ConstraintError::InvalidValue)?;
        let mut spring = spring.to_jph();
        // SAFETY: as in `set_motor_state`; `spring` is a live local that joltc copies.
        unsafe { JPH_HingeConstraint_SetLimitsSpringSettings(self.ptr(), &mut spring) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the friction torque in N·m applied while the motor is off, finite and at least 0.
    /// Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_max_friction_torque(&mut self, torque: f32) -> Result<(), ConstraintError> {
        check_friction(torque)?;
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_HingeConstraint_SetMaxFrictionTorque(self.ptr(), torque) };
        self.wake_bodies();
        Ok(())
    }

    /// The current limits spring.
    fn limits_spring(&self) -> SpringSettings {
        // SAFETY: an all-zero `JPH_SpringSettings` is valid; the getter fills it from a member.
        let mut spring: JPH_SpringSettings = unsafe { std::mem::zeroed() };
        // SAFETY: as in `set_motor_state`; `spring` is a live local.
        unsafe { JPH_HingeConstraint_GetLimitsSpringSettings(self.ptr(), &mut spring) };
        SpringSettings::from_jph(spring)
    }
}

/// `Err(InvalidValue)` unless `rotation` is a finite unit quaternion.
fn check_rotation(rotation: Quat) -> Result<(), ConstraintError> {
    if rotation.is_valid_rotation() {
        Ok(())
    } else {
        Err(ConstraintError::InvalidValue(
            "target orientation must be a finite unit quaternion",
        ))
    }
}

/// `Err(InvalidValue)` unless `velocity` is an angular velocity within
/// [`limits::MAX_ANGULAR_VELOCITY`].
fn check_angular_velocity(velocity: Vec3) -> Result<(), ConstraintError> {
    if limits::is_angular_velocity(velocity) {
        Ok(())
    } else {
        Err(ConstraintError::InvalidValue(
            "target angular velocity must be finite and at most limits::MAX_ANGULAR_VELOCITY long",
        ))
    }
}

/// A cone constraint (Jolt `ConeConstraintSettings`): a ball joint whose body 2 twist axis
/// stays within a cone of half angle `half_cone_angle` around body 1's; the twist about it is
/// free.
///
/// The default is Jolt's: both points at the origin with twist axis +X, world space, half cone
/// angle 0.
#[derive(Clone, Debug, PartialEq)]
pub struct ConeConstraintSettings {
    space: ConstraintSpace,
    point1: RVec3,
    twist_axis1: Vec3,
    point2: RVec3,
    twist_axis2: Vec3,
    half_cone_angle: f32,
    pub(super) solver: SolverOverrides,
}

impl Default for ConeConstraintSettings {
    fn default() -> Self {
        Self::new(RVec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 0.0)
    }
}

impl ConeConstraintSettings {
    /// A cone at `point` around the unit `twist_axis` on both bodies, in world space, with half
    /// angle `half_cone_angle`, radians in `[0, π]`.
    pub fn new(point: RVec3, twist_axis: Vec3, half_cone_angle: f32) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            point1: point,
            twist_axis1: twist_axis,
            point2: point,
            twist_axis2: twist_axis,
            half_cone_angle,
            solver: SolverOverrides::default(),
        }
    }

    /// The space the points and axes are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The point and unit twist axis on body 1.
    #[must_use]
    pub fn frame1(mut self, point: RVec3, twist_axis: Vec3) -> Self {
        self.point1 = point;
        self.twist_axis1 = twist_axis;
        self
    }

    /// The point and unit twist axis on body 2.
    #[must_use]
    pub fn frame2(mut self, point: RVec3, twist_axis: Vec3) -> Self {
        self.point2 = point;
        self.twist_axis2 = twist_axis;
        self
    }
}

/// What a cone's half angle must satisfy, as Jolt's `ConeConstraint::SetHalfConeAngle` asserts.
fn validate_half_cone_angle(angle: f32) -> Result<(), &'static str> {
    if within(angle, 0.0, PI) {
        Ok(())
    } else {
        Err("half cone angle must be between 0 and pi")
    }
}

impl sealed::Settings for ConeConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        self.solver.validate()?;
        validate_point(self.point1)?;
        validate_point(self.point2)?;
        if !(is_unit(self.twist_axis1) && is_unit(self.twist_axis2)) {
            return Err(FRAME_AXES_RULE);
        }
        validate_half_cone_angle(self.half_cone_angle)
    }

    fn springs(&self) -> Vec<SpringSettings> {
        Vec::new()
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(point_anchors(self.space, self.point1, self.point2))
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = JPH_ConeConstraintSettings {
            base: self.solver.to_jph(),
            space: self.space.to_jph(),
            point1: self.point1.to_jph(),
            twistAxis1: self.twist_axis1.to_jph(),
            point2: self.point2.to_jph(),
            twistAxis2: self.twist_axis2.to_jph(),
            halfConeAngle: self.half_cone_angle,
        };
        // SAFETY: the caller locks both live bodies (trait contract); `settings` is a live,
        // validated local that joltc converts.
        unsafe { JPH_ConeConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for ConeConstraintSettings {
    type Kind = ConeConstraint;
}

impl ConstraintRef<'_, ConeConstraint> {
    /// The half angle of the cone, radians, as Jolt stores it (its cosine) turned back into an
    /// angle.
    pub fn half_cone_angle(&self) -> f32 {
        // SAFETY: the world borrowed here owns the constraint; the getter reads a member.
        let cos = unsafe { JPH_ConeConstraint_GetCosHalfConeAngle(self.ptr()) };
        cos.clamp(-1.0, 1.0).acos()
    }

    /// The impulse in N·s that kept the points together in the last collision step.
    pub fn total_lambda_position(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `half_cone_angle`; `value` is a live local.
        unsafe { JPH_ConeConstraint_GetTotalLambdaPosition(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The angular impulse in N·m·s the cone applied in the last collision step.
    pub fn total_lambda_rotation(&self) -> f32 {
        // SAFETY: as in `half_cone_angle`.
        unsafe { JPH_ConeConstraint_GetTotalLambdaRotation(self.ptr()) }
    }
}

impl ConstraintMut<'_, ConeConstraint> {
    /// Sets the half angle of the cone, radians in `[0, π]`. Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_half_cone_angle(&mut self, angle: f32) -> Result<(), ConstraintError> {
        validate_half_cone_angle(angle).map_err(ConstraintError::InvalidValue)?;
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. Jolt asserts the range checked above.
        unsafe { JPH_ConeConstraint_SetHalfConeAngle(self.ptr(), angle) };
        self.wake_bodies();
        Ok(())
    }
}

impl sealed::Settings for SwingTwistConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        SwingTwistConstraintSettings::validate(self)
    }

    fn springs(&self) -> Vec<SpringSettings> {
        SwingTwistConstraintSettings::springs(self).collect()
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(point_anchors(self.space, self.position1, self.position2))
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = self.to_jph();
        // SAFETY: as in `ConeConstraintSettings::create`.
        unsafe { JPH_SwingTwistConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for SwingTwistConstraintSettings {
    type Kind = SwingTwistConstraint;
}

impl ConstraintRef<'_, SwingTwistConstraint> {
    /// The rotation of body 2's constraint frame relative to body 1's.
    pub fn rotation_in_constraint_space(&self) -> Quat {
        let mut value = Quat::IDENTITY.to_jph();
        // SAFETY: the world borrowed here owns the constraint. Jolt's
        // `GetRotationInConstraintSpace` is const and reads the frames and the bodies'
        // rotations, which change only behind `&mut PhysicsWorld`; `value` is a live local.
        unsafe { JPH_SwingTwistConstraint_GetRotationInConstraintSpace(self.ptr(), &mut value) };
        Quat::from_jph(value)
    }

    /// The swing motor's state.
    pub fn swing_motor_state(&self) -> MotorState {
        // SAFETY: as in `rotation_in_constraint_space`; the getter reads a member.
        MotorState::from_jph(unsafe { JPH_SwingTwistConstraint_GetSwingMotorState(self.ptr()) })
    }

    /// The twist motor's state.
    pub fn twist_motor_state(&self) -> MotorState {
        // SAFETY: as in `swing_motor_state`.
        MotorState::from_jph(unsafe { JPH_SwingTwistConstraint_GetTwistMotorState(self.ptr()) })
    }

    /// The swing motor's settings.
    pub fn swing_motor_settings(&self) -> MotorSettings {
        // SAFETY: an all-zero `JPH_MotorSettings` is valid (floats and an enum with a zero
        // value).
        let mut motor: JPH_MotorSettings = unsafe { std::mem::zeroed() };
        // SAFETY: as in `swing_motor_state`; joltc copies the member into `motor`, a live local.
        unsafe { JPH_SwingTwistConstraint_GetSwingMotorSettings(self.ptr(), &mut motor) };
        MotorSettings::from_jph(motor)
    }

    /// The twist motor's settings.
    pub fn twist_motor_settings(&self) -> MotorSettings {
        // SAFETY: as in `swing_motor_settings`.
        let mut motor: JPH_MotorSettings = unsafe { std::mem::zeroed() };
        // SAFETY: as in `swing_motor_settings`.
        unsafe { JPH_SwingTwistConstraint_GetTwistMotorSettings(self.ptr(), &mut motor) };
        MotorSettings::from_jph(motor)
    }

    /// The orientation position motors drive to, in constraint space, as Jolt stores it after
    /// clamping it to the limits.
    pub fn target_orientation_cs(&self) -> Quat {
        let mut value = Quat::IDENTITY.to_jph();
        // SAFETY: as in `swing_motor_state`; `value` is a live local.
        unsafe { JPH_SwingTwistConstraint_GetTargetOrientationCS(self.ptr(), &mut value) };
        Quat::from_jph(value)
    }

    /// The angular velocity velocity motors drive to, rad/s in constraint space.
    pub fn target_angular_velocity_cs(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `target_orientation_cs`.
        unsafe { JPH_SwingTwistConstraint_GetTargetAngularVelocityCS(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The swing half angles about the normal and the plane axis, radians.
    pub fn half_cone_angles(&self) -> (f32, f32) {
        // SAFETY: as in `swing_motor_state`; Jolt's getters are const member reads, joltc
        // declares the handle of the first one mutable.
        unsafe {
            (
                JPH_SwingTwistConstraint_GetNormalHalfConeAngle(self.ptr()),
                JPH_SwingTwistConstraint_GetPlaneHalfConeAngle(self.ptr()),
            )
        }
    }

    /// The twist range, radians: `(min, max)`.
    pub fn twist_limits(&self) -> (f32, f32) {
        // SAFETY: as in `swing_motor_state`.
        unsafe {
            (
                JPH_SwingTwistConstraint_GetTwistMinAngle(self.ptr()),
                JPH_SwingTwistConstraint_GetTwistMaxAngle(self.ptr()),
            )
        }
    }

    /// The friction torque in N·m applied while no motor drives the joint.
    pub fn max_friction_torque(&self) -> f32 {
        // SAFETY: as in `swing_motor_state`.
        unsafe { JPH_SwingTwistConstraint_GetMaxFrictionTorque(self.ptr()) }
    }

    /// The impulse in N·s that kept the points together in the last collision step.
    pub fn total_lambda_position(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `target_orientation_cs`.
        unsafe { JPH_SwingTwistConstraint_GetTotalLambdaPosition(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The angular impulses in N·m·s the twist, swing Y and swing Z limit parts applied in the
    /// last collision step, each about an axis Jolt picks for that step. A limit with a range
    /// only pushes back and turns its axis round at its minimum, so a twist limit reads zero or
    /// less at either end; a locked axis reads either sign. With both swing axes limited Jolt solves one swing
    /// limit, about the axis from the nearest allowed swing to the current one, in the swing Y
    /// part, and swing Z reads 0. Compare the length of the three with a torque limit, not one
    /// component (see [docs/constraints.md#breakable-constraints]).
    ///
    /// [docs/constraints.md#breakable-constraints]: https://github.com/pockerhead/oxijolt/blob/main/docs/constraints.md#breakable-constraints
    pub fn total_lambda_limits(&self) -> [f32; 3] {
        // SAFETY: as in `swing_motor_state`.
        unsafe {
            [
                JPH_SwingTwistConstraint_GetTotalLambdaTwist(self.ptr()),
                JPH_SwingTwistConstraint_GetTotalLambdaSwingY(self.ptr()),
                JPH_SwingTwistConstraint_GetTotalLambdaSwingZ(self.ptr()),
            ]
        }
    }

    /// The angular impulses in N·m·s the motors, or the friction while they are off, applied in
    /// the last collision step, per constraint axis (twist, swing Y, swing Z).
    pub fn total_lambda_motor(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `target_orientation_cs`.
        unsafe { JPH_SwingTwistConstraint_GetTotalLambdaMotor(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }
}

impl ConstraintMut<'_, SwingTwistConstraint> {
    /// Switches the swing motor and wakes the constraint's bodies.
    pub fn set_swing_motor_state(&mut self, state: MotorState) {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. Jolt asserts valid motor settings for a running motor: every setting stored
        // was validated.
        unsafe { JPH_SwingTwistConstraint_SetSwingMotorState(self.ptr(), state.to_jph()) };
        self.wake_bodies();
    }

    /// Switches the twist motor and wakes the constraint's bodies.
    pub fn set_twist_motor_state(&mut self, state: MotorState) {
        // SAFETY: as in `set_swing_motor_state`.
        unsafe { JPH_SwingTwistConstraint_SetTwistMotorState(self.ptr(), state.to_jph()) };
        self.wake_bodies();
    }

    /// Sets the orientation position motors drive to: body 2's constraint frame relative to body
    /// 1's, a finite unit quaternion. Jolt clamps it to the limits. Wakes the constraint's
    /// bodies.
    pub fn set_target_orientation_cs(&mut self, rotation: Quat) -> Result<(), ConstraintError> {
        check_rotation(rotation)?;
        let rotation = rotation.to_jph();
        // SAFETY: as in `set_swing_motor_state`; `rotation` is a live local.
        unsafe { JPH_SwingTwistConstraint_SetTargetOrientationCS(self.ptr(), &rotation) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the angular velocity velocity motors drive to, rad/s in constraint space, at most
    /// [`limits::MAX_ANGULAR_VELOCITY`] long. Wakes the constraint's bodies.
    pub fn set_target_angular_velocity_cs(
        &mut self,
        velocity: Vec3,
    ) -> Result<(), ConstraintError> {
        check_angular_velocity(velocity)?;
        let velocity = velocity.to_jph();
        // SAFETY: as in `set_swing_motor_state`; `velocity` is a live local.
        unsafe { JPH_SwingTwistConstraint_SetTargetAngularVelocityCS(self.ptr(), &velocity) };
        self.wake_bodies();
        Ok(())
    }

    /// Replaces the swing motor settings, checked as at creation and bounded through the
    /// bodies' effective mass. Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_swing_motor_settings(
        &mut self,
        motor: MotorSettings,
    ) -> Result<(), ConstraintError> {
        check_motor(motor, self.effective_mass_bound())?;
        let motor = motor.to_jph();
        // SAFETY: as in `set_swing_motor_state`; `motor` is a live local that joltc copies.
        unsafe { JPH_SwingTwistConstraint_SetSwingMotorSettings(self.ptr(), &motor) };
        self.wake_bodies();
        Ok(())
    }

    /// Replaces the twist motor settings, checked as at creation and bounded through the
    /// bodies' effective mass. Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_twist_motor_settings(
        &mut self,
        motor: MotorSettings,
    ) -> Result<(), ConstraintError> {
        check_motor(motor, self.effective_mass_bound())?;
        let motor = motor.to_jph();
        // SAFETY: as in `set_swing_motor_settings`.
        unsafe { JPH_SwingTwistConstraint_SetTwistMotorSettings(self.ptr(), &motor) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the friction torque in N·m applied while no motor drives the joint, finite and at
    /// least 0. Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_max_friction_torque(&mut self, torque: f32) -> Result<(), ConstraintError> {
        check_friction(torque)?;
        // SAFETY: as in `set_swing_motor_state`.
        unsafe { JPH_SwingTwistConstraint_SetMaxFrictionTorque(self.ptr(), torque) };
        self.wake_bodies();
        Ok(())
    }
}

impl sealed::Settings for SixDofConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        SixDofConstraintSettings::validate(self)
    }

    fn springs(&self) -> Vec<SpringSettings> {
        SixDofConstraintSettings::springs(self).collect()
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(point_anchors(self.space, self.position1, self.position2))
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = self.to_jph();
        // SAFETY: as in `ConeConstraintSettings::create`.
        unsafe { JPH_SixDOFConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for SixDofConstraintSettings {
    type Kind = SixDofConstraint;
}

impl ConstraintRef<'_, SixDofConstraint> {
    /// The state of the motor of `axis`.
    pub fn motor_state(&self, axis: SixDofConstraintAxis) -> MotorState {
        // SAFETY: the world borrowed here owns the constraint; Jolt's getter is a const member
        // read, joltc only declares the handle mutable.
        MotorState::from_jph(unsafe {
            JPH_SixDOFConstraint_GetMotorState(self.ptr(), axis.to_jph())
        })
    }

    /// The rotation of body 2's constraint frame relative to body 1's.
    pub fn rotation_in_constraint_space(&self) -> Quat {
        let mut value = Quat::IDENTITY.to_jph();
        // SAFETY: as in `motor_state`; Jolt's getter is const and reads the frames and the
        // bodies' rotations. `value` is a live local.
        unsafe { JPH_SixDOFConstraint_GetRotationInConstraintSpace(self.ptr(), &mut value) };
        Quat::from_jph(value)
    }

    /// The limits of `axis`: `None` for a free axis, `(min, max)` otherwise; a fixed axis reads
    /// as Jolt stores it.
    pub fn limits(&self, axis: SixDofConstraintAxis) -> Option<(f32, f32)> {
        // SAFETY: as in `motor_state`.
        unsafe {
            (!JPH_SixDOFConstraint_IsFreeAxis(self.ptr(), axis.to_jph())).then(|| {
                (
                    JPH_SixDOFConstraint_GetLimitsMin(self.ptr(), axis.to_jph()),
                    JPH_SixDOFConstraint_GetLimitsMax(self.ptr(), axis.to_jph()),
                )
            })
        }
    }

    /// The velocity translation motors drive to, m/s in constraint space.
    pub fn target_velocity_cs(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `rotation_in_constraint_space`.
        unsafe { JPH_SixDOFConstraint_GetTargetVelocityCS(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The angular velocity rotation motors drive to, rad/s in constraint space.
    pub fn target_angular_velocity_cs(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `rotation_in_constraint_space`.
        unsafe { JPH_SixDOFConstraint_GetTargetAngularVelocityCS(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The position translation motors drive to, metres in constraint space.
    pub fn target_position_cs(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `rotation_in_constraint_space`.
        unsafe { JPH_SixDOFConstraint_GetTargetPositionCS(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The orientation rotation motors drive to, in constraint space, after Jolt's clamping.
    pub fn target_orientation_cs(&self) -> Quat {
        let mut value = Quat::IDENTITY.to_jph();
        // SAFETY: as in `rotation_in_constraint_space`.
        unsafe { JPH_SixDOFConstraint_GetTargetOrientationCS(self.ptr(), &mut value) };
        Quat::from_jph(value)
    }

    /// The impulse in N·s that held the translation in the last collision step: a world-space
    /// vector while all three translation axes are fixed and none has a soft limit spring
    /// ([`limits_spring`](crate::SixDofConstraintSettings::limits_spring)),
    /// otherwise the three translation axis parts (limits) in constraint-axis order.
    pub fn total_lambda_position(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `rotation_in_constraint_space`.
        unsafe { JPH_SixDOFConstraint_GetTotalLambdaPosition(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The angular impulse in N·m·s that held the rotation in the last collision step: a
    /// world-space vector while all three rotation axes are fixed, otherwise the twist, swing Y
    /// and swing Z limit parts, about axes Jolt picks each step as for a swing-twist constraint's
    /// [`total_lambda_limits`](ConstraintRef::total_lambda_limits).
    pub fn total_lambda_rotation(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `rotation_in_constraint_space`.
        unsafe { JPH_SixDOFConstraint_GetTotalLambdaRotation(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The impulses in N·s the translation motors, or the friction while they are off, applied
    /// in the last collision step, per constraint axis.
    pub fn total_lambda_motor_translation(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `rotation_in_constraint_space`.
        unsafe { JPH_SixDOFConstraint_GetTotalLambdaMotorTranslation(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The angular impulses in N·m·s the rotation motors, or the friction while they are off,
    /// applied in the last collision step, per constraint axis.
    pub fn total_lambda_motor_rotation(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `rotation_in_constraint_space`.
        unsafe { JPH_SixDOFConstraint_GetTotalLambdaMotorRotation(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }
}

impl ConstraintMut<'_, SixDofConstraint> {
    /// Switches the motor of `axis` and wakes the constraint's bodies.
    pub fn set_motor_state(&mut self, axis: SixDofConstraintAxis, state: MotorState) {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. Jolt asserts valid motor settings for a running motor: every setting stored
        // was validated.
        unsafe { JPH_SixDOFConstraint_SetMotorState(self.ptr(), axis.to_jph(), state.to_jph()) };
        self.wake_bodies();
    }

    /// Replaces the motor settings of `axis`, checked as at creation and bounded through the
    /// bodies' effective mass. Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_motor_settings(
        &mut self,
        axis: SixDofConstraintAxis,
        motor: MotorSettings,
    ) -> Result<(), ConstraintError> {
        check_motor(motor, self.effective_mass_bound())?;
        let motor = motor.to_jph();
        // SAFETY: as in `set_motor_state`; `motor` is a live local that joltc copies.
        unsafe { JPH_SixDOFConstraint_SetMotorSettings(self.ptr(), axis.to_jph(), &motor) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the velocity translation motors drive to, m/s in constraint space, at most
    /// [`limits::MAX_LINEAR_VELOCITY`] long. Wakes the constraint's bodies.
    pub fn set_target_velocity_cs(&mut self, velocity: Vec3) -> Result<(), ConstraintError> {
        if !limits::is_linear_velocity(velocity) {
            return Err(ConstraintError::InvalidValue(
                "target velocity must be finite and at most limits::MAX_LINEAR_VELOCITY long",
            ));
        }
        let mut velocity = velocity.to_jph();
        // SAFETY: as in `set_motor_state`; `velocity` is a live local that joltc copies.
        unsafe { JPH_SixDOFConstraint_SetTargetVelocityCS(self.ptr(), &mut velocity) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the angular velocity rotation motors drive to, rad/s in constraint space, at most
    /// [`limits::MAX_ANGULAR_VELOCITY`] long. Wakes the constraint's bodies.
    pub fn set_target_angular_velocity_cs(
        &mut self,
        velocity: Vec3,
    ) -> Result<(), ConstraintError> {
        check_angular_velocity(velocity)?;
        let mut velocity = velocity.to_jph();
        // SAFETY: as in `set_target_velocity_cs`.
        unsafe { JPH_SixDOFConstraint_SetTargetAngularVelocityCS(self.ptr(), &mut velocity) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the position translation motors drive to, metres in constraint space, within
    /// [`limits::MAX_SHAPE_EXTENT`] per axis. Wakes the constraint's bodies.
    pub fn set_target_position_cs(&mut self, position: Vec3) -> Result<(), ConstraintError> {
        if !limits::is_local_offset(position) {
            return Err(ConstraintError::InvalidValue(
                "target position must be finite and within limits::MAX_SHAPE_EXTENT per axis",
            ));
        }
        let mut position = position.to_jph();
        // SAFETY: as in `set_target_velocity_cs`.
        unsafe { JPH_SixDOFConstraint_SetTargetPositionCS(self.ptr(), &mut position) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the orientation rotation motors drive to: body 2's constraint frame relative to
    /// body 1's, a finite unit quaternion. Jolt clamps it to the limits. Wakes the constraint's
    /// bodies.
    pub fn set_target_orientation_cs(&mut self, rotation: Quat) -> Result<(), ConstraintError> {
        check_rotation(rotation)?;
        let mut rotation = rotation.to_jph();
        // SAFETY: as in `set_target_velocity_cs`.
        unsafe { JPH_SixDOFConstraint_SetTargetOrientationCS(self.ptr(), &mut rotation) };
        self.wake_bodies();
        Ok(())
    }
}
