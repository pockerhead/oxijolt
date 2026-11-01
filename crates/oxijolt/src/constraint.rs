//! Constraint settings, used for ragdoll joints and world constraints, with Jolt's defaults, and
//! the constraints a [`PhysicsWorld`](crate::PhysicsWorld) owns.
//!
//! The types here are plain Rust values, checked against what Jolt asserts or silently rewrites
//! before any of them reaches Jolt. Angles are in radians, torques in N·m, forces in N.
//!
//! # Frames
//! Each constraint attaches a frame to each of its two bodies: a point and two perpendicular unit
//! axes. With [`ConstraintSpace::WorldSpace`] (the default) the frames are given in world space at
//! the time the constraint is created (for a ragdoll joint, at the bodies' creation pose); with
//! [`ConstraintSpace::LocalToBodyCom`] relative to each body's centre of mass. In the constraint frame, X is the twist (or hinge) axis and Y and Z are the
//! swing axes.

mod coupling;
mod id;
mod lever;
mod linear;
mod path;
mod pulley;
mod rotational;
mod slider;
mod solver;
mod world;

use std::f32::consts::PI;

use oxijolt_sys::*;

pub use coupling::{GearConstraintSettings, RackAndPinionConstraintSettings};
pub use id::{AnyConstraintId, ConstraintId};
pub use linear::{
    DistanceConstraintSettings, DistanceRange, FixedConstraintSettings, PointConstraintSettings,
};
pub use path::{HermitePath, HermitePathPoint, PathConstraintSettings, PathRotationConstraint};
pub use pulley::{PulleyConstraintSettings, PulleyLength};
pub use rotational::ConeConstraintSettings;
pub use slider::SliderConstraintSettings;
pub(crate) use world::ConstraintEntry;
pub use world::{
    ConeConstraint, ConstraintKind, ConstraintMut, ConstraintRef, ConstraintSettings,
    ConstraintType, DistanceConstraint, FixedConstraint, GearConstraint, HingeConstraint,
    MotorState, PathConstraint, PointConstraint, PulleyConstraint, RackAndPinionConstraint,
    SixDofConstraint, SliderConstraint, SwingTwistConstraint,
};

use crate::limits;
use crate::math::is_unit;
use crate::{RVec3, Vec3};
use solver::SolverOverrides;

/// Tolerance of the perpendicularity checks on unit axes, `|a·b|` at most this.
const PERPENDICULAR_TOLERANCE: f32 = 1.0e-4;

/// Jolt's `ConstraintSettings` defaults: enabled, priority 0, the world's iteration counts.
pub(crate) fn constraint_base() -> JPH_ConstraintSettings {
    JPH_ConstraintSettings {
        enabled: true,
        constraintPriority: 0,
        numVelocityStepsOverride: 0,
        numPositionStepsOverride: 0,
        drawConstraintSize: 1.0,
        userData: 0,
    }
}

/// The space a constraint's frames are given in (Jolt `EConstraintSpace`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum ConstraintSpace {
    /// Relative to each body's centre of mass.
    LocalToBodyCom,
    /// In world space, at the bodies' poses when the constraint is created. The default.
    #[default]
    WorldSpace,
}

impl ConstraintSpace {
    fn to_jph(self) -> JPH_ConstraintSpace {
        match self {
            Self::LocalToBodyCom => JPH_ConstraintSpace_LocalToBodyCOM,
            Self::WorldSpace => JPH_ConstraintSpace_WorldSpace,
        }
    }
}

/// How the two swing limits combine (Jolt `ESwingType`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum SwingType {
    /// An elliptic cone around the twist axis; its limits are symmetric. The default.
    #[default]
    Cone,
    /// A pyramid: each swing axis has its own, possibly asymmetric, range.
    Pyramid,
}

impl SwingType {
    fn to_jph(self) -> JPH_SwingType {
        match self {
            Self::Cone => JPH_SwingType_Cone,
            Self::Pyramid => JPH_SwingType_Pyramid,
        }
    }
}

/// A spring (Jolt `SpringSettings`). A limit spring with frequency or stiffness 0 is rigid; a
/// position motor whose spring has frequency or stiffness 0 does nothing (Jolt deactivates it).
/// The default is rigid: frequency 0, damping 0.
///
/// Jolt turns every spring into a stiffness `k` and damping `c`, which must stay at most
/// [`limits::MAX_SPRING_COEFFICIENT`]. In stiffness mode they are the values given. In frequency
/// mode Jolt computes `k = m·ω²` and `c = 2·m·ζ·ω` with `ω = 2π·frequency`, where `m` is the
/// effective mass (or inertia) of the joint's bodies.
/// [`RagdollSettings::new`](crate::RagdollSettings::new) checks every joint spring against an
/// upper bound of `m` computed from the masses and inertias of all its parts, and
/// [`PhysicsWorld::create_constraint`](crate::PhysicsWorld::create_constraint) and the constraint
/// setters against one computed from the constraint's two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SpringSettings {
    /// Oscillation frequency in Hz and damping ratio (1 is critical damping), both at least 0.
    FrequencyAndDamping {
        /// Hz, finite and at least 0; with the damping ratio bounded through the parts' masses
        /// (see [`SpringSettings`]).
        frequency: f32,
        /// Damping ratio, finite and at least 0; bounded with the frequency.
        damping: f32,
    },
    /// Stiffness in N/m (N·m/rad for rotations) and damping in N·s/m (N·m·s/rad), both at
    /// least 0.
    StiffnessAndDamping {
        /// N/m or N·m/rad, between 0 and [`limits::MAX_SPRING_COEFFICIENT`].
        stiffness: f32,
        /// N·s/m or N·m·s/rad, between 0 and [`limits::MAX_SPRING_COEFFICIENT`].
        damping: f32,
    },
}

impl Default for SpringSettings {
    fn default() -> Self {
        Self::FrequencyAndDamping {
            frequency: 0.0,
            damping: 0.0,
        }
    }
}

impl SpringSettings {
    /// Frequency or stiffness, and damping.
    fn values(&self) -> (f32, f32) {
        match *self {
            Self::FrequencyAndDamping { frequency, damping } => (frequency, damping),
            Self::StiffnessAndDamping { stiffness, damping } => (stiffness, damping),
        }
    }

    /// Whether the spring is soft, which is what Jolt tests (`mFrequency > 0`, which also holds
    /// the stiffness).
    fn is_soft(&self) -> bool {
        self.values().0 > 0.0
    }

    fn validate(&self) -> Result<(), &'static str> {
        let (strength, damping) = self.values();
        if !(non_negative(strength) && non_negative(damping)) {
            return Err("spring frequency, stiffness and damping must be finite and not negative");
        }
        if let Self::StiffnessAndDamping { .. } = self {
            if strength > limits::MAX_SPRING_COEFFICIENT || damping > limits::MAX_SPRING_COEFFICIENT
            {
                return Err(
                    "spring stiffness and damping must be at most limits::MAX_SPRING_COEFFICIENT",
                );
            }
        }
        Ok(())
    }

    /// Whether the stiffness and damping Jolt derives from this valid spring stay at most
    /// [`limits::MAX_SPRING_COEFFICIENT`] for an effective mass of at most
    /// `effective_mass_bound` (kg, or kg·m² for rotations). A stiffness-mode spring is checked by
    /// [`validate`](Self::validate) alone.
    pub(crate) fn fits_effective_mass(&self, effective_mass_bound: f64) -> bool {
        let Self::FrequencyAndDamping { frequency, damping } = *self else {
            return true;
        };
        // A rigid spring has no coefficients; an infinite bound times 0 would be NaN.
        if frequency == 0.0 {
            return true;
        }
        let omega = 2.0 * std::f64::consts::PI * f64::from(frequency);
        let bound = f64::from(limits::MAX_SPRING_COEFFICIENT);
        effective_mass_bound * omega * omega <= bound
            && 2.0 * effective_mass_bound * f64::from(damping) * omega <= bound
    }

    fn from_jph(spring: JPH_SpringSettings) -> Self {
        let (strength, damping) = (spring.frequencyOrStiffness, spring.damping);
        if spring.mode == JPH_SpringMode_StiffnessAndDamping {
            Self::StiffnessAndDamping {
                stiffness: strength,
                damping,
            }
        } else {
            Self::FrequencyAndDamping {
                frequency: strength,
                damping,
            }
        }
    }

    pub(crate) fn to_jph(self) -> JPH_SpringSettings {
        let (strength, damping) = self.values();
        let mode = match self {
            Self::FrequencyAndDamping { .. } => JPH_SpringMode_FrequencyAndDamping,
            Self::StiffnessAndDamping { .. } => JPH_SpringMode_StiffnessAndDamping,
        };
        JPH_SpringSettings {
            mode,
            frequencyOrStiffness: strength,
            damping,
        }
    }
}

/// A constraint motor (Jolt `MotorSettings`): the spring that drives it to its target and the
/// force and torque it may use. The defaults are Jolt's: frequency 2 Hz, damping 1, and
/// unlimited (`∓f32::MAX`) force and torque.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorSettings {
    spring: SpringSettings,
    min_force_limit: f32,
    max_force_limit: f32,
    min_torque_limit: f32,
    max_torque_limit: f32,
}

impl Default for MotorSettings {
    fn default() -> Self {
        Self {
            spring: SpringSettings::FrequencyAndDamping {
                frequency: 2.0,
                damping: 1.0,
            },
            min_force_limit: -f32::MAX,
            max_force_limit: f32::MAX,
            min_torque_limit: -f32::MAX,
            max_torque_limit: f32::MAX,
        }
    }
}

impl MotorSettings {
    /// The spring that drives a position motor to its target. Default 2 Hz, damping 1. Its size
    /// is bounded as [`SpringSettings`] describes.
    #[must_use]
    pub fn spring(mut self, value: SpringSettings) -> Self {
        self.spring = value;
        self
    }

    /// Force limits in N of a translation motor, finite, `min <= max`.
    #[must_use]
    pub fn force_limits(mut self, min: f32, max: f32) -> Self {
        self.min_force_limit = min;
        self.max_force_limit = max;
        self
    }

    /// Torque limits in N·m of a rotation motor, finite, `min <= max`.
    #[must_use]
    pub fn torque_limits(mut self, min: f32, max: f32) -> Self {
        self.min_torque_limit = min;
        self.max_torque_limit = max;
        self
    }

    /// What Jolt's `MotorSettings::IsValid` asserts for a running motor, and finite limits.
    fn validate(&self) -> Result<(), &'static str> {
        self.spring.validate()?;
        let ordered = |min: f32, max: f32| min.is_finite() && max.is_finite() && min <= max;
        if ordered(self.min_force_limit, self.max_force_limit)
            && ordered(self.min_torque_limit, self.max_torque_limit)
        {
            Ok(())
        } else {
            Err("motor force and torque limits must be finite with min <= max")
        }
    }

    fn from_jph(motor: JPH_MotorSettings) -> Self {
        Self {
            spring: SpringSettings::from_jph(motor.springSettings),
            min_force_limit: motor.minForceLimit,
            max_force_limit: motor.maxForceLimit,
            min_torque_limit: motor.minTorqueLimit,
            max_torque_limit: motor.maxTorqueLimit,
        }
    }

    fn to_jph(self) -> JPH_MotorSettings {
        JPH_MotorSettings {
            springSettings: self.spring.to_jph(),
            minForceLimit: self.min_force_limit,
            maxForceLimit: self.max_force_limit,
            minTorqueLimit: self.min_torque_limit,
            maxTorqueLimit: self.max_torque_limit,
        }
    }
}

/// What a constraint's friction force must satisfy ([`non_negative`]).
const FRICTION_RULE: &str = "friction must be finite and not negative";

/// What a constraint's friction torque must satisfy ([`non_negative`]).
const FRICTION_TORQUE_RULE: &str = "friction torque must be finite and not negative";

/// Whether `value` is finite and not negative.
fn non_negative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

/// What a linear motor's target velocity must satisfy ([`within`] `±MAX_LINEAR_VELOCITY`).
const TARGET_VELOCITY_RULE: &str =
    "target velocity must be finite and at most limits::MAX_LINEAR_VELOCITY";

/// Whether `value` is finite and in `[min, max]`.
fn within(value: f32, min: f32, max: f32) -> bool {
    value.is_finite() && (min..=max).contains(&value)
}

/// Checks one constraint point: finite and within the frame.
fn validate_point(point: RVec3) -> Result<(), &'static str> {
    if limits::is_in_frame(point) {
        Ok(())
    } else {
        Err("constraint frame points must be finite and within limits::MAX_POSITION")
    }
}

/// Where constraints given in `space` hold body 1 by `point1` and body 2 by `point2`.
fn point_anchors(
    space: ConstraintSpace,
    point1: RVec3,
    point2: RVec3,
) -> [world::sealed::Anchor; 2] {
    [point1, point2].map(|point| match space {
        ConstraintSpace::WorldSpace => world::sealed::Anchor::World(point),
        // `Real` is `f32` without the `double-precision` feature, so the casts are no-ops there.
        #[allow(clippy::unnecessary_cast)]
        ConstraintSpace::LocalToBodyCom => world::sealed::Anchor::CenterOfMass(Vec3::new(
            point.x as f32,
            point.y as f32,
            point.z as f32,
        )),
    })
}

/// What the axes of a constraint frame must satisfy ([`is_unit`]).
const FRAME_AXES_RULE: &str = "constraint frame axes must be unit vectors";

/// Checks one frame: a finite point and two perpendicular unit axes.
fn validate_frame(point: RVec3, axis_a: Vec3, axis_b: Vec3) -> Result<(), &'static str> {
    validate_point(point)?;
    if !(is_unit(axis_a) && is_unit(axis_b)) {
        return Err(FRAME_AXES_RULE);
    }
    if axis_a.dot(axis_b).abs() > PERPENDICULAR_TOLERANCE {
        return Err("the two axes of a constraint frame must be perpendicular");
    }
    Ok(())
}

/// A swing-twist constraint (Jolt `SwingTwistConstraintSettings`), the usual shoulder, neck or
/// spine joint: body 2 twists about the twist axis within `[twist_min, twist_max]`, and the twist
/// axis swings away from body 1's within a cone (or pyramid) of half angles `normal` (about the
/// normal axis, `twist × plane`) and `plane` (about the plane axis).
///
/// The default is Jolt's: both frames at the origin with twist axis +X and plane axis +Y, a cone
/// with all limits 0 (locked), no friction and default motors.
#[derive(Clone, Debug, PartialEq)]
pub struct SwingTwistConstraintSettings {
    space: ConstraintSpace,
    position1: RVec3,
    twist_axis1: Vec3,
    plane_axis1: Vec3,
    position2: RVec3,
    twist_axis2: Vec3,
    plane_axis2: Vec3,
    swing_type: SwingType,
    normal_half_cone_angle: f32,
    plane_half_cone_angle: f32,
    twist_min_angle: f32,
    twist_max_angle: f32,
    max_friction_torque: f32,
    swing_motor: MotorSettings,
    twist_motor: MotorSettings,
    solver: SolverOverrides,
}

impl Default for SwingTwistConstraintSettings {
    fn default() -> Self {
        Self::new(
            RVec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
    }
}

impl SwingTwistConstraintSettings {
    /// A joint at `position` with the same frame on both bodies, in world space: `twist_axis`
    /// and the perpendicular `plane_axis` are unit vectors. The limits are 0 (locked) until set.
    pub fn new(position: RVec3, twist_axis: Vec3, plane_axis: Vec3) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            position1: position,
            twist_axis1: twist_axis,
            plane_axis1: plane_axis,
            position2: position,
            twist_axis2: twist_axis,
            plane_axis2: plane_axis,
            swing_type: SwingType::Cone,
            normal_half_cone_angle: 0.0,
            plane_half_cone_angle: 0.0,
            twist_min_angle: 0.0,
            twist_max_angle: 0.0,
            max_friction_torque: 0.0,
            swing_motor: MotorSettings::default(),
            twist_motor: MotorSettings::default(),
            solver: SolverOverrides::default(),
        }
    }

    /// The space the frames are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The frame on body 1 (the parent): a point and perpendicular unit twist and plane axes.
    #[must_use]
    pub fn frame1(mut self, position: RVec3, twist_axis: Vec3, plane_axis: Vec3) -> Self {
        self.position1 = position;
        self.twist_axis1 = twist_axis;
        self.plane_axis1 = plane_axis;
        self
    }

    /// The frame on body 2 (the child): a point and perpendicular unit twist and plane axes.
    #[must_use]
    pub fn frame2(mut self, position: RVec3, twist_axis: Vec3, plane_axis: Vec3) -> Self {
        self.position2 = position;
        self.twist_axis2 = twist_axis;
        self.plane_axis2 = plane_axis;
        self
    }

    /// How the swing limits combine. Default [`SwingType::Cone`].
    #[must_use]
    pub fn swing_type(mut self, value: SwingType) -> Self {
        self.swing_type = value;
        self
    }

    /// Half angles of the swing cone, radians in `[0, π]`: about the normal axis
    /// (`twist × plane`) and about the plane axis.
    #[must_use]
    pub fn half_cone_angles(mut self, normal: f32, plane: f32) -> Self {
        self.normal_half_cone_angle = normal;
        self.plane_half_cone_angle = plane;
        self
    }

    /// Twist range, radians in `[-π, π]` with `min <= max`.
    #[must_use]
    pub fn twist_limits(mut self, min: f32, max: f32) -> Self {
        self.twist_min_angle = min;
        self.twist_max_angle = max;
        self
    }

    /// Torque in N·m that friction applies while no motor drives the joint, at least 0.
    /// Default 0.
    #[must_use]
    pub fn max_friction_torque(mut self, value: f32) -> Self {
        self.max_friction_torque = value;
        self
    }

    /// The motor that drives the swing.
    #[must_use]
    pub fn swing_motor(mut self, value: MotorSettings) -> Self {
        self.swing_motor = value;
        self
    }

    /// The motor that drives the twist.
    #[must_use]
    pub fn twist_motor(mut self, value: MotorSettings) -> Self {
        self.twist_motor = value;
        self
    }

    pub(crate) fn validate(&self) -> Result<(), &'static str> {
        self.solver.validate()?;
        validate_frame(self.position1, self.twist_axis1, self.plane_axis1)?;
        validate_frame(self.position2, self.twist_axis2, self.plane_axis2)?;
        if !(within(self.normal_half_cone_angle, 0.0, PI)
            && within(self.plane_half_cone_angle, 0.0, PI))
        {
            return Err("swing-twist half cone angles must be between 0 and pi");
        }
        if !(within(self.twist_min_angle, -PI, PI)
            && within(self.twist_max_angle, -PI, PI)
            && self.twist_min_angle <= self.twist_max_angle)
        {
            return Err("swing-twist twist limits must be between -pi and pi with min <= max");
        }
        if !non_negative(self.max_friction_torque) {
            return Err(FRICTION_TORQUE_RULE);
        }
        self.swing_motor.validate()?;
        self.twist_motor.validate()
    }

    /// The springs of both motors.
    pub(crate) fn springs(&self) -> impl Iterator<Item = SpringSettings> {
        [self.swing_motor.spring, self.twist_motor.spring].into_iter()
    }

    pub(crate) fn to_jph(&self) -> JPH_SwingTwistConstraintSettings {
        JPH_SwingTwistConstraintSettings {
            base: self.solver.to_jph(),
            space: self.space.to_jph(),
            position1: self.position1.to_jph(),
            twistAxis1: self.twist_axis1.to_jph(),
            planeAxis1: self.plane_axis1.to_jph(),
            position2: self.position2.to_jph(),
            twistAxis2: self.twist_axis2.to_jph(),
            planeAxis2: self.plane_axis2.to_jph(),
            swingType: self.swing_type.to_jph(),
            normalHalfConeAngle: self.normal_half_cone_angle,
            planeHalfConeAngle: self.plane_half_cone_angle,
            twistMinAngle: self.twist_min_angle,
            twistMaxAngle: self.twist_max_angle,
            maxFrictionTorque: self.max_friction_torque,
            swingMotorSettings: self.swing_motor.to_jph(),
            twistMotorSettings: self.twist_motor.to_jph(),
        }
    }
}

/// A hinge (Jolt `HingeConstraintSettings`), the usual elbow or knee joint: body 2 rotates
/// about the hinge axis by an angle in `[limits_min, limits_max]`, measured from the pose where
/// both normal axes coincide.
///
/// The default is Jolt's: both frames at the origin with hinge axis +Y and normal axis +X, no
/// limits (`[-π, π]`), no friction and a default motor.
#[derive(Clone, Debug, PartialEq)]
pub struct HingeConstraintSettings {
    space: ConstraintSpace,
    point1: RVec3,
    hinge_axis1: Vec3,
    normal_axis1: Vec3,
    point2: RVec3,
    hinge_axis2: Vec3,
    normal_axis2: Vec3,
    limits_min: f32,
    limits_max: f32,
    limits_spring: SpringSettings,
    max_friction_torque: f32,
    motor: MotorSettings,
    solver: SolverOverrides,
}

impl Default for HingeConstraintSettings {
    fn default() -> Self {
        Self::new(
            RVec3::ZERO,
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
    }
}

impl HingeConstraintSettings {
    /// A hinge at `point` with the same frame on both bodies, in world space: `hinge_axis` and
    /// the perpendicular `normal_axis` are unit vectors. No limits until set.
    pub fn new(point: RVec3, hinge_axis: Vec3, normal_axis: Vec3) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            point1: point,
            hinge_axis1: hinge_axis,
            normal_axis1: normal_axis,
            point2: point,
            hinge_axis2: hinge_axis,
            normal_axis2: normal_axis,
            limits_min: -PI,
            limits_max: PI,
            limits_spring: SpringSettings::default(),
            max_friction_torque: 0.0,
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

    /// The frame on body 1 (the parent): a point and perpendicular unit hinge and normal axes.
    #[must_use]
    pub fn frame1(mut self, point: RVec3, hinge_axis: Vec3, normal_axis: Vec3) -> Self {
        self.point1 = point;
        self.hinge_axis1 = hinge_axis;
        self.normal_axis1 = normal_axis;
        self
    }

    /// The frame on body 2 (the child): a point and perpendicular unit hinge and normal axes.
    #[must_use]
    pub fn frame2(mut self, point: RVec3, hinge_axis: Vec3, normal_axis: Vec3) -> Self {
        self.point2 = point;
        self.hinge_axis2 = hinge_axis;
        self.normal_axis2 = normal_axis;
        self
    }

    /// The angle range in radians: `min` in `[-π, 0]`, `max` in `[0, π]`; `[-π, π]` means no
    /// limits. `min == max` needs a soft [`limits_spring`](Self::limits_spring).
    #[must_use]
    pub fn limits(mut self, min: f32, max: f32) -> Self {
        self.limits_min = min;
        self.limits_max = max;
        self
    }

    /// Makes the limits soft. Default rigid.
    #[must_use]
    pub fn limits_spring(mut self, value: SpringSettings) -> Self {
        self.limits_spring = value;
        self
    }

    /// Torque in N·m that friction applies while the motor is off, at least 0. Default 0.
    #[must_use]
    pub fn max_friction_torque(mut self, value: f32) -> Self {
        self.max_friction_torque = value;
        self
    }

    /// The motor.
    #[must_use]
    pub fn motor(mut self, value: MotorSettings) -> Self {
        self.motor = value;
        self
    }

    pub(crate) fn validate(&self) -> Result<(), &'static str> {
        self.solver.validate()?;
        validate_frame(self.point1, self.hinge_axis1, self.normal_axis1)?;
        validate_frame(self.point2, self.hinge_axis2, self.normal_axis2)?;
        self.limits_spring.validate()?;
        validate_hinge_limits(self.limits_min, self.limits_max, self.limits_spring)?;
        if !non_negative(self.max_friction_torque) {
            return Err(FRICTION_TORQUE_RULE);
        }
        self.motor.validate()
    }

    /// The limits spring and the motor's spring.
    pub(crate) fn springs(&self) -> impl Iterator<Item = SpringSettings> {
        [self.limits_spring, self.motor.spring].into_iter()
    }

    pub(crate) fn to_jph(&self) -> JPH_HingeConstraintSettings {
        JPH_HingeConstraintSettings {
            base: self.solver.to_jph(),
            space: self.space.to_jph(),
            point1: self.point1.to_jph(),
            hingeAxis1: self.hinge_axis1.to_jph(),
            normalAxis1: self.normal_axis1.to_jph(),
            point2: self.point2.to_jph(),
            hingeAxis2: self.hinge_axis2.to_jph(),
            normalAxis2: self.normal_axis2.to_jph(),
            limitsMin: self.limits_min,
            limitsMax: self.limits_max,
            limitsSpringSettings: self.limits_spring.to_jph(),
            maxFrictionTorque: self.max_friction_torque,
            motorSettings: self.motor.to_jph(),
        }
    }
}

/// Checks a hinge's limits against the rules of Jolt's `HingeConstraint`.
fn validate_hinge_limits(
    min: f32,
    max: f32,
    limits_spring: SpringSettings,
) -> Result<(), &'static str> {
    // Jolt asserts these ranges in `HingeConstraint::SetLimits`.
    if !(within(min, -PI, 0.0) && within(max, 0.0, PI)) {
        return Err("hinge limits must be min in [-pi, 0] and max in [0, pi]");
    }
    // Jolt asserts this in the `HingeConstraint` constructor.
    if min == max && !limits_spring.is_soft() {
        return Err("hinge limits with min == max need a soft limits spring");
    }
    Ok(())
}

/// One axis of a six-degree-of-freedom constraint (Jolt `SixDOFConstraintSettings::EAxis`).
/// Translations are along, rotations about, the axes of the constraint frame: X is the twist
/// axis, Y and Z the swing axes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SixDofConstraintAxis {
    /// Translation along X.
    TranslationX,
    /// Translation along Y.
    TranslationY,
    /// Translation along Z.
    TranslationZ,
    /// Rotation about X, the twist.
    RotationX,
    /// Rotation about Y, a swing.
    RotationY,
    /// Rotation about Z, a swing.
    RotationZ,
}

impl SixDofConstraintAxis {
    /// Every axis, in Jolt's order.
    pub(crate) const ALL: [Self; 6] = [
        Self::TranslationX,
        Self::TranslationY,
        Self::TranslationZ,
        Self::RotationX,
        Self::RotationY,
        Self::RotationZ,
    ];

    /// The three rotation axes.
    pub(crate) const ROTATIONS: [Self; 3] = [Self::RotationX, Self::RotationY, Self::RotationZ];

    /// Jolt's index of the axis.
    pub(crate) fn index(self) -> usize {
        self as usize
    }

    fn is_translation(self) -> bool {
        self.index() < 3
    }

    pub(crate) fn to_jph(self) -> JPH_SixDOFConstraintAxis {
        match self {
            Self::TranslationX => JPH_SixDOFConstraintAxis_TranslationX,
            Self::TranslationY => JPH_SixDOFConstraintAxis_TranslationY,
            Self::TranslationZ => JPH_SixDOFConstraintAxis_TranslationZ,
            Self::RotationX => JPH_SixDOFConstraintAxis_RotationX,
            Self::RotationY => JPH_SixDOFConstraintAxis_RotationY,
            Self::RotationZ => JPH_SixDOFConstraintAxis_RotationZ,
        }
    }
}

/// How one axis of a six-degree-of-freedom constraint may move.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum SixDofAxis {
    /// Moves freely (Jolt `MakeFreeAxis`). The default.
    #[default]
    Free,
    /// Does not move (Jolt `MakeFixedAxis`).
    Fixed,
    /// Moves within `[min, max]`, `min < max`: metres within
    /// [`limits::MAX_SHAPE_EXTENT`] for translations, radians in `[-π, π]` for rotations.
    Limited {
        /// Lower limit.
        min: f32,
        /// Upper limit.
        max: f32,
    },
}

impl SixDofAxis {
    /// The limits Jolt stores for this axis.
    fn limits(self) -> (f32, f32) {
        match self {
            Self::Free => (-f32::MAX, f32::MAX),
            Self::Fixed => (f32::MAX, -f32::MAX),
            Self::Limited { min, max } => (min, max),
        }
    }
}

/// A six-degree-of-freedom constraint (Jolt `SixDOFConstraintSettings`): each translation and
/// rotation axis of the constraint frame is free, fixed or limited on its own. With
/// [`SwingType::Pyramid`] the two swing ranges may be asymmetric, which suits hips.
///
/// The default is Jolt's: both frames at the origin with axes +X and +Y, every axis free, a cone
/// swing, no friction, rigid translation limits and default motors.
#[derive(Clone, Debug, PartialEq)]
pub struct SixDofConstraintSettings {
    space: ConstraintSpace,
    position1: RVec3,
    axis_x1: Vec3,
    axis_y1: Vec3,
    position2: RVec3,
    axis_x2: Vec3,
    axis_y2: Vec3,
    swing_type: SwingType,
    axes: [SixDofAxis; 6],
    max_friction: [f32; 6],
    limits_springs: [SpringSettings; 6],
    motors: [MotorSettings; 6],
    solver: SolverOverrides,
}

impl Default for SixDofConstraintSettings {
    fn default() -> Self {
        Self::new(
            RVec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
    }
}

impl SixDofConstraintSettings {
    /// A joint at `position` with the same frame on both bodies, in world space: `axis_x` (the
    /// twist axis) and the perpendicular `axis_y` are unit vectors. Every axis is free until set.
    pub fn new(position: RVec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            position1: position,
            axis_x1: axis_x,
            axis_y1: axis_y,
            position2: position,
            axis_x2: axis_x,
            axis_y2: axis_y,
            swing_type: SwingType::Cone,
            axes: [SixDofAxis::Free; 6],
            max_friction: [0.0; 6],
            limits_springs: [SpringSettings::default(); 6],
            motors: [MotorSettings::default(); 6],
            solver: SolverOverrides::default(),
        }
    }

    /// The space the frames are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The frame on body 1 (the parent): a point and perpendicular unit X and Y axes.
    #[must_use]
    pub fn frame1(mut self, position: RVec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        self.position1 = position;
        self.axis_x1 = axis_x;
        self.axis_y1 = axis_y;
        self
    }

    /// The frame on body 2 (the child): a point and perpendicular unit X and Y axes.
    #[must_use]
    pub fn frame2(mut self, position: RVec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        self.position2 = position;
        self.axis_x2 = axis_x;
        self.axis_y2 = axis_y;
        self
    }

    /// How the swing limits combine. Default [`SwingType::Cone`], which needs symmetric
    /// rotation Y and Z limits; [`SwingType::Pyramid`] allows asymmetric ones.
    #[must_use]
    pub fn swing_type(mut self, value: SwingType) -> Self {
        self.swing_type = value;
        self
    }

    /// How `which` may move. Default [`SixDofAxis::Free`].
    #[must_use]
    pub fn axis(mut self, which: SixDofConstraintAxis, value: SixDofAxis) -> Self {
        self.axes[which.index()] = value;
        self
    }

    /// Force (N) or torque (N·m) friction applies along `which` while its motor is off, at
    /// least 0. Default 0.
    #[must_use]
    pub fn max_friction(mut self, which: SixDofConstraintAxis, value: f32) -> Self {
        self.max_friction[which.index()] = value;
        self
    }

    /// Makes the limits of the limited translation axis `which` soft. Jolt has limit springs for
    /// translations only. Default rigid.
    #[must_use]
    pub fn limits_spring(mut self, which: SixDofConstraintAxis, value: SpringSettings) -> Self {
        self.limits_springs[which.index()] = value;
        self
    }

    /// The motor of `which`.
    #[must_use]
    pub fn motor(mut self, which: SixDofConstraintAxis, value: MotorSettings) -> Self {
        self.motors[which.index()] = value;
        self
    }

    pub(crate) fn validate(&self) -> Result<(), &'static str> {
        self.solver.validate()?;
        validate_frame(self.position1, self.axis_x1, self.axis_y1)?;
        validate_frame(self.position2, self.axis_x2, self.axis_y2)?;
        for which in SixDofConstraintAxis::ALL {
            let i = which.index();
            if let SixDofAxis::Limited { min, max } = self.axes[i] {
                if !(min.is_finite() && max.is_finite() && min < max) {
                    return Err(
                        "limited axes need finite limits with min < max; use Fixed for min == max",
                    );
                }
                if which.is_translation() {
                    let extent = limits::MAX_SHAPE_EXTENT;
                    if !(within(min, -extent, extent) && within(max, -extent, extent)) {
                        return Err("translation limits must be within limits::MAX_SHAPE_EXTENT");
                    }
                } else if !(within(min, -PI, PI) && within(max, -PI, PI)) {
                    return Err("rotation limits must be between -pi and pi");
                }
            }
            if !non_negative(self.max_friction[i]) {
                return Err(FRICTION_RULE);
            }
            let spring = self.limits_springs[i];
            spring.validate()?;
            // Jolt uses limit springs that are soft (`SixDOFConstraint::CacheHasSpringLimits`).
            if spring.is_soft() {
                if !which.is_translation() {
                    return Err("limit springs exist only for translation axes");
                }
                if self.axes[i] == SixDofAxis::Free {
                    return Err("a limit spring needs a limited or fixed translation axis");
                }
            }
            self.motors[i].validate()?;
        }
        // Jolt silently makes cone swing limits symmetric (`SixDOFConstraint::UpdateRotationLimits`).
        if self.swing_type == SwingType::Cone {
            for which in [
                SixDofConstraintAxis::RotationY,
                SixDofConstraintAxis::RotationZ,
            ] {
                if let SixDofAxis::Limited { min, max } = self.axes[which.index()] {
                    if min != -max {
                        return Err(
                            "cone swing limits must be symmetric; use SwingType::Pyramid for asymmetric ones",
                        );
                    }
                }
            }
        }
        Ok(())
    }

    /// The translation limit springs and the springs of all six motors.
    pub(crate) fn springs(&self) -> impl Iterator<Item = SpringSettings> + '_ {
        self.limits_springs[..3]
            .iter()
            .copied()
            .chain(self.motors.iter().map(|motor| motor.spring))
    }

    pub(crate) fn to_jph(&self) -> JPH_SixDOFConstraintSettings {
        let limits = self.axes.map(SixDofAxis::limits);
        JPH_SixDOFConstraintSettings {
            base: self.solver.to_jph(),
            space: self.space.to_jph(),
            position1: self.position1.to_jph(),
            axisX1: self.axis_x1.to_jph(),
            axisY1: self.axis_y1.to_jph(),
            position2: self.position2.to_jph(),
            axisX2: self.axis_x2.to_jph(),
            axisY2: self.axis_y2.to_jph(),
            maxFriction: self.max_friction,
            swingType: self.swing_type.to_jph(),
            limitMin: limits.map(|(min, _)| min),
            limitMax: limits.map(|(_, max)| max),
            limitsSpringSettings: [0, 1, 2].map(|i| self.limits_springs[i].to_jph()),
            motorSettings: self.motors.map(MotorSettings::to_jph),
        }
    }
}

#[cfg(test)]
mod tests;
