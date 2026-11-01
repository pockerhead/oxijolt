use super::*;
use crate::world::ensure_initialized;
use crate::Real;

fn assert_spring_eq(ours: JPH_SpringSettings, jolt: JPH_SpringSettings) {
    assert_eq!(ours.mode, jolt.mode);
    assert_eq!(ours.frequencyOrStiffness, jolt.frequencyOrStiffness);
    assert_eq!(ours.damping, jolt.damping);
}

fn assert_motor_eq(ours: JPH_MotorSettings, jolt: JPH_MotorSettings) {
    assert_spring_eq(ours.springSettings, jolt.springSettings);
    assert_eq!(ours.minForceLimit, jolt.minForceLimit);
    assert_eq!(ours.maxForceLimit, jolt.maxForceLimit);
    assert_eq!(ours.minTorqueLimit, jolt.minTorqueLimit);
    assert_eq!(ours.maxTorqueLimit, jolt.maxTorqueLimit);
}

fn assert_base_eq(ours: JPH_ConstraintSettings, jolt: JPH_ConstraintSettings) {
    assert_eq!(ours.enabled, jolt.enabled);
    assert_eq!(ours.constraintPriority, jolt.constraintPriority);
    assert_eq!(ours.numVelocityStepsOverride, jolt.numVelocityStepsOverride);
    assert_eq!(ours.numPositionStepsOverride, jolt.numPositionStepsOverride);
    assert_eq!(ours.drawConstraintSize, jolt.drawConstraintSize);
    assert_eq!(ours.userData, jolt.userData);
}

fn bits(v: JPH_Vec3) -> [u32; 3] {
    [v.x, v.y, v.z].map(f32::to_bits)
}

#[test]
fn constraint_base_is_jolts_default() {
    assert!(ensure_initialized());
    // SAFETY: an all-zero `JPH_VehicleConstraintSettings` is valid: floats, integers,
    // `false` and null pointers. joltc fills it with Jolt's defaults and allocates nothing.
    let mut settings: JPH_VehicleConstraintSettings = unsafe { std::mem::zeroed() };
    // SAFETY: Jolt is initialised and `settings` is a live local.
    unsafe { JPH_VehicleConstraintSettings_Init(&mut settings) };
    assert_base_eq(constraint_base(), settings.base);
}

#[test]
fn swing_twist_defaults_are_jolts() {
    assert!(ensure_initialized());
    // SAFETY: an all-zero `JPH_SwingTwistConstraintSettings` is valid: floats, integers and
    // enums with a zero value. joltc fills it with Jolt's defaults and allocates nothing.
    let mut jolt: JPH_SwingTwistConstraintSettings = unsafe { std::mem::zeroed() };
    // SAFETY: Jolt is initialised and `jolt` is a live local.
    unsafe { JPH_SwingTwistConstraintSettings_Init(&mut jolt) };
    let ours = SwingTwistConstraintSettings::default();
    assert_eq!(ours.validate(), Ok(()));
    let ours = ours.to_jph();
    assert_base_eq(ours.base, jolt.base);
    assert_eq!(ours.space, jolt.space);
    assert_eq!(
        RVec3::from_jph(ours.position1),
        RVec3::from_jph(jolt.position1)
    );
    assert_eq!(bits(ours.twistAxis1), bits(jolt.twistAxis1));
    assert_eq!(bits(ours.planeAxis1), bits(jolt.planeAxis1));
    assert_eq!(
        RVec3::from_jph(ours.position2),
        RVec3::from_jph(jolt.position2)
    );
    assert_eq!(bits(ours.twistAxis2), bits(jolt.twistAxis2));
    assert_eq!(bits(ours.planeAxis2), bits(jolt.planeAxis2));
    assert_eq!(ours.swingType, jolt.swingType);
    assert_eq!(ours.normalHalfConeAngle, jolt.normalHalfConeAngle);
    assert_eq!(ours.planeHalfConeAngle, jolt.planeHalfConeAngle);
    assert_eq!(ours.twistMinAngle, jolt.twistMinAngle);
    assert_eq!(ours.twistMaxAngle, jolt.twistMaxAngle);
    assert_eq!(ours.maxFrictionTorque, jolt.maxFrictionTorque);
    assert_motor_eq(ours.swingMotorSettings, jolt.swingMotorSettings);
    assert_motor_eq(ours.twistMotorSettings, jolt.twistMotorSettings);
}

#[test]
fn hinge_defaults_are_jolts() {
    assert!(ensure_initialized());
    // SAFETY: as in `swing_twist_defaults_are_jolts`, for `JPH_HingeConstraintSettings`.
    let mut jolt: JPH_HingeConstraintSettings = unsafe { std::mem::zeroed() };
    // SAFETY: Jolt is initialised and `jolt` is a live local.
    unsafe { JPH_HingeConstraintSettings_Init(&mut jolt) };
    let ours = HingeConstraintSettings::default();
    assert_eq!(ours.validate(), Ok(()));
    let ours = ours.to_jph();
    assert_base_eq(ours.base, jolt.base);
    assert_eq!(ours.space, jolt.space);
    assert_eq!(RVec3::from_jph(ours.point1), RVec3::from_jph(jolt.point1));
    assert_eq!(bits(ours.hingeAxis1), bits(jolt.hingeAxis1));
    assert_eq!(bits(ours.normalAxis1), bits(jolt.normalAxis1));
    assert_eq!(RVec3::from_jph(ours.point2), RVec3::from_jph(jolt.point2));
    assert_eq!(bits(ours.hingeAxis2), bits(jolt.hingeAxis2));
    assert_eq!(bits(ours.normalAxis2), bits(jolt.normalAxis2));
    assert_eq!(ours.limitsMin, jolt.limitsMin);
    assert_eq!(ours.limitsMax, jolt.limitsMax);
    assert_spring_eq(ours.limitsSpringSettings, jolt.limitsSpringSettings);
    assert_eq!(ours.maxFrictionTorque, jolt.maxFrictionTorque);
    assert_motor_eq(ours.motorSettings, jolt.motorSettings);
}

#[test]
fn six_dof_defaults_are_jolts() {
    assert!(ensure_initialized());
    // SAFETY: as in `swing_twist_defaults_are_jolts`, for `JPH_SixDOFConstraintSettings`.
    let mut jolt: JPH_SixDOFConstraintSettings = unsafe { std::mem::zeroed() };
    // SAFETY: Jolt is initialised and `jolt` is a live local.
    unsafe { JPH_SixDOFConstraintSettings_Init(&mut jolt) };
    let ours = SixDofConstraintSettings::default();
    assert_eq!(ours.validate(), Ok(()));
    let ours = ours.to_jph();
    assert_base_eq(ours.base, jolt.base);
    assert_eq!(ours.space, jolt.space);
    assert_eq!(
        RVec3::from_jph(ours.position1),
        RVec3::from_jph(jolt.position1)
    );
    assert_eq!(bits(ours.axisX1), bits(jolt.axisX1));
    assert_eq!(bits(ours.axisY1), bits(jolt.axisY1));
    assert_eq!(
        RVec3::from_jph(ours.position2),
        RVec3::from_jph(jolt.position2)
    );
    assert_eq!(bits(ours.axisX2), bits(jolt.axisX2));
    assert_eq!(bits(ours.axisY2), bits(jolt.axisY2));
    assert_eq!(ours.maxFriction, jolt.maxFriction);
    assert_eq!(ours.swingType, jolt.swingType);
    assert_eq!(ours.limitMin, jolt.limitMin);
    assert_eq!(ours.limitMax, jolt.limitMax);
    for (a, b) in ours
        .limitsSpringSettings
        .into_iter()
        .zip(jolt.limitsSpringSettings)
    {
        assert_spring_eq(a, b);
    }
    for (a, b) in ours.motorSettings.into_iter().zip(jolt.motorSettings) {
        assert_motor_eq(a, b);
    }
}

#[test]
fn fixed_and_free_map_to_jolts_sentinels() {
    assert!(ensure_initialized());
    let x = SixDofConstraintAxis::TranslationX;
    let ours = SixDofConstraintSettings::default()
        .axis(x, SixDofAxis::Fixed)
        .to_jph();
    // SAFETY: as in `six_dof_defaults_are_jolts`.
    let mut jolt: JPH_SixDOFConstraintSettings = unsafe { std::mem::zeroed() };
    // SAFETY: Jolt is initialised and `jolt` is a live local; the setters write its arrays.
    unsafe {
        JPH_SixDOFConstraintSettings_Init(&mut jolt);
        JPH_SixDOFConstraintSettings_MakeFixedAxis(&mut jolt, x.to_jph());
        assert!(JPH_SixDOFConstraintSettings_IsFixedAxis(&ours, x.to_jph()));
    }
    assert_eq!(ours.limitMin, jolt.limitMin);
    assert_eq!(ours.limitMax, jolt.limitMax);
    let free = SixDofConstraintSettings::default().to_jph();
    // SAFETY: `free` is a live local; the getter only reads it.
    assert!(unsafe { JPH_SixDOFConstraintSettings_IsFreeAxis(&free, x.to_jph()) });
}

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);

fn swing_twist() -> SwingTwistConstraintSettings {
    SwingTwistConstraintSettings::new(RVec3::ZERO, X, Y)
}

fn hinge() -> HingeConstraintSettings {
    HingeConstraintSettings::new(RVec3::ZERO, X, Y)
}

fn six_dof() -> SixDofConstraintSettings {
    SixDofConstraintSettings::new(RVec3::ZERO, X, Y)
}

#[test]
fn frames_are_validated() {
    let skewed = Vec3::new(0.0, 0.00005, 1.0).normalized_or_zero();
    let tilted = Vec3::new(0.001, 1.0, 0.0).normalized_or_zero();
    let not_unit = Vec3::new(0.0, 1.1, 0.0);
    let nan = RVec3::new(Real::NAN, 0.0, 0.0);
    assert!(swing_twist().frame1(nan, X, Y).validate().is_err());
    assert!(swing_twist()
        .frame2(RVec3::ZERO, X, not_unit)
        .validate()
        .is_err());
    assert!(hinge().frame1(RVec3::ZERO, X, tilted).validate().is_err());
    assert!(six_dof().frame2(RVec3::ZERO, Y, X).validate().is_ok());
    assert!(six_dof().frame2(RVec3::ZERO, skewed, Y).validate().is_ok());
    assert!(six_dof().frame2(RVec3::ZERO, X, tilted).validate().is_err());
}

#[test]
fn swing_twist_limits_are_validated() {
    assert!(swing_twist().half_cone_angles(PI, 0.0).validate().is_ok());
    assert!(swing_twist()
        .half_cone_angles(PI + 0.01, 0.0)
        .validate()
        .is_err());
    assert!(swing_twist()
        .half_cone_angles(0.5, -0.01)
        .validate()
        .is_err());
    assert!(swing_twist().twist_limits(-PI, PI).validate().is_ok());
    assert!(swing_twist()
        .twist_limits(-PI - 0.01, 0.0)
        .validate()
        .is_err());
    assert!(swing_twist().twist_limits(0.3, 0.2).validate().is_err());
    assert!(swing_twist().twist_limits(0.2, 0.2).validate().is_ok());
    assert!(swing_twist().max_friction_torque(-1.0).validate().is_err());
    assert!(swing_twist()
        .max_friction_torque(f32::NAN)
        .validate()
        .is_err());
}

#[test]
fn hinge_limits_are_validated() {
    assert!(hinge().limits(-PI, PI).validate().is_ok());
    assert!(hinge().limits(-1.0, 0.0).validate().is_ok());
    assert!(hinge().limits(0.0, 0.5).validate().is_ok());
    assert!(hinge().limits(0.1, 0.5).validate().is_err());
    assert!(hinge().limits(-0.5, -0.1).validate().is_err());
    assert!(hinge().limits(-PI - 0.01, 0.0).validate().is_err());
    assert!(hinge().limits(0.0, 0.0).validate().is_err());
    assert!(hinge().max_friction_torque(-1.0).validate().is_err());
    assert!(hinge()
        .max_friction_torque(f32::INFINITY)
        .validate()
        .is_err());
    let soft = SpringSettings::FrequencyAndDamping {
        frequency: 5.0,
        damping: 0.5,
    };
    assert!(hinge()
        .limits(0.0, 0.0)
        .limits_spring(soft)
        .validate()
        .is_ok());
}

#[test]
fn six_dof_limits_are_validated() {
    let rx = SixDofConstraintAxis::RotationX;
    let ry = SixDofConstraintAxis::RotationY;
    let tx = SixDofConstraintAxis::TranslationX;
    let limited = |min, max| SixDofAxis::Limited { min, max };
    assert!(six_dof().axis(rx, limited(-PI, PI)).validate().is_ok());
    assert!(six_dof()
        .axis(rx, limited(-PI - 0.01, 0.0))
        .validate()
        .is_err());
    assert!(six_dof().axis(rx, limited(0.2, 0.2)).validate().is_err());
    assert!(six_dof().axis(tx, limited(-5.0, 5.0)).validate().is_ok());
    let extent = limits::MAX_SHAPE_EXTENT;
    assert!(six_dof()
        .axis(tx, limited(-extent, extent))
        .validate()
        .is_ok());
    for (min, max) in [(-extent.next_up(), 0.0), (0.0, extent.next_up())] {
        assert!(six_dof().axis(tx, limited(min, max)).validate().is_err());
    }
    assert!(six_dof()
        .axis(tx, limited(f32::NEG_INFINITY, 0.0))
        .validate()
        .is_err());
    // Cone swings are symmetric; asymmetric ones need a pyramid.
    assert!(six_dof().axis(ry, limited(-0.3, 1.6)).validate().is_err());
    assert!(six_dof().axis(ry, limited(-0.5, 0.5)).validate().is_ok());
    assert!(six_dof()
        .swing_type(SwingType::Pyramid)
        .axis(ry, limited(-0.3, 1.6))
        .validate()
        .is_ok());
    let soft = SpringSettings::StiffnessAndDamping {
        stiffness: 100.0,
        damping: 1.0,
    };
    assert!(six_dof().limits_spring(tx, soft).validate().is_err());
    assert!(six_dof()
        .axis(tx, limited(-0.1, 0.1))
        .limits_spring(tx, soft)
        .validate()
        .is_ok());
    assert!(six_dof()
        .axis(rx, limited(-0.1, 0.1))
        .limits_spring(rx, soft)
        .validate()
        .is_err());
    assert!(six_dof().max_friction(rx, -1.0).validate().is_err());
}

#[test]
fn motors_and_springs_are_validated() {
    let negative = SpringSettings::FrequencyAndDamping {
        frequency: -1.0,
        damping: 0.0,
    };
    let motor = MotorSettings::default();
    assert!(hinge().motor(motor.spring(negative)).validate().is_err());
    assert!(hinge()
        .motor(motor.torque_limits(1.0, -1.0))
        .validate()
        .is_err());
    assert!(hinge()
        .motor(motor.force_limits(f32::NEG_INFINITY, 0.0))
        .validate()
        .is_err());
    assert!(hinge()
        .motor(motor.torque_limits(-50.0, 50.0))
        .validate()
        .is_ok());
    assert!(hinge().limits_spring(negative).validate().is_err());
    assert!(swing_twist()
        .twist_motor(motor.spring(negative))
        .validate()
        .is_err());
    assert!(six_dof()
        .motor(SixDofConstraintAxis::RotationZ, motor.spring(negative))
        .validate()
        .is_err());
}

#[test]
fn ragdoll_joint_kinds_bound_their_step_overrides() {
    // Ragdoll joints are checked by these `validate`s, not by `create_constraint`.
    let max = limits::MAX_SOLVER_STEPS_OVERRIDE;
    for (velocity, position, accepted) in
        [(max, max, true), (max + 1, 0, false), (0, max + 1, false)]
    {
        let swing_twist = SwingTwistConstraintSettings::default()
            .velocity_steps_override(velocity)
            .position_steps_override(position);
        let hinge = HingeConstraintSettings::default()
            .velocity_steps_override(velocity)
            .position_steps_override(position);
        let six_dof = SixDofConstraintSettings::default()
            .velocity_steps_override(velocity)
            .position_steps_override(position);
        assert_eq!(swing_twist.validate().is_ok(), accepted);
        assert_eq!(hinge.validate().is_ok(), accepted);
        assert_eq!(six_dof.validate().is_ok(), accepted);
    }
}

#[test]
fn solver_overrides_reach_jolts_settings() {
    let base = SixDofConstraintSettings::default()
        .constraint_priority(u32::MAX)
        .velocity_steps_override(40)
        .position_steps_override(7)
        .to_jph()
        .base;
    assert_eq!(base.constraintPriority, u32::MAX);
    assert_eq!(base.numVelocityStepsOverride, 40);
    assert_eq!(base.numPositionStepsOverride, 7);
    assert_base_eq(
        JPH_ConstraintSettings {
            constraintPriority: 0,
            numVelocityStepsOverride: 0,
            numPositionStepsOverride: 0,
            ..base
        },
        constraint_base(),
    );
}
