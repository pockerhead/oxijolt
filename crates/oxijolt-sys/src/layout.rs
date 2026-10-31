//! Compile-time layout checks for the joltc types the raw tests and the safe layer use.
//!
//! These numbers mirror `native/layout_checks.cpp`, which checks the same C types
//! against Jolt's C++ types; change both together.

use core::mem::{align_of, offset_of, size_of};

use crate::*;

const _: () = {
    assert!(size_of::<JPH_Vec3>() == 12);
    assert!(align_of::<JPH_Vec3>() == 4);
    assert!(offset_of!(JPH_Vec3, z) == 8);

    assert!(size_of::<JPH_Vec4>() == 16);
    assert!(align_of::<JPH_Vec4>() == 4);
    assert!(offset_of!(JPH_Vec4, w) == 12);

    assert!(size_of::<JPH_Quat>() == 16);
    assert!(align_of::<JPH_Quat>() == 4);
    assert!(offset_of!(JPH_Quat, w) == 12);

    assert!(size_of::<JPH_Mat4>() == 64);
    assert!(align_of::<JPH_Mat4>() == 4);
    assert!(offset_of!(JPH_Mat4, column) == 0);

    assert!(size_of::<JPH_AABox>() == 24);
    assert!(align_of::<JPH_AABox>() == 4);
    assert!(offset_of!(JPH_AABox, min) == 0);
    assert!(offset_of!(JPH_AABox, max) == 12);

    assert!(size_of::<JPH_MassProperties>() == 68);
    assert!(align_of::<JPH_MassProperties>() == 4);
    assert!(offset_of!(JPH_MassProperties, mass) == 0);
    assert!(offset_of!(JPH_MassProperties, inertia) == 4);

    assert!(size_of::<JPH_RayCastResult>() == 12);
    assert!(align_of::<JPH_RayCastResult>() == 4);
    assert!(offset_of!(JPH_RayCastResult, bodyID) == 0);
    assert!(offset_of!(JPH_RayCastResult, fraction) == 4);
    assert!(offset_of!(JPH_RayCastResult, subShapeID2) == 8);

    assert!(size_of::<JPH_CollidePointResult>() == 8);
    assert!(align_of::<JPH_CollidePointResult>() == 4);
    assert!(offset_of!(JPH_CollidePointResult, bodyID) == 0);
    assert!(offset_of!(JPH_CollidePointResult, subShapeID2) == 4);

    assert!(size_of::<JPH_IndexedTriangle>() == 20);
    assert!(align_of::<JPH_IndexedTriangle>() == 4);
    assert!(offset_of!(JPH_IndexedTriangle, i1) == 0);
    assert!(offset_of!(JPH_IndexedTriangle, i2) == 4);
    assert!(offset_of!(JPH_IndexedTriangle, i3) == 8);
    assert!(offset_of!(JPH_IndexedTriangle, materialIndex) == 12);
    assert!(offset_of!(JPH_IndexedTriangle, userData) == 16);

    assert!(size_of::<JPH_SoftVertex>() == 28);
    assert!(align_of::<JPH_SoftVertex>() == 4);
    assert!(offset_of!(JPH_SoftVertex, position) == 0);
    assert!(offset_of!(JPH_SoftVertex, velocity) == 12);
    assert!(offset_of!(JPH_SoftVertex, invMass) == 24);

    assert!(size_of::<JPH_SoftFace>() == 16);
    assert!(align_of::<JPH_SoftFace>() == 4);
    assert!(offset_of!(JPH_SoftFace, vertex1) == 0);
    assert!(offset_of!(JPH_SoftFace, vertex2) == 4);
    assert!(offset_of!(JPH_SoftFace, vertex3) == 8);
    assert!(offset_of!(JPH_SoftFace, materialIndex) == 12);

    assert!(size_of::<JPH_SoftBodyVertexAttributes>() == 20);
    assert!(align_of::<JPH_SoftBodyVertexAttributes>() == 4);
    assert!(offset_of!(JPH_SoftBodyVertexAttributes, compliance) == 0);
    assert!(offset_of!(JPH_SoftBodyVertexAttributes, shearCompliance) == 4);
    assert!(offset_of!(JPH_SoftBodyVertexAttributes, bendCompliance) == 8);
    assert!(offset_of!(JPH_SoftBodyVertexAttributes, lraType) == 12);
    assert!(offset_of!(JPH_SoftBodyVertexAttributes, lraMaxDistanceMultiplier) == 16);

    assert!(size_of::<JPH_SoftBodySkinWeight>() == 8);
    assert!(align_of::<JPH_SoftBodySkinWeight>() == 4);
    assert!(offset_of!(JPH_SoftBodySkinWeight, invBindIndex) == 0);
    assert!(offset_of!(JPH_SoftBodySkinWeight, weight) == 4);

    assert!(size_of::<JPH_SoftBodySkinnedConstraint>() == 48);
    assert!(align_of::<JPH_SoftBodySkinnedConstraint>() == 4);
    assert!(offset_of!(JPH_SoftBodySkinnedConstraint, vertex) == 0);
    assert!(offset_of!(JPH_SoftBodySkinnedConstraint, weights) == 4);
    assert!(offset_of!(JPH_SoftBodySkinnedConstraint, maxDistance) == 36);
    assert!(offset_of!(JPH_SoftBodySkinnedConstraint, backStopDistance) == 40);
    assert!(offset_of!(JPH_SoftBodySkinnedConstraint, backStopRadius) == 44);

    assert!(size_of::<JPH_ContactSettings>() == 52);
    assert!(align_of::<JPH_ContactSettings>() == 4);

    assert!(size_of::<JPH_CharacterContactSettings>() == 2);
    assert!(align_of::<JPH_CharacterContactSettings>() == 1);
    assert!(offset_of!(JPH_CharacterContactSettings, canPushCharacter) == 0);
    assert!(offset_of!(JPH_CharacterContactSettings, canReceiveImpulses) == 1);
    assert!(offset_of!(JPH_ContactSettings, combinedFriction) == 0);
    assert!(offset_of!(JPH_ContactSettings, combinedRestitution) == 4);
    assert!(offset_of!(JPH_ContactSettings, invMassScale1) == 8);
    assert!(offset_of!(JPH_ContactSettings, invInertiaScale1) == 12);
    assert!(offset_of!(JPH_ContactSettings, invMassScale2) == 16);
    assert!(offset_of!(JPH_ContactSettings, invInertiaScale2) == 20);
    assert!(offset_of!(JPH_ContactSettings, isSensor) == 24);
    assert!(offset_of!(JPH_ContactSettings, relativeLinearSurfaceVelocity) == 28);
    assert!(offset_of!(JPH_ContactSettings, relativeAngularSurfaceVelocity) == 40);

    assert!(size_of::<JPH_SoftBodyContactSettings>() == 16);
    assert!(align_of::<JPH_SoftBodyContactSettings>() == 4);
    assert!(offset_of!(JPH_SoftBodyContactSettings, invMassScale1) == 0);
    assert!(offset_of!(JPH_SoftBodyContactSettings, invMassScale2) == 4);
    assert!(offset_of!(JPH_SoftBodyContactSettings, invInertiaScale2) == 8);
    assert!(offset_of!(JPH_SoftBodyContactSettings, isSensor) == 12);

    assert!(size_of::<JPH_RayCastSettings>() == 12);
    assert!(align_of::<JPH_RayCastSettings>() == 4);
    assert!(offset_of!(JPH_RayCastSettings, backFaceModeTriangles) == 0);
    assert!(offset_of!(JPH_RayCastSettings, backFaceModeConvex) == 4);
    assert!(offset_of!(JPH_RayCastSettings, treatConvexAsSolid) == 8);

    assert!(size_of::<JPH_CollideSettingsBase>() == 28);
    assert!(align_of::<JPH_CollideSettingsBase>() == 4);
    assert!(offset_of!(JPH_CollideSettingsBase, activeEdgeMode) == 0);
    assert!(offset_of!(JPH_CollideSettingsBase, collectFacesMode) == 4);
    assert!(offset_of!(JPH_CollideSettingsBase, collisionTolerance) == 8);
    assert!(offset_of!(JPH_CollideSettingsBase, penetrationTolerance) == 12);
    assert!(offset_of!(JPH_CollideSettingsBase, activeEdgeMovementDirection) == 16);

    assert!(size_of::<JPH_CollideShapeSettings>() == 36);
    assert!(align_of::<JPH_CollideShapeSettings>() == 4);
    assert!(offset_of!(JPH_CollideShapeSettings, base) == 0);
    assert!(offset_of!(JPH_CollideShapeSettings, maxSeparationDistance) == 28);
    assert!(offset_of!(JPH_CollideShapeSettings, backFaceMode) == 32);

    assert!(size_of::<JPH_ShapeCastSettings>() == 40);
    assert!(align_of::<JPH_ShapeCastSettings>() == 4);
    assert!(offset_of!(JPH_ShapeCastSettings, base) == 0);
    assert!(offset_of!(JPH_ShapeCastSettings, backFaceModeTriangles) == 28);
    assert!(offset_of!(JPH_ShapeCastSettings, backFaceModeConvex) == 32);
    assert!(offset_of!(JPH_ShapeCastSettings, useShrunkenShapeAndConvexRadius) == 36);
    assert!(offset_of!(JPH_ShapeCastSettings, returnDeepestPoint) == 37);

    assert!(size_of::<JPH_ShapeCastResult>() == 60);
    assert!(align_of::<JPH_ShapeCastResult>() == 4);
    assert!(offset_of!(JPH_ShapeCastResult, contactPointOn1) == 0);
    assert!(offset_of!(JPH_ShapeCastResult, contactPointOn2) == 12);
    assert!(offset_of!(JPH_ShapeCastResult, penetrationAxis) == 24);
    assert!(offset_of!(JPH_ShapeCastResult, penetrationDepth) == 36);
    assert!(offset_of!(JPH_ShapeCastResult, subShapeID1) == 40);
    assert!(offset_of!(JPH_ShapeCastResult, subShapeID2) == 44);
    assert!(offset_of!(JPH_ShapeCastResult, bodyID2) == 48);
    assert!(offset_of!(JPH_ShapeCastResult, fraction) == 52);
    assert!(offset_of!(JPH_ShapeCastResult, isBackFaceHit) == 56);

    assert!(size_of::<JPH_Plane>() == 16);
    assert!(align_of::<JPH_Plane>() == 4);
    assert!(offset_of!(JPH_Plane, normal) == 0);
    assert!(offset_of!(JPH_Plane, distance) == 12);

    assert!(size_of::<JPH_ExtendedUpdateSettings>() == 48);
    assert!(align_of::<JPH_ExtendedUpdateSettings>() == 4);
    assert!(offset_of!(JPH_ExtendedUpdateSettings, stickToFloorStepDown) == 0);
    assert!(offset_of!(JPH_ExtendedUpdateSettings, walkStairsStepUp) == 12);
    assert!(offset_of!(JPH_ExtendedUpdateSettings, walkStairsMinStepForward) == 24);
    assert!(offset_of!(JPH_ExtendedUpdateSettings, walkStairsStepForwardTest) == 28);
    assert!(offset_of!(JPH_ExtendedUpdateSettings, walkStairsCosAngleForwardContact) == 32);
    assert!(offset_of!(JPH_ExtendedUpdateSettings, walkStairsStepDownExtra) == 36);

    assert!(size_of::<JPH_Point>() == 8);
    assert!(align_of::<JPH_Point>() == 4);
    assert!(offset_of!(JPH_Point, x) == 0);
    assert!(offset_of!(JPH_Point, y) == 4);

    assert!(size_of::<JPH_SpringSettings>() == 12);
    assert!(align_of::<JPH_SpringSettings>() == 4);
    assert!(offset_of!(JPH_SpringSettings, mode) == 0);
    assert!(offset_of!(JPH_SpringSettings, frequencyOrStiffness) == 4);
    assert!(offset_of!(JPH_SpringSettings, damping) == 8);

    assert!(size_of::<JPH_MotorSettings>() == 28);
    assert!(align_of::<JPH_MotorSettings>() == 4);
    assert!(offset_of!(JPH_MotorSettings, springSettings) == 0);
    assert!(offset_of!(JPH_MotorSettings, minForceLimit) == 12);
    assert!(offset_of!(JPH_MotorSettings, maxForceLimit) == 16);
    assert!(offset_of!(JPH_MotorSettings, minTorqueLimit) == 20);
    assert!(offset_of!(JPH_MotorSettings, maxTorqueLimit) == 24);

    assert!(size_of::<JPH_VehicleAntiRollBar>() == 12);
    assert!(align_of::<JPH_VehicleAntiRollBar>() == 4);
    assert!(offset_of!(JPH_VehicleAntiRollBar, leftWheel) == 0);
    assert!(offset_of!(JPH_VehicleAntiRollBar, rightWheel) == 4);
    assert!(offset_of!(JPH_VehicleAntiRollBar, stiffness) == 8);

    assert!(size_of::<JPH_VehicleDifferentialSettings>() == 24);
    assert!(align_of::<JPH_VehicleDifferentialSettings>() == 4);
    assert!(offset_of!(JPH_VehicleDifferentialSettings, leftWheel) == 0);
    assert!(offset_of!(JPH_VehicleDifferentialSettings, rightWheel) == 4);
    assert!(offset_of!(JPH_VehicleDifferentialSettings, differentialRatio) == 8);
    assert!(offset_of!(JPH_VehicleDifferentialSettings, leftRightSplit) == 12);
    assert!(offset_of!(JPH_VehicleDifferentialSettings, limitedSlipRatio) == 16);
    assert!(offset_of!(JPH_VehicleDifferentialSettings, engineTorqueRatio) == 20);

    assert!(size_of::<JPH_ConstraintSettings>() == 32);
    assert!(align_of::<JPH_ConstraintSettings>() == 8);
    assert!(offset_of!(JPH_ConstraintSettings, enabled) == 0);
    assert!(offset_of!(JPH_ConstraintSettings, constraintPriority) == 4);
    assert!(offset_of!(JPH_ConstraintSettings, numVelocityStepsOverride) == 8);
    assert!(offset_of!(JPH_ConstraintSettings, numPositionStepsOverride) == 12);
    assert!(offset_of!(JPH_ConstraintSettings, drawConstraintSize) == 16);
    assert!(offset_of!(JPH_ConstraintSettings, userData) == 24);

    assert!(size_of::<JPH_GearConstraintSettings>() == 64);
    assert!(align_of::<JPH_GearConstraintSettings>() == 8);
    assert!(offset_of!(JPH_GearConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_GearConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_GearConstraintSettings, hingeAxis1) == 36);
    assert!(offset_of!(JPH_GearConstraintSettings, hingeAxis2) == 48);
    assert!(offset_of!(JPH_GearConstraintSettings, ratio) == 60);

    assert!(size_of::<JPH_RackAndPinionConstraintSettings>() == 64);
    assert!(align_of::<JPH_RackAndPinionConstraintSettings>() == 8);
    assert!(offset_of!(JPH_RackAndPinionConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_RackAndPinionConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_RackAndPinionConstraintSettings, hingeAxis) == 36);
    assert!(offset_of!(JPH_RackAndPinionConstraintSettings, sliderAxis) == 48);
    assert!(offset_of!(JPH_RackAndPinionConstraintSettings, ratio) == 60);

    assert!(size_of::<JPH_PhysicsSettings>() == 84);
    assert!(align_of::<JPH_PhysicsSettings>() == 4);
    assert!(offset_of!(JPH_PhysicsSettings, maxInFlightBodyPairs) == 0);
    assert!(offset_of!(JPH_PhysicsSettings, stepListenersBatchSize) == 4);
    assert!(offset_of!(JPH_PhysicsSettings, stepListenerBatchesPerJob) == 8);
    assert!(offset_of!(JPH_PhysicsSettings, baumgarte) == 12);
    assert!(offset_of!(JPH_PhysicsSettings, speculativeContactDistance) == 16);
    assert!(offset_of!(JPH_PhysicsSettings, penetrationSlop) == 20);
    assert!(offset_of!(JPH_PhysicsSettings, linearCastThreshold) == 24);
    assert!(offset_of!(JPH_PhysicsSettings, linearCastMaxPenetration) == 28);
    assert!(offset_of!(JPH_PhysicsSettings, manifoldTolerance) == 32);
    assert!(offset_of!(JPH_PhysicsSettings, maxPenetrationDistance) == 36);
    assert!(offset_of!(JPH_PhysicsSettings, bodyPairCacheMaxDeltaPositionSq) == 40);
    assert!(offset_of!(JPH_PhysicsSettings, bodyPairCacheCosMaxDeltaRotationDiv2) == 44);
    assert!(offset_of!(JPH_PhysicsSettings, contactNormalCosMaxDeltaRotation) == 48);
    assert!(offset_of!(JPH_PhysicsSettings, contactPointPreserveLambdaMaxDistSq) == 52);
    assert!(offset_of!(JPH_PhysicsSettings, numVelocitySteps) == 56);
    assert!(offset_of!(JPH_PhysicsSettings, numPositionSteps) == 60);
    assert!(offset_of!(JPH_PhysicsSettings, minVelocityForRestitution) == 64);
    assert!(offset_of!(JPH_PhysicsSettings, timeBeforeSleep) == 68);
    assert!(offset_of!(JPH_PhysicsSettings, pointVelocitySleepThreshold) == 72);
    assert!(offset_of!(JPH_PhysicsSettings, deterministicSimulation) == 76);
    assert!(offset_of!(JPH_PhysicsSettings, constraintWarmStart) == 77);
    assert!(offset_of!(JPH_PhysicsSettings, useBodyPairContactCache) == 78);
    assert!(offset_of!(JPH_PhysicsSettings, useManifoldReduction) == 79);
    assert!(offset_of!(JPH_PhysicsSettings, useLargeIslandSplitter) == 80);
    assert!(offset_of!(JPH_PhysicsSettings, allowSleeping) == 81);
    assert!(offset_of!(JPH_PhysicsSettings, checkActiveEdges) == 82);

    assert!(size_of::<JobSystemThreadPoolConfig>() == 12);
    assert!(align_of::<JobSystemThreadPoolConfig>() == 4);
    assert!(offset_of!(JobSystemThreadPoolConfig, numThreads) == 8);

    assert!(size_of::<JPH_BodyID>() == 4);
    assert!(size_of::<JPH_SubShapeID>() == 4);
    assert!(size_of::<JPH_ObjectLayer>() == 4);
    assert!(size_of::<JPH_BroadPhaseLayer>() == 1);

    assert!(size_of::<JPH_MotionType>() == 4);
    assert!(size_of::<JPH_Activation>() == 4);
    assert!(size_of::<JPH_PhysicsUpdateError>() == 4);
    assert!(size_of::<JPH_MotionQuality>() == 4);
    assert!(size_of::<JPH_OverrideMassProperties>() == 4);
    assert!(size_of::<JPH_ShapeSubType>() == 4);
    assert!(size_of::<JPH_BackFaceMode>() == 4);
    assert!(size_of::<JPH_ActiveEdgeMode>() == 4);
    assert!(size_of::<JPH_CollectFacesMode>() == 4);
    assert!(size_of::<JPH_CollisionCollectorType>() == 4);
    assert!(size_of::<JPH_GroundState>() == 4);
    assert!(size_of::<JPH_SpringMode>() == 4);
    assert!(size_of::<JPH_TransmissionMode>() == 4);
    assert!(size_of::<JPH_ConstraintSubType>() == 4);
    assert!(size_of::<JPH_MotorState>() == 4);
    assert!(size_of::<JPH_SwingType>() == 4);
    assert!(size_of::<JPH_ConstraintSpace>() == 4);
    assert!(size_of::<JPH_SixDOFConstraintAxis>() == 4);
    assert!(size_of::<JPH_PathRotationConstraintType>() == 4);
    assert!(size_of::<JPH_SoftBodyValidateResult>() == 4);
    assert!(size_of::<JPH_AllowedDOFs>() == 4);
};

#[cfg(feature = "double-precision")]
const _: () = {
    assert!(size_of::<JPH_RVec3>() == 24);
    assert!(align_of::<JPH_RVec3>() == 8);
    assert!(offset_of!(JPH_RVec3, z) == 16);

    assert!(size_of::<JPH_RMat4>() == 72);
    assert!(align_of::<JPH_RMat4>() == 8);
    assert!(offset_of!(JPH_RMat4, column3) == 48);

    assert!(size_of::<JPH_HingeConstraintSettings>() == 192);
    assert!(align_of::<JPH_HingeConstraintSettings>() == 8);
    assert!(offset_of!(JPH_HingeConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_HingeConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_HingeConstraintSettings, point1) == 40);
    assert!(offset_of!(JPH_HingeConstraintSettings, hingeAxis1) == 64);
    assert!(offset_of!(JPH_HingeConstraintSettings, normalAxis1) == 76);
    assert!(offset_of!(JPH_HingeConstraintSettings, point2) == 88);
    assert!(offset_of!(JPH_HingeConstraintSettings, hingeAxis2) == 112);
    assert!(offset_of!(JPH_HingeConstraintSettings, normalAxis2) == 124);
    assert!(offset_of!(JPH_HingeConstraintSettings, limitsMin) == 136);
    assert!(offset_of!(JPH_HingeConstraintSettings, limitsMax) == 140);
    assert!(offset_of!(JPH_HingeConstraintSettings, limitsSpringSettings) == 144);
    assert!(offset_of!(JPH_HingeConstraintSettings, maxFrictionTorque) == 156);
    assert!(offset_of!(JPH_HingeConstraintSettings, motorSettings) == 160);

    assert!(size_of::<JPH_SwingTwistConstraintSettings>() == 216);
    assert!(align_of::<JPH_SwingTwistConstraintSettings>() == 8);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, position1) == 40);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, twistAxis1) == 64);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, planeAxis1) == 76);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, position2) == 88);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, twistAxis2) == 112);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, planeAxis2) == 124);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, swingType) == 136);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, normalHalfConeAngle) == 140);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, planeHalfConeAngle) == 144);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, twistMinAngle) == 148);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, twistMaxAngle) == 152);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, maxFrictionTorque) == 156);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, swingMotorSettings) == 160);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, twistMotorSettings) == 188);

    assert!(size_of::<JPH_SixDOFConstraintSettings>() == 416);
    assert!(align_of::<JPH_SixDOFConstraintSettings>() == 8);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, position1) == 40);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, axisX1) == 64);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, axisY1) == 76);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, position2) == 88);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, axisX2) == 112);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, axisY2) == 124);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, maxFriction) == 136);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, swingType) == 160);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, limitMin) == 164);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, limitMax) == 188);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, limitsSpringSettings) == 212);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, motorSettings) == 248);

    assert!(size_of::<JPH_FixedConstraintSettings>() == 136);
    assert!(align_of::<JPH_FixedConstraintSettings>() == 8);
    assert!(offset_of!(JPH_FixedConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_FixedConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_FixedConstraintSettings, autoDetectPoint) == 36);
    assert!(offset_of!(JPH_FixedConstraintSettings, point1) == 40);
    assert!(offset_of!(JPH_FixedConstraintSettings, axisX1) == 64);
    assert!(offset_of!(JPH_FixedConstraintSettings, axisY1) == 76);
    assert!(offset_of!(JPH_FixedConstraintSettings, point2) == 88);
    assert!(offset_of!(JPH_FixedConstraintSettings, axisX2) == 112);
    assert!(offset_of!(JPH_FixedConstraintSettings, axisY2) == 124);

    assert!(size_of::<JPH_PointConstraintSettings>() == 88);
    assert!(align_of::<JPH_PointConstraintSettings>() == 8);
    assert!(offset_of!(JPH_PointConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_PointConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_PointConstraintSettings, point1) == 40);
    assert!(offset_of!(JPH_PointConstraintSettings, point2) == 64);

    assert!(size_of::<JPH_DistanceConstraintSettings>() == 112);
    assert!(align_of::<JPH_DistanceConstraintSettings>() == 8);
    assert!(offset_of!(JPH_DistanceConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_DistanceConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_DistanceConstraintSettings, point1) == 40);
    assert!(offset_of!(JPH_DistanceConstraintSettings, point2) == 64);
    assert!(offset_of!(JPH_DistanceConstraintSettings, minDistance) == 88);
    assert!(offset_of!(JPH_DistanceConstraintSettings, maxDistance) == 92);
    assert!(offset_of!(JPH_DistanceConstraintSettings, limitsSpringSettings) == 96);

    assert!(size_of::<JPH_SliderConstraintSettings>() == 192);
    assert!(align_of::<JPH_SliderConstraintSettings>() == 8);
    assert!(offset_of!(JPH_SliderConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_SliderConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_SliderConstraintSettings, autoDetectPoint) == 36);
    assert!(offset_of!(JPH_SliderConstraintSettings, point1) == 40);
    assert!(offset_of!(JPH_SliderConstraintSettings, sliderAxis1) == 64);
    assert!(offset_of!(JPH_SliderConstraintSettings, normalAxis1) == 76);
    assert!(offset_of!(JPH_SliderConstraintSettings, point2) == 88);
    assert!(offset_of!(JPH_SliderConstraintSettings, sliderAxis2) == 112);
    assert!(offset_of!(JPH_SliderConstraintSettings, normalAxis2) == 124);
    assert!(offset_of!(JPH_SliderConstraintSettings, limitsMin) == 136);
    assert!(offset_of!(JPH_SliderConstraintSettings, limitsMax) == 140);
    assert!(offset_of!(JPH_SliderConstraintSettings, limitsSpringSettings) == 144);
    assert!(offset_of!(JPH_SliderConstraintSettings, maxFrictionForce) == 156);
    assert!(offset_of!(JPH_SliderConstraintSettings, motorSettings) == 160);

    assert!(size_of::<JPH_ConeConstraintSettings>() == 120);
    assert!(align_of::<JPH_ConeConstraintSettings>() == 8);
    assert!(offset_of!(JPH_ConeConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_ConeConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_ConeConstraintSettings, point1) == 40);
    assert!(offset_of!(JPH_ConeConstraintSettings, twistAxis1) == 64);
    assert!(offset_of!(JPH_ConeConstraintSettings, point2) == 80);
    assert!(offset_of!(JPH_ConeConstraintSettings, twistAxis2) == 104);
    assert!(offset_of!(JPH_ConeConstraintSettings, halfConeAngle) == 116);

    assert!(size_of::<JPH_PulleyConstraintSettings>() == 152);
    assert!(align_of::<JPH_PulleyConstraintSettings>() == 8);
    assert!(offset_of!(JPH_PulleyConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_PulleyConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_PulleyConstraintSettings, bodyPoint1) == 40);
    assert!(offset_of!(JPH_PulleyConstraintSettings, fixedPoint1) == 64);
    assert!(offset_of!(JPH_PulleyConstraintSettings, bodyPoint2) == 88);
    assert!(offset_of!(JPH_PulleyConstraintSettings, fixedPoint2) == 112);
    assert!(offset_of!(JPH_PulleyConstraintSettings, ratio) == 136);
    assert!(offset_of!(JPH_PulleyConstraintSettings, minLength) == 140);
    assert!(offset_of!(JPH_PulleyConstraintSettings, maxLength) == 144);
};

#[cfg(not(feature = "double-precision"))]
const _: () = {
    assert!(size_of::<JPH_RVec3>() == 12);
    assert!(size_of::<JPH_RMat4>() == 64);

    assert!(size_of::<JPH_HingeConstraintSettings>() == 160);
    assert!(align_of::<JPH_HingeConstraintSettings>() == 8);
    assert!(offset_of!(JPH_HingeConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_HingeConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_HingeConstraintSettings, point1) == 36);
    assert!(offset_of!(JPH_HingeConstraintSettings, hingeAxis1) == 48);
    assert!(offset_of!(JPH_HingeConstraintSettings, normalAxis1) == 60);
    assert!(offset_of!(JPH_HingeConstraintSettings, point2) == 72);
    assert!(offset_of!(JPH_HingeConstraintSettings, hingeAxis2) == 84);
    assert!(offset_of!(JPH_HingeConstraintSettings, normalAxis2) == 96);
    assert!(offset_of!(JPH_HingeConstraintSettings, limitsMin) == 108);
    assert!(offset_of!(JPH_HingeConstraintSettings, limitsMax) == 112);
    assert!(offset_of!(JPH_HingeConstraintSettings, limitsSpringSettings) == 116);
    assert!(offset_of!(JPH_HingeConstraintSettings, maxFrictionTorque) == 128);
    assert!(offset_of!(JPH_HingeConstraintSettings, motorSettings) == 132);

    assert!(size_of::<JPH_SwingTwistConstraintSettings>() == 192);
    assert!(align_of::<JPH_SwingTwistConstraintSettings>() == 8);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, position1) == 36);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, twistAxis1) == 48);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, planeAxis1) == 60);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, position2) == 72);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, twistAxis2) == 84);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, planeAxis2) == 96);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, swingType) == 108);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, normalHalfConeAngle) == 112);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, planeHalfConeAngle) == 116);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, twistMinAngle) == 120);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, twistMaxAngle) == 124);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, maxFrictionTorque) == 128);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, swingMotorSettings) == 132);
    assert!(offset_of!(JPH_SwingTwistConstraintSettings, twistMotorSettings) == 160);

    assert!(size_of::<JPH_SixDOFConstraintSettings>() == 392);
    assert!(align_of::<JPH_SixDOFConstraintSettings>() == 8);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, position1) == 36);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, axisX1) == 48);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, axisY1) == 60);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, position2) == 72);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, axisX2) == 84);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, axisY2) == 96);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, maxFriction) == 108);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, swingType) == 132);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, limitMin) == 136);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, limitMax) == 160);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, limitsSpringSettings) == 184);
    assert!(offset_of!(JPH_SixDOFConstraintSettings, motorSettings) == 220);

    assert!(size_of::<JPH_FixedConstraintSettings>() == 112);
    assert!(align_of::<JPH_FixedConstraintSettings>() == 8);
    assert!(offset_of!(JPH_FixedConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_FixedConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_FixedConstraintSettings, autoDetectPoint) == 36);
    assert!(offset_of!(JPH_FixedConstraintSettings, point1) == 40);
    assert!(offset_of!(JPH_FixedConstraintSettings, axisX1) == 52);
    assert!(offset_of!(JPH_FixedConstraintSettings, axisY1) == 64);
    assert!(offset_of!(JPH_FixedConstraintSettings, point2) == 76);
    assert!(offset_of!(JPH_FixedConstraintSettings, axisX2) == 88);
    assert!(offset_of!(JPH_FixedConstraintSettings, axisY2) == 100);

    assert!(size_of::<JPH_PointConstraintSettings>() == 64);
    assert!(align_of::<JPH_PointConstraintSettings>() == 8);
    assert!(offset_of!(JPH_PointConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_PointConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_PointConstraintSettings, point1) == 36);
    assert!(offset_of!(JPH_PointConstraintSettings, point2) == 48);

    assert!(size_of::<JPH_DistanceConstraintSettings>() == 80);
    assert!(align_of::<JPH_DistanceConstraintSettings>() == 8);
    assert!(offset_of!(JPH_DistanceConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_DistanceConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_DistanceConstraintSettings, point1) == 36);
    assert!(offset_of!(JPH_DistanceConstraintSettings, point2) == 48);
    assert!(offset_of!(JPH_DistanceConstraintSettings, minDistance) == 60);
    assert!(offset_of!(JPH_DistanceConstraintSettings, maxDistance) == 64);
    assert!(offset_of!(JPH_DistanceConstraintSettings, limitsSpringSettings) == 68);

    assert!(size_of::<JPH_SliderConstraintSettings>() == 168);
    assert!(align_of::<JPH_SliderConstraintSettings>() == 8);
    assert!(offset_of!(JPH_SliderConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_SliderConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_SliderConstraintSettings, autoDetectPoint) == 36);
    assert!(offset_of!(JPH_SliderConstraintSettings, point1) == 40);
    assert!(offset_of!(JPH_SliderConstraintSettings, sliderAxis1) == 52);
    assert!(offset_of!(JPH_SliderConstraintSettings, normalAxis1) == 64);
    assert!(offset_of!(JPH_SliderConstraintSettings, point2) == 76);
    assert!(offset_of!(JPH_SliderConstraintSettings, sliderAxis2) == 88);
    assert!(offset_of!(JPH_SliderConstraintSettings, normalAxis2) == 100);
    assert!(offset_of!(JPH_SliderConstraintSettings, limitsMin) == 112);
    assert!(offset_of!(JPH_SliderConstraintSettings, limitsMax) == 116);
    assert!(offset_of!(JPH_SliderConstraintSettings, limitsSpringSettings) == 120);
    assert!(offset_of!(JPH_SliderConstraintSettings, maxFrictionForce) == 132);
    assert!(offset_of!(JPH_SliderConstraintSettings, motorSettings) == 136);

    assert!(size_of::<JPH_ConeConstraintSettings>() == 88);
    assert!(align_of::<JPH_ConeConstraintSettings>() == 8);
    assert!(offset_of!(JPH_ConeConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_ConeConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_ConeConstraintSettings, point1) == 36);
    assert!(offset_of!(JPH_ConeConstraintSettings, twistAxis1) == 48);
    assert!(offset_of!(JPH_ConeConstraintSettings, point2) == 60);
    assert!(offset_of!(JPH_ConeConstraintSettings, twistAxis2) == 72);
    assert!(offset_of!(JPH_ConeConstraintSettings, halfConeAngle) == 84);

    assert!(size_of::<JPH_PulleyConstraintSettings>() == 96);
    assert!(align_of::<JPH_PulleyConstraintSettings>() == 8);
    assert!(offset_of!(JPH_PulleyConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_PulleyConstraintSettings, space) == 32);
    assert!(offset_of!(JPH_PulleyConstraintSettings, bodyPoint1) == 36);
    assert!(offset_of!(JPH_PulleyConstraintSettings, fixedPoint1) == 48);
    assert!(offset_of!(JPH_PulleyConstraintSettings, bodyPoint2) == 60);
    assert!(offset_of!(JPH_PulleyConstraintSettings, fixedPoint2) == 72);
    assert!(offset_of!(JPH_PulleyConstraintSettings, ratio) == 84);
    assert!(offset_of!(JPH_PulleyConstraintSettings, minLength) == 88);
    assert!(offset_of!(JPH_PulleyConstraintSettings, maxLength) == 92);
};

#[cfg(target_pointer_width = "64")]
const _: () = {
    assert!(size_of::<JPH_CollisionEstimationResult>() == 112);
    assert!(align_of::<JPH_CollisionEstimationResult>() == 8);
    assert!(offset_of!(JPH_CollisionEstimationResult, linearVelocity1) == 0);
    assert!(offset_of!(JPH_CollisionEstimationResult, angularVelocity1) == 12);
    assert!(offset_of!(JPH_CollisionEstimationResult, linearVelocity2) == 24);
    assert!(offset_of!(JPH_CollisionEstimationResult, angularVelocity2) == 36);
    assert!(offset_of!(JPH_CollisionEstimationResult, frictionPoint) == 48);
    assert!(offset_of!(JPH_CollisionEstimationResult, tangent1) == 60);
    assert!(offset_of!(JPH_CollisionEstimationResult, tangent2) == 72);
    assert!(offset_of!(JPH_CollisionEstimationResult, frictionImpulse1) == 84);
    assert!(offset_of!(JPH_CollisionEstimationResult, frictionImpulse2) == 88);
    assert!(offset_of!(JPH_CollisionEstimationResult, angularFrictionImpulse) == 92);
    assert!(offset_of!(JPH_CollisionEstimationResult, contactImpulseCount) == 96);
    assert!(offset_of!(JPH_CollisionEstimationResult, contactImpulses) == 104);

    assert!(size_of::<JPH_PhysicsSystemSettings>() == 48);
    assert!(align_of::<JPH_PhysicsSystemSettings>() == 8);
    assert!(offset_of!(JPH_PhysicsSystemSettings, maxContactConstraints) == 12);
    assert!(offset_of!(JPH_PhysicsSystemSettings, broadPhaseLayerInterface) == 24);
    assert!(offset_of!(JPH_PhysicsSystemSettings, objectLayerPairFilter) == 32);
    assert!(offset_of!(JPH_PhysicsSystemSettings, objectVsBroadPhaseLayerFilter) == 40);

    assert!(size_of::<JPH_JobSystemConfig>() == 32);
    assert!(align_of::<JPH_JobSystemConfig>() == 8);
    assert!(offset_of!(JPH_JobSystemConfig, context) == 0);
    assert!(offset_of!(JPH_JobSystemConfig, queueJob) == 8);
    assert!(offset_of!(JPH_JobSystemConfig, queueJobs) == 16);
    assert!(offset_of!(JPH_JobSystemConfig, maxConcurrency) == 24);
    assert!(offset_of!(JPH_JobSystemConfig, maxBarriers) == 28);

    assert!(size_of::<JPH_CollideShapeResult>() == 80);
    assert!(align_of::<JPH_CollideShapeResult>() == 8);
    assert!(offset_of!(JPH_CollideShapeResult, contactPointOn1) == 0);
    assert!(offset_of!(JPH_CollideShapeResult, contactPointOn2) == 12);
    assert!(offset_of!(JPH_CollideShapeResult, penetrationAxis) == 24);
    assert!(offset_of!(JPH_CollideShapeResult, penetrationDepth) == 36);
    assert!(offset_of!(JPH_CollideShapeResult, subShapeID1) == 40);
    assert!(offset_of!(JPH_CollideShapeResult, subShapeID2) == 44);
    assert!(offset_of!(JPH_CollideShapeResult, bodyID2) == 48);
    assert!(offset_of!(JPH_CollideShapeResult, shape1FaceCount) == 52);
    assert!(offset_of!(JPH_CollideShapeResult, shape1Faces) == 56);
    assert!(offset_of!(JPH_CollideShapeResult, shape2FaceCount) == 64);
    assert!(offset_of!(JPH_CollideShapeResult, shape2Faces) == 72);

    assert!(size_of::<JPH_ObjectLayerFilter_Procs>() == 8);
    assert!(align_of::<JPH_ObjectLayerFilter_Procs>() == 8);
    assert!(offset_of!(JPH_ObjectLayerFilter_Procs, ShouldCollide) == 0);

    assert!(size_of::<JPH_BodyFilter_Procs>() == 16);
    assert!(align_of::<JPH_BodyFilter_Procs>() == 8);
    assert!(offset_of!(JPH_BodyFilter_Procs, ShouldCollide) == 0);
    assert!(offset_of!(JPH_BodyFilter_Procs, ShouldCollideLocked) == 8);

    assert!(size_of::<JPH_ShapeFilter_Procs>() == 16);
    assert!(align_of::<JPH_ShapeFilter_Procs>() == 8);
    assert!(offset_of!(JPH_ShapeFilter_Procs, ShouldCollide) == 0);
    assert!(offset_of!(JPH_ShapeFilter_Procs, ShouldCollide2) == 8);

    assert!(size_of::<JPH_ContactListener_Procs>() == 32);
    assert!(align_of::<JPH_ContactListener_Procs>() == 8);
    assert!(offset_of!(JPH_ContactListener_Procs, OnContactValidate) == 0);
    assert!(offset_of!(JPH_ContactListener_Procs, OnContactAdded) == 8);
    assert!(offset_of!(JPH_ContactListener_Procs, OnContactPersisted) == 16);
    assert!(offset_of!(JPH_ContactListener_Procs, OnContactRemoved) == 24);

    assert!(size_of::<JPH_BodyActivationListener_Procs>() == 16);
    assert!(align_of::<JPH_BodyActivationListener_Procs>() == 8);
    assert!(offset_of!(JPH_BodyActivationListener_Procs, OnBodyActivated) == 0);
    assert!(offset_of!(JPH_BodyActivationListener_Procs, OnBodyDeactivated) == 8);

    assert!(size_of::<JPH_SoftBodyContactListener_Procs>() == 16);
    assert!(align_of::<JPH_SoftBodyContactListener_Procs>() == 8);
    assert!(offset_of!(JPH_SoftBodyContactListener_Procs, OnSoftBodyContactValidate) == 0);
    assert!(offset_of!(JPH_SoftBodyContactListener_Procs, OnSoftBodyContactAdded) == 8);

    assert!(size_of::<JPH_CharacterBaseSettings>() == 48);
    assert!(align_of::<JPH_CharacterBaseSettings>() == 8);
    assert!(offset_of!(JPH_CharacterBaseSettings, up) == 0);
    assert!(offset_of!(JPH_CharacterBaseSettings, supportingVolume) == 12);
    assert!(offset_of!(JPH_CharacterBaseSettings, maxSlopeAngle) == 28);
    assert!(offset_of!(JPH_CharacterBaseSettings, enhancedInternalEdgeRemoval) == 32);
    assert!(offset_of!(JPH_CharacterBaseSettings, shape) == 40);

    assert!(size_of::<JPH_CharacterVirtualSettings>() == 128);
    assert!(align_of::<JPH_CharacterVirtualSettings>() == 8);
    assert!(offset_of!(JPH_CharacterVirtualSettings, base) == 0);
    assert!(offset_of!(JPH_CharacterVirtualSettings, ID) == 48);
    assert!(offset_of!(JPH_CharacterVirtualSettings, mass) == 52);
    assert!(offset_of!(JPH_CharacterVirtualSettings, maxStrength) == 56);
    assert!(offset_of!(JPH_CharacterVirtualSettings, shapeOffset) == 60);
    assert!(offset_of!(JPH_CharacterVirtualSettings, backFaceMode) == 72);
    assert!(offset_of!(JPH_CharacterVirtualSettings, predictiveContactDistance) == 76);
    assert!(offset_of!(JPH_CharacterVirtualSettings, maxCollisionIterations) == 80);
    assert!(offset_of!(JPH_CharacterVirtualSettings, maxConstraintIterations) == 84);
    assert!(offset_of!(JPH_CharacterVirtualSettings, minTimeRemaining) == 88);
    assert!(offset_of!(JPH_CharacterVirtualSettings, collisionTolerance) == 92);
    assert!(offset_of!(JPH_CharacterVirtualSettings, characterPadding) == 96);
    assert!(offset_of!(JPH_CharacterVirtualSettings, maxNumHits) == 100);
    assert!(offset_of!(JPH_CharacterVirtualSettings, hitReductionCosMaxAngle) == 104);
    assert!(offset_of!(JPH_CharacterVirtualSettings, penetrationRecoverySpeed) == 108);
    assert!(offset_of!(JPH_CharacterVirtualSettings, innerBodyShape) == 112);
    assert!(offset_of!(JPH_CharacterVirtualSettings, innerBodyIDOverride) == 120);
    assert!(offset_of!(JPH_CharacterVirtualSettings, innerBodyLayer) == 124);

    assert!(size_of::<JPH_VehicleConstraintSettings>() == 96);
    assert!(align_of::<JPH_VehicleConstraintSettings>() == 8);
    assert!(offset_of!(JPH_VehicleConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_VehicleConstraintSettings, up) == 32);
    assert!(offset_of!(JPH_VehicleConstraintSettings, forward) == 44);
    assert!(offset_of!(JPH_VehicleConstraintSettings, maxPitchRollAngle) == 56);
    assert!(offset_of!(JPH_VehicleConstraintSettings, wheelsCount) == 60);
    assert!(offset_of!(JPH_VehicleConstraintSettings, wheels) == 64);
    assert!(offset_of!(JPH_VehicleConstraintSettings, antiRollBarsCount) == 72);
    assert!(offset_of!(JPH_VehicleConstraintSettings, antiRollBars) == 80);
    assert!(offset_of!(JPH_VehicleConstraintSettings, controller) == 88);

    assert!(size_of::<JPH_CollisionGroup>() == 16);
    assert!(align_of::<JPH_CollisionGroup>() == 8);
    assert!(offset_of!(JPH_CollisionGroup, groupFilter) == 0);
    assert!(offset_of!(JPH_CollisionGroup, groupID) == 8);
    assert!(offset_of!(JPH_CollisionGroup, subGroupID) == 12);

    assert!(size_of::<JPH_VehicleEngineSettings>() == 32);
    assert!(align_of::<JPH_VehicleEngineSettings>() == 8);
    assert!(offset_of!(JPH_VehicleEngineSettings, maxTorque) == 0);
    assert!(offset_of!(JPH_VehicleEngineSettings, minRPM) == 4);
    assert!(offset_of!(JPH_VehicleEngineSettings, maxRPM) == 8);
    assert!(offset_of!(JPH_VehicleEngineSettings, normalizedTorque) == 16);
    assert!(offset_of!(JPH_VehicleEngineSettings, inertia) == 24);
    assert!(offset_of!(JPH_VehicleEngineSettings, angularDamping) == 28);

    assert!(size_of::<JPH_VehicleTrackSettings>() == 40);
    assert!(align_of::<JPH_VehicleTrackSettings>() == 8);
    assert!(offset_of!(JPH_VehicleTrackSettings, drivenWheel) == 0);
    assert!(offset_of!(JPH_VehicleTrackSettings, wheels) == 8);
    assert!(offset_of!(JPH_VehicleTrackSettings, wheelsCount) == 16);
    assert!(offset_of!(JPH_VehicleTrackSettings, inertia) == 20);
    assert!(offset_of!(JPH_VehicleTrackSettings, angularDamping) == 24);
    assert!(offset_of!(JPH_VehicleTrackSettings, maxBrakeTorque) == 28);
    assert!(offset_of!(JPH_VehicleTrackSettings, differentialRatio) == 32);

    assert!(size_of::<JPH_CollisionGroup>() == 16);
    assert!(align_of::<JPH_CollisionGroup>() == 8);
    assert!(offset_of!(JPH_CollisionGroup, groupFilter) == 0);
    assert!(offset_of!(JPH_CollisionGroup, groupID) == 8);
    assert!(offset_of!(JPH_CollisionGroup, subGroupID) == 12);

    assert!(align_of::<JPH_CharacterContact>() == 8);
    assert!(offset_of!(JPH_CharacterContact, hash) == 0);
    assert!(offset_of!(JPH_CharacterContact, bodyB) == 8);
    assert!(offset_of!(JPH_CharacterContact, characterIDB) == 12);
    assert!(offset_of!(JPH_CharacterContact, subShapeIDB) == 16);

    assert!(size_of::<JPH_PathConstraintSettings>() == 112);
    assert!(align_of::<JPH_PathConstraintSettings>() == 8);
    assert!(offset_of!(JPH_PathConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_PathConstraintSettings, path) == 32);
    assert!(offset_of!(JPH_PathConstraintSettings, pathPosition) == 40);
    assert!(offset_of!(JPH_PathConstraintSettings, pathRotation) == 52);
    assert!(offset_of!(JPH_PathConstraintSettings, pathFraction) == 68);
    assert!(offset_of!(JPH_PathConstraintSettings, maxFrictionForce) == 72);
    assert!(offset_of!(JPH_PathConstraintSettings, rotationConstraintType) == 76);
    assert!(offset_of!(JPH_PathConstraintSettings, positionMotorSettings) == 80);
};

// The `JPH_CharacterContact` fields from its `JPH_RVec3` position on, whose size depends on
// the precision.
#[cfg(all(target_pointer_width = "64", feature = "double-precision"))]
const _: () = {
    assert!(size_of::<JPH_CharacterContact>() == 136);
    assert!(offset_of!(JPH_CharacterContact, position) == 24);
    assert!(offset_of!(JPH_CharacterContact, linearVelocity) == 48);
    assert!(offset_of!(JPH_CharacterContact, contactNormal) == 60);
    assert!(offset_of!(JPH_CharacterContact, surfaceNormal) == 72);
    assert!(offset_of!(JPH_CharacterContact, distance) == 84);
    assert!(offset_of!(JPH_CharacterContact, fraction) == 88);
    assert!(offset_of!(JPH_CharacterContact, motionTypeB) == 92);
    assert!(offset_of!(JPH_CharacterContact, isSensorB) == 96);
    assert!(offset_of!(JPH_CharacterContact, characterB) == 104);
    assert!(offset_of!(JPH_CharacterContact, userData) == 112);
    assert!(offset_of!(JPH_CharacterContact, material) == 120);
    assert!(offset_of!(JPH_CharacterContact, hadCollision) == 128);
    assert!(offset_of!(JPH_CharacterContact, wasDiscarded) == 129);
    assert!(offset_of!(JPH_CharacterContact, canPushCharacter) == 130);
    assert!(offset_of!(JPH_CharacterContact, isBackFacingContact) == 131);
};

#[cfg(all(target_pointer_width = "64", not(feature = "double-precision")))]
const _: () = {
    assert!(size_of::<JPH_CharacterContact>() == 120);
    assert!(offset_of!(JPH_CharacterContact, position) == 20);
    assert!(offset_of!(JPH_CharacterContact, linearVelocity) == 32);
    assert!(offset_of!(JPH_CharacterContact, contactNormal) == 44);
    assert!(offset_of!(JPH_CharacterContact, surfaceNormal) == 56);
    assert!(offset_of!(JPH_CharacterContact, distance) == 68);
    assert!(offset_of!(JPH_CharacterContact, fraction) == 72);
    assert!(offset_of!(JPH_CharacterContact, motionTypeB) == 76);
    assert!(offset_of!(JPH_CharacterContact, isSensorB) == 80);
    assert!(offset_of!(JPH_CharacterContact, characterB) == 88);
    assert!(offset_of!(JPH_CharacterContact, userData) == 96);
    assert!(offset_of!(JPH_CharacterContact, material) == 104);
    assert!(offset_of!(JPH_CharacterContact, hadCollision) == 112);
    assert!(offset_of!(JPH_CharacterContact, wasDiscarded) == 113);
    assert!(offset_of!(JPH_CharacterContact, canPushCharacter) == 114);
    assert!(offset_of!(JPH_CharacterContact, isBackFacingContact) == 115);
};

#[cfg(feature = "debug-renderer")]
const _: () = {
    assert!(size_of::<JPH_DebugRenderer_Procs>() == 24);
    assert!(align_of::<JPH_DebugRenderer_Procs>() == 8);
    assert!(offset_of!(JPH_DebugRenderer_Procs, DrawLine) == 0);
    assert!(offset_of!(JPH_DebugRenderer_Procs, DrawTriangle) == 8);
    assert!(offset_of!(JPH_DebugRenderer_Procs, DrawText3D) == 16);
};
