//! The 32-bit numbers of the pointer-dependent structs; mirror of `native/layout_checks_ptr32.h`.

use core::mem::{align_of, offset_of, size_of};

use crate::*;

const _: () = {
    assert!(size_of::<JPH_CollisionEstimationResult>() == 104);
    assert!(align_of::<JPH_CollisionEstimationResult>() == 4);
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
    assert!(offset_of!(JPH_CollisionEstimationResult, contactImpulses) == 100);

    assert!(size_of::<JPH_PhysicsSystemSettings>() == 32);
    assert!(align_of::<JPH_PhysicsSystemSettings>() == 4);
    assert!(offset_of!(JPH_PhysicsSystemSettings, maxContactConstraints) == 12);
    assert!(offset_of!(JPH_PhysicsSystemSettings, broadPhaseLayerInterface) == 20);
    assert!(offset_of!(JPH_PhysicsSystemSettings, objectLayerPairFilter) == 24);
    assert!(offset_of!(JPH_PhysicsSystemSettings, objectVsBroadPhaseLayerFilter) == 28);

    assert!(size_of::<JPH_JobSystemConfig>() == 20);
    assert!(align_of::<JPH_JobSystemConfig>() == 4);
    assert!(offset_of!(JPH_JobSystemConfig, context) == 0);
    assert!(offset_of!(JPH_JobSystemConfig, queueJob) == 4);
    assert!(offset_of!(JPH_JobSystemConfig, queueJobs) == 8);
    assert!(offset_of!(JPH_JobSystemConfig, maxConcurrency) == 12);
    assert!(offset_of!(JPH_JobSystemConfig, maxBarriers) == 16);

    assert!(size_of::<JPH_CollideShapeResult>() == 68);
    assert!(align_of::<JPH_CollideShapeResult>() == 4);
    assert!(offset_of!(JPH_CollideShapeResult, contactPointOn1) == 0);
    assert!(offset_of!(JPH_CollideShapeResult, contactPointOn2) == 12);
    assert!(offset_of!(JPH_CollideShapeResult, penetrationAxis) == 24);
    assert!(offset_of!(JPH_CollideShapeResult, penetrationDepth) == 36);
    assert!(offset_of!(JPH_CollideShapeResult, subShapeID1) == 40);
    assert!(offset_of!(JPH_CollideShapeResult, subShapeID2) == 44);
    assert!(offset_of!(JPH_CollideShapeResult, bodyID2) == 48);
    assert!(offset_of!(JPH_CollideShapeResult, shape1FaceCount) == 52);
    assert!(offset_of!(JPH_CollideShapeResult, shape1Faces) == 56);
    assert!(offset_of!(JPH_CollideShapeResult, shape2FaceCount) == 60);
    assert!(offset_of!(JPH_CollideShapeResult, shape2Faces) == 64);

    assert!(size_of::<JPH_ObjectLayerFilter_Procs>() == 4);
    assert!(align_of::<JPH_ObjectLayerFilter_Procs>() == 4);
    assert!(offset_of!(JPH_ObjectLayerFilter_Procs, ShouldCollide) == 0);

    assert!(size_of::<JPH_BodyFilter_Procs>() == 8);
    assert!(align_of::<JPH_BodyFilter_Procs>() == 4);
    assert!(offset_of!(JPH_BodyFilter_Procs, ShouldCollide) == 0);
    assert!(offset_of!(JPH_BodyFilter_Procs, ShouldCollideLocked) == 4);

    assert!(size_of::<JPH_ShapeFilter_Procs>() == 8);
    assert!(align_of::<JPH_ShapeFilter_Procs>() == 4);
    assert!(offset_of!(JPH_ShapeFilter_Procs, ShouldCollide) == 0);
    assert!(offset_of!(JPH_ShapeFilter_Procs, ShouldCollide2) == 4);

    assert!(size_of::<JPH_ContactListener_Procs>() == 16);
    assert!(align_of::<JPH_ContactListener_Procs>() == 4);
    assert!(offset_of!(JPH_ContactListener_Procs, OnContactValidate) == 0);
    assert!(offset_of!(JPH_ContactListener_Procs, OnContactAdded) == 4);
    assert!(offset_of!(JPH_ContactListener_Procs, OnContactPersisted) == 8);
    assert!(offset_of!(JPH_ContactListener_Procs, OnContactRemoved) == 12);

    assert!(size_of::<JPH_BodyActivationListener_Procs>() == 8);
    assert!(align_of::<JPH_BodyActivationListener_Procs>() == 4);
    assert!(offset_of!(JPH_BodyActivationListener_Procs, OnBodyActivated) == 0);
    assert!(offset_of!(JPH_BodyActivationListener_Procs, OnBodyDeactivated) == 4);

    assert!(size_of::<JPH_SoftBodyContactListener_Procs>() == 8);
    assert!(align_of::<JPH_SoftBodyContactListener_Procs>() == 4);
    assert!(offset_of!(JPH_SoftBodyContactListener_Procs, OnSoftBodyContactValidate) == 0);
    assert!(offset_of!(JPH_SoftBodyContactListener_Procs, OnSoftBodyContactAdded) == 4);

    assert!(size_of::<JPH_CharacterBaseSettings>() == 40);
    assert!(align_of::<JPH_CharacterBaseSettings>() == 4);
    assert!(offset_of!(JPH_CharacterBaseSettings, up) == 0);
    assert!(offset_of!(JPH_CharacterBaseSettings, supportingVolume) == 12);
    assert!(offset_of!(JPH_CharacterBaseSettings, maxSlopeAngle) == 28);
    assert!(offset_of!(JPH_CharacterBaseSettings, enhancedInternalEdgeRemoval) == 32);
    assert!(offset_of!(JPH_CharacterBaseSettings, shape) == 36);

    assert!(size_of::<JPH_CharacterVirtualSettings>() == 116);
    assert!(align_of::<JPH_CharacterVirtualSettings>() == 4);
    assert!(offset_of!(JPH_CharacterVirtualSettings, base) == 0);
    assert!(offset_of!(JPH_CharacterVirtualSettings, ID) == 40);
    assert!(offset_of!(JPH_CharacterVirtualSettings, mass) == 44);
    assert!(offset_of!(JPH_CharacterVirtualSettings, maxStrength) == 48);
    assert!(offset_of!(JPH_CharacterVirtualSettings, shapeOffset) == 52);
    assert!(offset_of!(JPH_CharacterVirtualSettings, backFaceMode) == 64);
    assert!(offset_of!(JPH_CharacterVirtualSettings, predictiveContactDistance) == 68);
    assert!(offset_of!(JPH_CharacterVirtualSettings, maxCollisionIterations) == 72);
    assert!(offset_of!(JPH_CharacterVirtualSettings, maxConstraintIterations) == 76);
    assert!(offset_of!(JPH_CharacterVirtualSettings, minTimeRemaining) == 80);
    assert!(offset_of!(JPH_CharacterVirtualSettings, collisionTolerance) == 84);
    assert!(offset_of!(JPH_CharacterVirtualSettings, characterPadding) == 88);
    assert!(offset_of!(JPH_CharacterVirtualSettings, maxNumHits) == 92);
    assert!(offset_of!(JPH_CharacterVirtualSettings, hitReductionCosMaxAngle) == 96);
    assert!(offset_of!(JPH_CharacterVirtualSettings, penetrationRecoverySpeed) == 100);
    assert!(offset_of!(JPH_CharacterVirtualSettings, innerBodyShape) == 104);
    assert!(offset_of!(JPH_CharacterVirtualSettings, innerBodyIDOverride) == 108);
    assert!(offset_of!(JPH_CharacterVirtualSettings, innerBodyLayer) == 112);

    assert!(size_of::<JPH_VehicleConstraintSettings>() == 80);
    assert!(align_of::<JPH_VehicleConstraintSettings>() == 8);
    assert!(offset_of!(JPH_VehicleConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_VehicleConstraintSettings, up) == 32);
    assert!(offset_of!(JPH_VehicleConstraintSettings, forward) == 44);
    assert!(offset_of!(JPH_VehicleConstraintSettings, maxPitchRollAngle) == 56);
    assert!(offset_of!(JPH_VehicleConstraintSettings, wheelsCount) == 60);
    assert!(offset_of!(JPH_VehicleConstraintSettings, wheels) == 64);
    assert!(offset_of!(JPH_VehicleConstraintSettings, antiRollBarsCount) == 68);
    assert!(offset_of!(JPH_VehicleConstraintSettings, antiRollBars) == 72);
    assert!(offset_of!(JPH_VehicleConstraintSettings, controller) == 76);

    assert!(size_of::<JPH_CollisionGroup>() == 12);
    assert!(align_of::<JPH_CollisionGroup>() == 4);
    assert!(offset_of!(JPH_CollisionGroup, groupFilter) == 0);
    assert!(offset_of!(JPH_CollisionGroup, groupID) == 4);
    assert!(offset_of!(JPH_CollisionGroup, subGroupID) == 8);

    assert!(size_of::<JPH_VehicleEngineSettings>() == 24);
    assert!(align_of::<JPH_VehicleEngineSettings>() == 4);
    assert!(offset_of!(JPH_VehicleEngineSettings, maxTorque) == 0);
    assert!(offset_of!(JPH_VehicleEngineSettings, minRPM) == 4);
    assert!(offset_of!(JPH_VehicleEngineSettings, maxRPM) == 8);
    assert!(offset_of!(JPH_VehicleEngineSettings, normalizedTorque) == 12);
    assert!(offset_of!(JPH_VehicleEngineSettings, inertia) == 16);
    assert!(offset_of!(JPH_VehicleEngineSettings, angularDamping) == 20);

    assert!(size_of::<JPH_VehicleTrackSettings>() == 28);
    assert!(align_of::<JPH_VehicleTrackSettings>() == 4);
    assert!(offset_of!(JPH_VehicleTrackSettings, drivenWheel) == 0);
    assert!(offset_of!(JPH_VehicleTrackSettings, wheels) == 4);
    assert!(offset_of!(JPH_VehicleTrackSettings, wheelsCount) == 8);
    assert!(offset_of!(JPH_VehicleTrackSettings, inertia) == 12);
    assert!(offset_of!(JPH_VehicleTrackSettings, angularDamping) == 16);
    assert!(offset_of!(JPH_VehicleTrackSettings, maxBrakeTorque) == 20);
    assert!(offset_of!(JPH_VehicleTrackSettings, differentialRatio) == 24);

    assert!(align_of::<JPH_CharacterContact>() == 8);
    assert!(offset_of!(JPH_CharacterContact, hash) == 0);
    assert!(offset_of!(JPH_CharacterContact, bodyB) == 8);
    assert!(offset_of!(JPH_CharacterContact, characterIDB) == 12);
    assert!(offset_of!(JPH_CharacterContact, subShapeIDB) == 16);

    assert!(size_of::<JPH_PathConstraintSettings>() == 104);
    assert!(align_of::<JPH_PathConstraintSettings>() == 8);
    assert!(offset_of!(JPH_PathConstraintSettings, base) == 0);
    assert!(offset_of!(JPH_PathConstraintSettings, path) == 32);
    assert!(offset_of!(JPH_PathConstraintSettings, pathPosition) == 36);
    assert!(offset_of!(JPH_PathConstraintSettings, pathRotation) == 48);
    assert!(offset_of!(JPH_PathConstraintSettings, pathFraction) == 64);
    assert!(offset_of!(JPH_PathConstraintSettings, maxFrictionForce) == 68);
    assert!(offset_of!(JPH_PathConstraintSettings, rotationConstraintType) == 72);
    assert!(offset_of!(JPH_PathConstraintSettings, positionMotorSettings) == 76);
};

// The `JPH_CharacterContact` fields from its `JPH_RVec3` position on, whose size depends on
// the precision.
#[cfg(feature = "double-precision")]
const _: () = {
    assert!(size_of::<JPH_CharacterContact>() == 120);
    assert!(offset_of!(JPH_CharacterContact, position) == 24);
    assert!(offset_of!(JPH_CharacterContact, linearVelocity) == 48);
    assert!(offset_of!(JPH_CharacterContact, contactNormal) == 60);
    assert!(offset_of!(JPH_CharacterContact, surfaceNormal) == 72);
    assert!(offset_of!(JPH_CharacterContact, distance) == 84);
    assert!(offset_of!(JPH_CharacterContact, fraction) == 88);
    assert!(offset_of!(JPH_CharacterContact, motionTypeB) == 92);
    assert!(offset_of!(JPH_CharacterContact, isSensorB) == 96);
    assert!(offset_of!(JPH_CharacterContact, characterB) == 100);
    assert!(offset_of!(JPH_CharacterContact, userData) == 104);
    assert!(offset_of!(JPH_CharacterContact, material) == 112);
    assert!(offset_of!(JPH_CharacterContact, hadCollision) == 116);
    assert!(offset_of!(JPH_CharacterContact, wasDiscarded) == 117);
    assert!(offset_of!(JPH_CharacterContact, canPushCharacter) == 118);
    assert!(offset_of!(JPH_CharacterContact, isBackFacingContact) == 119);
};

#[cfg(not(feature = "double-precision"))]
const _: () = {
    assert!(size_of::<JPH_CharacterContact>() == 104);
    assert!(offset_of!(JPH_CharacterContact, position) == 20);
    assert!(offset_of!(JPH_CharacterContact, linearVelocity) == 32);
    assert!(offset_of!(JPH_CharacterContact, contactNormal) == 44);
    assert!(offset_of!(JPH_CharacterContact, surfaceNormal) == 56);
    assert!(offset_of!(JPH_CharacterContact, distance) == 68);
    assert!(offset_of!(JPH_CharacterContact, fraction) == 72);
    assert!(offset_of!(JPH_CharacterContact, motionTypeB) == 76);
    assert!(offset_of!(JPH_CharacterContact, isSensorB) == 80);
    assert!(offset_of!(JPH_CharacterContact, characterB) == 84);
    assert!(offset_of!(JPH_CharacterContact, userData) == 88);
    assert!(offset_of!(JPH_CharacterContact, material) == 96);
    assert!(offset_of!(JPH_CharacterContact, hadCollision) == 100);
    assert!(offset_of!(JPH_CharacterContact, wasDiscarded) == 101);
    assert!(offset_of!(JPH_CharacterContact, canPushCharacter) == 102);
    assert!(offset_of!(JPH_CharacterContact, isBackFacingContact) == 103);
};

#[cfg(feature = "debug-renderer")]
const _: () = {
    assert!(size_of::<JPH_DebugRenderer_Procs>() == 12);
    assert!(align_of::<JPH_DebugRenderer_Procs>() == 4);
    assert!(offset_of!(JPH_DebugRenderer_Procs, DrawLine) == 0);
    assert!(offset_of!(JPH_DebugRenderer_Procs, DrawTriangle) == 4);
    assert!(offset_of!(JPH_DebugRenderer_Procs, DrawText3D) == 8);
};
