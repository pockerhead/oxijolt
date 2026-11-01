// Included only by layout_checks.cpp inside its 32-bit branch; uses its macros and includes.
// Same numbers as src/layout/ptr32.rs.

#pragma once

// JPH_EstimateCollisionResponse fills JPH_CollisionEstimationResult field by
// field (it is not a mirror of Jolt's SIMD struct), and oxijolt passes it by
// pointer and frees it with JPH_CollisionEstimationResult_FreeMembers.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CollisionEstimationResult, 104, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, linearVelocity1, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, angularVelocity1, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, linearVelocity2, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, angularVelocity2, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, frictionPoint, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, tangent1, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, tangent2, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, frictionImpulse1, 84);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, frictionImpulse2, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, angularFrictionImpulse, 92);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, contactImpulseCount, 96);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, contactImpulses, 100);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_PhysicsSystemSettings, 32, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, maxContactConstraints, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, broadPhaseLayerInterface, 20);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, objectLayerPairFilter, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, objectVsBroadPhaseLayerFilter, 28);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_JobSystemConfig, 20, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_JobSystemConfig, context, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_JobSystemConfig, queueJob, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_JobSystemConfig, queueJobs, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_JobSystemConfig, maxConcurrency, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_JobSystemConfig, maxBarriers, 16);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_PathConstraintSettings, 104, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, path, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, pathPosition, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, pathRotation, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, pathFraction, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, maxFrictionForce, 68);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, rotationConstraintType, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, positionMotorSettings, 76);

OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CollideShapeResult, 68, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, contactPointOn1, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, contactPointOn2, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, penetrationAxis, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, penetrationDepth, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, subShapeID1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, subShapeID2, 44);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, bodyID2, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, shape1FaceCount, 52);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, shape1Faces, 56);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, shape2FaceCount, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, shape2Faces, 64);

// Filter proc tables that oxijolt fills with its callbacks.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ObjectLayerFilter_Procs, 4, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ObjectLayerFilter_Procs, ShouldCollide, 0);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_BodyFilter_Procs, 8, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_BodyFilter_Procs, ShouldCollide, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_BodyFilter_Procs, ShouldCollideLocked, 4);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ShapeFilter_Procs, 8, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeFilter_Procs, ShouldCollide, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeFilter_Procs, ShouldCollide2, 4);

// Listener proc tables that oxijolt fills with its callbacks.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ContactListener_Procs, 16, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactListener_Procs, OnContactValidate, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactListener_Procs, OnContactAdded, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactListener_Procs, OnContactPersisted, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactListener_Procs, OnContactRemoved, 12);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_BodyActivationListener_Procs, 8, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_BodyActivationListener_Procs, OnBodyActivated, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_BodyActivationListener_Procs, OnBodyDeactivated, 4);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SoftBodyContactListener_Procs, 8, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyContactListener_Procs, OnSoftBodyContactValidate, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyContactListener_Procs, OnSoftBodyContactAdded, 4);

#ifdef JPH_DEBUG_RENDERER
// Debug renderer proc table that oxijolt fills with its line callback.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_DebugRenderer_Procs, 12, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DebugRenderer_Procs, DrawLine, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DebugRenderer_Procs, DrawTriangle, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DebugRenderer_Procs, DrawText3D, 8);
#endif

// Character settings and contacts, which hold pointers. joltc converts these
// field by field, so only the C ABI is pinned here.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CharacterBaseSettings, 40, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, up, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, supportingVolume, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, maxSlopeAngle, 28);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, enhancedInternalEdgeRemoval, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, shape, 36);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CharacterVirtualSettings, 116, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, ID, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, mass, 44);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, maxStrength, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, shapeOffset, 52);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, backFaceMode, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, predictiveContactDistance, 68);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, maxCollisionIterations, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, maxConstraintIterations, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, minTimeRemaining, 80);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, collisionTolerance, 84);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, characterPadding, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, maxNumHits, 92);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, hitReductionCosMaxAngle, 96);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, penetrationRecoverySpeed, 100);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, innerBodyShape, 104);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, innerBodyIDOverride, 108);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, innerBodyLayer, 112);
// Vehicle settings, which hold pointers. joltc converts these field by field,
// so only the C ABI is pinned here.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_VehicleConstraintSettings, 80, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, up, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, forward, 44);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, maxPitchRollAngle, 56);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, wheelsCount, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, wheels, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, antiRollBarsCount, 68);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, antiRollBars, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, controller, 76);
// A collision group holds a group filter pointer; joltc converts it field by field.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CollisionGroup, 12, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionGroup, groupFilter, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionGroup, groupID, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionGroup, subGroupID, 8);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_VehicleEngineSettings, 24, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleEngineSettings, maxTorque, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleEngineSettings, minRPM, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleEngineSettings, maxRPM, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleEngineSettings, normalizedTorque, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleEngineSettings, inertia, 16);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleEngineSettings, angularDamping, 20);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_VehicleTrackSettings, 28, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, drivenWheel, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, wheels, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, wheelsCount, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, inertia, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, angularDamping, 16);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, maxBrakeTorque, 20);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, differentialRatio, 24);

OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, hash, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, bodyB, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, characterIDB, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, subShapeIDB, 16);
#ifdef JPH_DOUBLE_PRECISION
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CharacterContact, 120, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, position, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, linearVelocity, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, contactNormal, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, surfaceNormal, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, distance, 84);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, fraction, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, motionTypeB, 92);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, isSensorB, 96);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, characterB, 100);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, userData, 104);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, material, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, hadCollision, 116);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, wasDiscarded, 117);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, canPushCharacter, 118);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, isBackFacingContact, 119);
#else
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CharacterContact, 104, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, position, 20);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, linearVelocity, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, contactNormal, 44);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, surfaceNormal, 56);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, distance, 68);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, fraction, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, motionTypeB, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, isSensorB, 80);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, characterB, 84);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, userData, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, material, 96);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, hadCollision, 100);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, wasDiscarded, 101);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, canPushCharacter, 102);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, isBackFacingContact, 103);
#endif
