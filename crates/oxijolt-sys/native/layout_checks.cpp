// Compile-time layout checks for the joltc C types that the Rust bindings use.
//
// joltc declares C mirrors of Jolt values and enums and converts or casts them
// to the C++ types. These checks pin the C ABI view (the same numbers are
// asserted on the Rust side in src/layout.rs; change both together) and the
// agreement between the C mirrors and Jolt's C++ types.
//
// More types are added here when the safe layer starts to use them.

#include <Jolt/Jolt.h>

#include <Jolt/Math/Mat44.h>
#include <Jolt/Physics/Body/AllowedDOFs.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Body/MotionQuality.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Character/CharacterBase.h>
#include <Jolt/Physics/Collision/ActiveEdgeMode.h>
#include <Jolt/Physics/Collision/BackFaceMode.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/CollectFacesMode.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Constraints/ContactConstraintManager.h>
#include <Jolt/Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.h>
#include <Jolt/Physics/Constraints/MotorSettings.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>
#include <Jolt/Physics/EActivation.h>
#include <Jolt/Physics/EPhysicsUpdateError.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/Vehicle/VehicleTransmission.h>

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "joltc.h"
#include "joltc_ext/joltc_ext.h"

#define OXIJOLT_SYS_ASSERT_LAYOUT(T, size, align)                                    \
    static_assert(sizeof(T) == (size), #T ": unexpected size");                \
    static_assert(alignof(T) == (align), #T ": unexpected alignment")

#define OXIJOLT_SYS_ASSERT_OFFSET(T, field, offset)                                  \
    static_assert(offsetof(T, field) == (offset), #T "." #field ": unexpected offset")

// C ABI value structs passed by the raw tests and oxijolt.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_Vec3, 12, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_Vec3, z, 8);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_Vec4, 16, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_Vec4, w, 12);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_Quat, 16, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_Quat, w, 12);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_Mat4, 64, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_Mat4, column, 0);

#ifdef JPH_DOUBLE_PRECISION
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_RVec3, 24, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RVec3, z, 16);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_RMat4, 72, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RMat4, column3, 48);
#else
static_assert(std::is_same_v<JPH_RVec3, JPH_Vec3>, "JPH_RVec3 is JPH_Vec3 in single precision");
static_assert(std::is_same_v<JPH_RMat4, JPH_Mat4>, "JPH_RMat4 is JPH_Mat4 in single precision");
#endif

// joltc converts JPH_AABox and JPH_MassProperties field by field, so they have
// no C++ layout to compare against.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_AABox, 24, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_AABox, min, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_AABox, max, 12);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_MassProperties, 68, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_MassProperties, mass, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_MassProperties, inertia, 4);
// JPH_NarrowPhaseQuery_CastRay fills JPH_RayCastResult field by field, and
// oxijolt passes it by pointer.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_RayCastResult, 12, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RayCastResult, bodyID, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RayCastResult, fraction, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RayCastResult, subShapeID2, 8);
// JPH_Shape_CollidePoint2 and JPH_NarrowPhaseQuery_CollidePoint2 fill a local
// JPH_CollidePointResult field by field and pass oxijolt a pointer to it.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CollidePointResult, 8, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollidePointResult, bodyID, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollidePointResult, subShapeID2, 4);

// Mesh triangles. joltc and the extension copy JPH_IndexedTriangle field by field
// (JPH_MeshShapeSettings_Create2/_Create3), so only the C ABI is pinned here.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_IndexedTriangle, 20, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_IndexedTriangle, i1, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_IndexedTriangle, i2, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_IndexedTriangle, i3, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_IndexedTriangle, materialIndex, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_IndexedTriangle, userData, 16);
static_assert(sizeof(JPH_Mesh_Shape_BuildQuality) == 4, "JPH_Mesh_Shape_BuildQuality: unexpected size");
static_assert(int(JPH_Mesh_Shape_BuildQuality_FavorRuntimePerformance) == int(JPH::MeshShapeSettings::EBuildQuality::FavorRuntimePerformance),
              "JPH_Mesh_Shape_BuildQuality_FavorRuntimePerformance");
static_assert(int(JPH_Mesh_Shape_BuildQuality_FavorBuildSpeed) == int(JPH::MeshShapeSettings::EBuildQuality::FavorBuildSpeed),
              "JPH_Mesh_Shape_BuildQuality_FavorBuildSpeed");

// Soft body values. joltc copies JPH_SoftVertex and JPH_SoftFace field by field
// (JPH_SoftBodySharedSettings_AddVertices/_AddFaces), and the extension copies
// JPH_SoftBodyVertexAttributes field by field, so only the C ABI is pinned here.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SoftVertex, 28, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftVertex, position, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftVertex, velocity, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftVertex, invMass, 24);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SoftFace, 16, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftFace, vertex1, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftFace, vertex2, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftFace, vertex3, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftFace, materialIndex, 12);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SoftBodyVertexAttributes, 20, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyVertexAttributes, compliance, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyVertexAttributes, shearCompliance, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyVertexAttributes, bendCompliance, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyVertexAttributes, lraType, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyVertexAttributes, lraMaxDistanceMultiplier, 16);
static_assert(int(JPH_SoftBodyBendType_None) == int(JPH::SoftBodySharedSettings::EBendType::None), "JPH_SoftBodyBendType_None");
static_assert(int(JPH_SoftBodyBendType_Distance) == int(JPH::SoftBodySharedSettings::EBendType::Distance), "JPH_SoftBodyBendType_Distance");
static_assert(int(JPH_SoftBodyBendType_Dihedral) == int(JPH::SoftBodySharedSettings::EBendType::Dihedral), "JPH_SoftBodyBendType_Dihedral");
static_assert(int(JPH_SoftBodyLRAType_None) == int(JPH::SoftBodySharedSettings::ELRAType::None), "JPH_SoftBodyLRAType_None");
static_assert(int(JPH_SoftBodyLRAType_EuclideanDistance) == int(JPH::SoftBodySharedSettings::ELRAType::EuclideanDistance), "JPH_SoftBodyLRAType_EuclideanDistance");
static_assert(int(JPH_SoftBodyLRAType_GeodesicDistance) == int(JPH::SoftBodySharedSettings::ELRAType::GeodesicDistance), "JPH_SoftBodyLRAType_GeodesicDistance");
// The extension copies JPH_SoftBodySkinWeight and JPH_SoftBodySkinnedConstraint field by field
// (JPH_SoftBodySharedSettings_AddSkinnedConstraint), so only the C ABI is pinned here.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SoftBodySkinWeight, 8, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodySkinWeight, invBindIndex, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodySkinWeight, weight, 4);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SoftBodySkinnedConstraint, 48, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodySkinnedConstraint, vertex, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodySkinnedConstraint, weights, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodySkinnedConstraint, maxDistance, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodySkinnedConstraint, backStopDistance, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodySkinnedConstraint, backStopRadius, 44);
static_assert(JPH::SoftBodySharedSettings::Skinned::cMaxSkinWeights == 4, "JPH_SoftBodySkinnedConstraint: weight count");

// Contact settings. joltc's listener and the extension's JPH_ContactListener2 copy
// JPH_ContactSettings field by field (FromJolt/ToJolt of ContactSettings), the extension's soft body
// listener copies JPH_SoftBodyContactSettings and joltc's character contact listener
// JPH_CharacterContactSettings the same way, so only the C ABI is pinned here.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ContactSettings, 52, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactSettings, combinedFriction, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactSettings, combinedRestitution, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactSettings, invMassScale1, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactSettings, invInertiaScale1, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactSettings, invMassScale2, 16);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactSettings, invInertiaScale2, 20);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactSettings, isSensor, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactSettings, relativeLinearSurfaceVelocity, 28);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactSettings, relativeAngularSurfaceVelocity, 40);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SoftBodyContactSettings, 16, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyContactSettings, invMassScale1, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyContactSettings, invMassScale2, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyContactSettings, invInertiaScale2, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyContactSettings, isSensor, 12);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CharacterContactSettings, 2, 1);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContactSettings, canPushCharacter, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContactSettings, canReceiveImpulses, 1);

// Scene query settings and results. joltc converts these field by field
// (ToJolt/FromJolt), so only the C ABI is pinned here, not agreement with the
// C++ layout.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_RayCastSettings, 12, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RayCastSettings, backFaceModeTriangles, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RayCastSettings, backFaceModeConvex, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RayCastSettings, treatConvexAsSolid, 8);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CollideSettingsBase, 28, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideSettingsBase, activeEdgeMode, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideSettingsBase, collectFacesMode, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideSettingsBase, collisionTolerance, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideSettingsBase, penetrationTolerance, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideSettingsBase, activeEdgeMovementDirection, 16);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CollideShapeSettings, 36, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeSettings, maxSeparationDistance, 28);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeSettings, backFaceMode, 32);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ShapeCastSettings, 40, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastSettings, backFaceModeTriangles, 28);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastSettings, backFaceModeConvex, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastSettings, useShrunkenShapeAndConvexRadius, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastSettings, returnDeepestPoint, 37);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ShapeCastResult, 60, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, contactPointOn1, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, contactPointOn2, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, penetrationAxis, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, penetrationDepth, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, subShapeID1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, subShapeID2, 44);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, bodyID2, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, fraction, 52);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeCastResult, isBackFaceHit, 56);

// Character values. joltc converts these field by field, so only the C ABI is
// pinned here, not agreement with the C++ layout.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_Plane, 16, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_Plane, normal, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_Plane, distance, 12);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ExtendedUpdateSettings, 48, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ExtendedUpdateSettings, stickToFloorStepDown, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ExtendedUpdateSettings, walkStairsStepUp, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ExtendedUpdateSettings, walkStairsMinStepForward, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ExtendedUpdateSettings, walkStairsStepForwardTest, 28);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ExtendedUpdateSettings, walkStairsCosAngleForwardContact, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ExtendedUpdateSettings, walkStairsStepDownExtra, 36);

// Vehicle, constraint and physics settings values. joltc converts these field
// by field, so only the C ABI is pinned here, not agreement with the C++ layout.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_Point, 8, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_Point, x, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_Point, y, 4);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SpringSettings, 12, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SpringSettings, mode, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SpringSettings, frequencyOrStiffness, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SpringSettings, damping, 8);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_MotorSettings, 28, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_MotorSettings, springSettings, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_MotorSettings, minForceLimit, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_MotorSettings, maxForceLimit, 16);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_MotorSettings, minTorqueLimit, 20);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_MotorSettings, maxTorqueLimit, 24);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_VehicleAntiRollBar, 12, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleAntiRollBar, leftWheel, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleAntiRollBar, rightWheel, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleAntiRollBar, stiffness, 8);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_VehicleDifferentialSettings, 24, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleDifferentialSettings, leftWheel, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleDifferentialSettings, rightWheel, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleDifferentialSettings, differentialRatio, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleDifferentialSettings, leftRightSplit, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleDifferentialSettings, limitedSlipRatio, 16);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleDifferentialSettings, engineTorqueRatio, 20);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ConstraintSettings, 32, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConstraintSettings, enabled, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConstraintSettings, constraintPriority, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConstraintSettings, numVelocityStepsOverride, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConstraintSettings, numPositionStepsOverride, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConstraintSettings, drawConstraintSize, 16);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConstraintSettings, userData, 24);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_GearConstraintSettings, 64, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_GearConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_GearConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_GearConstraintSettings, hingeAxis1, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_GearConstraintSettings, hingeAxis2, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_GearConstraintSettings, ratio, 60);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_RackAndPinionConstraintSettings, 64, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RackAndPinionConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RackAndPinionConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RackAndPinionConstraintSettings, hingeAxis, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RackAndPinionConstraintSettings, sliderAxis, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_RackAndPinionConstraintSettings, ratio, 60);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_PhysicsSettings, 84, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, maxInFlightBodyPairs, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, stepListenersBatchSize, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, stepListenerBatchesPerJob, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, baumgarte, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, speculativeContactDistance, 16);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, penetrationSlop, 20);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, linearCastThreshold, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, linearCastMaxPenetration, 28);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, manifoldTolerance, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, maxPenetrationDistance, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, bodyPairCacheMaxDeltaPositionSq, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, bodyPairCacheCosMaxDeltaRotationDiv2, 44);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, contactNormalCosMaxDeltaRotation, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, contactPointPreserveLambdaMaxDistSq, 52);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, numVelocitySteps, 56);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, numPositionSteps, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, minVelocityForRestitution, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, timeBeforeSleep, 68);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, pointVelocitySleepThreshold, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, deterministicSimulation, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, constraintWarmStart, 77);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, useBodyPairContactCache, 78);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, useManifoldReduction, 79);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, useLargeIslandSplitter, 80);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, allowSleeping, 81);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSettings, checkActiveEdges, 82);

// Constraint settings, which hold JPH_RVec3. joltc converts these field by field,
// so only the C ABI is pinned here.
#ifdef JPH_DOUBLE_PRECISION
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_HingeConstraintSettings, 192, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, point1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, hingeAxis1, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, normalAxis1, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, point2, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, hingeAxis2, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, normalAxis2, 124);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, limitsMin, 136);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, limitsMax, 140);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, limitsSpringSettings, 144);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, maxFrictionTorque, 156);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, motorSettings, 160);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SwingTwistConstraintSettings, 216, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, position1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, twistAxis1, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, planeAxis1, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, position2, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, twistAxis2, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, planeAxis2, 124);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, swingType, 136);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, normalHalfConeAngle, 140);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, planeHalfConeAngle, 144);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, twistMinAngle, 148);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, twistMaxAngle, 152);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, maxFrictionTorque, 156);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, swingMotorSettings, 160);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, twistMotorSettings, 188);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SixDOFConstraintSettings, 416, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, position1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, axisX1, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, axisY1, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, position2, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, axisX2, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, axisY2, 124);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, maxFriction, 136);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, swingType, 160);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, limitMin, 164);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, limitMax, 188);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, limitsSpringSettings, 212);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, motorSettings, 248);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_FixedConstraintSettings, 136, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, autoDetectPoint, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, point1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, axisX1, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, axisY1, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, point2, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, axisX2, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, axisY2, 124);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_PointConstraintSettings, 88, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PointConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PointConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PointConstraintSettings, point1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PointConstraintSettings, point2, 64);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_DistanceConstraintSettings, 112, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, point1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, point2, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, minDistance, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, maxDistance, 92);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, limitsSpringSettings, 96);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SliderConstraintSettings, 192, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, autoDetectPoint, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, point1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, sliderAxis1, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, normalAxis1, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, point2, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, sliderAxis2, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, normalAxis2, 124);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, limitsMin, 136);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, limitsMax, 140);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, limitsSpringSettings, 144);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, maxFrictionForce, 156);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, motorSettings, 160);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ConeConstraintSettings, 120, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, point1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, twistAxis1, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, point2, 80);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, twistAxis2, 104);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, halfConeAngle, 116);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_PulleyConstraintSettings, 152, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, bodyPoint1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, fixedPoint1, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, bodyPoint2, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, fixedPoint2, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, ratio, 136);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, minLength, 140);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, maxLength, 144);
#else
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_HingeConstraintSettings, 160, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, point1, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, hingeAxis1, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, normalAxis1, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, point2, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, hingeAxis2, 84);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, normalAxis2, 96);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, limitsMin, 108);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, limitsMax, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, limitsSpringSettings, 116);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, maxFrictionTorque, 128);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_HingeConstraintSettings, motorSettings, 132);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SwingTwistConstraintSettings, 192, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, position1, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, twistAxis1, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, planeAxis1, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, position2, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, twistAxis2, 84);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, planeAxis2, 96);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, swingType, 108);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, normalHalfConeAngle, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, planeHalfConeAngle, 116);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, twistMinAngle, 120);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, twistMaxAngle, 124);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, maxFrictionTorque, 128);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, swingMotorSettings, 132);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SwingTwistConstraintSettings, twistMotorSettings, 160);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SixDOFConstraintSettings, 392, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, position1, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, axisX1, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, axisY1, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, position2, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, axisX2, 84);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, axisY2, 96);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, maxFriction, 108);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, swingType, 132);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, limitMin, 136);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, limitMax, 160);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, limitsSpringSettings, 184);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SixDOFConstraintSettings, motorSettings, 220);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_FixedConstraintSettings, 112, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, autoDetectPoint, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, point1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, axisX1, 52);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, axisY1, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, point2, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, axisX2, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_FixedConstraintSettings, axisY2, 100);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_PointConstraintSettings, 64, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PointConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PointConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PointConstraintSettings, point1, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PointConstraintSettings, point2, 48);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_DistanceConstraintSettings, 80, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, point1, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, point2, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, minDistance, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, maxDistance, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DistanceConstraintSettings, limitsSpringSettings, 68);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SliderConstraintSettings, 168, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, autoDetectPoint, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, point1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, sliderAxis1, 52);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, normalAxis1, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, point2, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, sliderAxis2, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, normalAxis2, 100);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, limitsMin, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, limitsMax, 116);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, limitsSpringSettings, 120);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, maxFrictionForce, 132);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SliderConstraintSettings, motorSettings, 136);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ConeConstraintSettings, 88, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, point1, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, twistAxis1, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, point2, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, twistAxis2, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ConeConstraintSettings, halfConeAngle, 84);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_PulleyConstraintSettings, 96, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, space, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, bodyPoint1, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, fixedPoint1, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, bodyPoint2, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, fixedPoint2, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, ratio, 84);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, minLength, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PulleyConstraintSettings, maxLength, 92);
#endif

OXIJOLT_SYS_ASSERT_LAYOUT(JobSystemThreadPoolConfig, 12, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JobSystemThreadPoolConfig, numThreads, 8);

#if UINTPTR_MAX == UINT64_MAX
// JPH_EstimateCollisionResponse fills JPH_CollisionEstimationResult field by
// field (it is not a mirror of Jolt's SIMD struct), and oxijolt passes it by
// pointer and frees it with JPH_CollisionEstimationResult_FreeMembers.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CollisionEstimationResult, 112, 8);
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
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionEstimationResult, contactImpulses, 104);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_PhysicsSystemSettings, 48, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, maxContactConstraints, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, broadPhaseLayerInterface, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, objectLayerPairFilter, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PhysicsSystemSettings, objectVsBroadPhaseLayerFilter, 40);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_JobSystemConfig, 32, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_JobSystemConfig, context, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_JobSystemConfig, queueJob, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_JobSystemConfig, queueJobs, 16);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_JobSystemConfig, maxConcurrency, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_JobSystemConfig, maxBarriers, 28);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_PathConstraintSettings, 112, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, path, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, pathPosition, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, pathRotation, 52);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, pathFraction, 68);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, maxFrictionForce, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, rotationConstraintType, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_PathConstraintSettings, positionMotorSettings, 80);

OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CollideShapeResult, 80, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, contactPointOn1, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, contactPointOn2, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, penetrationAxis, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, penetrationDepth, 36);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, subShapeID1, 40);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, subShapeID2, 44);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, bodyID2, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, shape1FaceCount, 52);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, shape1Faces, 56);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, shape2FaceCount, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollideShapeResult, shape2Faces, 72);

// Filter proc tables that oxijolt fills with its callbacks.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ObjectLayerFilter_Procs, 8, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ObjectLayerFilter_Procs, ShouldCollide, 0);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_BodyFilter_Procs, 16, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_BodyFilter_Procs, ShouldCollide, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_BodyFilter_Procs, ShouldCollideLocked, 8);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ShapeFilter_Procs, 16, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeFilter_Procs, ShouldCollide, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ShapeFilter_Procs, ShouldCollide2, 8);

// Listener proc tables that oxijolt fills with its callbacks.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_ContactListener_Procs, 32, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactListener_Procs, OnContactValidate, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactListener_Procs, OnContactAdded, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactListener_Procs, OnContactPersisted, 16);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_ContactListener_Procs, OnContactRemoved, 24);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_BodyActivationListener_Procs, 16, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_BodyActivationListener_Procs, OnBodyActivated, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_BodyActivationListener_Procs, OnBodyDeactivated, 8);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_SoftBodyContactListener_Procs, 16, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyContactListener_Procs, OnSoftBodyContactValidate, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_SoftBodyContactListener_Procs, OnSoftBodyContactAdded, 8);

#ifdef JPH_DEBUG_RENDERER
// Debug renderer proc table that oxijolt fills with its line callback.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_DebugRenderer_Procs, 24, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DebugRenderer_Procs, DrawLine, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DebugRenderer_Procs, DrawTriangle, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_DebugRenderer_Procs, DrawText3D, 16);
#endif

// Character settings and contacts, which hold pointers. joltc converts these
// field by field, so only the C ABI is pinned here.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CharacterBaseSettings, 48, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, up, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, supportingVolume, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, maxSlopeAngle, 28);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, enhancedInternalEdgeRemoval, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterBaseSettings, shape, 40);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CharacterVirtualSettings, 128, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, ID, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, mass, 52);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, maxStrength, 56);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, shapeOffset, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, backFaceMode, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, predictiveContactDistance, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, maxCollisionIterations, 80);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, maxConstraintIterations, 84);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, minTimeRemaining, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, collisionTolerance, 92);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, characterPadding, 96);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, maxNumHits, 100);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, hitReductionCosMaxAngle, 104);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, penetrationRecoverySpeed, 108);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, innerBodyShape, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, innerBodyIDOverride, 120);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterVirtualSettings, innerBodyLayer, 124);
// Vehicle settings, which hold pointers. joltc converts these field by field,
// so only the C ABI is pinned here.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_VehicleConstraintSettings, 96, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, base, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, up, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, forward, 44);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, maxPitchRollAngle, 56);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, wheelsCount, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, wheels, 64);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, antiRollBarsCount, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, antiRollBars, 80);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleConstraintSettings, controller, 88);
// A collision group holds a group filter pointer; joltc converts it field by field.
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CollisionGroup, 16, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionGroup, groupFilter, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionGroup, groupID, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CollisionGroup, subGroupID, 12);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_VehicleEngineSettings, 32, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleEngineSettings, maxTorque, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleEngineSettings, minRPM, 4);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleEngineSettings, maxRPM, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleEngineSettings, normalizedTorque, 16);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleEngineSettings, inertia, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleEngineSettings, angularDamping, 28);
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_VehicleTrackSettings, 40, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, drivenWheel, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, wheels, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, wheelsCount, 16);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, inertia, 20);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, angularDamping, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, maxBrakeTorque, 28);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_VehicleTrackSettings, differentialRatio, 32);

OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, hash, 0);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, bodyB, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, characterIDB, 12);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, subShapeIDB, 16);
#ifdef JPH_DOUBLE_PRECISION
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CharacterContact, 136, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, position, 24);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, linearVelocity, 48);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, contactNormal, 60);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, surfaceNormal, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, distance, 84);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, fraction, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, motionTypeB, 92);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, isSensorB, 96);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, characterB, 104);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, userData, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, material, 120);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, hadCollision, 128);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, wasDiscarded, 129);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, canPushCharacter, 130);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, isBackFacingContact, 131);
#else
OXIJOLT_SYS_ASSERT_LAYOUT(JPH_CharacterContact, 120, 8);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, position, 20);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, linearVelocity, 32);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, contactNormal, 44);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, surfaceNormal, 56);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, distance, 68);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, fraction, 72);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, motionTypeB, 76);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, isSensorB, 80);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, characterB, 88);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, userData, 96);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, material, 104);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, hadCollision, 112);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, wasDiscarded, 113);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, canPushCharacter, 114);
OXIJOLT_SYS_ASSERT_OFFSET(JPH_CharacterContact, isBackFacingContact, 115);
#endif
#endif

// joltc copies JPH_Mat4 to and from JPH::Mat44 with memcpy, so the sizes must
// agree. The alignments do not: Mat44 is 16-aligned while JPH_Mat4 is
// 4-aligned, which is why build.rs leaves out the joltc functions that
// reinterpret_cast JPH_Mat4 arrays to Mat44 arrays.
static_assert(sizeof(JPH_Mat4) == sizeof(JPH::Mat44), "JPH_Mat4 must have the size of JPH::Mat44");
static_assert(alignof(JPH::Mat44) == 16, "JPH::Mat44 is expected to be 16-aligned");
// In double precision joltc converts JPH_RVec3 and JPH_RMat4 field by field,
// so they have no C++ mirror to check against.

// Scalar handles cast between C and C++.
static_assert(sizeof(JPH_BodyID) == sizeof(JPH::BodyID), "JPH_BodyID must have the size of JPH::BodyID");
static_assert(sizeof(JPH_BodyID) == 4, "JPH_BodyID is expected to be 4 bytes");
static_assert(sizeof(JPH_SubShapeID) == sizeof(JPH::SubShapeID), "JPH_SubShapeID must have the size of JPH::SubShapeID");
static_assert(sizeof(JPH_SubShapeID) == 4, "JPH_SubShapeID is expected to be 4 bytes");
static_assert(sizeof(JPH_ObjectLayer) == sizeof(JPH::ObjectLayer), "JPH_ObjectLayer must match Jolt's OBJECT_LAYER_BITS");
static_assert(sizeof(JPH_BroadPhaseLayer) == sizeof(JPH::BroadPhaseLayer::Type), "JPH_BroadPhaseLayer must match JPH::BroadPhaseLayer::Type");

// Enums cast across the boundary: 4 bytes each and every enumerator equal to
// Jolt's value.
static_assert(sizeof(JPH_MotionType) == 4, "JPH_MotionType: unexpected size");
static_assert(sizeof(JPH_Activation) == 4, "JPH_Activation: unexpected size");
static_assert(sizeof(JPH_PhysicsUpdateError) == 4, "JPH_PhysicsUpdateError: unexpected size");
static_assert(sizeof(JPH_MotionQuality) == 4, "JPH_MotionQuality: unexpected size");
static_assert(sizeof(JPH_OverrideMassProperties) == 4, "JPH_OverrideMassProperties: unexpected size");
static_assert(sizeof(JPH_ShapeSubType) == 4, "JPH_ShapeSubType: unexpected size");
static_assert(sizeof(JPH_BackFaceMode) == 4, "JPH_BackFaceMode: unexpected size");
static_assert(sizeof(JPH_ActiveEdgeMode) == 4, "JPH_ActiveEdgeMode: unexpected size");
static_assert(sizeof(JPH_CollectFacesMode) == 4, "JPH_CollectFacesMode: unexpected size");
static_assert(sizeof(JPH_CollisionCollectorType) == 4, "JPH_CollisionCollectorType: unexpected size");
static_assert(sizeof(JPH_GroundState) == 4, "JPH_GroundState: unexpected size");
static_assert(sizeof(JPH_SpringMode) == 4, "JPH_SpringMode: unexpected size");
static_assert(sizeof(JPH_TransmissionMode) == 4, "JPH_TransmissionMode: unexpected size");
static_assert(sizeof(JPH_ConstraintSubType) == 4, "JPH_ConstraintSubType: unexpected size");
static_assert(sizeof(JPH_MotorState) == 4, "JPH_MotorState: unexpected size");
static_assert(sizeof(JPH_SwingType) == 4, "JPH_SwingType: unexpected size");
static_assert(sizeof(JPH_ConstraintSpace) == 4, "JPH_ConstraintSpace: unexpected size");
static_assert(sizeof(JPH_SixDOFConstraintAxis) == 4, "JPH_SixDOFConstraintAxis: unexpected size");
static_assert(sizeof(JPH_SoftBodyValidateResult) == 4, "JPH_SoftBodyValidateResult: unexpected size");
static_assert(sizeof(JPH_AllowedDOFs) == 4, "JPH_AllowedDOFs: unexpected size");

static_assert(int(JPH_MotionType_Static) == int(JPH::EMotionType::Static), "JPH_MotionType_Static");
static_assert(int(JPH_MotionType_Kinematic) == int(JPH::EMotionType::Kinematic), "JPH_MotionType_Kinematic");
static_assert(int(JPH_MotionType_Dynamic) == int(JPH::EMotionType::Dynamic), "JPH_MotionType_Dynamic");

static_assert(int(JPH_Activation_Activate) == int(JPH::EActivation::Activate), "JPH_Activation_Activate");
static_assert(int(JPH_Activation_DontActivate) == int(JPH::EActivation::DontActivate), "JPH_Activation_DontActivate");

static_assert(int(JPH_AllowedDOFs_All) == int(JPH::EAllowedDOFs::All), "JPH_AllowedDOFs_All");
static_assert(int(JPH_AllowedDOFs_TranslationX) == int(JPH::EAllowedDOFs::TranslationX), "JPH_AllowedDOFs_TranslationX");
static_assert(int(JPH_AllowedDOFs_TranslationY) == int(JPH::EAllowedDOFs::TranslationY), "JPH_AllowedDOFs_TranslationY");
static_assert(int(JPH_AllowedDOFs_TranslationZ) == int(JPH::EAllowedDOFs::TranslationZ), "JPH_AllowedDOFs_TranslationZ");
static_assert(int(JPH_AllowedDOFs_RotationX) == int(JPH::EAllowedDOFs::RotationX), "JPH_AllowedDOFs_RotationX");
static_assert(int(JPH_AllowedDOFs_RotationY) == int(JPH::EAllowedDOFs::RotationY), "JPH_AllowedDOFs_RotationY");
static_assert(int(JPH_AllowedDOFs_RotationZ) == int(JPH::EAllowedDOFs::RotationZ), "JPH_AllowedDOFs_RotationZ");
static_assert(int(JPH_AllowedDOFs_Plane2D) == int(JPH::EAllowedDOFs::Plane2D), "JPH_AllowedDOFs_Plane2D");

static_assert(int(JPH_PhysicsUpdateError_None) == int(JPH::EPhysicsUpdateError::None), "JPH_PhysicsUpdateError_None");
static_assert(int(JPH_PhysicsUpdateError_ManifoldCacheFull) == int(JPH::EPhysicsUpdateError::ManifoldCacheFull),
              "JPH_PhysicsUpdateError_ManifoldCacheFull");
static_assert(int(JPH_PhysicsUpdateError_BodyPairCacheFull) == int(JPH::EPhysicsUpdateError::BodyPairCacheFull),
              "JPH_PhysicsUpdateError_BodyPairCacheFull");
static_assert(int(JPH_PhysicsUpdateError_ContactConstraintsFull) == int(JPH::EPhysicsUpdateError::ContactConstraintsFull),
              "JPH_PhysicsUpdateError_ContactConstraintsFull");

// Jolt declares these as uint8 enums; joltc casts the 4-byte C values.
static_assert(int(JPH_MotionQuality_Discrete) == int(JPH::EMotionQuality::Discrete), "JPH_MotionQuality_Discrete");
static_assert(int(JPH_MotionQuality_LinearCast) == int(JPH::EMotionQuality::LinearCast), "JPH_MotionQuality_LinearCast");

static_assert(int(JPH_OverrideMassProperties_CalculateMassAndInertia) == int(JPH::EOverrideMassProperties::CalculateMassAndInertia),
              "JPH_OverrideMassProperties_CalculateMassAndInertia");
static_assert(int(JPH_OverrideMassProperties_CalculateInertia) == int(JPH::EOverrideMassProperties::CalculateInertia),
              "JPH_OverrideMassProperties_CalculateInertia");
static_assert(int(JPH_OverrideMassProperties_MassAndInertiaProvided) == int(JPH::EOverrideMassProperties::MassAndInertiaProvided),
              "JPH_OverrideMassProperties_MassAndInertiaProvided");

// Jolt declares EShapeSubType as a uint8 enum; joltc casts it to the 4-byte C value.
static_assert(int(JPH_ShapeSubType_Sphere) == int(JPH::EShapeSubType::Sphere), "JPH_ShapeSubType_Sphere");
static_assert(int(JPH_ShapeSubType_Box) == int(JPH::EShapeSubType::Box), "JPH_ShapeSubType_Box");
static_assert(int(JPH_ShapeSubType_Capsule) == int(JPH::EShapeSubType::Capsule), "JPH_ShapeSubType_Capsule");
static_assert(int(JPH_ShapeSubType_Cylinder) == int(JPH::EShapeSubType::Cylinder), "JPH_ShapeSubType_Cylinder");
static_assert(int(JPH_ShapeSubType_StaticCompound) == int(JPH::EShapeSubType::StaticCompound), "JPH_ShapeSubType_StaticCompound");
static_assert(int(JPH_ShapeSubType_MutableCompound) == int(JPH::EShapeSubType::MutableCompound), "JPH_ShapeSubType_MutableCompound");
static_assert(int(JPH_ShapeSubType_HeightField) == int(JPH::EShapeSubType::HeightField), "JPH_ShapeSubType_HeightField");

// Jolt declares these as uint8 enums; joltc casts the 4-byte C values.
// JPH_CollisionCollectorType is joltc's own and has no Jolt counterpart.
static_assert(int(JPH_BackFaceMode_IgnoreBackFaces) == int(JPH::EBackFaceMode::IgnoreBackFaces), "JPH_BackFaceMode_IgnoreBackFaces");
static_assert(int(JPH_BackFaceMode_CollideWithBackFaces) == int(JPH::EBackFaceMode::CollideWithBackFaces), "JPH_BackFaceMode_CollideWithBackFaces");
static_assert(int(JPH_ActiveEdgeMode_CollideOnlyWithActive) == int(JPH::EActiveEdgeMode::CollideOnlyWithActive), "JPH_ActiveEdgeMode_CollideOnlyWithActive");
static_assert(int(JPH_ActiveEdgeMode_CollideWithAll) == int(JPH::EActiveEdgeMode::CollideWithAll), "JPH_ActiveEdgeMode_CollideWithAll");
static_assert(int(JPH_CollectFacesMode_CollectFaces) == int(JPH::ECollectFacesMode::CollectFaces), "JPH_CollectFacesMode_CollectFaces");
static_assert(int(JPH_CollectFacesMode_NoFaces) == int(JPH::ECollectFacesMode::NoFaces), "JPH_CollectFacesMode_NoFaces");

// Jolt declares EGroundState as a uint8 enum; joltc casts it to the 4-byte C value.
static_assert(int(JPH_GroundState_OnGround) == int(JPH::CharacterBase::EGroundState::OnGround), "JPH_GroundState_OnGround");
static_assert(int(JPH_GroundState_OnSteepGround) == int(JPH::CharacterBase::EGroundState::OnSteepGround), "JPH_GroundState_OnSteepGround");
static_assert(int(JPH_GroundState_NotSupported) == int(JPH::CharacterBase::EGroundState::NotSupported), "JPH_GroundState_NotSupported");
static_assert(int(JPH_GroundState_InAir) == int(JPH::CharacterBase::EGroundState::InAir), "JPH_GroundState_InAir");

// Jolt declares ESpringMode and ETransmissionMode as uint8 enums; joltc casts the
// 4-byte C values.
static_assert(int(JPH_SpringMode_FrequencyAndDamping) == int(JPH::ESpringMode::FrequencyAndDamping), "JPH_SpringMode_FrequencyAndDamping");
static_assert(int(JPH_SpringMode_StiffnessAndDamping) == int(JPH::ESpringMode::StiffnessAndDamping), "JPH_SpringMode_StiffnessAndDamping");
static_assert(int(JPH_TransmissionMode_Auto) == int(JPH::ETransmissionMode::Auto), "JPH_TransmissionMode_Auto");
static_assert(int(JPH_TransmissionMode_Manual) == int(JPH::ETransmissionMode::Manual), "JPH_TransmissionMode_Manual");
static_assert(int(JPH_ConstraintSubType_Vehicle) == int(JPH::EConstraintSubType::Vehicle), "JPH_ConstraintSubType_Vehicle");

// Constraint enums. Jolt declares ESwingType as a uint8 enum; joltc casts the 4-byte C values.
static_assert(int(JPH_ConstraintSubType_Hinge) == int(JPH::EConstraintSubType::Hinge), "JPH_ConstraintSubType_Hinge");
static_assert(int(JPH_ConstraintSubType_SwingTwist) == int(JPH::EConstraintSubType::SwingTwist), "JPH_ConstraintSubType_SwingTwist");
static_assert(int(JPH_ConstraintSubType_SixDOF) == int(JPH::EConstraintSubType::SixDOF), "JPH_ConstraintSubType_SixDOF");
static_assert(int(JPH_ConstraintSubType_Fixed) == int(JPH::EConstraintSubType::Fixed), "JPH_ConstraintSubType_Fixed");
static_assert(int(JPH_ConstraintSubType_Point) == int(JPH::EConstraintSubType::Point), "JPH_ConstraintSubType_Point");
static_assert(int(JPH_ConstraintSubType_Distance) == int(JPH::EConstraintSubType::Distance), "JPH_ConstraintSubType_Distance");
static_assert(int(JPH_ConstraintSubType_Slider) == int(JPH::EConstraintSubType::Slider), "JPH_ConstraintSubType_Slider");
static_assert(int(JPH_ConstraintSubType_Cone) == int(JPH::EConstraintSubType::Cone), "JPH_ConstraintSubType_Cone");
static_assert(int(JPH_ConstraintSubType_Path) == int(JPH::EConstraintSubType::Path), "JPH_ConstraintSubType_Path");
static_assert(int(JPH_ConstraintSubType_RackAndPinion) == int(JPH::EConstraintSubType::RackAndPinion), "JPH_ConstraintSubType_RackAndPinion");
static_assert(int(JPH_ConstraintSubType_Gear) == int(JPH::EConstraintSubType::Gear), "JPH_ConstraintSubType_Gear");
static_assert(int(JPH_ConstraintSubType_Pulley) == int(JPH::EConstraintSubType::Pulley), "JPH_ConstraintSubType_Pulley");
static_assert(int(JPH_PathRotationConstraintType_Free) == int(JPH::EPathRotationConstraintType::Free), "JPH_PathRotationConstraintType_Free");
static_assert(int(JPH_PathRotationConstraintType_ConstrainAroundTangent) == int(JPH::EPathRotationConstraintType::ConstrainAroundTangent), "JPH_PathRotationConstraintType_ConstrainAroundTangent");
static_assert(int(JPH_PathRotationConstraintType_ConstrainAroundNormal) == int(JPH::EPathRotationConstraintType::ConstrainAroundNormal), "JPH_PathRotationConstraintType_ConstrainAroundNormal");
static_assert(int(JPH_PathRotationConstraintType_ConstrainAroundBinormal) == int(JPH::EPathRotationConstraintType::ConstrainAroundBinormal), "JPH_PathRotationConstraintType_ConstrainAroundBinormal");
static_assert(int(JPH_PathRotationConstraintType_ConstrainToPath) == int(JPH::EPathRotationConstraintType::ConstrainToPath), "JPH_PathRotationConstraintType_ConstrainToPath");
static_assert(int(JPH_PathRotationConstraintType_FullyConstrained) == int(JPH::EPathRotationConstraintType::FullyConstrained), "JPH_PathRotationConstraintType_FullyConstrained");
static_assert(int(JPH_MotorState_Off) == int(JPH::EMotorState::Off), "JPH_MotorState_Off");
static_assert(int(JPH_MotorState_Velocity) == int(JPH::EMotorState::Velocity), "JPH_MotorState_Velocity");
static_assert(int(JPH_MotorState_Position) == int(JPH::EMotorState::Position), "JPH_MotorState_Position");
static_assert(int(JPH_SwingType_Cone) == int(JPH::ESwingType::Cone), "JPH_SwingType_Cone");
static_assert(int(JPH_SwingType_Pyramid) == int(JPH::ESwingType::Pyramid), "JPH_SwingType_Pyramid");
static_assert(int(JPH_ConstraintSpace_LocalToBodyCOM) == int(JPH::EConstraintSpace::LocalToBodyCOM), "JPH_ConstraintSpace_LocalToBodyCOM");
static_assert(int(JPH_ConstraintSpace_WorldSpace) == int(JPH::EConstraintSpace::WorldSpace), "JPH_ConstraintSpace_WorldSpace");
static_assert(int(JPH_SixDOFConstraintAxis_TranslationX) == int(JPH::SixDOFConstraintSettings::EAxis::TranslationX), "JPH_SixDOFConstraintAxis_TranslationX");
static_assert(int(JPH_SixDOFConstraintAxis_TranslationY) == int(JPH::SixDOFConstraintSettings::EAxis::TranslationY), "JPH_SixDOFConstraintAxis_TranslationY");
static_assert(int(JPH_SixDOFConstraintAxis_TranslationZ) == int(JPH::SixDOFConstraintSettings::EAxis::TranslationZ), "JPH_SixDOFConstraintAxis_TranslationZ");
static_assert(int(JPH_SixDOFConstraintAxis_RotationX) == int(JPH::SixDOFConstraintSettings::EAxis::RotationX), "JPH_SixDOFConstraintAxis_RotationX");
static_assert(int(JPH_SixDOFConstraintAxis_RotationY) == int(JPH::SixDOFConstraintSettings::EAxis::RotationY), "JPH_SixDOFConstraintAxis_RotationY");
static_assert(int(JPH_SixDOFConstraintAxis_RotationZ) == int(JPH::SixDOFConstraintSettings::EAxis::RotationZ), "JPH_SixDOFConstraintAxis_RotationZ");
static_assert(int(_JPH_SixDOFConstraintAxis_Num) == int(JPH::SixDOFConstraintSettings::EAxis::Num), "_JPH_SixDOFConstraintAxis_Num");

// Not a layout: oxijolt bounds WorldSettings::max_contact_constraints by
// MAX_CONTACT_CONSTRAINTS (2^20), which must stay within the count above which
// ContactConstraintManager::Init asserts.
static_assert(JPH::ContactConstraintManager::cMaxContactConstraintsLimit >= (1u << 20), "oxijolt WorldSettings::MAX_CONTACT_CONSTRAINTS must stay within Jolt's limit; change both together");
