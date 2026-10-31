// Functions this fork adds to joltc, in joltc's naming and style. They are compiled into the
// joltc archive (see ../CMakeLists.txt) and are meant to be offered upstream.
// Licensed under the MIT License (MIT), like joltc.

#ifndef JOLT_C_EXT_H_
#define JOLT_C_EXT_H_ 1

#include "joltc.h"

/* StateRecorder */
typedef struct JPH_StateRecorder JPH_StateRecorder; /* JPH::StateRecorderImpl */

JPH_CAPI JPH_StateRecorder* JPH_StateRecorder_Create(void);
JPH_CAPI void JPH_StateRecorder_Destroy(JPH_StateRecorder* recorder);
JPH_CAPI void JPH_StateRecorder_Rewind(JPH_StateRecorder* recorder);
JPH_CAPI void JPH_StateRecorder_WriteBytes(JPH_StateRecorder* recorder, const void* data, size_t size);
JPH_CAPI size_t JPH_StateRecorder_GetDataSize(JPH_StateRecorder* recorder);
/* Copies min(size, data size) bytes of the recorded data to data, which may be null when that is 0. */
JPH_CAPI void JPH_StateRecorder_CopyData(JPH_StateRecorder* recorder, void* data, size_t size);
JPH_CAPI bool JPH_StateRecorder_IsFailed(const JPH_StateRecorder* recorder);

/* Which parts of a physics system's state JPH_PhysicsSystem_SaveState writes (JPH::EStateRecorderState). */
typedef enum JPH_StateRecorderState {
	JPH_StateRecorderState_None = 0,
	JPH_StateRecorderState_Global = 1 << 0,
	JPH_StateRecorderState_Bodies = 1 << 1,
	JPH_StateRecorderState_Contacts = 1 << 2,
	JPH_StateRecorderState_Constraints = 1 << 3,
	JPH_StateRecorderState_All = 15,

	_JPH_StateRecorderState_Force32 = 0x7fffffff
} JPH_StateRecorderState;

/* PhysicsSystem state */
/* bodies null: every body is saved; otherwise only the bodies whose id is among the bodyCount ids
   (any order, duplicates allowed), none for bodyCount 0, when bodies need not point to any object.
   Contacts and constraints are never filtered. */
JPH_CAPI void JPH_PhysicsSystem_SaveState(const JPH_PhysicsSystem* system, JPH_StateRecorder* recorder, JPH_StateRecorderState state, const JPH_BodyID* bodies, uint32_t bodyCount);
/* Reads from the recorder's current read position (rewind it first). Never validating. Returns
   false when Jolt could not restore; the system may then be partly restored. */
JPH_CAPI bool JPH_PhysicsSystem_RestoreState(JPH_PhysicsSystem* system, JPH_StateRecorder* recorder);

/* CharacterVirtual */
JPH_CAPI void JPH_CharacterVirtual_SaveState(const JPH_CharacterVirtual* character, JPH_StateRecorder* recorder);
JPH_CAPI void JPH_CharacterVirtual_RestoreState(JPH_CharacterVirtual* character, JPH_StateRecorder* recorder);

/* Like JPH_CharacterVirtual_ExtendedUpdate and JPH_CharacterVirtual_RefreshContacts, with explicit
   gravity, filters (null accepts everything) and temp allocator instead of the system's gravity, its
   layer tables and joltc's global temp allocator. */
JPH_CAPI void JPH_CharacterVirtual_ExtendedUpdate2(JPH_CharacterVirtual* character, float deltaTime,
	const JPH_Vec3* gravity, const JPH_ExtendedUpdateSettings* settings,
	const JPH_BroadPhaseLayerFilter* broadPhaseLayerFilter, const JPH_ObjectLayerFilter* objectLayerFilter,
	const JPH_BodyFilter* bodyFilter, const JPH_ShapeFilter* shapeFilter, JPH_TempAllocator* tempAllocator);
JPH_CAPI void JPH_CharacterVirtual_RefreshContacts2(JPH_CharacterVirtual* character,
	const JPH_BroadPhaseLayerFilter* broadPhaseLayerFilter, const JPH_ObjectLayerFilter* objectLayerFilter,
	const JPH_BodyFilter* bodyFilter, const JPH_ShapeFilter* shapeFilter, JPH_TempAllocator* tempAllocator);

/* VehicleConstraint */
/* The Constraint base of the vehicle constraint, for JPH_PhysicsSystem_AddConstraint, _RemoveConstraint
   and JPH_Constraint_Destroy; counterpart of JPH_VehicleConstraint_AsPhysicsStepListener. */
JPH_CAPI JPH_Constraint* JPH_VehicleConstraint_AsConstraint(JPH_VehicleConstraint* constraint);

/* VehicleStepListener */
typedef struct JPH_VehicleStepListener JPH_VehicleStepListener; /* a JPH::PhysicsStepListener that forwards to one vehicle */

/* A step listener to register instead of JPH_VehicleConstraint_AsPhysicsStepListener. It runs the
   vehicle's VehicleConstraint::OnStep in every collision step, but a gravity override acts once per
   collision step, as world gravity does: Jolt's OnStep adds the override force to the chassis in
   every collision step while the force accumulator is reset only after the last one, so collision
   step k of one PhysicsSystem::Update would get k times the override. From the second time OnStep
   adds it in one update, the listener restores the chassis force it had before OnStep. With one
   collision step it only forwards. The listener keeps a plain pointer to constraint, which must
   outlive its registration; the caller owns the listener and destroys it with
   JPH_VehicleStepListener_Destroy after JPH_PhysicsSystem_RemoveStepListener. Not reference counted. */
JPH_CAPI JPH_VehicleStepListener* JPH_VehicleStepListener_Create(JPH_VehicleConstraint* constraint);
JPH_CAPI void JPH_VehicleStepListener_Destroy(JPH_VehicleStepListener* listener);
/* The PhysicsStepListener base, for JPH_PhysicsSystem_AddStepListener and _RemoveStepListener. */
JPH_CAPI JPH_PhysicsStepListener* JPH_VehicleStepListener_AsPhysicsStepListener(JPH_VehicleStepListener* listener);

/* VehicleCollisionTester */
/* Like JPH_VehicleCollisionTesterRay_Create, _CastSphere_Create and _CastCylinder_Create, but the tester
   skips soft bodies as well as the chassis vehicleBody (Jolt's default body filter skips only the
   chassis): Jolt's VehicleConstraint treats the body under a wheel as rigid. Each returns a new tester
   holding one reference. */
JPH_CAPI JPH_VehicleCollisionTesterRay* JPH_VehicleCollisionTesterRay_Create2(JPH_ObjectLayer layer, const JPH_Vec3* up, float maxSlopeAngle, JPH_BodyID vehicleBody);
JPH_CAPI JPH_VehicleCollisionTesterCastSphere* JPH_VehicleCollisionTesterCastSphere_Create2(JPH_ObjectLayer layer, float radius, const JPH_Vec3* up, float maxSlopeAngle, JPH_BodyID vehicleBody);
JPH_CAPI JPH_VehicleCollisionTesterCastCylinder* JPH_VehicleCollisionTesterCastCylinder_Create2(JPH_ObjectLayer layer, float convexRadiusFraction, JPH_BodyID vehicleBody);

/* MotorcycleController */
/* Reads the target lean the lean controller steers the chassis' up toward, a world-space vector
   (Jolt's protected MotorcycleController::mTargetLean, part of its saved state): the zero vector
   before the first step, world up after a step with the lean controller off. Returns false, and
   leaves result untouched, when controller is not a MotorcycleController. The kind check
   allocates and releases one settings object (VehicleController::GetSettings). */
JPH_CAPI bool JPH_MotorcycleController_GetTargetLean(const JPH_VehicleController* controller, JPH_Vec3* result);
/* Replaces the target lean, for moving a world into a rotated frame; the next step smooths from
   it. Returns false, changing nothing, when controller is not a MotorcycleController. */
JPH_CAPI bool JPH_MotorcycleController_SetTargetLean(JPH_VehicleController* controller, const JPH_Vec3* value);

/* Live drivetrains: WheeledVehicleController and TrackedVehicleController */
/* Each setter copies configuration only and keeps the runtime state (engine rpm; current gear, clutch
   friction and the transmission's timers; track angular velocity). The change takes effect in the
   next step; the caller has exclusive access to the controller (no step runs). The
   JPH_WheeledVehicleController functions also accept a JPH_MotorcycleController, which derives from
   it by single inheritance (joltc's cast convention). */
/* Copies max torque, rpm range, inertia, angular damping and a deep copy of the normalized torque
   curve. The engine's current rpm is kept; Jolt clamps it to the new range before using it. */
JPH_CAPI void JPH_WheeledVehicleController_SetEngine(JPH_WheeledVehicleController* controller, const JPH_VehicleEngineSettings* settings);
JPH_CAPI void JPH_TrackedVehicleController_SetEngine(JPH_TrackedVehicleController* controller, const JPH_VehicleEngineSettings* settings);
/* Returns false and changes nothing when the number of forward or reverse gears, or the clutch
   release time, differs from the controller's: Jolt's state keeps the current gear and the
   clutch release countdown, which index the ratios and divide by the release time. */
JPH_CAPI bool JPH_WheeledVehicleController_SetTransmission(JPH_WheeledVehicleController* controller, const JPH_VehicleTransmissionSettings* settings);
JPH_CAPI bool JPH_TrackedVehicleController_SetTransmission(JPH_TrackedVehicleController* controller, const JPH_VehicleTransmissionSettings* settings);
/* Replaces the differentials with count copies of values (which may be null when count is 0). Each
   wheel index must be -1 or name a wheel of the vehicle: Jolt does not check them. */
JPH_CAPI void JPH_WheeledVehicleController_SetDifferentials(JPH_WheeledVehicleController* controller, const JPH_VehicleDifferentialSettings* values, uint32_t count);
JPH_CAPI void JPH_WheeledVehicleController_SetDifferentialLimitedSlipRatio(JPH_WheeledVehicleController* controller, float value);
/* Copies inertia, angular damping, max brake torque and differential ratio into the track on side.
   Returns false and changes nothing when side is not a track side, or when the driven wheel or
   the wheel list differs from the track's: Jolt re-reads them every step. */
JPH_CAPI bool JPH_TrackedVehicleController_SetTrack(JPH_TrackedVehicleController* controller, JPH_TrackSide side, const JPH_VehicleTrackSettings* track);
/* Gives the controller Jolt's default tire max impulse callback again (friction times suspension
   impulse); the callback and userData set before are no longer called once this returns. */
JPH_CAPI void JPH_WheeledVehicleController_ResetTireMaxImpulseCallback(JPH_WheeledVehicleController* controller);

/* RagdollSettings */
/* Copies every BodyCreationSettings field (shape reference, collision group, damping, velocities, mass
   override, ...) into part partIndex; the part's constraint to its parent is kept. */
JPH_CAPI void JPH_RagdollSettings_SetPart(JPH_RagdollSettings* settings, int partIndex, const JPH_BodyCreationSettings* bodySettings);
/* Like JPH_RagdollSettings_SetPartToParent, but every field reaches Jolt, base settings, spring modes and
   torque limits included; null removes the constraint. */
JPH_CAPI void JPH_RagdollSettings_SetPartToParentSwingTwist(JPH_RagdollSettings* settings, int partIndex, const JPH_SwingTwistConstraintSettings* constraintSettings);
JPH_CAPI void JPH_RagdollSettings_SetPartToParentHinge(JPH_RagdollSettings* settings, int partIndex, const JPH_HingeConstraintSettings* constraintSettings);
JPH_CAPI void JPH_RagdollSettings_SetPartToParentSixDOF(JPH_RagdollSettings* settings, int partIndex, const JPH_SixDOFConstraintSettings* constraintSettings);
JPH_CAPI void JPH_RagdollSettings_CalculateConstraintPriorities(JPH_RagdollSettings* settings, uint32_t basePriority);

/* SwingTwistConstraint */
JPH_CAPI void JPH_SwingTwistConstraint_SetSwingMotorState(JPH_SwingTwistConstraint* constraint, JPH_MotorState state);
JPH_CAPI JPH_MotorState JPH_SwingTwistConstraint_GetSwingMotorState(const JPH_SwingTwistConstraint* constraint);
JPH_CAPI void JPH_SwingTwistConstraint_SetTwistMotorState(JPH_SwingTwistConstraint* constraint, JPH_MotorState state);
JPH_CAPI JPH_MotorState JPH_SwingTwistConstraint_GetTwistMotorState(const JPH_SwingTwistConstraint* constraint);
JPH_CAPI void JPH_SwingTwistConstraint_SetTargetOrientationBS(JPH_SwingTwistConstraint* constraint, const JPH_Quat* orientation);
JPH_CAPI void JPH_SwingTwistConstraint_GetRotationInConstraintSpace(const JPH_SwingTwistConstraint* constraint, JPH_Quat* result);
JPH_CAPI void JPH_SwingTwistConstraint_SetTargetAngularVelocityCS(JPH_SwingTwistConstraint* constraint, const JPH_Vec3* angularVelocity);
JPH_CAPI void JPH_SwingTwistConstraint_GetTargetAngularVelocityCS(const JPH_SwingTwistConstraint* constraint, JPH_Vec3* result);
JPH_CAPI void JPH_SwingTwistConstraint_SetTargetOrientationCS(JPH_SwingTwistConstraint* constraint, const JPH_Quat* orientation);
JPH_CAPI void JPH_SwingTwistConstraint_GetTargetOrientationCS(const JPH_SwingTwistConstraint* constraint, JPH_Quat* result);
JPH_CAPI void JPH_SwingTwistConstraint_SetSwingMotorSettings(JPH_SwingTwistConstraint* constraint, const JPH_MotorSettings* settings);
JPH_CAPI void JPH_SwingTwistConstraint_GetSwingMotorSettings(const JPH_SwingTwistConstraint* constraint, JPH_MotorSettings* result);
JPH_CAPI void JPH_SwingTwistConstraint_SetTwistMotorSettings(JPH_SwingTwistConstraint* constraint, const JPH_MotorSettings* settings);
JPH_CAPI void JPH_SwingTwistConstraint_GetTwistMotorSettings(const JPH_SwingTwistConstraint* constraint, JPH_MotorSettings* result);
JPH_CAPI void JPH_SwingTwistConstraint_SetMaxFrictionTorque(JPH_SwingTwistConstraint* constraint, float frictionTorque);
JPH_CAPI float JPH_SwingTwistConstraint_GetMaxFrictionTorque(const JPH_SwingTwistConstraint* constraint);
JPH_CAPI float JPH_SwingTwistConstraint_GetPlaneHalfConeAngle(const JPH_SwingTwistConstraint* constraint);
JPH_CAPI float JPH_SwingTwistConstraint_GetTwistMinAngle(const JPH_SwingTwistConstraint* constraint);
JPH_CAPI float JPH_SwingTwistConstraint_GetTwistMaxAngle(const JPH_SwingTwistConstraint* constraint);

/* HingeConstraint */
JPH_CAPI void JPH_HingeConstraint_SetTargetOrientationBS(JPH_HingeConstraint* constraint, const JPH_Quat* orientation);

/* SixDOFConstraint */
JPH_CAPI void JPH_SixDOFConstraint_SetMotorSettings(JPH_SixDOFConstraint* constraint, JPH_SixDOFConstraintAxis axis, const JPH_MotorSettings* settings);

/* PathConstraintPath */
/* A path for a path constraint (JPH::PathConstraintPath). JPH_PathConstraintPathHermite_Create returns
   it holding one reference, which JPH_PathConstraintPath_Destroy releases; the path constraint settings
   and every path constraint created from them take their own reference (RefConst). */
typedef struct JPH_PathConstraintPath JPH_PathConstraintPath; /* JPH::PathConstraintPath */

JPH_CAPI void JPH_PathConstraintPath_Destroy(JPH_PathConstraintPath* path);
JPH_CAPI void JPH_PathConstraintPath_SetIsLooping(JPH_PathConstraintPath* path, bool isLooping);
JPH_CAPI bool JPH_PathConstraintPath_IsLooping(const JPH_PathConstraintPath* path);
JPH_CAPI float JPH_PathConstraintPath_GetPathMaxFraction(const JPH_PathConstraintPath* path);
JPH_CAPI float JPH_PathConstraintPath_GetClosestPoint(const JPH_PathConstraintPath* path, const JPH_Vec3* position, float fractionHint);

/* A JPH::PathConstraintPathHermite, as its JPH::PathConstraintPath base. */
JPH_CAPI JPH_PathConstraintPath* JPH_PathConstraintPathHermite_Create(void);
/* path must come from JPH_PathConstraintPathHermite_Create. */
JPH_CAPI void JPH_PathConstraintPathHermite_AddPoint(JPH_PathConstraintPath* path, const JPH_Vec3* position, const JPH_Vec3* tangent, const JPH_Vec3* normal);

/* PathConstraint */
typedef enum JPH_PathRotationConstraintType {
	JPH_PathRotationConstraintType_Free = 0,
	JPH_PathRotationConstraintType_ConstrainAroundTangent = 1,
	JPH_PathRotationConstraintType_ConstrainAroundNormal = 2,
	JPH_PathRotationConstraintType_ConstrainAroundBinormal = 3,
	JPH_PathRotationConstraintType_ConstrainToPath = 4,
	JPH_PathRotationConstraintType_FullyConstrained = 5,

	_JPH_PathRotationConstraintType_Count,
	_JPH_PathRotationConstraintType_Force32 = 0x7fffffff
} JPH_PathRotationConstraintType;

typedef struct JPH_PathConstraintSettings {
	JPH_ConstraintSettings			base;    /* Inherits JPH_ConstraintSettings */

	const JPH_PathConstraintPath*	path;
	JPH_Vec3						pathPosition;
	JPH_Quat						pathRotation;
	float							pathFraction;
	float							maxFrictionForce;
	JPH_PathRotationConstraintType	rotationConstraintType;
	JPH_MotorSettings				positionMotorSettings;
} JPH_PathConstraintSettings;

typedef struct JPH_PathConstraint JPH_PathConstraint; /* JPH::PathConstraint */

/* Jolt's defaults, with a null path. */
JPH_CAPI void JPH_PathConstraintSettings_Init(JPH_PathConstraintSettings* settings);
JPH_CAPI JPH_PathConstraint* JPH_PathConstraint_Create(const JPH_PathConstraintSettings* settings, JPH_Body* body1, JPH_Body* body2);
/* The constraint's path, borrowed from the constraint. */
JPH_CAPI const JPH_PathConstraintPath* JPH_PathConstraint_GetPath(const JPH_PathConstraint* constraint);
JPH_CAPI float JPH_PathConstraint_GetPathFraction(const JPH_PathConstraint* constraint);
JPH_CAPI void JPH_PathConstraint_SetMaxFrictionForce(JPH_PathConstraint* constraint, float frictionForce);
JPH_CAPI float JPH_PathConstraint_GetMaxFrictionForce(const JPH_PathConstraint* constraint);
JPH_CAPI void JPH_PathConstraint_SetPositionMotorSettings(JPH_PathConstraint* constraint, const JPH_MotorSettings* settings);
JPH_CAPI void JPH_PathConstraint_GetPositionMotorSettings(const JPH_PathConstraint* constraint, JPH_MotorSettings* result);
JPH_CAPI void JPH_PathConstraint_SetPositionMotorState(JPH_PathConstraint* constraint, JPH_MotorState state);
JPH_CAPI JPH_MotorState JPH_PathConstraint_GetPositionMotorState(const JPH_PathConstraint* constraint);
JPH_CAPI void JPH_PathConstraint_SetTargetVelocity(JPH_PathConstraint* constraint, float velocity);
JPH_CAPI float JPH_PathConstraint_GetTargetVelocity(const JPH_PathConstraint* constraint);
JPH_CAPI void JPH_PathConstraint_SetTargetPathFraction(JPH_PathConstraint* constraint, float fraction);
JPH_CAPI float JPH_PathConstraint_GetTargetPathFraction(const JPH_PathConstraint* constraint);
JPH_CAPI void JPH_PathConstraint_GetTotalLambdaPosition(const JPH_PathConstraint* constraint, float result[2]);
JPH_CAPI float JPH_PathConstraint_GetTotalLambdaPositionLimits(const JPH_PathConstraint* constraint);
JPH_CAPI float JPH_PathConstraint_GetTotalLambdaMotor(const JPH_PathConstraint* constraint);
JPH_CAPI void JPH_PathConstraint_GetTotalLambdaRotationHinge(const JPH_PathConstraint* constraint, float result[2]);
JPH_CAPI void JPH_PathConstraint_GetTotalLambdaRotation(const JPH_PathConstraint* constraint, JPH_Vec3* result);

/* PulleyConstraint */
typedef struct JPH_PulleyConstraintSettings {
	JPH_ConstraintSettings		base;    /* Inherits JPH_ConstraintSettings */

	JPH_ConstraintSpace			space;
	JPH_RVec3					bodyPoint1;
	JPH_RVec3					fixedPoint1;
	JPH_RVec3					bodyPoint2;
	JPH_RVec3					fixedPoint2;
	float						ratio;
	float						minLength;
	float						maxLength;
} JPH_PulleyConstraintSettings;

typedef struct JPH_PulleyConstraint JPH_PulleyConstraint; /* JPH::PulleyConstraint */

JPH_CAPI void JPH_PulleyConstraintSettings_Init(JPH_PulleyConstraintSettings* settings);
JPH_CAPI JPH_PulleyConstraint* JPH_PulleyConstraint_Create(const JPH_PulleyConstraintSettings* settings, JPH_Body* body1, JPH_Body* body2);
/* The body points come back relative to the centres of mass (LocalToBodyCOM), the fixed points in
   world space, and the lengths as resolved when the constraint was created. */
JPH_CAPI void JPH_PulleyConstraint_GetSettings(const JPH_PulleyConstraint* constraint, JPH_PulleyConstraintSettings* settings);
JPH_CAPI void JPH_PulleyConstraint_SetLength(JPH_PulleyConstraint* constraint, float minLength, float maxLength);
JPH_CAPI float JPH_PulleyConstraint_GetMinLength(const JPH_PulleyConstraint* constraint);
JPH_CAPI float JPH_PulleyConstraint_GetMaxLength(const JPH_PulleyConstraint* constraint);
JPH_CAPI float JPH_PulleyConstraint_GetCurrentLength(const JPH_PulleyConstraint* constraint);
JPH_CAPI float JPH_PulleyConstraint_GetTotalLambdaPosition(const JPH_PulleyConstraint* constraint);

/* RackAndPinionConstraint */
typedef struct JPH_RackAndPinionConstraintSettings {
	JPH_ConstraintSettings		base;    /* Inherits JPH_ConstraintSettings */

	JPH_ConstraintSpace			space;
	JPH_Vec3					hingeAxis;
	JPH_Vec3					sliderAxis;
	float						ratio;
} JPH_RackAndPinionConstraintSettings;

typedef struct JPH_RackAndPinionConstraint JPH_RackAndPinionConstraint; /* JPH::RackAndPinionConstraint */

JPH_CAPI void JPH_RackAndPinionConstraintSettings_Init(JPH_RackAndPinionConstraintSettings* settings);
JPH_CAPI JPH_RackAndPinionConstraint* JPH_RackAndPinionConstraint_Create(const JPH_RackAndPinionConstraintSettings* settings, JPH_Body* body1, JPH_Body* body2);
/* pinion must be a hinge and rack a slider constraint; the constraint takes a reference to each. */
JPH_CAPI void JPH_RackAndPinionConstraint_SetConstraints(JPH_RackAndPinionConstraint* constraint, const JPH_Constraint* pinion, const JPH_Constraint* rack);
JPH_CAPI float JPH_RackAndPinionConstraint_GetTotalLambda(const JPH_RackAndPinionConstraint* constraint);

/* SoftBodySharedSettings */
typedef enum JPH_SoftBodyLRAType {
	JPH_SoftBodyLRAType_None = 0,
	JPH_SoftBodyLRAType_EuclideanDistance = 1,
	JPH_SoftBodyLRAType_GeodesicDistance = 2,

	_JPH_SoftBodyLRAType_Force32 = 0x7fffffff
} JPH_SoftBodyLRAType;

/* JPH::SoftBodySharedSettings::VertexAttributes, copied field by field (not a layout mirror). */
typedef struct JPH_SoftBodyVertexAttributes {
	float					compliance;
	float					shearCompliance;
	float					bendCompliance;				/* FLT_MAX: no bend constraint */
	JPH_SoftBodyLRAType		lraType;
	float					lraMaxDistanceMultiplier;
} JPH_SoftBodyVertexAttributes;

/* Jolt's defaults: compliance 0, shear compliance 0, bend compliance FLT_MAX, no LRA, multiplier 1. */
JPH_CAPI void JPH_SoftBodyVertexAttributes_Init(JPH_SoftBodyVertexAttributes* attributes);
/* Like JPH_SoftBodySharedSettings_CreateConstraints, with every attribute field and per-vertex values:
   vertex v uses attributes[min(v, attributeCount - 1)], so attributeCount must be at least 1. Replaces
   the edge constraints already in the settings; Jolt's default angle tolerance (8 degrees). */
JPH_CAPI void JPH_SoftBodySharedSettings_CreateConstraints2(JPH_SoftBodySharedSettings* settings,
	const JPH_SoftBodyVertexAttributes* attributes, uint32_t attributeCount, JPH_SoftBodyBendType bendType);
JPH_CAPI void JPH_SoftBodySharedSettings_AddEdgeConstraint(JPH_SoftBodySharedSettings* settings, uint32_t vertex1, uint32_t vertex2, float compliance);
JPH_CAPI void JPH_SoftBodySharedSettings_AddDihedralBendConstraint(JPH_SoftBodySharedSettings* settings, uint32_t vertex1, uint32_t vertex2, uint32_t vertex3, uint32_t vertex4, float compliance);
JPH_CAPI void JPH_SoftBodySharedSettings_AddVolumeConstraint(JPH_SoftBodySharedSettings* settings, uint32_t vertex1, uint32_t vertex2, uint32_t vertex3, uint32_t vertex4, float compliance);
JPH_CAPI void JPH_SoftBodySharedSettings_CalculateEdgeLengths(JPH_SoftBodySharedSettings* settings);
JPH_CAPI void JPH_SoftBodySharedSettings_CalculateBendConstraintConstants(JPH_SoftBodySharedSettings* settings);
JPH_CAPI void JPH_SoftBodySharedSettings_CalculateVolumeConstraintVolumes(JPH_SoftBodySharedSettings* settings);
JPH_CAPI uint32_t JPH_SoftBodySharedSettings_GetEdgeConstraintCount(const JPH_SoftBodySharedSettings* settings);
JPH_CAPI uint32_t JPH_SoftBodySharedSettings_GetDihedralBendConstraintCount(const JPH_SoftBodySharedSettings* settings);
JPH_CAPI uint32_t JPH_SoftBodySharedSettings_GetVolumeConstraintCount(const JPH_SoftBodySharedSettings* settings);
JPH_CAPI uint32_t JPH_SoftBodySharedSettings_GetLRAConstraintCount(const JPH_SoftBodySharedSettings* settings);

/* Soft body Body */
/* Does nothing unless body is a soft body. Copies min(count, vertex count) vertices: positions in world
   space with Real precision, velocities in world space, inverse masses; any output may be null. */
JPH_CAPI void JPH_Body_GetSoftBodyVertices(const JPH_Body* body, JPH_RVec3* outPositions,
	JPH_Vec3* outVelocities, float* outInvMasses, uint32_t count);
/* Does nothing unless body is a soft body. Copies min(count, vertex count) vertex positions as Jolt
   stores them: relative to the centre of mass, in the body frame. */
JPH_CAPI void JPH_Body_GetSoftBodyVertexLocalPositions(const JPH_Body* body, JPH_Vec3* outPositions, uint32_t count);
/* Does nothing unless body is a soft body and index < vertex count; velocity in world space. */
JPH_CAPI void JPH_Body_SetSoftBodyVertexVelocity(JPH_Body* body, uint32_t index, const JPH_Vec3* velocity);
/* Does nothing unless body is a soft body and index < vertex count; then recomputes the body's mass and
   inertia (JPH::SoftBodyMotionProperties::CalculateMassAndInertia). */
JPH_CAPI void JPH_Body_SetSoftBodyVertexInvMass(JPH_Body* body, uint32_t index, float invMass);

/* Soft body skinning */
/* JPH::SoftBodySharedSettings::SkinWeight, copied field by field (not a layout mirror). */
typedef struct JPH_SoftBodySkinWeight {
	uint32_t invBindIndex;
	float weight;
} JPH_SoftBodySkinWeight;

/* JPH::SoftBodySharedSettings::Skinned without mNormalInfo, copied field by field (not a layout
   mirror). The weights end at the first zero weight, as in Jolt, and should add up to 1. maxDistance
   FLT_MAX is unlimited and 0 hard-skins the vertex. Jolt applies the back stop only while
   backStopDistance is below maxDistance times the body's skinned max distance multiplier, which is
   infinite for FLT_MAX times a multiplier above 1: INFINITY turns the back stop off for every
   multiplier (Jolt's default FLT_MAX does not; its default radius is 40). */
typedef struct JPH_SoftBodySkinnedConstraint {
	uint32_t vertex;
	JPH_SoftBodySkinWeight weights[4];
	float maxDistance;
	float backStopDistance;
	float backStopRadius;
} JPH_SoftBodySkinnedConstraint;

/* Appends a JPH::SoftBodySharedSettings::InvBind: invBind takes a vertex in its bind pose (the
   settings' vertex position) to the local space of joint jointIndex. invBind may be 4-aligned.
   Returns false and adds nothing when jointIndex is UINT32_MAX, whose joint count does not fit in a
   uint32_t. */
JPH_CAPI bool JPH_SoftBodySharedSettings_AddInvBind(JPH_SoftBodySharedSettings* settings, uint32_t jointIndex, const JPH_Mat4* invBind);
/* Appends a skinned constraint. Add it after its vertex and the inverse binds it names: returns false
   and adds nothing when vertex >= the vertex count or a weight before the first zero weight names
   invBindIndex >= the inverse bind count. */
JPH_CAPI bool JPH_SoftBodySharedSettings_AddSkinnedConstraint(JPH_SoftBodySharedSettings* settings, const JPH_SoftBodySkinnedConstraint* constraint);
/* Jolt's CalculateSkinnedConstraintNormals, from the faces whose three vertices are all skinned. Call
   it after the faces and skinned constraints are added and before JPH_SoftBodySharedSettings_Optimize.
   The caller keeps every skinned vertex below 256 such faces and their total below 2^24 (Jolt
   asserts). */
JPH_CAPI void JPH_SoftBodySharedSettings_CalculateSkinnedConstraintNormals(JPH_SoftBodySharedSettings* settings);
JPH_CAPI uint32_t JPH_SoftBodySharedSettings_GetInvBindCount(const JPH_SoftBodySharedSettings* settings);
JPH_CAPI uint32_t JPH_SoftBodySharedSettings_GetSkinnedConstraintCount(const JPH_SoftBodySharedSettings* settings);

/* The number of joint matrices JPH_Body_SkinSoftBodyVertices needs: 1 + the largest joint index of the
   inverse binds. 0 for a rigid body or a soft body without skinned constraints. */
JPH_CAPI uint32_t JPH_Body_GetSoftBodySkinJointCount(const JPH_Body* body);
/* Jolt's SoftBodyMotionProperties::SkinVertices with the body's centre of mass transform and a temp
   allocator of its own. jointMatrices holds jointCount model matrices relative to
   Body::GetCenterOfMassTransform() and may be 4-aligned. With hardSkinAll the skinned vertices move to
   their skinned positions and their velocities become zero. Jolt does not call this during a step: the
   caller holds the body write lock while no step runs, and the next step uses the result. Returns
   false and changes nothing when body is null, rigid or has no skinned constraints, jointMatrices is
   null, jointCount is not above every inverse bind's joint index, or the inverse binds do not fit in
   UINT32_MAX bytes of matrices. Nothing transfers ownership. */
JPH_CAPI bool JPH_Body_SkinSoftBodyVertices(JPH_Body* body, const JPH_Mat4* jointMatrices, uint32_t jointCount, bool hardSkinAll);
/* Jolt's multiplier of every skinned constraint's maxDistance (0 hard-skins all of them). 1 for a body
   that is not a soft body. */
JPH_CAPI float JPH_Body_GetSoftBodySkinnedMaxDistanceMultiplier(const JPH_Body* body);
/* Sets the multiplier; returns false and changes nothing for a body that is not a soft body. The
   caller passes a finite, non-negative value and holds the body write lock. */
JPH_CAPI bool JPH_Body_SetSoftBodySkinnedMaxDistanceMultiplier(JPH_Body* body, float multiplier);

/* Body (buoyancy) */
/* Jolt's Body::GetSubmergedVolume for a fluid surface through surfacePosition with surfaceNormal pointing
   out of the fluid: total volume, submerged volume and the submerged volume's centre relative to the
   centre of mass, in world space. Writes 0 to every output first, then returns false for a soft body and
   for a shape that must be static (meshes, heightfields, planes and anything holding one), for which
   Jolt has no volume; otherwise true. */
JPH_CAPI bool JPH_Body_GetSubmergedVolume(const JPH_Body* body, const JPH_RVec3* surfacePosition, const JPH_Vec3* surfaceNormal,
	float* outTotalVolume, float* outSubmergedVolume, JPH_Vec3* outRelativeCenterOfBuoyancy);
/* Jolt's Body::ApplyBuoyancyImpulse overload that takes the volumes (usually from
   JPH_Body_GetSubmergedVolume). Returns false and changes nothing unless body is a dynamic rigid body
   with a positive inverse mass and totalVolume > 0; otherwise returns whether submergedVolume > 0.
   Neither wakes the body nor clamps its velocities. The caller passes finite values whose products stay
   finite in float. */
JPH_CAPI bool JPH_Body_ApplyBuoyancyImpulse2(JPH_Body* body, float totalVolume, float submergedVolume,
	const JPH_Vec3* relativeCenterOfBuoyancy, float buoyancy, float linearDrag, float angularDrag,
	const JPH_Vec3* fluidVelocity, const JPH_Vec3* gravity, float deltaTime);

/* PhysicsMaterial */
/* A JPH::PhysicsMaterialSimple that also carries userData. Returns one reference, released by
   JPH_PhysicsMaterial_Destroy. Not serializable. */
JPH_CAPI JPH_PhysicsMaterial* JPH_PhysicsMaterial_Create2(const char* name, uint32_t color, uint64_t userData);
/* true and *userData set for a material made by JPH_PhysicsMaterial_Create2; false, *userData
   untouched, for null, JPH::PhysicsMaterial::sDefault and every other material. */
JPH_CAPI bool JPH_PhysicsMaterial_GetUserData(const JPH_PhysicsMaterial* material, uint64_t* userData);
/* Adds one reference to material and returns it for the caller, who releases it with
   JPH_PhysicsMaterial_Destroy. Returns null for null. Works for every material, also one that
   holds references only from Jolt objects (shapes, character contacts). */
JPH_CAPI JPH_PhysicsMaterial* JPH_PhysicsMaterial_AddRef(const JPH_PhysicsMaterial* material);
/* Jolt's RefTarget::GetRefCount of material: the references every owner holds together. 0 for
   null. For tests and diagnostics; another thread may change it at any time. */
JPH_CAPI uint32_t JPH_PhysicsMaterial_GetRefCount(const JPH_PhysicsMaterial* material);
/* Sets JPH::ConvexShapeSettings::mMaterial (null selects the default material); the settings keep their
   own reference. Call it before the first CreateShape: Jolt caches the created shape. */
JPH_CAPI void JPH_ConvexShapeSettings_SetMaterial(JPH_ConvexShapeSettings* settings, const JPH_PhysicsMaterial* material);
/* Like JPH_HeightFieldShapeSettings_Create, with materials: materialIndices holds (sampleCount - 1)^2
   indices into materials[0..materialCount). Returns null when materialCount is 0 or materialIndices or
   materials is null. Each list entry keeps its own reference. Jolt refuses more than 256 materials and
   indices beyond the list when the shape is created. */
JPH_CAPI JPH_HeightFieldShapeSettings* JPH_HeightFieldShapeSettings_Create2(const float* samples, const JPH_Vec3* offset, const JPH_Vec3* scale, uint32_t sampleCount, const uint8_t* materialIndices, const JPH_PhysicsMaterial* const* materials, uint32_t materialCount);
/* Like JPH_MeshShapeSettings_Create2, with materials: each triangle's materialIndex addresses
   materials[0..materialCount); without a list (materials null or materialCount 0) every materialIndex
   must be 0. Returns settings holding one reference; each list entry keeps its own reference.
   Preconditions, not checked: every vertex index is below vertexCount, and vertexCount and
   triangleCount are at most INT32_MAX. Jolt's constructor removes degenerate and duplicate triangles
   (MeshShapeSettings::Sanitize) and reads vertices[index] without a range check. */
JPH_CAPI JPH_MeshShapeSettings* JPH_MeshShapeSettings_Create3(const JPH_Vec3* vertices, uint32_t vertexCount, const JPH_IndexedTriangle* triangles, uint32_t triangleCount, const JPH_PhysicsMaterial* const* materials, uint32_t materialCount);
/* Number of triangles the settings hold, after the constructor removed degenerate and duplicate ones. */
JPH_CAPI uint32_t JPH_MeshShapeSettings_GetTriangleCount(const JPH_MeshShapeSettings* settings);

/* ShapeSettings */
/* Runs the settings' Create once and returns the shape holding one reference for the caller, or null
   when Jolt refused the settings. When error is not null and errorCapacity is not 0, error receives
   Jolt's message on failure (at most errorCapacity - 1 bytes, always NUL-terminated) and "" on success.
   The settings keep the cached result either way (Jolt behaviour), so a second call returns the same
   shape with one more reference. Works for every joltc shape settings type. */
JPH_CAPI JPH_Shape* JPH_ShapeSettings_CreateShapeWithError(const JPH_ShapeSettings* settings, char* error, uint32_t errorCapacity);

/* Shape */
/* The triangles of a leaf shape's surface (Jolt's GetTrianglesStart/Next over everything, unscaled),
   in the shape's centre of mass space, three vertices each. Returns the triangle count; writes the
   first min(count, maxTriangles) triangles to vertices, which holds 3 * maxTriangles vertices and may
   be null when maxTriangles is 0. Meshes and heightfields give their stored (quantized) triangles;
   convex shapes give a triangulation of their surface. Compound and decorated shapes (scaled,
   rotated-translated, offset centre of mass) return 0 without reading their parts: Jolt reaches their
   leaves through CollectTransformedShapes, and their GetTrianglesStart asserts. */
JPH_CAPI uint32_t JPH_Shape_GetTriangles(const JPH_Shape* shape, JPH_Vec3* vertices, uint32_t maxTriangles);

/* SubShapeIDPair */
/* pair is the JPH::SubShapeIDPair joltc passes to JPH_ContactListener_Procs::OnContactRemoved, read
   through Jolt's accessors; callers never read JPH_SubShapeIDPair's fields. */
JPH_CAPI JPH_BodyID JPH_SubShapeIDPair_GetBody1ID(const JPH_SubShapeIDPair* pair);
JPH_CAPI JPH_SubShapeID JPH_SubShapeIDPair_GetSubShapeID1(const JPH_SubShapeIDPair* pair);
JPH_CAPI JPH_BodyID JPH_SubShapeIDPair_GetBody2ID(const JPH_SubShapeIDPair* pair);
JPH_CAPI JPH_SubShapeID JPH_SubShapeIDPair_GetSubShapeID2(const JPH_SubShapeIDPair* pair);

/* ContactListener2 */
/* A JPH::ContactListener calling one process-global table of joltc's JPH_ContactListener_Procs.
   OnContactValidate receives a JPH_CollideShapeResult filled field by field without faces
   (shape1FaceCount and shape2FaceCount 0, shape1Faces and shape2Faces null): nothing is allocated
   per call. A null proc accepts (AcceptAllContactsForThisBodyPair) or does nothing. Added and
   persisted copy JPH::ContactSettings field by field as joltc does; the manifold and the removed pair
   are the Jolt objects cast, read with joltc's JPH_ContactManifold_* and the JPH_SubShapeIDPair_*
   getters above. Not ref-counted: detach it, then delete it with JPH_ContactListener2_Destroy. */
typedef struct JPH_ContactListener2 JPH_ContactListener2;

JPH_CAPI void JPH_ContactListener2_SetProcs(const JPH_ContactListener_Procs* procs);
JPH_CAPI JPH_ContactListener2* JPH_ContactListener2_Create(void* userData);
JPH_CAPI void JPH_ContactListener2_Destroy(JPH_ContactListener2* listener);
/* listener may be null; the system does not own it. */
JPH_CAPI void JPH_PhysicsSystem_SetContactListener2(JPH_PhysicsSystem* system, JPH_ContactListener2* listener);

/* SoftBodyContactListener */
typedef enum JPH_SoftBodyValidateResult {
	JPH_SoftBodyValidateResult_AcceptContact = 0,
	JPH_SoftBodyValidateResult_RejectContact = 1,

	_JPH_SoftBodyValidateResult_Force32 = 0x7fffffff
} JPH_SoftBodyValidateResult;

/* JPH::SoftBodyContactSettings, copied field by field (not a layout mirror). */
typedef struct JPH_SoftBodyContactSettings {
	float invMassScale1;
	float invMassScale2;
	float invInertiaScale2;
	bool isSensor;
} JPH_SoftBodyContactSettings;

/* JPH::SoftBodyManifold, valid only during OnSoftBodyContactAdded. */
typedef struct JPH_SoftBodyManifold JPH_SoftBodyManifold;
typedef struct JPH_SoftBodyContactListener JPH_SoftBodyContactListener;

/* One process-global table, as for joltc's other listeners. A null proc accepts the contact or does
   nothing. Both are called while Jolt holds the bodies, so they must not lock bodies. */
typedef struct JPH_SoftBodyContactListener_Procs {
	JPH_SoftBodyValidateResult(JPH_API_CALL* OnSoftBodyContactValidate)(void* userData,
		const JPH_Body* softBody,
		const JPH_Body* otherBody,
		JPH_SoftBodyContactSettings* settings);

	void(JPH_API_CALL* OnSoftBodyContactAdded)(void* userData,
		const JPH_Body* softBody,
		const JPH_SoftBodyManifold* manifold);
} JPH_SoftBodyContactListener_Procs;

JPH_CAPI void JPH_SoftBodyContactListener_SetProcs(const JPH_SoftBodyContactListener_Procs* procs);
JPH_CAPI JPH_SoftBodyContactListener* JPH_SoftBodyContactListener_Create(void* userData);
JPH_CAPI void JPH_SoftBodyContactListener_Destroy(JPH_SoftBodyContactListener* listener);
/* listener may be null; the system does not own it. */
JPH_CAPI void JPH_PhysicsSystem_SetSoftBodyContactListener(JPH_PhysicsSystem* system, JPH_SoftBodyContactListener* listener);

/* SoftBodyManifold. An index at or beyond the count gives false, JPH_BodyID 0xFFFFFFFF (invalid) or
   leaves the output untouched. The contact point and normal are in the soft body's centre of mass
   frame and are written only for a vertex that has a contact; the normal is Jolt's GetContactNormal,
   minus the vertex's collision plane normal. */
JPH_CAPI uint32_t JPH_SoftBodyManifold_GetVertexCount(const JPH_SoftBodyManifold* manifold);
JPH_CAPI bool JPH_SoftBodyManifold_HasContact(const JPH_SoftBodyManifold* manifold, uint32_t index);
JPH_CAPI bool JPH_SoftBodyManifold_GetLocalContactPoint(const JPH_SoftBodyManifold* manifold, uint32_t index, JPH_Vec3* result);
JPH_CAPI bool JPH_SoftBodyManifold_GetContactNormal(const JPH_SoftBodyManifold* manifold, uint32_t index, JPH_Vec3* result);
JPH_CAPI JPH_BodyID JPH_SoftBodyManifold_GetContactBodyID(const JPH_SoftBodyManifold* manifold, uint32_t index);
JPH_CAPI uint32_t JPH_SoftBodyManifold_GetNumSensorContacts(const JPH_SoftBodyManifold* manifold);
JPH_CAPI JPH_BodyID JPH_SoftBodyManifold_GetSensorContactBodyID(const JPH_SoftBodyManifold* manifold, uint32_t index);

/* SkeletonMapper */
/* Like joltc's JPH_SkeletonMapper_Initialize, _LockAllTranslations, _LockTranslations, _Map and
   _MapReverse, for JPH_Mat4 arrays of any alignment: every array is copied through 16-byte aligned
   storage. Outputs are written only on success and only after every input was copied, so an output
   may alias an input. Each function returns false, and changes and writes nothing, for an input that
   would make Jolt assert or read out of bounds; the cases are listed per function. The mapper keeps
   no pointer to the skeletons or poses. Skeleton 1 is the ragdoll (low detail) skeleton, skeleton 2
   the animation skeleton; poses are matrices whose bottom row is exactly (0, 0, 0, 1).
   Preconditions, not checked: live handles, arrays of at least count elements (lockedTranslations:
   count2 bools), count2 small enough for the bool array LockAllTranslations allocates on the stack,
   and the same skeleton 2 for every call on one mapper (indices are range-checked, their meaning is
   not). */
/* false unless 1 <= count1 <= count2, the counts equal the skeletons' joint counts, both skeletons
   list parents first, every matrix is finite with that bottom row, and the mapper was never
   initialised. Joints are mapped by name. */
JPH_CAPI bool JPH_SkeletonMapper_Initialize2(JPH_SkeletonMapper* mapper, const JPH_Skeleton* skeleton1, const JPH_Mat4* neutralPose1, uint32_t count1, const JPH_Skeleton* skeleton2, const JPH_Mat4* neutralPose2, uint32_t count2);
/* false unless count2 is skeleton 2's joint count, the skeleton lists parents first, every matrix is
   finite with that bottom row, the mapper has a mapping whose skeleton 2 joint is below count2, and
   it has no locked translations yet. */
JPH_CAPI bool JPH_SkeletonMapper_LockAllTranslations2(JPH_SkeletonMapper* mapper, const JPH_Skeleton* skeleton2, const JPH_Mat4* neutralPose2, uint32_t count2);
/* As JPH_SkeletonMapper_LockAllTranslations2, with these joints of skeleton 2 instead of every
   descendant of the first mapped joint; false also when a root (a joint without parent) is flagged,
   because Map sets a locked joint from its parent, and when the mapper was never initialised. */
JPH_CAPI bool JPH_SkeletonMapper_LockTranslations2(JPH_SkeletonMapper* mapper, const JPH_Skeleton* skeleton2, const bool* lockedTranslations, const JPH_Mat4* neutralPose2, uint32_t count2);
/* Pose 1 (model space) and pose 2 (local space) to pose 2 (model space). *outDegenerateJoint1 (may be
   null) is -1, or the skeleton 1 joint at the end of a chain this call refused. false unless the
   mapper has a mapping, every index it stores is below the counts and every stored transform is
   finite with that bottom row, each skeleton 2 joint is mapped at most once, every input matrix is
   finite with that bottom row, and every chain is usable: its desired direction (between the skeleton
   1 joints) is exactly zero or at least 1 mm long, its actual direction (along the skeleton 2 chain)
   is at least 1 mm long after an allowance for rounding, both are at most 1e18 long (the actual one
   with its allowance), their lengths' product is at most 4e18, and a norm bound on the chain's
   products stays below 1e30. The derivation is in docs/limits.md of the oxijolt repository, section
   "Skeleton mapper chains". */
JPH_CAPI bool JPH_SkeletonMapper_Map2(const JPH_SkeletonMapper* mapper, const JPH_Mat4* pose1ModelSpace, uint32_t count1, const JPH_Mat4* pose2LocalSpace, uint32_t count2, JPH_Mat4* outPose2ModelSpace, int* outDegenerateJoint1);
/* Pose 2 (model space) to pose 1 (model space) through the direct mappings; a skeleton 1 joint
   without a mapping comes back as identity. false unless the mapper has a mapping, every mapping is
   below the counts with finite transforms of that bottom row, and every input matrix is finite with
   that bottom row. */
JPH_CAPI bool JPH_SkeletonMapper_MapReverse2(const JPH_SkeletonMapper* mapper, const JPH_Mat4* pose2ModelSpace, uint32_t count2, JPH_Mat4* outPose1ModelSpace, uint32_t count1);

/* Shape binary state */
/* A shape saved with its children and materials (Jolt's cooked shape data): one record per shape,
   children before their parents, each record holding the sub-shape type, child and material indices
   and the bytes of Jolt's Shape::SaveBinaryState. The bytes are in the host's byte order and are
   restored only by a build of the same Jolt and joltc commits and the same version below; the
   caller records and checks those, and a checksum if the bytes are stored or sent. */
typedef struct JPH_ShapeBinaryState JPH_ShapeBinaryState; /* a byte array */

/* The version of the record layout; it changes whenever the layout does. */
JPH_CAPI uint32_t JPH_Shape_GetBinaryStateVersion(void);
/* Saves shape with its children and materials. Supported shapes: sphere, box, capsule, tapered
   capsule, cylinder, tapered cylinder, convex hull, static and mutable compound, rotated-translated,
   scaled, offset centre of mass, mesh, heightfield, plane and empty. Supported materials: none (the
   default), JPH::PhysicsMaterial::sDefault, a JPH::PhysicsMaterialSimple (JPH_PhysicsMaterial_Create)
   and a JPH_PhysicsMaterial_Create2 material; a shared child or material is saved once. Returns the
   state, destroyed with JPH_ShapeBinaryState_Destroy, and an empty message; or null and a message
   for any other shape or material. error may be null; the message is cut to errorCapacity - 1 bytes. */
JPH_CAPI JPH_ShapeBinaryState* JPH_Shape_SaveBinaryState(const JPH_Shape* shape, char* error, uint32_t errorCapacity);
JPH_CAPI size_t JPH_ShapeBinaryState_GetSize(const JPH_ShapeBinaryState* state);
/* Copies min(size, state size) bytes to data, which may be null when that is 0. */
JPH_CAPI void JPH_ShapeBinaryState_CopyData(const JPH_ShapeBinaryState* state, void* data, size_t size);
JPH_CAPI void JPH_ShapeBinaryState_Destroy(JPH_ShapeBinaryState* state);
/* Restores the bytes of a JPH_ShapeBinaryState. Before Jolt reads a record it checks that every
   length lies inside the data, that the sub-shape type is a supported one and matches Jolt's bytes,
   that child indices name earlier records and material indices existing materials, and that the
   data ends with the last record; after Jolt read a record, that Jolt read exactly its bytes and that
   the child and material counts are the ones the shape takes (decorated shapes one child, compounds
   their sub-shape count, convex shapes and planes one material). Every record but the last must be
   used. Returns the last record's shape holding one reference (release it with JPH_Shape_Destroy)
   and an empty message, or null and a message naming the failed check.
   Precondition, not checked: the bytes inside each record were written by Jolt's SaveBinaryState of
   this build. Jolt does not validate them (array lengths, tree offsets, hull indices), so crafted
   records can make it read and write out of bounds. */
JPH_CAPI JPH_Shape* JPH_Shape_RestoreBinaryState(const void* data, size_t size, char* error, uint32_t errorCapacity);

/* JobSystem */
/* Jolt's JobSystemSingleThreaded: runs each job on the calling thread as soon as it is ready, with
   concurrency 1 and no worker threads. maxJobs is how many jobs may be allocated at once, 0 for
   Jolt's cMaxPhysicsJobs; a smaller value must cover every job alive during one update, or Jolt
   asserts and a release build reads out of bounds. Call after JPH_Init. The caller owns the result
   and destroys it with JPH_JobSystem_Destroy, after the last JPH_PhysicsSystem_Update2 that used it
   returned. Not a reference-counted object; the handle is opaque. */
JPH_CAPI JPH_JobSystem* JPH_JobSystemSingleThreaded_Create(uint32_t maxJobs);

#endif /* JOLT_C_EXT_H_ */
