// Body additions: buoyancy in two parts, the submerged volume and Jolt's volume overload of
// ApplyBuoyancyImpulse, so a caller can check the volumes Jolt will use before it applies them; and
// the stored motion quality. Part of this fork's joltc additions (see joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// Handles: a JPH_Body is a reinterpret_cast of the JPH::Body and a JPH_MotionProperties of the
// JPH::MotionProperties, as in joltc.

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/MotionProperties.h>

#include "joltc_ext_internal.h"

namespace
{
	const JPH::Body& AsBody(const JPH_Body* body)
	{
		return *reinterpret_cast<const JPH::Body*>(body);
	}

	JPH::Body& AsBody(JPH_Body* body)
	{
		return *reinterpret_cast<JPH::Body*>(body);
	}

	// JPH_RVec3 is three Reals and JPH::RVec3 a SIMD type, so it is built field by field.
	JPH::RVec3 ToRVec3(const JPH_RVec3& vec)
	{
		return JPH::RVec3(vec.x, vec.y, vec.z);
	}

	void FromVec3(JPH::Vec3Arg vec, JPH_Vec3* result)
	{
		result->x = vec.GetX();
		result->y = vec.GetY();
		result->z = vec.GetZ();
	}
}

bool JPH_Body_GetSubmergedVolume(const JPH_Body* body, const JPH_RVec3* surfacePosition, const JPH_Vec3* surfaceNormal, float* outTotalVolume, float* outSubmergedVolume, JPH_Vec3* outRelativeCenterOfBuoyancy)
{
	*outTotalVolume = 0.0f;
	*outSubmergedVolume = 0.0f;
	FromVec3(JPH::Vec3::sZero(), outRelativeCenterOfBuoyancy);

	// Meshes, heightfields and planes (and shapes containing them) assert "Not supported" in
	// GetSubmergedVolume; every such shape is static-only.
	const JPH::Body& joltBody = AsBody(body);
	if (!joltBody.IsRigidBody() || joltBody.GetShape()->MustBeStatic())
	{
		return false;
	}

	float totalVolume, submergedVolume;
	JPH::Vec3 relativeCenterOfBuoyancy;
	joltBody.GetSubmergedVolume(ToRVec3(*surfacePosition), joltc_ext::ToVec3(*surfaceNormal), totalVolume, submergedVolume, relativeCenterOfBuoyancy);
	*outTotalVolume = totalVolume;
	*outSubmergedVolume = submergedVolume;
	FromVec3(relativeCenterOfBuoyancy, outRelativeCenterOfBuoyancy);
	return true;
}

bool JPH_Body_ApplyBuoyancyImpulse2(JPH_Body* body, float totalVolume, float submergedVolume, const JPH_Vec3* relativeCenterOfBuoyancy, float buoyancy, float linearDrag, float angularDrag, const JPH_Vec3* fluidVelocity, const JPH_Vec3* gravity, float deltaTime)
{
	// Jolt divides by the total volume and by the inverse mass, and its velocity setters assert a
	// dynamic body.
	JPH::Body& joltBody = AsBody(body);
	if (!joltBody.IsRigidBody() || !joltBody.IsDynamic() || !(joltBody.GetMotionProperties()->GetInverseMass() > 0.0f) || !(totalVolume > 0.0f))
	{
		return false;
	}

	return joltBody.ApplyBuoyancyImpulse(
		totalVolume,
		submergedVolume,
		joltc_ext::ToVec3(*relativeCenterOfBuoyancy),
		buoyancy,
		linearDrag,
		angularDrag,
		joltc_ext::ToVec3(*fluidVelocity),
		joltc_ext::ToVec3(*gravity),
		deltaTime);
}

JPH_MotionQuality JPH_MotionProperties_GetMotionQuality(const JPH_MotionProperties* properties)
{
	return static_cast<JPH_MotionQuality>(reinterpret_cast<const JPH::MotionProperties*>(properties)->GetMotionQuality());
}
