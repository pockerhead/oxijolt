// CharacterVirtual and CharacterBase settings that change on a live character. Part of this fork's
// joltc additions (see joltc_ext.cpp, which also explains the handle conversions).
// Licensed under the MIT License (MIT), like joltc.

#include <Jolt/Jolt.h>

#include <Jolt/Geometry/Plane.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include "joltc_ext_internal.h"

using joltc_ext::AsCharacterVirtual;
using joltc_ext::AsTempAllocator;
using joltc_ext::ToJolt;
using joltc_ext::ToVec3;

bool JPH_CharacterVirtual_SetShape2(JPH_CharacterVirtual* character, const JPH_Shape* shape,
	float maxPenetrationDepth,
	const JPH_BroadPhaseLayerFilter* broadPhaseLayerFilter, const JPH_ObjectLayerFilter* objectLayerFilter,
	const JPH_BodyFilter* bodyFilter, const JPH_ShapeFilter* shapeFilter, JPH_TempAllocator* tempAllocator)
{
	JPH_ASSERT(character && shape && tempAllocator);

	return AsCharacterVirtual(character)->SetShape(
		reinterpret_cast<const JPH::Shape*>(shape),
		maxPenetrationDepth,
		ToJolt(broadPhaseLayerFilter),
		ToJolt(objectLayerFilter),
		ToJolt(bodyFilter),
		ToJolt(shapeFilter),
		AsTempAllocator(tempAllocator)
	);
}

/* CharacterBase */
void JPH_CharacterBase_GetSupportingVolume(JPH_CharacterBase* character, JPH_Plane* result)
{
	JPH_ASSERT(character && result);

	// JPH::Plane is a SIMD Vec4, so it is copied member by member, never cast.
	const JPH::Plane& plane = reinterpret_cast<const JPH::CharacterBase*>(character)->GetSupportingVolume();
	const JPH::Vec3 normal = plane.GetNormal();
	result->normal = JPH_Vec3{ normal.GetX(), normal.GetY(), normal.GetZ() };
	result->distance = plane.GetConstant();
}

void JPH_CharacterBase_SetSupportingVolume(JPH_CharacterBase* character, const JPH_Plane* value)
{
	JPH_ASSERT(character && value);

	reinterpret_cast<JPH::CharacterBase*>(character)->SetSupportingVolume(JPH::Plane(ToVec3(value->normal), value->distance));
}
