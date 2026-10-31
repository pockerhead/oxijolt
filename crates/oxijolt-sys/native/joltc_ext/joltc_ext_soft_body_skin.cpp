// Soft body skinning: inverse binds and skinned constraints in the shared settings, and skinning a
// live soft body to joint matrices. Part of this fork's joltc additions (see joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// Handles: a JPH_Body is a reinterpret_cast of the JPH::Body and a JPH_SoftBodySharedSettings of the
// JPH::SoftBodySharedSettings, as in joltc.

#include <Jolt/Jolt.h>

#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "joltc_ext_internal.h"

using joltc_ext::AsSoftBodyMotionProperties;
using joltc_ext::AsSoftBodySharedSettings;

namespace
{
	using Skinned = JPH::SoftBodySharedSettings::Skinned;

	static_assert(Skinned::cMaxSkinWeights == 4, "JPH_SoftBodySkinnedConstraint holds four weights");

	// JPH_Mat4 may be 4-aligned and JPH::Mat44 is 16-aligned, so the matrix is copied, never cast.
	JPH::Mat44 ToMat44(const JPH_Mat4& matrix)
	{
		JPH::Mat44 result;
		std::memcpy(&result, &matrix, sizeof(JPH_Mat4));
		return result;
	}

	// Whether every weight before the first zero weight names an existing inverse bind.
	bool HasValidWeights(const JPH_SoftBodySkinnedConstraint& constraint, size_t invBindCount)
	{
		for (const JPH_SoftBodySkinWeight& weight : constraint.weights)
		{
			if (weight.weight == 0.0f)
				return true;
			if (weight.invBindIndex >= invBindCount)
				return false;
		}
		return true;
	}

	// 1 + the largest joint index of the inverse binds, 0 without inverse binds. AddInvBind refuses
	// the joint index UINT32_MAX, so the sum does not wrap.
	uint32_t SkinJointCount(const JPH::SoftBodySharedSettings& settings)
	{
		uint32_t count = 0;
		for (const JPH::SoftBodySharedSettings::InvBind& invBind : settings.mInvBindMatrices)
			count = std::max(count, invBind.mJointIndex + 1);
		return count;
	}

	// Whether jointCount joint matrices cover the joint index of every inverse bind.
	bool CoversEveryJoint(const JPH::SoftBodySharedSettings& settings, uint32_t jointCount)
	{
		for (const JPH::SoftBodySharedSettings::InvBind& invBind : settings.mInvBindMatrices)
			if (invBind.mJointIndex >= jointCount)
				return false;
		return true;
	}

	// The motion properties of a soft body with skinned constraints, or null.
	JPH::SoftBodyMotionProperties* AsSkinnedSoftBody(JPH_Body* body)
	{
		JPH::SoftBodyMotionProperties* mp = AsSoftBodyMotionProperties(reinterpret_cast<JPH::Body*>(body));
		if (mp == nullptr || mp->GetSettings()->mSkinnedConstraints.empty())
			return nullptr;
		return mp;
	}
}

bool JPH_SoftBodySharedSettings_AddInvBind(JPH_SoftBodySharedSettings* settings, uint32_t jointIndex, const JPH_Mat4* invBind)
{
	JPH_ASSERT(settings && invBind);

	// The joint count of such a bind, 2^32, does not fit the uint32_t of SkinSoftBodyVertices.
	if (jointIndex == UINT32_MAX)
		return false;
	AsSoftBodySharedSettings(settings)->mInvBindMatrices.emplace_back(jointIndex, ToMat44(*invBind));
	return true;
}

bool JPH_SoftBodySharedSettings_AddSkinnedConstraint(JPH_SoftBodySharedSettings* settings, const JPH_SoftBodySkinnedConstraint* constraint)
{
	JPH_ASSERT(settings && constraint);

	JPH::SoftBodySharedSettings* joltSettings = AsSoftBodySharedSettings(settings);
	if (constraint->vertex >= joltSettings->mVertices.size()
		|| !HasValidWeights(*constraint, joltSettings->mInvBindMatrices.size()))
		return false;

	Skinned skinned(constraint->vertex, constraint->maxDistance, constraint->backStopDistance, constraint->backStopRadius);
	for (uint32_t i = 0; i < Skinned::cMaxSkinWeights; ++i)
		skinned.mWeights[i] = JPH::SoftBodySharedSettings::SkinWeight(constraint->weights[i].invBindIndex, constraint->weights[i].weight);
	joltSettings->mSkinnedConstraints.push_back(skinned);
	return true;
}

void JPH_SoftBodySharedSettings_CalculateSkinnedConstraintNormals(JPH_SoftBodySharedSettings* settings)
{
	AsSoftBodySharedSettings(settings)->CalculateSkinnedConstraintNormals();
}

uint32_t JPH_SoftBodySharedSettings_GetInvBindCount(const JPH_SoftBodySharedSettings* settings)
{
	return (uint32_t)AsSoftBodySharedSettings(settings)->mInvBindMatrices.size();
}

uint32_t JPH_SoftBodySharedSettings_GetSkinnedConstraintCount(const JPH_SoftBodySharedSettings* settings)
{
	return (uint32_t)AsSoftBodySharedSettings(settings)->mSkinnedConstraints.size();
}

uint32_t JPH_Body_GetSoftBodySkinJointCount(const JPH_Body* body)
{
	const JPH::SoftBodyMotionProperties* mp = AsSoftBodyMotionProperties(reinterpret_cast<const JPH::Body*>(body));
	if (mp == nullptr || mp->GetSettings()->mSkinnedConstraints.empty())
		return 0;
	return SkinJointCount(*mp->GetSettings());
}

bool JPH_Body_SkinSoftBodyVertices(JPH_Body* body, const JPH_Mat4* jointMatrices, uint32_t jointCount, bool hardSkinAll)
{
	JPH::SoftBodyMotionProperties* mp = AsSkinnedSoftBody(body);
	if (mp == nullptr || jointMatrices == nullptr)
		return false;

	// SkinVertices allocates the skin matrices with a uint byte count.
	const JPH::SoftBodySharedSettings& settings = *mp->GetSettings();
	if (uint64_t(settings.mInvBindMatrices.size()) * sizeof(JPH::Mat44) > UINT32_MAX)
		return false;
	// SkinVertices indexes the joint matrices with the joint indices unchecked.
	if (!CoversEveryJoint(settings, jointCount))
		return false;

	JPH::Array<JPH::Mat44> matrices;
	matrices.reserve(jointCount);
	for (uint32_t i = 0; i < jointCount; ++i)
		matrices.push_back(ToMat44(jointMatrices[i]));

	JPH::TempAllocatorMalloc allocator;
	const JPH::Body& joltBody = *reinterpret_cast<const JPH::Body*>(body);
	mp->SkinVertices(joltBody.GetCenterOfMassTransform(), matrices.data(), jointCount, hardSkinAll, allocator);
	return true;
}

float JPH_Body_GetSoftBodySkinnedMaxDistanceMultiplier(const JPH_Body* body)
{
	const JPH::SoftBodyMotionProperties* mp = AsSoftBodyMotionProperties(reinterpret_cast<const JPH::Body*>(body));
	return mp != nullptr ? mp->GetSkinnedMaxDistanceMultiplier() : 1.0f;
}

bool JPH_Body_SetSoftBodySkinnedMaxDistanceMultiplier(JPH_Body* body, float multiplier)
{
	JPH::SoftBodyMotionProperties* mp = AsSoftBodyMotionProperties(reinterpret_cast<JPH::Body*>(body));
	if (mp == nullptr)
		return false;
	mp->SetSkinnedMaxDistanceMultiplier(multiplier);
	return true;
}
