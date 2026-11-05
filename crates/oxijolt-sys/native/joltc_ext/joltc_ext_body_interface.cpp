// Batch adding and removing of bodies: Jolt's BodyInterface::AddBodiesPrepare, AddBodiesFinalize,
// AddBodiesAbort, RemoveBodies and DestroyBodies. Part of this fork's joltc additions (see
// joltc_ext.cpp).
// Licensed under the MIT License (MIT), like joltc.
//
// Handles: a JPH_BodyInterface is a reinterpret_cast of the JPH::BodyInterface, as in joltc. Jolt
// reorders the id arrays it is given; these functions work on copies, as joltc's
// JPH_BodyInterface_ActivateBodies does, so the caller's array is never changed. Each checks its
// preconditions first and changes nothing when one fails.

#include <Jolt/Jolt.h>

#include <algorithm>

#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Body/BodyInterface.h>

#include "joltc_ext_internal.h"

// The ids of a prepared batch, in the order Jolt left them, and Jolt's add state, which points
// into that array until the batch is finalized or aborted.
struct JPH_AddBodiesState
{
	JPH::Array<JPH::BodyID> bodies;
	JPH::BodyInterface::AddState joltState = nullptr;
};

namespace
{
	JPH::BodyInterface& AsBodyInterface(JPH_BodyInterface* bodyInterface)
	{
		return *reinterpret_cast<JPH::BodyInterface*>(bodyInterface);
	}

	// The count ids at bodies as Jolt ids, sorted ascending; false when an id repeats.
	bool SortedBodyIDs(const JPH_BodyID* bodies, uint32_t count, JPH::Array<JPH::BodyID>& outIDs)
	{
		outIDs.clear();
		outIDs.reserve(count);
		for (uint32_t i = 0; i < count; ++i)
			outIDs.push_back(JPH::BodyID(bodies[i]));
		std::sort(outIDs.begin(), outIDs.end());
		return std::adjacent_find(outIDs.begin(), outIDs.end()) == outIDs.end();
	}

	// Whether id names a body of the system that is not in its broad phase. Every body has a shape,
	// and GetShape returns null exactly when the id names no body.
	bool IsDetached(const JPH::BodyInterface& bodyInterface, const JPH::BodyID& id)
	{
		return bodyInterface.GetShape(id) != nullptr && !bodyInterface.IsAdded(id);
	}
}

JPH_AddBodiesState* JPH_BodyInterface_AddBodiesPrepare(JPH_BodyInterface* bodyInterface, const JPH_BodyID* bodies, uint32_t count)
{
	JPH_ASSERT(bodyInterface && (bodies || count == 0));

	JPH::BodyInterface& joltInterface = AsBodyInterface(bodyInterface);
	JPH_AddBodiesState* state = new JPH_AddBodiesState();
	bool valid = SortedBodyIDs(bodies, count, state->bodies);
	for (const JPH::BodyID& id : state->bodies)
		valid = valid && IsDetached(joltInterface, id);
	if (!valid)
	{
		delete state;
		return nullptr;
	}

	state->joltState = joltInterface.AddBodiesPrepare(state->bodies.data(), (int)state->bodies.size());
	return state;
}

void JPH_BodyInterface_AddBodiesFinalize(JPH_BodyInterface* bodyInterface, JPH_AddBodiesState* state, JPH_Activation activationMode)
{
	JPH_ASSERT(bodyInterface && state);

	AsBodyInterface(bodyInterface).AddBodiesFinalize(state->bodies.data(), (int)state->bodies.size(), state->joltState, (JPH::EActivation)activationMode);
	delete state;
}

void JPH_BodyInterface_AddBodiesAbort(JPH_BodyInterface* bodyInterface, JPH_AddBodiesState* state)
{
	JPH_ASSERT(bodyInterface && state);

	AsBodyInterface(bodyInterface).AddBodiesAbort(state->bodies.data(), (int)state->bodies.size(), state->joltState);
	delete state;
}

bool JPH_BodyInterface_RemoveBodies(JPH_BodyInterface* bodyInterface, const JPH_BodyID* bodies, uint32_t count)
{
	JPH_ASSERT(bodyInterface && (bodies || count == 0));

	JPH::BodyInterface& joltInterface = AsBodyInterface(bodyInterface);
	JPH::Array<JPH::BodyID> ids;
	if (!SortedBodyIDs(bodies, count, ids))
		return false;
	for (const JPH::BodyID& id : ids)
		if (!joltInterface.IsAdded(id))
			return false;

	joltInterface.RemoveBodies(ids.data(), (int)ids.size());
	return true;
}

bool JPH_BodyInterface_DestroyBodies(JPH_BodyInterface* bodyInterface, const JPH_BodyID* bodies, uint32_t count)
{
	JPH_ASSERT(bodyInterface && (bodies || count == 0));

	JPH::BodyInterface& joltInterface = AsBodyInterface(bodyInterface);
	JPH::Array<JPH::BodyID> ids;
	if (!SortedBodyIDs(bodies, count, ids))
		return false;
	for (const JPH::BodyID& id : ids)
		if (!IsDetached(joltInterface, id))
			return false;

	// Jolt frees the ids in this order, and the free list decides the ids of later bodies, so
	// the caller's order is kept.
	for (uint32_t i = 0; i < count; ++i)
		ids[i] = JPH::BodyID(bodies[i]);
	joltInterface.DestroyBodies(ids.data(), (int)count);
	return true;
}
