//! Creating, removing and finding bodies, and broad-phase lookups.

use std::any::Any;
use std::ffi::c_void;
use std::marker::PhantomData;
use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::ptr::{null, NonNull};

use oxijolt_sys::*;

use super::structure::{check_shape_for, ShapeUse};
use super::{
    with_locked_body, with_read_locked_body, AllowedDofs, BodyId, BodyMut, BodyRef, BodySettings,
    CreationSettings, INVALID_BODY_ID, STATIC_DOFS_RULE,
};
use crate::limits::is_in_frame;
use crate::owned::Owned;
use crate::{BodyError, PhysicsWorld, RVec3, Real, Shape, Vec3};

impl PhysicsWorld {
    /// Creates a body from `shape` and adds it to the world. The body keeps its own reference
    /// to the shape, so `shape` may be dropped afterwards.
    ///
    /// Fails with [`BodyError::InvalidValue`] when a setting is out of range, when a sensor's
    /// shape is one that only static bodies may use, and, for a dynamic or kinematic body or a
    /// static one that may move ([`BodySettings::allow_dynamic_or_kinematic`]), when:
    /// - the shape is one that only static bodies may use: a heightfield, or a compound or
    ///   decorated shape that contains one;
    /// - the shape contains a mesh and the body is dynamic, or not dynamic without
    ///   [`BodySettings::mass`] (Jolt computes no mass for a mesh);
    /// - the mass or inertia (overridden, or computed from a tiny shape) has no finite inverse;
    /// - the inertia tensor is not diagonal (a rotated or offset compound child, an offset centre
    ///   of mass) and too badly conditioned for Jolt to decompose, such as a slender shape in a
    ///   rotated child ([docs/limits.md#rigid-body-inertia]).
    ///
    /// A dynamic body's mass (overridden or computed) must also be within
    /// [`limits::MIN_MASS`]`..=`[`limits::MAX_MASS`]. Kinematic bodies are exempt from this
    /// range: Jolt gives them infinite mass in the solver.
    ///
    /// [`check_body`](Self::check_body) runs the same checks without creating the body. A full
    /// world fails with [`BodyError::TooManyBodies`].
    ///
    /// [docs/limits.md#rigid-body-inertia]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#rigid-body-inertia
    /// [`limits::MIN_MASS`]: crate::limits::MIN_MASS
    /// [`limits::MAX_MASS`]: crate::limits::MAX_MASS
    pub fn create_body(
        &mut self,
        shape: &Shape,
        settings: &BodySettings,
    ) -> Result<BodyId, BodyError> {
        self.check_body(shape, settings)?;
        let creation = CreationSettings::new(shape, settings)?;
        if !self.has_room_for_bodies(1) {
            return Err(BodyError::TooManyBodies);
        }
        self.note_structure_change();
        // SAFETY: the body interface belongs to this live world, borrowed mutably; `creation`
        // is a fully set up settings object whose layer exists in this world.
        let raw = unsafe {
            JPH_BodyInterface_CreateAndAddBody(
                self.body_interface.as_ptr(),
                creation.as_ptr(),
                settings.activation.to_jph(),
            )
        };
        if raw == INVALID_BODY_ID {
            return Err(BodyError::TooManyBodies);
        }
        Ok(BodyId::new(raw, self.tag))
    }

    /// Checks whether [`create_body`](Self::create_body) with `shape` and `settings` would
    /// succeed, without creating, adding or waking anything: it returns the error `create_body`
    /// would return for these arguments, or `Ok` when `create_body` with them succeeds unless the
    /// world is full. Earlier [`WorldState`](crate::WorldState)s stay restorable.
    ///
    /// `create_body` runs exactly these checks before it creates the body. They read only the
    /// arguments and the world's number of object layers, which is fixed when the world is
    /// created, so creating, removing, changing or stepping bodies does not change the verdict.
    ///
    /// Capacity is not checked: this never returns [`BodyError::TooManyBodies`]. Compare
    /// [`body_count`](Self::body_count) with [`max_bodies`](Self::max_bodies) for that.
    ///
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let ball = Shape::new_sphere(0.5)?;
    /// let settings = BodySettings::new_dynamic();
    /// world.check_body(&ball, &settings)?;
    /// assert_eq!(world.body_count(), 0);
    /// world.create_body(&ball, &settings)?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn check_body(&self, shape: &Shape, settings: &BodySettings) -> Result<(), BodyError> {
        settings.validate(self.object_layer_count)?;
        if !settings.can_move() && settings.allowed_dofs != AllowedDofs::ALL {
            return Err(BodyError::InvalidValue(STATIC_DOFS_RULE));
        }
        // Jolt computes mass properties for every body that can move
        // (`BodyCreationSettings::HasMassProperties`).
        check_shape_for(
            shape,
            ShapeUse {
                motion_type: settings.motion_type,
                can_move: settings.can_move(),
                sensor: settings.sensor,
                mass: settings.mass,
            },
        )?;
        Ok(())
    }

    /// `Ok` if `id` names a body that is in this world now; [`BodyError::Detached`] for a body
    /// created by [`create_body_detached`](Self::create_body_detached) and not added.
    pub(crate) fn check(&self, id: BodyId) -> Result<(), BodyError> {
        if id.world != self.tag {
            return Err(BodyError::WrongWorld(id));
        }
        // SAFETY: the body interface belongs to this live world. `IsAdded` locks the body and
        // compares index and sequence number, so any id value is safe to pass.
        if unsafe { JPH_BodyInterface_IsAdded(self.body_interface.as_ptr(), id.raw) } {
            Ok(())
        } else if with_read_locked_body(self.body_lock_interface, id, |_| ()).is_some() {
            Err(BodyError::Detached(id))
        } else {
            Err(BodyError::NotFound(id))
        }
    }

    /// Whether `id` names a body that is in this world now (Jolt `BodyInterface::IsAdded`);
    /// `false` for a detached body ([`create_body_detached`](Self::create_body_detached)).
    pub fn contains_body(&self, id: BodyId) -> bool {
        self.check(id).is_ok()
    }

    /// Every body in the world in ascending id order ([`BodyId::to_raw`]), including soft
    /// bodies, ragdoll parts and characters' inner bodies (Jolt `PhysicsSystem::GetBodies`).
    /// Detached bodies ([`create_body_detached`](Self::create_body_detached)) are listed too, as
    /// [`body_count`](Self::body_count) counts them; [`contains_body`](Self::contains_body) tells
    /// them apart.
    ///
    /// It borrows the world mutably although it changes nothing: joltc copies as many ids as it
    /// is asked for, so the body count must not change between counting and copying. The ids
    /// are copied before this returns, so the iterator does not borrow the world.
    ///
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let shape = Shape::new_sphere(0.5)?;
    /// let a = world.create_body(&shape, &BodySettings::new_dynamic())?;
    /// let b = world.create_body(&shape, &BodySettings::new_static())?;
    /// assert!(world.body_ids().eq([a, b]));
    /// for id in world.body_ids() {
    ///     world.body_mut(id)?.activate();
    /// }
    /// # Ok(())
    /// # }
    /// ```
    pub fn body_ids(&mut self) -> impl Iterator<Item = BodyId> {
        let count = self.body_count();
        let mut raw = vec![INVALID_BODY_ID; count as usize];
        if count > 0 {
            // SAFETY: the system is live and `raw` holds `count` ids. joltc writes `count` ids
            // without clamping them to the bodies it lists (joltc.cpp:5759-5772), so `count`
            // must be at most that number: it is `GetNumBodies`, which Jolt's list matches
            // (`BodyManager::GetBodyIDs` asserts it), and the world is borrowed mutably, so no
            // body is added or removed meanwhile.
            unsafe { JPH_PhysicsSystem_GetBodies(self.system.as_ptr(), raw.as_mut_ptr(), count) };
        }
        raw.sort_unstable();
        let tag = self.tag;
        raw.into_iter().map(move |raw| BodyId::new(raw, tag))
    }

    /// Read access to a body. Fails with [`BodyError::Detached`] for a body that is not added.
    pub fn body(&self, id: BodyId) -> Result<BodyRef<'_>, BodyError> {
        self.check(id)?;
        Ok(BodyRef {
            body_interface: self.body_interface,
            body_lock_interface: self.body_lock_interface,
            id,
            _world: PhantomData,
        })
    }

    /// Read and write access to a body. Fails with [`BodyError::Detached`] for a body that is not
    /// added.
    pub fn body_mut(&mut self, id: BodyId) -> Result<BodyMut<'_>, BodyError> {
        self.check(id)?;
        Ok(BodyMut {
            inner: BodyRef {
                body_interface: self.body_interface,
                body_lock_interface: self.body_lock_interface,
                id,
                _world: PhantomData,
            },
            world: self,
        })
    }

    /// Removes a body from the world and destroys it; soft bodies are removed the same way.
    ///
    /// Jolt does not wake the bodies around a removed one by itself, so oxijolt wakes every
    /// non-static body whose current bounds come within the [speculative contact distance] of
    /// the removed body's bounds, in body-id order. The woken set depends only on body poses, not on broad-phase maintenance or
    /// worker threads; a stack whose bottom is removed falls.
    ///
    /// The inner body of a character cannot be removed this way
    /// ([`BodyError::OwnedByCharacter`]); it goes with
    /// [`remove_character`](Self::remove_character). The chassis of a vehicle cannot be removed
    /// while the vehicle exists ([`BodyError::UsedByVehicle`]); remove the vehicle first with
    /// [`remove_vehicle`](Self::remove_vehicle). A part of a ragdoll goes only with its ragdoll
    /// ([`BodyError::OwnedByRagdoll`], [`remove_ragdoll`](Self::remove_ragdoll)). A body that a
    /// constraint uses cannot be removed while the constraint exists
    /// ([`BodyError::UsedByConstraint`]); remove the constraint first with
    /// [`remove_constraint`](Self::remove_constraint). A detached body is refused
    /// ([`BodyError::Detached`]); it goes with [`destroy_body`](Self::destroy_body).
    /// [`remove_bodies`](Self::remove_bodies) removes several bodies at once.
    ///
    /// [speculative contact distance]: crate::WorldSettings::speculative_contact_distance
    pub fn remove_body(&mut self, id: BodyId) -> Result<(), BodyError> {
        self.check(id)?;
        self.check_not_owned(id)?;
        let mut bounds = JPH_AABox {
            min: Vec3::ZERO.to_jph(),
            max: Vec3::ZERO.to_jph(),
        };
        with_locked_body(self.body_lock_interface, id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure; `bounds` is
            // a live local.
            unsafe { JPH_Body_GetWorldSpaceBounds(body.as_ptr(), &mut bounds) };
        })
        .ok_or(BodyError::NotFound(id))?;
        self.note_structure_change();
        // SAFETY: `check` just confirmed the id names a body in this world, and `&mut self`
        // keeps anyone else from removing it in between, so the body is removed exactly once
        // (Jolt does not validate ids in `DestroyBody`). This thread holds no body lock.
        unsafe { JPH_BodyInterface_RemoveAndDestroyBody(self.body_interface.as_ptr(), id.raw) };
        self.pending_cache_invalidations.remove(&id.raw);
        self.wake_bodies_near(&bounds, None);
        Ok(())
    }

    /// Refuses a body that a character, vehicle, ragdoll or constraint holds: removing it, or
    /// changing its shape or motion type, would break what that owner relies on. Every API that
    /// destroys a body or changes its shape or motion type calls this.
    pub(crate) fn check_not_owned(&self, id: BodyId) -> Result<(), BodyError> {
        // The character's destructor destroys its inner body, and Jolt does not validate ids in
        // `DestroyBody`: removing it here first would make that a double destroy. The character
        // also moves the body as a kinematic body of its own shape.
        if self.is_inner_body(id) {
            return Err(BodyError::OwnedByCharacter(id));
        }
        // A vehicle keeps a pointer to its chassis and dereferences it on every step, and its
        // checks assume a dynamic chassis of the mass it was created with.
        if self.is_vehicle_body(id) {
            return Err(BodyError::UsedByVehicle(id));
        }
        // A ragdoll destroys its parts when it is released, and Jolt does not validate ids in
        // `DestroyBody`, so removing a part here would make that a double destroy. Its joints
        // were checked against the parts' shapes and masses.
        if self.is_ragdoll_body(id) {
            return Err(BodyError::OwnedByRagdoll(id));
        }
        // A constraint keeps pointers to its bodies and dereferences them on every step, and
        // its lever-arm, spring and dynamic-body checks used the bodies' motion types, masses
        // and inertias at creation.
        if self.is_constraint_body(id) {
            return Err(BodyError::UsedByConstraint(id));
        }
        Ok(())
    }

    /// Wakes every non-static body whose current bounds overlap or touch the box from `min` to
    /// `max` (world space, metres), in body-id order, recording an
    /// [`ActivationEvent::Activated`](crate::ActivationEvent::Activated) for each that was
    /// asleep.
    ///
    /// The bodies are found by their exact bounds now, compared in the precision of [`Real`],
    /// so the result does not depend on the broad phase's history or on worker threads; Jolt's
    /// own `ActivateBodiesInAABox` does depend on them and is not used. Both corners must lie
    /// within [`limits::MAX_POSITION`](crate::limits::MAX_POSITION) with `min <= max` on every
    /// axis, otherwise [`BodyError::InvalidValue`] is returned and nothing wakes.
    pub fn activate_bodies_in_box(&mut self, min: RVec3, max: RVec3) -> Result<(), BodyError> {
        check_box(min, max)?;
        self.wake_bodies_overlapping(min, max, None);
        Ok(())
    }

    /// Whether bodies `a` and `b` touched in the last [`step`](Self::step) (Jolt
    /// `PhysicsSystem::WereBodiesInContact`).
    ///
    /// Jolt answers from the contact cache of the last step, and only for pairs of which at
    /// least one body was awake in it; bodies removed since are allowed. A soft body's
    /// vertices collide inside the soft body solver and never enter that cache, so this is
    /// `false` for every pair with a soft body, even one lying on the other body. Fails with
    /// [`BodyError::WrongWorld`] for an id of another world.
    pub fn were_bodies_in_contact(&self, a: BodyId, b: BodyId) -> Result<bool, BodyError> {
        for id in [a, b] {
            if id.world != self.tag {
                return Err(BodyError::WrongWorld(id));
            }
        }
        // SAFETY: the system is live and no step runs (`step` needs `&mut self`); Jolt looks the
        // pair up in its contact cache and never dereferences a body, so any id is safe.
        Ok(unsafe { JPH_PhysicsSystem_WereBodiesInContact(self.system.as_ptr(), a.raw, b.raw) })
    }

    /// Wakes every non-static body other than `except` whose world bounds come within the
    /// speculative contact distance of `bounds`, in body-id order: the neighbour wake after a
    /// body is removed or changed.
    ///
    /// Jolt pairs two bodies when their bounds come within that distance
    /// (`QuadTree::FindCollidingPairs` widens them in `f32` the same way), so a body resting on a
    /// speculative contact need not touch the exact bounds.
    pub(crate) fn wake_bodies_near(&mut self, bounds: &JPH_AABox, except: Option<BodyId>) {
        let (min, max) = self.near_box(bounds);
        self.wake_bodies_overlapping(min, max, except);
    }

    /// Wakes every non-static body other than `except` whose world bounds come within the
    /// speculative contact distance of any of the boxes `(min, max)`, each once and in body-id
    /// order: [`wake_bodies_near`](Self::wake_bodies_near) for boxes given in [`Real`], each
    /// first rounded outward to `f32`. No boxes wake nothing.
    pub(crate) fn wake_bodies_near_boxes(
        &mut self,
        boxes: &[(RVec3, RVec3)],
        except: Option<BodyId>,
    ) {
        let mut woken = Vec::new();
        for &(min, max) in boxes {
            let (min, max) = self.near_box(&outward_box(min, max));
            let candidates = self.broad_phase_bodies(&outward_box(min, max));
            woken.extend(self.overlapping_movable_bodies(min, max, &candidates));
        }
        woken.sort_unstable();
        woken.dedup();
        self.activate_all_except(woken, except);
    }

    /// `bounds` widened by the speculative contact distance in `f32`, as Jolt widens bounds
    /// when it pairs bodies, with the corners in [`Real`].
    fn near_box(&self, bounds: &JPH_AABox) -> (RVec3, RVec3) {
        let margin = self.speculative_contact_distance();
        let widen = |v: JPH_Vec3, by: f32| JPH_Vec3 {
            x: v.x + by,
            y: v.y + by,
            z: v.z + by,
        };
        corners(&JPH_AABox {
            min: widen(bounds.min, -margin),
            max: widen(bounds.max, margin),
        })
    }

    /// Wakes every non-static body other than `except` whose world bounds overlap or touch the
    /// box from `min` to `max`, in body-id order.
    ///
    /// Jolt's broad phase keeps widened bounds for moved bodies until its next maintenance, so
    /// which bodies it reports depends on that history (Jolt docs, "Deterministic Simulation").
    /// Here it only proposes candidates, from the box rounded outward to `f32`; each is kept only
    /// when its exact bounds overlap the box in the caller's precision.
    pub(crate) fn wake_bodies_overlapping(
        &mut self,
        min: RVec3,
        max: RVec3,
        except: Option<BodyId>,
    ) {
        let candidates = self.broad_phase_bodies(&outward_box(min, max));
        let woken = self.overlapping_movable_bodies(min, max, &candidates);
        self.activate_all_except(woken, except);
    }

    /// Wakes the bodies `woken`, in that order, leaving out `except`.
    pub(super) fn activate_all_except(
        &mut self,
        mut woken: Vec<JPH_BodyID>,
        except: Option<BodyId>,
    ) {
        if let Some(except) = except {
            woken.retain(|&raw| raw != except.raw);
        }
        if woken.is_empty() {
            return;
        }
        // SAFETY: the body interface belongs to this live world, borrowed mutably; `woken` holds
        // `woken.len()` ids and lives for the call. This thread holds no body lock:
        // `overlapping_movable_bodies` has released its locks, which `ActivateBodies` takes
        // again (Jolt's body mutexes are not recursive).
        unsafe {
            JPH_BodyInterface_ActivateBodies(
                self.body_interface.as_ptr(),
                woken.as_ptr(),
                woken.len() as u32,
            );
        }
    }

    /// The ids of the bodies whose broad-phase bounds overlap `bounds`, sorted and without
    /// duplicates. The broad phase may keep widened bounds, so callers check exact bounds.
    pub(crate) fn broad_phase_bodies(&self, bounds: &JPH_AABox) -> Vec<JPH_BodyID> {
        let mut hits = BroadPhaseHits {
            ids: Vec::with_capacity(self.body_count() as usize),
            panic: None,
        };
        // SAFETY: the broad-phase query belongs to this live world; a step needs
        // `&mut PhysicsWorld`, so none runs during this `&self` call. `bounds` is live for the
        // call. `hits` is a live local that only the collector touches during the call, as the
        // `BroadPhaseHits` it expects. Null filters select joltc's accept-all defaults.
        unsafe {
            JPH_BroadPhaseQuery_CollideAABox(
                self.broad_phase_query.as_ptr(),
                bounds,
                Some(collect_broad_phase_hit),
                (&mut hits as *mut BroadPhaseHits).cast(),
                null(),
                null(),
            );
        }
        if let Some(payload) = hits.panic {
            resume_unwind(payload);
        }
        let mut candidates = hits.ids;
        // Jolt's `BodyID::operator<` compares the raw value.
        candidates.sort_unstable();
        candidates.dedup();
        candidates
    }

    /// Those of the sorted `candidates` that are non-static bodies whose world bounds overlap
    /// the box from `min` to `max`, in the same order. Locks all candidates at once and releases
    /// them before returning.
    fn overlapping_movable_bodies(
        &self,
        min: RVec3,
        max: RVec3,
        candidates: &[JPH_BodyID],
    ) -> Vec<JPH_BodyID> {
        let mut woken = Vec::new();
        if candidates.is_empty() {
            return woken;
        }
        // SAFETY: the lock interface belongs to this live world. joltc copies the ids into the
        // lock object, so `candidates` only has to live for the call. The handle takes over the
        // lock and releases it when it goes out of scope.
        let lock = unsafe {
            Owned::from_raw(JPH_BodyLockInterface_LockMultiWrite(
                self.body_lock_interface.as_ptr(),
                candidates.as_ptr(),
                candidates.len() as u32,
            ))
        };
        let Some(lock) = lock else {
            return woken;
        };
        for (index, &candidate) in candidates.iter().enumerate() {
            // SAFETY: `lock` is live and holds `candidates.len()` ids, so `index` is in range.
            // Jolt returns null unless the id still names a live body.
            let body = unsafe { JPH_BodyLockMultiWrite_GetBody(lock.as_ptr(), index as u32) };
            let Some(body) = NonNull::new(body) else {
                continue;
            };
            let mut candidate_bounds = JPH_AABox {
                min: Vec3::ZERO.to_jph(),
                max: Vec3::ZERO.to_jph(),
            };
            // SAFETY: `body` is locked for writing while `lock` lives; the getters only read
            // it, and `candidate_bounds` is a live local.
            let is_static = unsafe {
                JPH_Body_GetWorldSpaceBounds(body.as_ptr(), &mut candidate_bounds);
                JPH_Body_IsStatic(body.as_ptr())
            };
            if !is_static && bounds_overlap(min, max, &candidate_bounds) {
                woken.push(candidate);
            }
        }
        woken
    }
}

/// What the broad-phase collector of `PhysicsWorld::broad_phase_bodies` gathers.
struct BroadPhaseHits {
    /// Candidate ids, at most as many as the capacity reserved before the query.
    ids: Vec<JPH_BodyID>,
    /// The payload of a panic caught in the collector, re-raised after the query.
    panic: Option<Box<dyn Any + Send>>,
}

/// Jolt's `CollisionCollectorTraitsCollideShape::InitialEarlyOutFraction`: keep collecting.
const KEEP_COLLECTING: f32 = f32::MAX;
/// Jolt's `CollisionCollectorTraitsCollideShape::ShouldEarlyOutFraction`: stop the query.
const STOP_COLLECTING: f32 = -f32::MAX;

/// Broad-phase collector of `PhysicsWorld::broad_phase_bodies`: records each candidate id
/// without allocating and never lets a panic unwind into joltc.
///
/// # Safety
/// Called only by joltc during the `JPH_BroadPhaseQuery_CollideAABox` call of
/// `broad_phase_bodies`, with that call's live `*mut BroadPhaseHits` as `user_data`, which
/// nothing else accesses during the call.
unsafe extern "C" fn collect_broad_phase_hit(user_data: *mut c_void, body: JPH_BodyID) -> f32 {
    // SAFETY: guaranteed by the caller (function contract); this is the only reference to the
    // hits during the call.
    let hits = unsafe { &mut *user_data.cast::<BroadPhaseHits>() };
    let ids = &mut hits.ids;
    let collected = catch_unwind(AssertUnwindSafe(|| {
        if ids.len() < ids.capacity() {
            ids.push(body);
        }
    }));
    match collected {
        Ok(()) => KEEP_COLLECTING,
        Err(payload) => {
            hits.panic = Some(payload);
            STOP_COLLECTING
        }
    }
}

/// Refuses a box whose corners lie beyond [`limits::MAX_POSITION`](crate::limits::MAX_POSITION)
/// or are not finite, or whose `min` exceeds `max` on some axis.
pub(crate) fn check_box(min: RVec3, max: RVec3) -> Result<(), BodyError> {
    let ordered = min.x <= max.x && min.y <= max.y && min.z <= max.z;
    if is_in_frame(min) && is_in_frame(max) && ordered {
        Ok(())
    } else {
        Err(BodyError::InvalidValue(
            "box corners must be within limits::MAX_POSITION with min <= max",
        ))
    }
}

/// The corners of `bounds`, widened to [`Real`].
pub(crate) fn corners(bounds: &JPH_AABox) -> (RVec3, RVec3) {
    let widen = |v: JPH_Vec3| RVec3::new(Real::from(v.x), Real::from(v.y), Real::from(v.z));
    (widen(bounds.min), widen(bounds.max))
}

/// `value` as `f32`, rounded down for `up == false` and up otherwise, so that the result
/// encloses `value` on that side.
// `Real` is `f32` without the `double-precision` feature, so the cast is a no-op there.
#[allow(clippy::unnecessary_cast)]
fn rounded_outward(value: Real, up: bool) -> f32 {
    let rounded = value as f32;
    if up && Real::from(rounded) < value {
        rounded.next_up()
    } else if !up && Real::from(rounded) > value {
        rounded.next_down()
    } else {
        rounded
    }
}

/// The smallest `f32` box that contains the box from `min` to `max`.
pub(super) fn outward_box(min: RVec3, max: RVec3) -> JPH_AABox {
    let side = |v: RVec3, up| {
        Vec3::new(
            rounded_outward(v.x, up),
            rounded_outward(v.y, up),
            rounded_outward(v.z, up),
        )
        .to_jph()
    };
    JPH_AABox {
        min: side(min, false),
        max: side(max, true),
    }
}

/// Whether `bounds` overlaps the box from `min` to `max`, touching included (Jolt
/// `AABox::Overlaps`), compared in [`Real`].
pub(super) fn bounds_overlap(min: RVec3, max: RVec3, bounds: &JPH_AABox) -> bool {
    let (other_min, other_max) = corners(bounds);
    let axis =
        |a_min: Real, a_max: Real, b_min: Real, b_max: Real| a_min <= b_max && b_min <= a_max;
    axis(min.x, max.x, other_min.x, other_max.x)
        && axis(min.y, max.y, other_min.y, other_max.y)
        && axis(min.z, max.z, other_min.z, other_max.z)
}
