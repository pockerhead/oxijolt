//! Bodies created without being added, and adding and removing bodies in batches.

use std::marker::PhantomData;
use std::mem::ManuallyDrop;
use std::ptr::NonNull;

use oxijolt_sys::*;

use super::access::corners;
use super::{with_read_locked_body, Activation, BodyId, BodySettings, CreationSettings};
use crate::{BodyError, PhysicsWorld, Shape, Vec3};

impl PhysicsWorld {
    /// Creates a body from `shape` as [`create_body`](Self::create_body) does, with the same
    /// checks and errors, but does not add it to the world (Jolt `BodyInterface::CreateBody`).
    /// [`BodySettings::activation`] is not used; [`add_body`](Self::add_body) and
    /// [`add_bodies`](Self::add_bodies) take the activation.
    ///
    /// The id is assigned now, in call order, so bodies created this way get the same ids as
    /// the same bodies created with `create_body`. Until it is added, the body is detached:
    /// - it takes a place in the world: [`body_count`](Self::body_count) counts it,
    ///   [`body_ids`](Self::body_ids) lists it, and a full world refuses more bodies;
    /// - it is not in the world: [`contains_body`](Self::contains_body) is `false`, queries and
    ///   steps do not see it, and [`body`](Self::body), [`body_mut`](Self::body_mut),
    ///   [`remove_body`](Self::remove_body), [`remove_bodies`](Self::remove_bodies) and every
    ///   call that takes a body (constraints, vehicles, state selections) fail with
    ///   [`BodyError::Detached`], and so does [`rebase`](Self::rebase), which must list every
    ///   body;
    /// - Jolt's saved states do not hold it.
    ///
    /// Creating it changes the world's structure as `create_body` does: earlier
    /// [`WorldState`](crate::WorldState)s are no longer restorable. The world destroys a body
    /// still detached when it is dropped.
    ///
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    /// let mut ids = Vec::new();
    /// for x in 0..100 {
    ///     let at = RVec3::new(2.0 * x as Real, 0.5, 0.0);
    ///     ids.push(world.create_body_detached(&cube, &BodySettings::new_dynamic().position(at))?);
    /// }
    /// assert!(!world.contains_body(ids[0]));
    /// world.add_bodies(&ids, Activation::Activate)?;
    /// assert!(world.contains_body(ids[0]));
    /// # Ok(())
    /// # }
    /// ```
    pub fn create_body_detached(
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
        let body = unsafe {
            JPH_BodyInterface_CreateBody(self.body_interface.as_ptr(), creation.as_ptr())
        };
        let Some(body) = NonNull::new(body) else {
            return Err(BodyError::TooManyBodies);
        };
        // SAFETY: `body` was just created and is not added, and the world is borrowed mutably,
        // so no other thread reaches it; reading its id needs no lock.
        let raw = unsafe { JPH_Body_GetID(body.as_ptr()) };
        Ok(BodyId::new(raw, self.tag))
    }

    /// Adds a detached body ([`create_body_detached`](Self::create_body_detached)) to the world
    /// (Jolt `BodyInterface::AddBody`). With [`Activation::Activate`] a body that is not static
    /// wakes. Bodies around it stay as they are, as with [`create_body`](Self::create_body).
    ///
    /// Fails with [`BodyError::AlreadyAdded`] for a body in the world, and with
    /// [`BodyError::NotFound`] or [`BodyError::WrongWorld`] for an id that names no body of this
    /// world; nothing changes then. Adding changes the world's structure: earlier
    /// [`WorldState`](crate::WorldState)s are no longer restorable.
    pub fn add_body(&mut self, id: BodyId, activation: Activation) -> Result<(), BodyError> {
        self.check_detached(id)?;
        self.note_structure_change();
        // SAFETY: `check_detached` confirmed that the id names a body of this world that is not
        // added, and `&mut self` keeps it so. This thread holds no body lock.
        unsafe {
            JPH_BodyInterface_AddBody(self.body_interface.as_ptr(), id.raw, activation.to_jph())
        };
        Ok(())
    }

    /// Destroys a detached body ([`create_body_detached`](Self::create_body_detached)) without
    /// ever adding it (Jolt `BodyInterface::DestroyBody`). Its id is freed as
    /// [`remove_body`](Self::remove_body) frees one.
    ///
    /// Fails with [`BodyError::AlreadyAdded`] for a body in the world (use `remove_body`), and
    /// with [`BodyError::NotFound`] or [`BodyError::WrongWorld`] for an id that names no body of
    /// this world; nothing changes then. Destroying changes the world's structure: earlier
    /// [`WorldState`](crate::WorldState)s are no longer restorable.
    pub fn destroy_body(&mut self, id: BodyId) -> Result<(), BodyError> {
        self.check_detached(id)?;
        self.check_not_owned(id)?;
        self.note_structure_change();
        // SAFETY: `check_detached` confirmed that the id names a body of this world that is not
        // added, and `&mut self` keeps anyone else from destroying it in between, so it is
        // destroyed exactly once (Jolt does not validate ids in `DestroyBody`).
        unsafe { JPH_BodyInterface_DestroyBody(self.body_interface.as_ptr(), id.raw) };
        Ok(())
    }

    /// Adds detached bodies ([`create_body_detached`](Self::create_body_detached)) to the world
    /// in one batch (Jolt `BodyInterface::AddBodiesPrepare` and `AddBodiesFinalize`). Jolt builds
    /// one broad-phase subtree per broad-phase layer for the batch, which keeps queries and the next step fast where
    /// adding the bodies one by one would leave the broad phase unbalanced until
    /// [`optimize_broad_phase`](Self::optimize_broad_phase) or later steps rebuild it.
    ///
    /// Every id is checked before anything changes: an id of another world
    /// ([`BodyError::WrongWorld`]), of no body ([`BodyError::NotFound`]), of a body in the world
    /// ([`BodyError::AlreadyAdded`]) or listed twice ([`BodyError::DuplicateId`]) fails the call
    /// and adds nothing. An empty slice does nothing.
    ///
    /// The bodies are added in ascending id order whatever the order of `ids`, and `ids` is not
    /// reordered. With [`Activation::Activate`] the bodies that are not static wake in ascending
    /// id order, which is the order of their
    /// [`ActivationEvent::Activated`](crate::ActivationEvent::Activated) events. The batch builds
    /// a different broad-phase tree than adding the same bodies one by one, so the two call
    /// histories may simulate differently; the same calls give the same results ([docs/determinism.md]).
    /// Adding changes the world's structure: earlier [`WorldState`](crate::WorldState)s are no
    /// longer restorable.
    ///
    /// [docs/determinism.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/determinism.md#batches-of-bodies
    pub fn add_bodies(&mut self, ids: &[BodyId], activation: Activation) -> Result<(), BodyError> {
        for &id in ids {
            self.check_detached(id)?;
        }
        let sorted = self.sorted_unique(ids)?;
        if sorted.is_empty() {
            return Ok(());
        }
        self.note_structure_change();
        self.prepare_add(&sorted).finalize(Activation::DontActivate);
        if activation == Activation::Activate {
            self.activate_all_except(sorted, None);
        }
        Ok(())
    }

    /// Removes bodies from the world and destroys them in one batch (Jolt
    /// `BodyInterface::RemoveBodies` and `DestroyBodies`), as [`remove_body`](Self::remove_body)
    /// does for each.
    ///
    /// Every id is checked first, as `remove_body` checks one: an id of another world, of no
    /// body, of a detached body ([`BodyError::Detached`]; see
    /// [`destroy_body`](Self::destroy_body)) or of a body a character, vehicle, ragdoll or
    /// constraint holds fails the call, and so does an id listed twice
    /// ([`BodyError::DuplicateId`]); nothing is removed then. An empty slice does nothing.
    ///
    /// The ids are freed in the order of `ids`, which decides the ids later bodies get, so
    /// removing `[a, b]` here and `a` then `b` with `remove_body` leave the same ids to reuse.
    /// `ids` is not reordered. The bodies around the removed ones wake as `remove_body` wakes
    /// them, once for the whole batch: every non-static body within the
    /// [speculative contact distance] of a removed body's bounds, each once, in ascending id
    /// order. The activation events differ from those of one-by-one removal: the removed bodies
    /// that are awake report [`ActivationEvent::Deactivated`](crate::ActivationEvent::Deactivated)
    /// in ascending id order, all before the neighbours' `Activated` events, and a sleeping body
    /// of the batch reports nothing, also where `remove_body` of an earlier id would have woken
    /// it first. Removing changes the world's structure: earlier
    /// [`WorldState`](crate::WorldState)s are no longer restorable.
    ///
    /// [speculative contact distance]: crate::WorldSettings::speculative_contact_distance
    pub fn remove_bodies(&mut self, ids: &[BodyId]) -> Result<(), BodyError> {
        for &id in ids {
            self.check(id)?;
            self.check_not_owned(id)?;
        }
        self.sorted_unique(ids)?;
        if ids.is_empty() {
            return Ok(());
        }
        let mut boxes = Vec::with_capacity(ids.len());
        for &id in ids {
            let mut bounds = JPH_AABox {
                min: Vec3::ZERO.to_jph(),
                max: Vec3::ZERO.to_jph(),
            };
            with_read_locked_body(self.body_lock_interface, id, |body| {
                // SAFETY: `body` is locked for reading for the duration of the closure; `bounds`
                // is a live local.
                unsafe { JPH_Body_GetWorldSpaceBounds(body.as_ptr(), &mut bounds) };
            })
            .ok_or(BodyError::NotFound(id))?;
            boxes.push(corners(&bounds));
        }
        let raw: Vec<JPH_BodyID> = ids.iter().map(|id| id.raw).collect();
        self.note_structure_change();
        // SAFETY: the body interface belongs to this live world, borrowed mutably; `raw` holds
        // `raw.len()` ids for the calls. The checks above confirmed that the ids are distinct
        // bodies of this world that are added, which the extension checks again before it
        // changes anything. This thread holds no body lock.
        let (removed, destroyed) = unsafe {
            let interface = self.body_interface.as_ptr();
            let count = raw.len() as u32;
            let removed = JPH_BodyInterface_RemoveBodies(interface, raw.as_ptr(), count);
            (
                removed,
                JPH_BodyInterface_DestroyBodies(interface, raw.as_ptr(), count),
            )
        };
        assert!(removed && destroyed, "checked bodies were not removed");
        for raw in &raw {
            self.pending_cache_invalidations.remove(raw);
        }
        self.wake_bodies_near_boxes(&boxes, None);
        Ok(())
    }

    /// `Ok` if `id` names a body of this world that is not added; [`BodyError::AlreadyAdded`]
    /// for one that is.
    fn check_detached(&self, id: BodyId) -> Result<(), BodyError> {
        match self.check(id) {
            Ok(()) => Err(BodyError::AlreadyAdded(id)),
            Err(BodyError::Detached(_)) => Ok(()),
            Err(error) => Err(error),
        }
    }

    /// The raw ids of `ids`, ascending; [`BodyError::DuplicateId`] for the smallest id listed
    /// more than once.
    pub(super) fn sorted_unique(&self, ids: &[BodyId]) -> Result<Vec<JPH_BodyID>, BodyError> {
        let mut sorted: Vec<JPH_BodyID> = ids.iter().map(|id| id.raw).collect();
        sorted.sort_unstable();
        match sorted.windows(2).find(|pair| pair[0] == pair[1]) {
            Some(pair) => Err(BodyError::DuplicateId(BodyId::new(pair[0], self.tag))),
            None => Ok(sorted),
        }
    }

    /// Prepares adding the bodies `ids`: distinct, detached bodies of this world.
    fn prepare_add(&mut self, ids: &[JPH_BodyID]) -> PreparedAdd<'_> {
        // SAFETY: the body interface belongs to this live world, borrowed mutably for as long as
        // the prepared batch lives; `ids` holds `ids.len()` ids, which the extension copies.
        let state = unsafe {
            JPH_BodyInterface_AddBodiesPrepare(
                self.body_interface.as_ptr(),
                ids.as_ptr(),
                ids.len() as u32,
            )
        };
        let state = NonNull::new(state).expect("checked bodies were refused for adding");
        PreparedAdd {
            body_interface: self.body_interface,
            state,
            _world: PhantomData,
        }
    }
}

/// A batch of bodies prepared for adding. [`finalize`](Self::finalize) adds them; dropping it
/// instead aborts the batch (Jolt `AddBodiesAbort`) and leaves the bodies detached. It borrows
/// the world mutably, so nothing adds, removes or destroys its bodies in between.
struct PreparedAdd<'w> {
    body_interface: NonNull<JPH_BodyInterface>,
    state: NonNull<JPH_AddBodiesState>,
    _world: PhantomData<&'w mut PhysicsWorld>,
}

impl PreparedAdd<'_> {
    /// Adds the prepared bodies to the broad phase; with [`Activation::Activate`] Jolt wakes
    /// those that are not static, in its own order.
    fn finalize(self, activation: Activation) {
        let this = ManuallyDrop::new(self);
        // SAFETY: the state comes from `prepare_add` on this live body interface and goes to
        // finalize exactly once: `ManuallyDrop` keeps `Drop` from aborting it as well.
        unsafe {
            JPH_BodyInterface_AddBodiesFinalize(
                this.body_interface.as_ptr(),
                this.state.as_ptr(),
                activation.to_jph(),
            )
        };
    }
}

impl Drop for PreparedAdd<'_> {
    fn drop(&mut self) {
        // SAFETY: the state comes from `prepare_add` on this live body interface and was not
        // finalized (`finalize` does not drop it), so it is aborted exactly once.
        unsafe {
            JPH_BodyInterface_AddBodiesAbort(self.body_interface.as_ptr(), self.state.as_ptr())
        };
    }
}

#[cfg(test)]
mod tests {
    use crate::*;

    /// Two detached cubes of a new world without gravity.
    fn detached_pair() -> (PhysicsWorld, [BodyId; 2]) {
        let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO)).unwrap();
        let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
        let mut create = |x: Real| {
            let settings = BodySettings::new_dynamic().position(RVec3::new(x, 0.0, 0.0));
            world.create_body_detached(&cube, &settings).unwrap()
        };
        let ids = [create(0.0), create(3.0)];
        (world, ids)
    }

    #[test]
    fn a_dropped_prepared_batch_is_aborted() {
        let (mut world, ids) = detached_pair();
        let sorted = world.sorted_unique(&ids).unwrap();
        drop(world.prepare_add(&sorted));
        assert!(ids.iter().all(|&id| !world.contains_body(id)));
        assert_eq!(world.check(ids[0]), Err(BodyError::Detached(ids[0])));

        world.add_bodies(&ids, Activation::Activate).unwrap();
        assert!(ids.iter().all(|&id| world.body(id).unwrap().is_active()));
    }

    #[test]
    fn finalize_activates_through_jolt() {
        let (mut world, ids) = detached_pair();
        let sorted = world.sorted_unique(&ids).unwrap();
        world.prepare_add(&sorted).finalize(Activation::Activate);
        assert!(ids.iter().all(|&id| world.body(id).unwrap().is_active()));
    }
}
