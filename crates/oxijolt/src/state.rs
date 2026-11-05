//! Saving and restoring a world's simulation state, for rollback and replay (Jolt
//! `PhysicsSystem::SaveState` and `RestoreState`, plus every character's own state).

mod record;
mod selection;

use std::fmt;
use std::mem::MaybeUninit;

use oxijolt_sys::*;

pub use selection::BodySelection;

use crate::world::WorldTag;
use crate::{CharacterState, PhysicsWorld, StateError};

/// A world's simulation state at one moment, from [`PhysicsWorld::save_state`],
/// [`PhysicsWorld::save_state_of`] or [`PhysicsWorld::save_state_into`], to go back to with
/// [`PhysicsWorld::restore_state`].
///
/// It holds Jolt's saved state of the physics system (bodies with their poses, velocities,
/// forces and sleep data, soft body vertices, the contact cache, each constraint's own state,
/// vehicles' driver input and drivetrain, tracked vehicles' track speeds and motorcycles' target
/// lean), every character's [`CharacterState`] and the contact-cache invalidations
/// ([`BodyMut::invalidate_contact_cache`](crate::BodyMut::invalidate_contact_cache)) that no
/// step has applied yet, which a state of some bodies ([`PhysicsWorld::save_state_of`]) holds
/// for every body too.
///
/// A state restores only into the world that saved it, and only while that world's structure is
/// unchanged: creating or removing a body, character, vehicle, ragdoll or constraint, setting a
/// body's or a ragdoll's motion type, setting a body's shape or sensor flag, changing a
/// character's or its inner body's shape
/// ([`CharacterMut::set_shape`](crate::CharacterMut::set_shape),
/// [`set_inner_body_shape`](crate::CharacterMut::set_inner_body_shape)), or a
/// [`rebase`](PhysicsWorld::rebase) that moves anything makes every earlier state unrestorable
/// ([`StateError::WorldChanged`]). Calls that fail their
/// checks keep states restorable.
///
/// Configuration Jolt does not save, such as a body's friction, damping, object layer and
/// collision group, constraint limits, motor settings, a vehicle's gravity override, soft
/// body vertex inverse masses and a character's settings such as its mass, stays as it is when a state is restored. So does a vehicle's world up, which a zero gravity keeps from the step before. Jolt
/// does not save a motorcycle's integrated lean angle, which is why its integration coefficient
/// must be 0.
/// [docs/state.md] lists every such setter.
///
/// A state is opaque: it gives neither its bytes nor `==`, because Jolt's stream may hold bytes
/// without a defined value (a wheel's contact data before its first contact). Compare what a
/// world reports instead.
///
/// For rollback, allocate the states once ([`WorldState::new`]) and save into them with
/// [`PhysicsWorld::save_state_into`], which reuses their memory.
///
/// [docs/state.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md
#[derive(Clone)]
pub struct WorldState {
    /// The world that saved it; `None` for a state no world saved ([`WorldState::new`]).
    world: Option<WorldTag>,
    epoch: u64,
    body_count: u32,
    constraint_count: u32,
    character_ids: Vec<u32>,
    /// Jolt's stream, which may hold bytes without a defined value (see above).
    jolt: Vec<MaybeUninit<u8>>,
    /// The saved state of each character of `character_ids`, in that order; entries after them
    /// are buffers of earlier saves, kept to reuse their memory.
    characters: Vec<CharacterState>,
    /// Raw ids of the bodies with a pending contact-cache invalidation, in ascending order.
    cache_invalidations: Vec<u32>,
    /// The raw ids of the last save's selected bodies, kept to reuse its memory.
    selection: Vec<u32>,
}

impl WorldState {
    /// A state that no world saved, holding no memory yet: a buffer for
    /// [`PhysicsWorld::save_state_into`]. Restoring it fails with [`StateError::WrongWorld`].
    pub fn new() -> Self {
        Self {
            world: None,
            epoch: 0,
            body_count: 0,
            constraint_count: 0,
            character_ids: Vec::new(),
            jolt: Vec::new(),
            characters: Vec::new(),
            cache_invalidations: Vec::new(),
            selection: Vec::new(),
        }
    }

    /// The size in bytes of Jolt's saved stream, which makes up most of a state; 0 for
    /// [`WorldState::new`]. [docs/state.md] lists what each body, contact and constraint adds.
    ///
    /// [docs/state.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#size
    pub fn data_size(&self) -> usize {
        self.jolt.len()
    }
}

impl Default for WorldState {
    /// The same as [`WorldState::new`].
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Debug for WorldState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("WorldState")
            .field("epoch", &self.epoch)
            .field("body_count", &self.body_count)
            .field("constraint_count", &self.constraint_count)
            .field("characters", &self.character_ids.len())
            .field("jolt_bytes", &self.jolt.len())
            .field("cache_invalidations", &self.cache_invalidations.len())
            .field("saved", &self.world.is_some())
            .finish_non_exhaustive()
    }
}

impl PhysicsWorld {
    /// Saves the whole simulation state of the world, every body included. See [`WorldState`]
    /// for what it holds and when it can be restored.
    ///
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let ball_shape = Shape::new_sphere(0.5)?;
    /// let ball = world.create_body(
    ///     &ball_shape,
    ///     &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
    /// )?;
    ///
    /// let saved = world.save_state();
    /// assert!(world.step(1.0 / 60.0)?.is_complete());
    /// let after_one_step = world.body(ball)?.position();
    ///
    /// world.restore_state(&saved)?;
    /// assert_eq!(world.body(ball)?.position(), RVec3::new(0.0, 2.0, 0.0));
    /// assert!(world.step(1.0 / 60.0)?.is_complete());
    /// assert_eq!(world.body(ball)?.position(), after_one_step);
    /// # Ok(())
    /// # }
    /// ```
    pub fn save_state(&self) -> WorldState {
        let mut state = WorldState::new();
        self.record_into(None, &mut state.jolt);
        self.save_world_parts_into(&mut state);
        state
    }

    /// Saves the simulation state with only the bodies `bodies` selects; global state, contacts,
    /// constraints and characters are saved whole.
    ///
    /// Restoring it leaves every other body in its current state, except that a character's
    /// inner body moves to the restored character's pose, so a replay from it is exact
    /// only when those bodies did not change since the save, for example static bodies the caller
    /// never moved. Fails with [`StateError::Body`] for an id of another world or a removed body,
    /// before anything is saved.
    pub fn save_state_of(&self, bodies: BodySelection<'_>) -> Result<WorldState, StateError> {
        let mut state = WorldState::new();
        self.save_state_into(bodies, &mut state)?;
        Ok(state)
    }

    /// Saves what [`save_state_of`](Self::save_state_of) saves into `state`, reusing the memory
    /// `state` holds.
    ///
    /// `state` may be new ([`WorldState::new`]), of another world or of this one; it is
    /// overwritten and then belongs to this world. `state` keeps every buffer it grows, so the
    /// call allocates on the Rust heap only for what no earlier save into `state` needed: a
    /// longer Jolt stream, more bodies, more characters or a larger character state, more
    /// pending contact-cache invalidations, or a [`BodySelection::Only`] list longer than the
    /// world's body count. Jolt's recorder still allocates its own stream for each save. On an
    /// error `state` is unchanged.
    ///
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let ball_shape = Shape::new_sphere(0.5)?;
    /// world.create_body(&ball_shape, &BodySettings::new_dynamic())?;
    ///
    /// // One buffer per tick of the rollback window, allocated before the game runs.
    /// let mut ring = vec![WorldState::new(); 8];
    /// for tick in 0..60 {
    ///     world.save_state_into(BodySelection::All, &mut ring[tick % 8])?;
    ///     assert!(world.step(1.0 / 60.0)?.is_complete());
    /// }
    /// // Back to the start of tick 55.
    /// world.restore_state(&ring[55 % 8])?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn save_state_into(
        &self,
        bodies: BodySelection<'_>,
        state: &mut WorldState,
    ) -> Result<(), StateError> {
        if let BodySelection::All = bodies {
            // Sized here so that a later `Movable` or `Only` save needs no more memory.
            state.selection.clear();
            state.selection.reserve(self.body_count() as usize);
        }
        let listed = self.select_bodies(bodies, &mut state.selection)?;
        self.record_into(listed, &mut state.jolt);
        self.save_world_parts_into(state);
        Ok(())
    }

    /// Returns the world to `state`. See [`WorldState`] for what this restores and what it
    /// leaves alone.
    ///
    /// Fails with [`StateError::WrongWorld`] for a state of another world and with
    /// [`StateError::WorldChanged`] when the world's structure changed since the save; the world
    /// is unchanged then. [`StateError::RestoreFailed`] is not expected for a state the world
    /// accepts; if it happens, the world may be partly restored.
    ///
    /// After a restore, the same calls as in the original run give bit-identical results, with
    /// the configuration and the vehicle world up in zero gravity that [`WorldState`] lists as
    /// not saved left aside; the tests check this with 1 and 4 worker threads, in one process
    /// and across two.
    pub fn restore_state(&mut self, state: &WorldState) -> Result<(), StateError> {
        self.check_restorable(state)?;
        self.restore_checked(state)
    }

    /// Returns the world to `state` for the bodies `bodies` selects; every other body keeps its
    /// current state bit for bit. Global state, contacts, constraints, characters and pending
    /// contact-cache invalidations come from `state` whole, as in
    /// [`restore_state`](Self::restore_state). A character's inner body always counts as
    /// selected, because restoring the character moves it. A selected body that `state` does not
    /// hold (see [`save_state_of`](Self::save_state_of)) keeps its current state too.
    ///
    /// The same calls give the same result with 1 and 4 worker threads (the tests compare two
    /// processes), but the world is not one that never left `state`: the restored contacts were
    /// made at the saved poses, also for bodies that now stay where they are.
    /// [`BodySelection::All`] is [`restore_state`](Self::restore_state).
    ///
    /// Fails like `restore_state`, and with [`StateError::Body`] for an id of another world or a
    /// removed body; the world is unchanged then.
    ///
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let ball_shape = Shape::new_sphere(0.5)?;
    /// let settings = BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0));
    /// let ball = world.create_body(&ball_shape, &settings)?;
    /// let other = world.create_body(&ball_shape, &settings.position(RVec3::new(3.0, 2.0, 0.0)))?;
    ///
    /// let saved = world.save_state();
    /// assert!(world.step(1.0 / 60.0)?.is_complete());
    /// let other_now = world.body(other)?.position();
    ///
    /// world.restore_state_of(&saved, BodySelection::Only(&[ball]))?;
    /// assert_eq!(world.body(ball)?.position(), RVec3::new(0.0, 2.0, 0.0));
    /// assert_eq!(world.body(other)?.position(), other_now);
    /// # Ok(())
    /// # }
    /// ```
    pub fn restore_state_of(
        &mut self,
        state: &WorldState,
        bodies: BodySelection<'_>,
    ) -> Result<(), StateError> {
        self.check_restorable(state)?;
        let mut selection = Vec::new();
        let Some(selected) = self.select_bodies(bodies, &mut selection)? else {
            return self.restore_checked(state);
        };
        let mut kept = Vec::new();
        self.unselected_body_ids_into(selected, &mut kept);
        let mut current = Vec::new();
        self.record_bodies_into(&kept, &mut current);

        self.restore_checked(state)?;
        // SAFETY: `current` was recorded above from this world, and `restore_checked` adds and
        // removes nothing.
        let restored = unsafe { self.restore_jolt(&current) };
        debug_assert!(restored, "the unselected bodies failed to restore");
        if restored {
            Ok(())
        } else {
            Err(StateError::RestoreFailed)
        }
    }

    /// `Ok` if `state` restores into this world now: saved by it at the current structure.
    fn check_restorable(&self, state: &WorldState) -> Result<(), StateError> {
        if state.world != Some(self.tag) {
            return Err(StateError::WrongWorld);
        }
        let same_characters = self.characters.keys().eq(state.character_ids.iter());
        if state.epoch != self.structure_epoch
            || state.body_count != self.body_count()
            || state.constraint_count != self.jolt_constraint_count()
            || !same_characters
        {
            return Err(StateError::WorldChanged);
        }
        Ok(())
    }

    /// Restores `state`, which [`check_restorable`](Self::check_restorable) accepted.
    fn restore_checked(&mut self, state: &WorldState) -> Result<(), StateError> {
        // The characters go first, so that the system restore has the last word on their inner
        // bodies: restoring a character moves its inner body to the character's pose, which
        // resets that body's sleep timer. Inner bodies are kinematic and Jolt runs no sleep test
        // on kinematic bodies, so the order does not change any step result today.
        let ids: Vec<_> = self.character_ids().collect();
        for (id, character_state) in ids.into_iter().zip(&state.characters) {
            let restored = self
                .character_mut(id)
                .map_err(|_| ())
                .and_then(|mut character| character.restore_state(character_state).map_err(|_| ()));
            if restored.is_err() {
                return Err(StateError::RestoreFailed);
            }
            // Restored contacts point to Jolt's default material.
            self.retain_contact_materials(id);
        }

        // SAFETY: `state` was saved by this world (`record_into`, the only writer of a
        // `WorldState`'s stream) at the current structure epoch, which `check_restorable`
        // checked.
        let restored = unsafe { self.restore_jolt(&state.jolt) };
        debug_assert!(restored, "a saved world state failed to restore");
        if restored {
            self.pending_cache_invalidations = state.cache_invalidations.iter().copied().collect();
            Ok(())
        } else {
            Err(StateError::RestoreFailed)
        }
    }

    /// Number of constraints in the Jolt system, vehicles and ragdoll joints included.
    fn jolt_constraint_count(&self) -> u32 {
        // SAFETY: the system is live; Jolt counts under its constraint mutex.
        unsafe { JPH_PhysicsSystem_GetNumConstraints(self.system.as_ptr()) }
    }

    /// Writes everything of a state of this world now but Jolt's stream into `state`, reusing
    /// its memory.
    fn save_world_parts_into(&self, state: &mut WorldState) {
        state.world = Some(self.tag);
        state.epoch = self.structure_epoch;
        state.body_count = self.body_count();
        state.constraint_count = self.jolt_constraint_count();
        state.character_ids.clear();
        state.character_ids.extend(self.characters.keys());
        for (index, id) in self.character_ids().enumerate() {
            let character = self
                .character(id)
                .unwrap_or_else(|_| unreachable!("listed by the world"));
            match state.characters.get_mut(index) {
                Some(saved) => character.save_state_into(saved),
                None => state.characters.push(character.save_state()),
            }
        }
        state.cache_invalidations.clear();
        state
            .cache_invalidations
            .extend(&self.pending_cache_invalidations);
    }
}
