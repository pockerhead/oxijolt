//! Which bodies a world state saves or restores: [`BodySelection`] and the id lists it resolves to.

use oxijolt_sys::*;

use crate::body::INVALID_BODY_ID;
use crate::{BodyId, MotionType, PhysicsWorld, StateError};

/// Which bodies [`PhysicsWorld::save_state_of`] and [`PhysicsWorld::save_state_into`] save, and
/// which bodies [`PhysicsWorld::restore_state_of`] restores.
///
/// Global state, contacts, constraints, characters and pending contact-cache invalidations are
/// saved and restored whole whatever the selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum BodySelection<'a> {
    /// Every body.
    All,
    /// Every body that is not static, asleep or awake: dynamic, kinematic and soft bodies,
    /// ragdoll parts and characters' inner bodies. Static bodies are left out, which makes the
    /// state smaller; a static body the caller moves after the save keeps its new pose when the
    /// state is restored.
    Movable,
    /// The listed bodies, in any order, duplicates allowed; no body for an empty slice. Every id
    /// must name a body of this world, or the call fails with [`StateError::Body`].
    Only(&'a [BodyId]),
}

impl PhysicsWorld {
    /// Writes the raw ids of the bodies `selection` names into `ids`, ascending and without
    /// duplicates, and returns them; `None` for [`BodySelection::All`], which leaves `ids` alone.
    /// Checks every listed id before it touches `ids`.
    pub(super) fn select_bodies<'v>(
        &self,
        selection: BodySelection<'_>,
        ids: &'v mut Vec<u32>,
    ) -> Result<Option<&'v [u32]>, StateError> {
        match selection {
            BodySelection::All => return Ok(None),
            BodySelection::Movable => {
                self.all_body_ids_into(ids);
                ids.retain(|&raw| self.raw_motion_type(raw) != MotionType::Static);
            }
            BodySelection::Only(listed) => {
                for &id in listed {
                    self.check(id).map_err(StateError::Body)?;
                }
                ids.clear();
                ids.extend(listed.iter().map(|id| id.to_raw()));
                ids.sort_unstable();
                ids.dedup();
            }
        }
        Ok(Some(ids))
    }

    /// Writes the raw ids of every body of the world into `ids`, ascending, reusing its memory.
    pub(super) fn all_body_ids_into(&self, ids: &mut Vec<u32>) {
        let count = self.body_count();
        ids.clear();
        ids.resize(count as usize, INVALID_BODY_ID);
        if count > 0 {
            // SAFETY: the system is live and `ids` holds `count` ids. joltc writes `count` ids
            // without clamping them to the bodies it lists (joltc.cpp:5759-5772), so `count` must
            // be at most that number: it is `GetNumBodies`, which Jolt's list matches
            // (`BodyManager::GetBodyIDs` asserts it), and no body is added or removed meanwhile,
            // because every call that creates or destroys one (`create_body`, `create_body_detached`,
            // `create_soft_body`, `create_character`, `create_ragdoll`, their removals,
            // `destroy_body`) takes `&mut self` and this borrow is shared. Jolt copies the list under its bodies mutex.
            unsafe { JPH_PhysicsSystem_GetBodies(self.system.as_ptr(), ids.as_mut_ptr(), count) };
        }
        ids.sort_unstable();
    }

    /// Writes the raw ids of the bodies outside `selected` (ascending raw ids) into `ids`,
    /// ascending, leaving out characters' inner bodies, which follow their characters.
    pub(super) fn unselected_body_ids_into(&self, selected: &[u32], ids: &mut Vec<u32>) {
        self.all_body_ids_into(ids);
        ids.retain(|raw| selected.binary_search(raw).is_err() && !self.inner_bodies.contains(raw));
    }

    /// The motion type of the body with the raw id `raw`, one of this world's bodies.
    pub(super) fn raw_motion_type(&self, raw: u32) -> MotionType {
        // SAFETY: the body interface belongs to this live world; the locking interface checks
        // the id and reads the motion type under the body's read lock, and this thread holds no
        // body lock.
        MotionType::from_jph(unsafe {
            JPH_BodyInterface_GetMotionType(self.body_interface.as_ptr(), raw)
        })
    }
}
