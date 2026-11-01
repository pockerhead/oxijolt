//! Collision groups (Jolt `CollisionGroup` and `GroupFilterTable`): which bodies the solver lets
//! collide, beyond their object layers.
//!
//! A [`GroupFilterTable`] says which of its sub groups collide with each other; it is built once
//! with a [`GroupFilterTableBuilder`] and never changes afterwards, so bodies and Jolt's worker
//! threads can share it. A [`CollisionGroup`] puts a body into a group of a table, given to the
//! body at creation with [`BodySettings::collision_group`](crate::BodySettings::collision_group)
//! or [`SoftBodySettings::collision_group`](crate::SoftBodySettings::collision_group), or later
//! with [`BodyMut::set_collision_group`](crate::BodyMut::set_collision_group).

use std::fmt;
use std::sync::Arc;

use oxijolt_sys::*;

use crate::owned::{JoltObject, Owned};
use crate::world::ensure_initialized;
use crate::CollisionGroupError;

/// A group filter table, owned with one reference: the one `JPH_GroupFilterTable_Create`
/// returns. Creation settings that use the table hold their own reference while they exist, and
/// each body created with it holds one for its life.
impl JoltObject for JPH_GroupFilterTable {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract). `GroupFilterTable` derives from
        // `GroupFilter` with single inheritance, as joltc's header notes, so the pointer is the
        // `GroupFilter` that `Release` acts on.
        unsafe { JPH_GroupFilter_Destroy(ptr.cast()) };
    }
}

/// Builds a [`GroupFilterTable`]: which of its sub groups collide with each other.
///
/// ```
/// use oxijolt::{CollisionGroup, GroupFilterTableBuilder};
///
/// # fn main() -> Result<(), oxijolt::CollisionGroupError> {
/// // A chain of three links in which neighbours do not collide.
/// let mut builder = GroupFilterTableBuilder::new(3)?;
/// builder.disable_collision(0, 1)?;
/// builder.disable_collision(1, 2)?;
/// let table = builder.build();
/// assert!(table.is_collision_enabled(0, 2)?);
/// let links: Vec<CollisionGroup> = (0..3)
///     .map(|link| CollisionGroup::new(&table, 1, link))
///     .collect::<Result<_, _>>()?;
/// assert!(!links[0].can_collide(&links[1]));
/// assert!(links[0].can_collide(&links[2]));
/// # Ok(())
/// # }
/// ```
pub struct GroupFilterTableBuilder {
    table: Owned<JPH_GroupFilterTable>,
    sub_groups: u32,
}

impl GroupFilterTableBuilder {
    /// A table of `sub_groups` sub groups (ids `0..sub_groups`) in which every pair of different
    /// sub groups collides; a sub group never collides with itself. Refused for 0 sub groups and
    /// for more than [`GroupFilterTable::MAX_SUB_GROUPS`].
    pub fn new(sub_groups: u32) -> Result<Self, CollisionGroupError> {
        if sub_groups == 0 {
            return Err(CollisionGroupError::NoSubGroups);
        }
        if sub_groups > GroupFilterTable::MAX_SUB_GROUPS {
            return Err(CollisionGroupError::TooManySubGroups(sub_groups));
        }
        if !ensure_initialized() {
            return Err(CollisionGroupError::InitFailed);
        }
        // SAFETY: Jolt is initialised (its allocation hooks are set). The handle takes over the
        // one reference joltc's `JPH_GroupFilterTable_Create` returns.
        let table = unsafe { Owned::from_raw(JPH_GroupFilterTable_Create(sub_groups)) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the table"));
        Ok(Self { table, sub_groups })
    }

    /// Makes the sub groups `a` and `b` (in either order) not collide.
    pub fn disable_collision(&mut self, a: u32, b: u32) -> Result<(), CollisionGroupError> {
        self.check_pair(a, b)?;
        // SAFETY: the table is live and only this builder references it; both sub groups are
        // below its size and differ, as `GroupFilterTable::GetBit` asserts.
        unsafe { JPH_GroupFilterTable_DisableCollision(self.table.as_ptr(), a, b) };
        Ok(())
    }

    /// Makes the sub groups `a` and `b` (in either order) collide again.
    pub fn enable_collision(&mut self, a: u32, b: u32) -> Result<(), CollisionGroupError> {
        self.check_pair(a, b)?;
        // SAFETY: as in `disable_collision`.
        unsafe { JPH_GroupFilterTable_EnableCollision(self.table.as_ptr(), a, b) };
        Ok(())
    }

    /// The table, which no longer changes.
    pub fn build(self) -> GroupFilterTable {
        GroupFilterTable(Arc::new(TableInner {
            table: self.table,
            sub_groups: self.sub_groups,
        }))
    }

    fn check_pair(&self, a: u32, b: u32) -> Result<(), CollisionGroupError> {
        check_pair(self.sub_groups, a, b)
    }
}

impl fmt::Debug for GroupFilterTableBuilder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GroupFilterTableBuilder")
            .field("sub_groups", &self.sub_groups)
            .finish_non_exhaustive()
    }
}

/// Both sub groups below `sub_groups`, and different.
fn check_pair(sub_groups: u32, a: u32, b: u32) -> Result<(), CollisionGroupError> {
    check_sub_group(sub_groups, a)?;
    check_sub_group(sub_groups, b)?;
    if a == b {
        return Err(CollisionGroupError::SameSubGroup(a));
    }
    Ok(())
}

fn check_sub_group(sub_groups: u32, id: u32) -> Result<(), CollisionGroupError> {
    if id >= sub_groups {
        return Err(CollisionGroupError::SubGroupOutOfRange(id));
    }
    Ok(())
}

// SAFETY: the builder owns its native table alone (no group or body references it before
// `build`), and Jolt's `GroupFilterTable` has no thread affinity; `&mut self` serialises edits.
unsafe impl Send for GroupFilterTableBuilder {}

/// Which sub groups of a collision group collide with each other (Jolt `GroupFilterTable`),
/// built with a [`GroupFilterTableBuilder`] and fixed from then on. Cloning shares the table.
///
/// Two tables are equal only when they are the same table, as Jolt compares group filters by
/// pointer: two groups with the same group id but different tables never collide.
#[derive(Clone)]
pub struct GroupFilterTable(Arc<TableInner>);

/// The native table and its size.
struct TableInner {
    table: Owned<JPH_GroupFilterTable>,
    sub_groups: u32,
}

// SAFETY: the native table is never written after `GroupFilterTableBuilder::build`; Jolt's
// `RefTarget` count is atomic, and Jolt's own worker threads call the const `CanCollide`
// concurrently during a step (`Body.inl:74-76`, `PhysicsSystem.cpp:1992`).
unsafe impl Send for TableInner {}
// SAFETY: as for `Send`: every access after `build` only reads the table or changes its atomic
// reference count.
unsafe impl Sync for TableInner {}

impl GroupFilterTable {
    /// The most sub groups a table may have, an oxijolt bound: the table takes about n²/16
    /// bytes, and Jolt's `int` bit index (`GroupFilterTable::GetBit`) overflows above 65 536.
    /// See [docs/limits.md#group-filter-table-size].
    ///
    /// [docs/limits.md#group-filter-table-size]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#group-filter-table-size
    pub const MAX_SUB_GROUPS: u32 = 4096;

    /// How many sub groups the table has; their ids are `0..sub_groups()`.
    pub fn sub_groups(&self) -> u32 {
        self.0.sub_groups
    }

    /// Whether the sub groups `a` and `b` collide; a sub group never collides with itself.
    /// Refused for a sub group not in the table.
    pub fn is_collision_enabled(&self, a: u32, b: u32) -> Result<bool, CollisionGroupError> {
        check_sub_group(self.0.sub_groups, a)?;
        check_sub_group(self.0.sub_groups, b)?;
        if a == b {
            return Ok(false);
        }
        // SAFETY: the table is live and no longer written; both sub groups are below its size
        // and differ. joltc's parameter is mutable but the call is Jolt's const
        // `IsCollisionEnabled`.
        Ok(unsafe { JPH_GroupFilterTable_IsCollisionEnabled(self.as_ptr(), a, b) })
    }

    fn as_ptr(&self) -> *mut JPH_GroupFilterTable {
        self.0.table.as_ptr()
    }
}

impl PartialEq for GroupFilterTable {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

impl Eq for GroupFilterTable {}

impl fmt::Debug for GroupFilterTable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GroupFilterTable")
            .field("sub_groups", &self.0.sub_groups)
            .finish_non_exhaustive()
    }
}

/// The collision group of a body (Jolt `CollisionGroup`): a group id and a sub group of a
/// [`GroupFilterTable`].
///
/// Jolt's rule for two bodies that both have a group: different group ids collide; the same
/// group id with different tables never collides; within one table the same sub group never
/// collides, and different sub groups collide when the table says so. A body without a group
/// collides by its object layer alone. The group only filters what the solver collides: scene
/// queries, character movement and vehicle wheel casts do not look at it.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct CollisionGroup {
    table: GroupFilterTable,
    group_id: u32,
    sub_group_id: u32,
}

impl CollisionGroup {
    /// The largest group id a caller may use. The ids above it belong to ragdolls: the parts of
    /// a ragdoll use group id `2^31 +` [`RagdollId::to_raw`](crate::RagdollId::to_raw).
    pub const MAX_GROUP_ID: u32 = (1 << 31) - 1;

    /// The group `group_id` with sub group `sub_group_id` of `table`. Refused for a group id
    /// above [`MAX_GROUP_ID`](Self::MAX_GROUP_ID) and a sub group not in the table.
    pub fn new(
        table: &GroupFilterTable,
        group_id: u32,
        sub_group_id: u32,
    ) -> Result<Self, CollisionGroupError> {
        if group_id > Self::MAX_GROUP_ID {
            return Err(CollisionGroupError::GroupIdOutOfRange(group_id));
        }
        check_sub_group(table.sub_groups(), sub_group_id)?;
        Ok(Self {
            table: table.clone(),
            group_id,
            sub_group_id,
        })
    }

    /// The table of the group.
    pub fn table(&self) -> &GroupFilterTable {
        &self.table
    }

    /// The group id.
    pub fn group_id(&self) -> u32 {
        self.group_id
    }

    /// The sub group, below the table's [`sub_groups`](GroupFilterTable::sub_groups).
    pub fn sub_group_id(&self) -> u32 {
        self.sub_group_id
    }

    /// Whether bodies of the two groups collide, by Jolt's rule (`CollisionGroup::CanCollide`).
    pub fn can_collide(&self, other: &CollisionGroup) -> bool {
        let (a, b) = (self.to_jph(), other.to_jph());
        // SAFETY: both tables are live, and `a` and `b` are live locals whose sub groups are
        // below their tables' sizes (`new`). joltc builds two temporary Jolt groups, each adding
        // and releasing a reference to its table within the call; the table is only read.
        unsafe { JPH_GroupFilter_CanCollide(self.table.as_ptr().cast(), &a, &b) }
    }

    /// The group as joltc's value; the table pointer stays owned by `self`.
    pub(crate) fn to_jph(&self) -> JPH_CollisionGroup {
        JPH_CollisionGroup {
            // `GroupFilterTable` derives from `GroupFilter` with single inheritance.
            groupFilter: self.table.as_ptr().cast_const().cast(),
            groupID: self.group_id,
            subGroupID: self.sub_group_id,
        }
    }
}

/// Jolt's default `CollisionGroup`, which a body created without a group has: no table, and
/// `cInvalidGroup` and `cInvalidSubGroup` as ids (`CollisionGroup.h:27-28`).
pub(crate) const NO_COLLISION_GROUP: JPH_CollisionGroup = JPH_CollisionGroup {
    groupFilter: std::ptr::null(),
    groupID: u32::MAX,
    subGroupID: u32::MAX,
};

/// Whether `a` and `b` are the same group: the same table pointer and the same ids. Neither
/// table is dereferenced.
pub(crate) fn same_group(a: &JPH_CollisionGroup, b: &JPH_CollisionGroup) -> bool {
    std::ptr::eq(a.groupFilter, b.groupFilter)
        && a.groupID == b.groupID
        && a.subGroupID == b.subGroupID
}
