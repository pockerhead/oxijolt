//! Reading the poses, and the poses with velocities, of every awake body at once.

use std::ptr::NonNull;

use oxijolt_sys::*;

use super::BodyId;
use crate::owned::Owned;
use crate::{PhysicsWorld, Quat, RVec3, Vec3};

/// The pose of one awake body, as returned by [`PhysicsWorld::active_body_poses`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct BodyPose {
    /// The body.
    pub id: BodyId,
    /// The body's origin in world space, as [`BodyRef::position`] (not its centre of mass).
    ///
    /// [`BodyRef::position`]: crate::BodyRef::position
    pub position: RVec3,
    /// The body's rotation, as [`BodyRef::rotation`].
    ///
    /// [`BodyRef::rotation`]: crate::BodyRef::rotation
    pub rotation: Quat,
}

/// The pose and velocities of one awake body, as returned by
/// [`PhysicsWorld::active_body_states`].
///
/// Every field is bit for bit what the matching [`BodyRef`] getter returns for the same body at
/// the same time. For a soft body the velocities are the body's own, as [`BodyRef`] reports them,
/// not those of its vertices, which come from
/// [`SoftBodyRef::vertices`](crate::SoftBodyRef::vertices).
///
/// [`BodyRef`]: crate::BodyRef
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct BodyMotionState {
    /// The body.
    pub id: BodyId,
    /// The body's origin in world space, in metres, as [`BodyRef::position`] (not its centre of
    /// mass).
    ///
    /// [`BodyRef::position`]: crate::BodyRef::position
    pub position: RVec3,
    /// The body's rotation, as [`BodyRef::rotation`].
    ///
    /// [`BodyRef::rotation`]: crate::BodyRef::rotation
    pub rotation: Quat,
    /// The velocity of the centre of mass in m/s, as [`BodyRef::linear_velocity`]; for a soft
    /// body the average of its vertex velocities.
    ///
    /// [`BodyRef::linear_velocity`]: crate::BodyRef::linear_velocity
    pub linear_velocity: Vec3,
    /// The angular velocity in rad/s, as [`BodyRef::angular_velocity`]; for a soft body the
    /// average of `p × v` over its vertices (in m²/s, relative to the body origin).
    ///
    /// [`BodyRef::angular_velocity`]: crate::BodyRef::angular_velocity
    pub angular_velocity: Vec3,
}

impl PhysicsWorld {
    /// The id, position and rotation of every awake body, in ascending [`BodyId`] order.
    ///
    /// The order does not depend on the worker threads or on the order in which Jolt woke the
    /// bodies. The poses are those of the bodies awake at the time of the call, which is not
    /// the set of bodies that moved: static and sleeping bodies are absent, including a body
    /// that moved in the last step and fell asleep at its end (its [`Deactivated`] event
    /// reports it) and a body moved while asleep. The [guide] says how a game syncs those.
    /// Characters' inner bodies, ragdoll parts, vehicle chassis and soft bodies appear like any
    /// other awake body. A soft body's pose is its body transform; its deformed vertices come
    /// from [`SoftBodyRef::vertices`](crate::SoftBodyRef::vertices). A character's inner body
    /// has its own pose, which is not the character's position.
    ///
    /// One call takes one multi-body read lock over the awake bodies instead of one lock per
    /// body as [`body`](Self::body) does; once there are at least as many awake bodies as Jolt
    /// has body mutexes, that lock takes every body mutex shared. To reuse the result's
    /// allocation, use [`active_body_poses_into`](Self::active_body_poses_into); each call
    /// still allocates the list of awake ids and Jolt's lock.
    ///
    /// ```
    /// use oxijolt::prelude::math::*;
    /// use oxijolt::prelude::*;
    ///
    /// # fn main() -> oxijolt::error::Result<()> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let ball = world.create_body(
    ///     &Shape::new_sphere(0.5)?,
    ///     &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
    /// )?;
    /// assert!(world.step(1.0 / 60.0)?.is_complete());
    ///
    /// let poses = world.active_body_poses();
    /// assert_eq!(poses.len(), 1);
    /// assert_eq!(poses[0].id, ball);
    /// assert!(poses[0].position.y < 2.0);
    /// # Ok(())
    /// # }
    /// ```
    ///
    /// [`Deactivated`]: crate::ActivationEvent::Deactivated
    /// [guide]: https://github.com/pockerhead/oxijolt/blob/main/docs/guide.md#stepping
    pub fn active_body_poses(&self) -> Vec<BodyPose> {
        let mut poses = Vec::new();
        self.active_body_poses_into(&mut poses);
        poses
    }

    /// Like [`active_body_poses`](Self::active_body_poses), into `out`, which is cleared first;
    /// reusing it avoids reallocating the result.
    pub fn active_body_poses_into(&self, out: &mut Vec<BodyPose>) {
        self.read_active_bodies(out, |id, body| {
            let mut position = RVec3::ZERO.to_jph();
            let mut rotation = Quat::IDENTITY.to_jph();
            // SAFETY: `read_active_bodies` passes a body locked for reading for this call; the
            // getters only read it, and `position` and `rotation` are live locals.
            unsafe {
                JPH_Body_GetPosition(body.as_ptr(), &mut position);
                JPH_Body_GetRotation(body.as_ptr(), &mut rotation);
            }
            BodyPose {
                id,
                position: RVec3::from_jph(position),
                rotation: Quat::from_jph(rotation),
            }
        });
    }

    /// The id, position, rotation and both velocities of every awake body, in ascending
    /// [`BodyId`] order, read under one multi-body read lock.
    ///
    /// The same bodies in the same order as [`active_body_poses`](Self::active_body_poses), whose
    /// documentation says which bodies are absent and how a game syncs them. A body created with
    /// [`create_body_detached`](Self::create_body_detached) is absent until it is added: Jolt
    /// wakes only bodies in the world. Reading the velocities here instead of through
    /// [`body`](Self::body) saves a body lock per getter. To reuse the result's allocation, use
    /// [`active_body_states_into`](Self::active_body_states_into).
    ///
    /// ```
    /// use oxijolt::prelude::math::*;
    /// use oxijolt::prelude::*;
    ///
    /// # fn main() -> oxijolt::error::Result<()> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let ball = world.create_body(
    ///     &Shape::new_sphere(0.5)?,
    ///     &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
    /// )?;
    /// assert!(world.step(1.0 / 60.0)?.is_complete());
    ///
    /// let states = world.active_body_states();
    /// assert_eq!(states.len(), 1);
    /// assert_eq!(states[0].id, ball);
    /// assert!(states[0].linear_velocity.y < 0.0);
    /// # Ok(())
    /// # }
    /// ```
    pub fn active_body_states(&self) -> Vec<BodyMotionState> {
        let mut states = Vec::new();
        self.active_body_states_into(&mut states);
        states
    }

    /// Like [`active_body_states`](Self::active_body_states), into `out`, which is cleared
    /// first; reusing it avoids reallocating the result.
    pub fn active_body_states_into(&self, out: &mut Vec<BodyMotionState>) {
        self.read_active_bodies(out, |id, body| {
            let mut position = RVec3::ZERO.to_jph();
            let mut rotation = Quat::IDENTITY.to_jph();
            let mut linear_velocity = Vec3::ZERO.to_jph();
            let mut angular_velocity = Vec3::ZERO.to_jph();
            // SAFETY: `read_active_bodies` passes a body locked for reading for this call. The
            // getters only read it: the velocity getters take a mutable pointer but call Jolt's
            // const `Body::GetLinearVelocity` and `GetAngularVelocity` (joltc.cpp). The outputs
            // are live locals.
            unsafe {
                JPH_Body_GetPosition(body.as_ptr(), &mut position);
                JPH_Body_GetRotation(body.as_ptr(), &mut rotation);
                JPH_Body_GetLinearVelocity(body.as_ptr(), &mut linear_velocity);
                JPH_Body_GetAngularVelocity(body.as_ptr(), &mut angular_velocity);
            }
            BodyMotionState {
                id,
                position: RVec3::from_jph(position),
                rotation: Quat::from_jph(rotation),
                linear_velocity: Vec3::from_jph(linear_velocity),
                angular_velocity: Vec3::from_jph(angular_velocity),
            }
        });
    }

    /// Clears `out`, then pushes `read(id, body)` for every awake body in ascending id order,
    /// all of them locked for reading under one multi-body read lock that outlives every `read`
    /// call. `read` only reads the body it is given and takes no lock.
    fn read_active_bodies<T>(
        &self,
        out: &mut Vec<T>,
        mut read: impl FnMut(BodyId, NonNull<JPH_Body>) -> T,
    ) {
        out.clear();
        // Jolt ranks its locks (`PhysicsLock.h`): a body lock may not be taken while the
        // higher-ranked active-list mutex is held. The id copies release that mutex before the
        // bodies are locked.
        let ids = self.active_body_ids();
        if ids.is_empty() {
            return;
        }
        out.reserve(ids.len());
        // SAFETY: the lock interface belongs to this live world. joltc copies the ids into the
        // lock object, so `ids` only has to live for the call. This thread holds no body lock
        // and no active-list lock here: both id copies have returned, and `read` runs only
        // after this call. The handle takes over the lock and releases it when it goes
        // out of scope. The count fits `u32`: worlds hold at most `1 << 23` bodies (validated in
        // `WorldSettings`).
        let lock = unsafe {
            Owned::from_raw(JPH_BodyLockInterface_LockMultiRead(
                self.body_lock_interface.as_ptr(),
                ids.as_ptr(),
                ids.len() as u32,
            ))
        };
        let Some(lock) = lock else {
            return;
        };
        for (index, &raw) in ids.iter().enumerate() {
            // SAFETY: `lock` is live and holds `ids.len()` ids, so `index` is in range. Jolt
            // returns null unless the id still names a live body.
            let body = unsafe { JPH_BodyLockMultiRead_GetBody(lock.as_ptr(), index as u32) };
            let Some(body) = NonNull::new(body.cast_mut()) else {
                continue;
            };
            out.push(read(BodyId::new(raw, self.tag), body));
        }
    }

    /// The raw ids of every awake rigid and soft body, sorted (Jolt's `BodyID::operator<`
    /// compares the raw value, as `BodyId`'s `Ord` does).
    fn active_body_ids(&self) -> Vec<JPH_BodyID> {
        let system = self.system.as_ptr();
        // SAFETY: the system is this live world's; the getter only reads the atomic active-list
        // length. The copies below rely on it staying put, which the shared world borrow ensures.
        let (rigid, soft) = unsafe {
            (
                JPH_PhysicsSystem_GetNumActiveBodies(system, JPH_BodyType_Rigid),
                JPH_PhysicsSystem_GetNumActiveBodies(system, JPH_BodyType_Soft),
            )
        };
        let mut ids = vec![0; rigid as usize + soft as usize];
        let (rigid_ids, soft_ids) = ids.split_at_mut(rigid as usize);
        for (kind, segment) in [
            (JPH_BodyType_Rigid, rigid_ids),
            (JPH_BodyType_Soft, soft_ids),
        ] {
            if segment.is_empty() {
                continue;
            }
            // SAFETY: the system is this live world's. Every change of the active lists (step,
            // create, remove, wake) takes `&mut self`, so under `&self` the counts read above
            // are the lists' lengths and the call fills `segment` exactly. joltc copies at most
            // `segment.len()` ids, so the writes stay inside `segment`.
            unsafe {
                JPH_PhysicsSystem_GetActiveBodies(
                    system,
                    kind,
                    segment.as_mut_ptr(),
                    segment.len() as u32,
                );
            }
        }
        ids.sort_unstable();
        ids
    }
}

#[cfg(test)]
mod tests;
