//! Ragdolls: Jolt's `Ragdoll`, a set of bodies joined by constraints, owned by a
//! [`PhysicsWorld`] and created from [`RagdollSettings`].
//!
//! # Ragdolls and worlds
//! A ragdoll's parts are ordinary bodies of the world, listed by [`RagdollRef::body_ids`], that
//! the world destroys only with the ragdoll ([`PhysicsWorld::remove_ragdoll`]). Parts of one
//! ragdoll never collide with each other; parts of different ragdolls do. For ragdolls in a
//! second world next to the main one, create the static bodies of both from the same [`Shape`]
//! handles; Jolt shapes are shared, not copied.
//!
//! For the caller's own (for example radial) gravity, give every part
//! [`gravity_factor(0.0)`](crate::BodySettings::gravity_factor) and add `g * mass` to each part
//! every tick ([`BodyRef::mass`](crate::BodyRef::mass), [`BodyMut::add_force`](crate::BodyMut::add_force)).
//!
//! [`Shape`]: crate::Shape

mod mapper;
mod settings;
mod settle;

use std::fmt;
use std::ptr::NonNull;

use oxijolt_sys::*;

use crate::body::{kinematic_velocities, with_locked_body};
use crate::constraint::SixDofConstraintAxis;
use crate::limits::{self, ANGULAR_VELOCITY_RULE, LINEAR_VELOCITY_RULE};
use crate::owned::{JoltObject, Owned};
use crate::world::{advance_structure_epoch, WorldTag, DELTA_TIME_RULE};
use crate::{Activation, BodyId, MotionType, PhysicsWorld, Quat, RVec3, RagdollError, Vec3};

pub use mapper::{MappedSkeleton, SkeletonMapper, TranslationLocks};
use settings::JointKind;
pub use settings::{
    JointTransform, RagdollJoint, RagdollPart, RagdollSettings, Skeleton, SkeletonJoint,
    SkeletonPose,
};
pub use settle::SettleDetector;

/// Identifies a ragdoll in the world that created it.
///
/// The raw value is 1 for the first ragdoll of a world, then 2, 3 and so on; ids are never
/// reused within a world, so the same creation history gives the same ids. Its parts use the
/// Jolt collision group `2^31 + to_raw()`, above every
/// [`CollisionGroup::MAX_GROUP_ID`](crate::CollisionGroup::MAX_GROUP_ID). Using an id with
/// another world returns [`RagdollError::WrongWorld`].
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct RagdollId {
    // Declared first, so ids order by creation before the world.
    raw: u32,
    pub(crate) world: WorldTag,
}

impl RagdollId {
    pub(crate) fn new(raw: u32, world: WorldTag) -> Self {
        Self { raw, world }
    }

    /// The id's number within its world.
    pub fn to_raw(self) -> u32 {
        self.raw
    }
}

impl fmt::Debug for RagdollId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("RagdollId").field(&self.raw).finish()
    }
}

/// A ragdoll, of which the world holds the one reference `JPH_RagdollSettings_CreateRagdoll`
/// adds. Jolt's `~Ragdoll` destroys the parts through the physics system it was created in, so
/// the world removes the ragdoll from the system before releasing it, and releases it before the
/// system is destroyed (`Drop for PhysicsWorld`).
impl JoltObject for JPH_Ragdoll {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds the last reference (trait contract) and has removed the
        // ragdoll from its live system (`remove_ragdoll`, `remove_all_ragdolls`).
        unsafe { JPH_Ragdoll_Destroy(ptr) };
    }
}

/// One ragdoll of a world.
pub(crate) struct RagdollEntry {
    ragdoll: Owned<JPH_Ragdoll>,
    /// The part bodies, in part order.
    bodies: Vec<BodyId>,
    parents: Vec<Option<u32>>,
    joints: Vec<Option<JointKind>>,
    /// The ragdoll's constraint index of each part's joint.
    constraint_of_part: Vec<Option<u32>>,
}

impl RagdollEntry {
    /// The constraint joining `part` to its parent, with its kind.
    fn constraint(&self, part: usize) -> Option<(JointKind, *mut JPH_TwoBodyConstraint)> {
        let kind = self.joints[part]?;
        let index = self.constraint_of_part[part]?;
        // SAFETY: the ragdoll is live and owns its constraints; `index` counts the jointed parts
        // before `part`, as Jolt's `CalculateBodyIndexToConstraintIndex` does, so it is below the
        // constraint count. The getter returns a pointer to a member.
        let constraint = unsafe { JPH_Ragdoll_GetConstraint(self.ragdoll.as_ptr(), index as i32) };
        debug_assert_eq!(
            // SAFETY: the constraint is live; the getter is a virtual call that only reads it.
            unsafe { JPH_Constraint_GetSubType(constraint.cast()) },
            kind.sub_type(),
            "the recorded joint kind matches the Jolt constraint"
        );
        Some((kind, constraint))
    }
}

/// What a joint reads now, in its constraint space.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum JointReading {
    /// The rotation of a swing-twist joint's child frame in its parent frame (Jolt
    /// `GetRotationInConstraintSpace`); its twist is about X, its swing about Y and Z.
    SwingTwist {
        /// A unit quaternion.
        rotation_in_constraint_space: Quat,
    },
    /// The angle of a hinge, radians, as of the last step (Jolt `GetCurrentAngle`).
    Hinge {
        /// Radians.
        current_angle: f32,
    },
    /// The rotation of a six-degree-of-freedom joint's child frame in its parent frame; its
    /// twist is about X, its swing about Y and Z.
    SixDof {
        /// A unit quaternion.
        rotation_in_constraint_space: Quat,
    },
}

/// Read access to one ragdoll, borrowed from its world.
pub struct RagdollRef<'w> {
    world: &'w PhysicsWorld,
    id: RagdollId,
    entry: &'w RagdollEntry,
}

impl fmt::Debug for RagdollRef<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RagdollRef")
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}

impl RagdollRef<'_> {
    /// The ragdoll's id.
    pub fn id(&self) -> RagdollId {
        self.id
    }

    /// Number of parts, one per skeleton joint.
    pub fn part_count(&self) -> u32 {
        self.entry.bodies.len() as u32
    }

    /// The part bodies, in part order. Read them with [`PhysicsWorld::body`].
    pub fn body_ids(&self) -> &[BodyId] {
        &self.entry.bodies
    }

    /// The pose of every part: the root offset at part 0's body origin, each translation
    /// relative to it, rotations in world space (Jolt `Ragdoll::GetPose`).
    pub fn pose(&self) -> SkeletonPose {
        let transforms: Vec<(RVec3, Quat)> = self
            .entry
            .bodies
            .iter()
            .map(|&id| {
                let body = self.world.body(id).unwrap_or_else(|_| {
                    unreachable!("the world removes parts only with the ragdoll")
                });
                (body.position(), body.rotation())
            })
            .collect();
        let root_offset = transforms[0].0;
        // `Real` is `f32` without the `double-precision` feature, so the casts are no-ops there.
        #[allow(clippy::unnecessary_cast)]
        let joints = transforms
            .iter()
            .map(|&(position, rotation)| JointTransform {
                translation: Vec3::new(
                    (position.x - root_offset.x) as f32,
                    (position.y - root_offset.y) as f32,
                    (position.z - root_offset.z) as f32,
                ),
                rotation,
            })
            .collect();
        SkeletonPose {
            root_offset,
            joints,
        }
    }

    /// Position and rotation of the root part's body (Jolt `Ragdoll::GetRootTransform`).
    pub fn root_transform(&self) -> (RVec3, Quat) {
        let mut position = RVec3::ZERO.to_jph();
        let mut rotation = Quat::IDENTITY.to_jph();
        // SAFETY: the world borrowed here owns the ragdoll and no step runs; the call locks the
        // root body for reading and writes the two live locals.
        unsafe {
            JPH_Ragdoll_GetRootTransform(
                self.entry.ragdoll.as_ptr(),
                &mut position,
                &mut rotation,
                true,
            )
        };
        (RVec3::from_jph(position), Quat::from_jph(rotation))
    }

    /// Whether any part is awake.
    pub fn is_active(&self) -> bool {
        // SAFETY: as in `root_transform`; the call locks the parts for reading.
        unsafe { JPH_Ragdoll_IsActive(self.entry.ragdoll.as_ptr(), true) }
    }

    /// What the joint between `part` and its parent reads now; `None` for the root part and for
    /// parts that do not exist.
    pub fn joint(&self, part: u32) -> Option<JointReading> {
        let part = part as usize;
        if part >= self.entry.bodies.len() {
            return None;
        }
        let (kind, constraint) = self.entry.constraint(part)?;
        let mut rotation = Quat::IDENTITY.to_jph();
        // SAFETY: the constraint is live and of `kind` (checked against Jolt's subtype in debug
        // builds); each is the most derived object, as joltc casts it. The getters read
        // constraint members and the two bodies' rotations, which only `step` and `&mut` setters
        // change, and write the live local.
        let reading = unsafe {
            match kind {
                JointKind::SwingTwist => {
                    JPH_SwingTwistConstraint_GetRotationInConstraintSpace(
                        constraint.cast_const().cast(),
                        &mut rotation,
                    );
                    JointReading::SwingTwist {
                        rotation_in_constraint_space: Quat::from_jph(rotation),
                    }
                }
                JointKind::Hinge => JointReading::Hinge {
                    current_angle: JPH_HingeConstraint_GetCurrentAngle(constraint.cast()),
                },
                JointKind::SixDof => {
                    JPH_SixDOFConstraint_GetRotationInConstraintSpace(
                        constraint.cast(),
                        &mut rotation,
                    );
                    JointReading::SixDof {
                        rotation_in_constraint_space: Quat::from_jph(rotation),
                    }
                }
            }
        };
        Some(reading)
    }

    /// Whether the rotation motors of the joint between `part` and its parent are on, as
    /// [`RagdollMut::drive_to_pose_using_motors`] leaves them until
    /// [`RagdollMut::stop_motors`]; `None` for the root part and for parts that do not exist.
    pub fn joint_motors_on(&self, part: u32) -> Option<bool> {
        let part = part as usize;
        if part >= self.entry.bodies.len() {
            return None;
        }
        let (kind, constraint) = self.entry.constraint(part)?;
        let on = |state: JPH_MotorState| state != JPH_MotorState_Off;
        // SAFETY: as in `joint`; the getters read the motor state members, which only `&mut`
        // setters change.
        let motors_on = unsafe {
            match kind {
                JointKind::SwingTwist => {
                    let constraint = constraint.cast_const().cast();
                    on(JPH_SwingTwistConstraint_GetSwingMotorState(constraint))
                        || on(JPH_SwingTwistConstraint_GetTwistMotorState(constraint))
                }
                JointKind::Hinge => on(JPH_HingeConstraint_GetMotorState(constraint.cast())),
                JointKind::SixDof => SixDofConstraintAxis::ROTATIONS.iter().any(|axis| {
                    on(JPH_SixDOFConstraint_GetMotorState(
                        constraint.cast(),
                        axis.to_jph(),
                    ))
                }),
            }
        };
        Some(motors_on)
    }

    /// Whether every part moves slower than `max_linear_speed` (m/s) and `max_angular_speed`
    /// (rad/s), strictly. A sleeping part is calm.
    pub fn is_calm(&self, max_linear_speed: f32, max_angular_speed: f32) -> bool {
        self.entry.bodies.iter().all(|&id| {
            let body = self
                .world
                .body(id)
                .unwrap_or_else(|_| unreachable!("the world removes parts only with the ragdoll"));
            !body.is_active()
                || (body.linear_velocity().length() < max_linear_speed
                    && body.angular_velocity().length() < max_angular_speed)
        })
    }
}

/// Write access to one ragdoll, borrowed mutably from its world.
///
/// Every method checks all of its input before it changes any part, so an error leaves the
/// ragdoll unchanged. Read the ragdoll through [`PhysicsWorld::ragdoll`] once this borrow ends.
pub struct RagdollMut<'w> {
    entry: &'w mut RagdollEntry,
    body_interface: NonNull<JPH_BodyInterface>,
    structure_epoch: &'w mut u64,
}

impl fmt::Debug for RagdollMut<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RagdollMut").finish_non_exhaustive()
    }
}

/// What the motion type of a ragdoll part must satisfy: not static.
const PART_MOTION_RULE: &str = "ragdoll parts are dynamic or kinematic";

/// The parts of a ragdoll use the collision group id `RAGDOLL_GROUP_BASE + RagdollId::to_raw()`,
/// above every caller group id ([`CollisionGroup::MAX_GROUP_ID`]), so a caller group never
/// shares an id with a ragdoll: Jolt never lets two bodies with the same group id and different
/// tables collide.
///
/// [`CollisionGroup::MAX_GROUP_ID`]: crate::CollisionGroup::MAX_GROUP_ID
const RAGDOLL_GROUP_BASE: u32 = 1 << 31;

impl RagdollMut<'_> {
    fn interface(&self) -> *mut JPH_BodyInterface {
        self.body_interface.as_ptr()
    }

    fn validate(&self, pose: &SkeletonPose) -> Result<(), RagdollError> {
        pose.validate(self.entry.bodies.len())
    }

    /// Moves every part to `pose` at once, without waking it, and clears the joints' warm start
    /// (Jolt `Ragdoll::SetPose` and `ResetWarmStart`). Velocities are kept.
    pub fn set_pose(&mut self, pose: &SkeletonPose) -> Result<(), RagdollError> {
        self.validate(pose)?;
        place_parts(self.body_interface, &self.entry.bodies, pose);
        // SAFETY: the world is borrowed mutably through this view and owns the ragdoll; no step
        // runs. The call writes members of the ragdoll's constraints.
        unsafe { JPH_Ragdoll_ResetWarmStart(self.entry.ragdoll.as_ptr()) };
        Ok(())
    }

    /// Sets each part's velocities so that it reaches `pose` in `delta_time` seconds (Jolt
    /// `Ragdoll::DriveToPoseUsingKinematics`), waking it when it moves. Meant for kinematic
    /// parts ([`set_motion_type`](Self::set_motion_type)); a dynamic part gets the velocity
    /// too, and collisions and joints change it. `delta_time` follows the rule of
    /// [`PhysicsWorld::step`].
    ///
    /// Jolt writes the velocities without clamping them, so every part's velocity is checked
    /// first: at most [`limits::MAX_LINEAR_VELOCITY`] and [`limits::MAX_ANGULAR_VELOCITY`] long,
    /// as Jolt computes them. If one part fails, no part changes.
    pub fn drive_to_pose_using_kinematics(
        &mut self,
        pose: &SkeletonPose,
        delta_time: f32,
    ) -> Result<(), RagdollError> {
        self.validate(pose)?;
        if !PhysicsWorld::is_valid_delta_time(delta_time) {
            return Err(RagdollError::InvalidValue(DELTA_TIME_RULE));
        }
        for (index, &id) in self.entry.bodies.iter().enumerate() {
            let rotation = pose.joints[index].rotation;
            // SAFETY: the world is borrowed mutably through this view and holds the part, a
            // rigid body; this thread holds no body lock.
            let velocities = unsafe {
                kinematic_velocities(
                    self.body_interface,
                    id,
                    pose.position(index),
                    rotation,
                    delta_time,
                )
            };
            let within_limits = velocities.is_some_and(|(linear, angular)| {
                limits::is_linear_velocity(linear) && limits::is_angular_velocity(angular)
            });
            if !within_limits {
                return Err(RagdollError::InvalidValue(KINEMATIC_DRIVE_RULE));
            }
        }
        for (index, &id) in self.entry.bodies.iter().enumerate() {
            let mut position = pose.position(index).to_jph();
            let mut rotation = pose.joints[index].rotation.to_jph();
            // SAFETY: the world is borrowed mutably through this view and holds the part, which
            // is dynamic or kinematic; the pose was validated and `delta_time` is in range. This
            // thread holds no body lock, and joltc only reads the live locals.
            unsafe {
                JPH_BodyInterface_MoveKinematic(
                    self.interface(),
                    id.to_raw(),
                    &mut position,
                    &mut rotation,
                    delta_time,
                )
            };
        }
        Ok(())
    }

    /// Turns on every joint's position motors and aims them at `pose`, then wakes the ragdoll
    /// (Jolt `Ragdoll::DriveToPoseUsingMotors`, extended to six-degree-of-freedom joints, whose
    /// rotation motors are driven and translation motors left off).
    ///
    /// Each joint's target is its part's rotation relative to its parent part in `pose`
    /// (`parent⁻¹ · part`); translations and the root part are not driven. The motors keep their
    /// target until it is set again or [`stop_motors`](Self::stop_motors) is called.
    pub fn drive_to_pose_using_motors(&mut self, pose: &SkeletonPose) -> Result<(), RagdollError> {
        self.validate(pose)?;
        for part in 0..self.entry.bodies.len() {
            let Some(parent) = self.entry.parents[part] else {
                continue;
            };
            let Some((kind, constraint)) = self.entry.constraint(part) else {
                continue;
            };
            let parent_rotation = pose.joints[parent as usize].rotation;
            let target = parent_rotation
                .conjugated()
                .product(pose.joints[part].rotation)
                .normalized()
                .to_jph();
            // SAFETY: the world is borrowed mutably through this view and owns the constraint,
            // which is of `kind` and the most derived object, as joltc casts it; no step runs.
            // Every motor setting was validated when the settings were built, so Jolt's
            // `MotorSettings::IsValid` assertion for a running motor holds. `target` is a live
            // local unit quaternion.
            unsafe {
                match kind {
                    JointKind::SwingTwist => {
                        let constraint = constraint.cast();
                        JPH_SwingTwistConstraint_SetSwingMotorState(
                            constraint,
                            JPH_MotorState_Position,
                        );
                        JPH_SwingTwistConstraint_SetTwistMotorState(
                            constraint,
                            JPH_MotorState_Position,
                        );
                        JPH_SwingTwistConstraint_SetTargetOrientationBS(constraint, &target);
                    }
                    JointKind::Hinge => {
                        let constraint = constraint.cast();
                        JPH_HingeConstraint_SetMotorState(constraint, JPH_MotorState_Position);
                        JPH_HingeConstraint_SetTargetOrientationBS(constraint, &target);
                    }
                    JointKind::SixDof => {
                        let constraint = constraint.cast();
                        for axis in SixDofConstraintAxis::ROTATIONS {
                            JPH_SixDOFConstraint_SetMotorState(
                                constraint,
                                axis.to_jph(),
                                JPH_MotorState_Position,
                            );
                        }
                        let mut target = target;
                        JPH_SixDOFConstraint_SetTargetOrientationBS(constraint, &mut target);
                    }
                }
            }
        }
        self.activate();
        Ok(())
    }

    /// Turns every joint motor off.
    pub fn stop_motors(&mut self) {
        for part in 0..self.entry.bodies.len() {
            let Some((kind, constraint)) = self.entry.constraint(part) else {
                continue;
            };
            // SAFETY: as in `drive_to_pose_using_motors`; turning a motor off needs no settings.
            unsafe {
                match kind {
                    JointKind::SwingTwist => {
                        let constraint = constraint.cast();
                        JPH_SwingTwistConstraint_SetSwingMotorState(constraint, JPH_MotorState_Off);
                        JPH_SwingTwistConstraint_SetTwistMotorState(constraint, JPH_MotorState_Off);
                    }
                    JointKind::Hinge => {
                        JPH_HingeConstraint_SetMotorState(constraint.cast(), JPH_MotorState_Off)
                    }
                    JointKind::SixDof => {
                        for axis in SixDofConstraintAxis::ALL {
                            JPH_SixDOFConstraint_SetMotorState(
                                constraint.cast(),
                                axis.to_jph(),
                                JPH_MotorState_Off,
                            );
                        }
                    }
                }
            }
        }
    }

    /// Makes every part dynamic or kinematic; static parts are refused
    /// ([`RagdollError::InvalidValue`]). The parts keep their object layers.
    pub fn set_motion_type(
        &mut self,
        motion_type: MotionType,
        activation: Activation,
    ) -> Result<(), RagdollError> {
        if motion_type == MotionType::Static {
            return Err(RagdollError::InvalidValue(PART_MOTION_RULE));
        }
        // Jolt does not save motion types.
        advance_structure_epoch(self.structure_epoch);
        for &id in &self.entry.bodies {
            // SAFETY: the world is borrowed mutably through this view and holds the part, which
            // has motion properties because it was created dynamic or kinematic. This thread
            // holds no body lock.
            unsafe {
                JPH_BodyInterface_SetMotionType(
                    self.interface(),
                    id.to_raw(),
                    motion_type.to_jph(),
                    activation.to_jph(),
                )
            };
        }
        Ok(())
    }

    /// Gives every part the linear velocity `linear` (m/s) and angular velocity `angular`
    /// (rad/s), the velocity at death: finite and at most [`limits::MAX_LINEAR_VELOCITY`] and
    /// [`limits::MAX_ANGULAR_VELOCITY`] long. Wakes the parts when they are not zero.
    pub fn set_linear_and_angular_velocity(
        &mut self,
        linear: Vec3,
        angular: Vec3,
    ) -> Result<(), RagdollError> {
        if !limits::is_linear_velocity(linear) {
            return Err(RagdollError::InvalidValue(LINEAR_VELOCITY_RULE));
        }
        if !limits::is_angular_velocity(angular) {
            return Err(RagdollError::InvalidValue(ANGULAR_VELOCITY_RULE));
        }
        for &id in &self.entry.bodies {
            let mut linear = linear.to_jph();
            let mut angular = angular.to_jph();
            // SAFETY: as in `set_motion_type`; joltc only reads the live locals.
            unsafe {
                JPH_BodyInterface_SetLinearAndAngularVelocity(
                    self.interface(),
                    id.to_raw(),
                    &mut linear,
                    &mut angular,
                )
            };
        }
        Ok(())
    }

    /// Wakes every part.
    pub fn activate(&mut self) {
        // SAFETY: the world is borrowed mutably through this view and owns the ragdoll; the call
        // locks the parts and wakes them.
        unsafe { JPH_Ragdoll_Activate(self.entry.ragdoll.as_ptr(), true) };
    }
}

/// What a kinematic drive must satisfy: every part gets a linear velocity within
/// [`limits::MAX_LINEAR_VELOCITY`] and an angular velocity within
/// [`limits::MAX_ANGULAR_VELOCITY`].
const KINEMATIC_DRIVE_RULE: &str =
    "a kinematic drive must keep every part within the limits velocity bounds";

/// Places each body of `bodies` at its transform in the validated `pose`, without waking it.
fn place_parts(body_interface: NonNull<JPH_BodyInterface>, bodies: &[BodyId], pose: &SkeletonPose) {
    for (index, &id) in bodies.iter().enumerate() {
        let position = pose.position(index).to_jph();
        let rotation = pose.joints[index].rotation.to_jph();
        // SAFETY: the body interface belongs to the live world that holds the body and is
        // borrowed mutably by the caller; the pose was validated (finite, unit rotations). This
        // thread holds no body lock.
        unsafe {
            JPH_BodyInterface_SetPositionAndRotation(
                body_interface.as_ptr(),
                id.to_raw(),
                &position,
                &rotation,
                JPH_Activation_DontActivate,
            )
        };
    }
}

impl PhysicsWorld {
    /// Creates a ragdoll from `settings`, adds it to the world and returns its id.
    ///
    /// The parts start at `pose` when given, otherwise at the bind pose of the settings, and are
    /// woken as `activation` says. Fails with [`RagdollError::UnknownObjectLayer`] when a part's
    /// layer is not in this world, with [`RagdollError::InvalidValue`] for an invalid pose, with
    /// [`RagdollError::TooManyRagdolls`] when the world has run out of ids and with
    /// [`RagdollError::TooManyBodies`] when it has no room for every part. Nothing is created on
    /// failure.
    pub fn create_ragdoll(
        &mut self,
        settings: &RagdollSettings,
        pose: Option<&SkeletonPose>,
        activation: Activation,
    ) -> Result<RagdollId, RagdollError> {
        if let Some(&layer) = settings
            .object_layers()
            .iter()
            .find(|layer| layer.get() >= self.object_layer_count)
        {
            return Err(RagdollError::UnknownObjectLayer(layer));
        }
        let parts = settings.object_layers().len();
        if let Some(pose) = pose {
            pose.validate(parts)?;
        }
        let raw = self.next_ragdoll_id;
        // The group id stays below `u32::MAX`, Jolt's invalid group.
        if raw > RAGDOLL_GROUP_BASE - 2 {
            return Err(RagdollError::TooManyRagdolls);
        }
        if !self.has_room_for_bodies(parts) {
            return Err(RagdollError::TooManyBodies);
        }
        self.note_structure_change();
        // SAFETY: the settings and the system are live, and the system is borrowed mutably, so no
        // other body is created meanwhile. Jolt's `BodyManager::AddBody` fails only when the
        // world holds `GetMaxBodies()` bodies, and the check above leaves room for every part,
        // so `CreateRagdoll` creates every body and returns a ragdoll, on which joltc's
        // unconditional `AddRef` is sound. The handle takes over that one reference.
        let ragdoll = unsafe {
            Owned::from_raw(JPH_RagdollSettings_CreateRagdoll(
                settings.as_ptr(),
                self.system.as_ptr(),
                RAGDOLL_GROUP_BASE + raw,
                0,
            ))
        }
        .ok_or(RagdollError::TooManyBodies)?;
        let bodies: Vec<BodyId> = (0..parts)
            .map(|part| {
                // SAFETY: the ragdoll is live and has one body per part.
                let raw = unsafe { JPH_Ragdoll_GetBodyID(ragdoll.as_ptr(), part as i32) };
                BodyId::new(raw, self.tag)
            })
            .collect();
        if let Some(pose) = pose {
            place_parts(self.body_interface, &bodies, pose);
        }
        // SAFETY: the ragdoll's bodies and constraints exist and are not yet in the system, which
        // is borrowed mutably; the call adds them and locks the bodies itself.
        unsafe { JPH_Ragdoll_AddToPhysicsSystem(ragdoll.as_ptr(), activation.to_jph(), true) };
        let mut next_constraint = 0;
        let constraint_of_part = settings
            .joints()
            .iter()
            .map(|joint| {
                joint.map(|_| {
                    next_constraint += 1;
                    next_constraint - 1
                })
            })
            .collect();
        for body in &bodies {
            self.ragdoll_bodies.insert(body.to_raw(), raw);
        }
        self.ragdolls.insert(
            raw,
            RagdollEntry {
                ragdoll,
                bodies,
                parents: settings.parents().to_vec(),
                joints: settings.joints().to_vec(),
                constraint_of_part,
            },
        );
        self.next_ragdoll_id += 1;
        Ok(RagdollId::new(raw, self.tag))
    }

    /// The entry of `id`, if it names a ragdoll of this world.
    fn ragdoll_entry(&self, id: RagdollId) -> Result<&RagdollEntry, RagdollError> {
        if id.world != self.tag {
            return Err(RagdollError::WrongWorld(id));
        }
        self.ragdolls.get(&id.raw).ok_or(RagdollError::NotFound(id))
    }

    /// Read access to a ragdoll.
    pub fn ragdoll(&self, id: RagdollId) -> Result<RagdollRef<'_>, RagdollError> {
        let entry = self.ragdoll_entry(id)?;
        Ok(RagdollRef {
            world: self,
            id,
            entry,
        })
    }

    /// Write access to a ragdoll.
    pub fn ragdoll_mut(&mut self, id: RagdollId) -> Result<RagdollMut<'_>, RagdollError> {
        self.ragdoll_entry(id)?;
        let body_interface = self.body_interface;
        let entry = self
            .ragdolls
            .get_mut(&id.raw)
            .unwrap_or_else(|| unreachable!("checked above"));
        Ok(RagdollMut {
            entry,
            body_interface,
            structure_epoch: &mut self.structure_epoch,
        })
    }

    /// Removes a ragdoll with its parts and joints.
    ///
    /// Like [`remove_body`](Self::remove_body), wakes every non-static body whose bounds come
    /// within the speculative contact distance of a removed part's, part by part in part order
    /// and within a part in body-id order.
    pub fn remove_ragdoll(&mut self, id: RagdollId) -> Result<(), RagdollError> {
        self.ragdoll_entry(id)?;
        self.note_structure_change();
        let entry = self
            .ragdolls
            .remove(&id.raw)
            .unwrap_or_else(|| unreachable!("checked above"));
        let bounds: Vec<JPH_AABox> = entry
            .bodies
            .iter()
            .map(|&body| {
                let mut bounds = JPH_AABox {
                    min: Vec3::ZERO.to_jph(),
                    max: Vec3::ZERO.to_jph(),
                };
                with_locked_body(self.body_lock_interface, body, |locked| {
                    // SAFETY: `locked` is locked for writing for the duration of the closure;
                    // `bounds` is a live local.
                    unsafe { JPH_Body_GetWorldSpaceBounds(locked.as_ptr(), &mut bounds) };
                })
                .unwrap_or_else(|| unreachable!("the world removes parts only with the ragdoll"));
                bounds
            })
            .collect();
        for body in &entry.bodies {
            self.ragdoll_bodies.remove(&body.to_raw());
            self.pending_cache_invalidations.remove(&body.to_raw());
        }
        self.unregister_ragdoll(entry);
        for bounds in &bounds {
            self.wake_bodies_near(bounds, None);
        }
        Ok(())
    }

    /// Takes the ragdoll out of the system, then releases it, which destroys its parts.
    fn unregister_ragdoll(&mut self, entry: RagdollEntry) {
        // SAFETY: the system and the ragdoll are live, the system is borrowed mutably and no step
        // runs; `create_ragdoll` added the ragdoll. The call removes its bodies and constraints
        // and locks the bodies itself.
        unsafe { JPH_Ragdoll_RemoveFromPhysicsSystem(entry.ragdoll.as_ptr(), true) };
        drop(entry);
    }

    /// Removes every ragdoll, in id order, without waking anything. Runs when the world is
    /// dropped, so no ragdoll outlives the system.
    pub(crate) fn remove_all_ragdolls(&mut self) {
        while let Some((_, entry)) = self.ragdolls.pop_first() {
            for body in &entry.bodies {
                self.ragdoll_bodies.remove(&body.to_raw());
            }
            self.unregister_ragdoll(entry);
        }
    }

    /// The ids of the world's ragdolls, in id order (creation order).
    pub fn ragdoll_ids(&self) -> impl Iterator<Item = RagdollId> + '_ {
        self.ragdolls
            .keys()
            .map(|&raw| RagdollId::new(raw, self.tag))
    }

    /// Number of ragdolls in the world.
    pub fn ragdoll_count(&self) -> u32 {
        // Every ragdoll has its own `u32` id, so the count fits.
        self.ragdolls.len() as u32
    }

    /// Whether `id` names a ragdoll that is in this world now.
    pub fn contains_ragdoll(&self, id: RagdollId) -> bool {
        self.ragdoll_entry(id).is_ok()
    }

    /// The ragdoll that `body` is a part of, if any.
    pub fn ragdoll_of_body(&self, body: BodyId) -> Option<RagdollId> {
        if body.world != self.tag {
            return None;
        }
        self.ragdoll_bodies
            .get(&body.to_raw())
            .map(|&raw| RagdollId::new(raw, self.tag))
    }

    /// Whether `body` is a part of a ragdoll of this world.
    pub(crate) fn is_ragdoll_body(&self, body: BodyId) -> bool {
        self.ragdoll_of_body(body).is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodySettings, Real, Shape, SwingTwistConstraintSettings, WorldSettings};

    /// The squared distance in metres between two world positions.
    fn distance_squared(a: RVec3, b: RVec3) -> Real {
        let d = [a.x - b.x, a.y - b.y, a.z - b.z];
        d.iter().map(|value| value * value).sum()
    }

    /// The last ragdoll id whose group id is below Jolt's invalid group is created, and its
    /// parts carry that group id; the next is refused before anything changes.
    #[test]
    fn ragdoll_group_ids_stop_below_the_invalid_group() {
        let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO)).unwrap();
        let skeleton = Skeleton::new(&[SkeletonJoint {
            name: "root",
            parent: None,
        }])
        .unwrap();
        let shape = Shape::new_sphere(0.2).unwrap();
        let parts = [RagdollPart {
            shape: &shape,
            body: BodySettings::new_dynamic(),
            joint: None,
        }];
        let settings = RagdollSettings::new(&skeleton, &parts).unwrap();
        world.next_ragdoll_id = RAGDOLL_GROUP_BASE - 2;
        let id = world
            .create_ragdoll(&settings, None, Activation::Activate)
            .unwrap();
        let part = world.ragdoll(id).unwrap().body_ids()[0];
        let mut group = JPH_CollisionGroup {
            groupFilter: std::ptr::null(),
            groupID: 0,
            subGroupID: 0,
        };
        // SAFETY: the body interface is the live world's and the part exists; `group` is a live
        // local that joltc overwrites with a borrowed table pointer, which is not used.
        unsafe {
            JPH_BodyInterface_GetCollisionGroup(
                world.body_interface.as_ptr(),
                part.to_raw(),
                &mut group,
            )
        };
        assert_eq!(group.groupID, u32::MAX - 1);

        let (epoch, bodies) = (world.structure_epoch, world.body_count());
        assert_eq!(
            world.create_ragdoll(&settings, None, Activation::Activate),
            Err(RagdollError::TooManyRagdolls)
        );
        assert_eq!((world.structure_epoch, world.body_count()), (epoch, bodies));
    }

    #[test]
    fn a_created_chain_reports_its_parts_and_joints() {
        let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO)).unwrap();
        let skeleton = Skeleton::new(&[
            SkeletonJoint {
                name: "root",
                parent: None,
            },
            SkeletonJoint {
                name: "child",
                parent: Some(0),
            },
        ])
        .unwrap();
        let shape = Shape::new_capsule(0.3, 0.2).unwrap();
        let joint = SwingTwistConstraintSettings::new(
            RVec3::new(0.0, 0.5, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .half_cone_angles(0.5, 0.5);
        let parts: Vec<RagdollPart<'_>> = (0..2)
            .map(|i| RagdollPart {
                shape: &shape,
                body: BodySettings::new_dynamic().position(RVec3::new(0.0, i as Real, 0.0)),
                joint: (i == 1).then(|| RagdollJoint::SwingTwist(joint.clone())),
            })
            .collect();
        let settings = RagdollSettings::new(&skeleton, &parts).unwrap();
        let id = world
            .create_ragdoll(&settings, None, Activation::Activate)
            .unwrap();
        let ragdoll = world.ragdoll(id).unwrap();
        assert_eq!(ragdoll.part_count(), 2);
        assert_eq!(ragdoll.joint(0), None);
        assert_eq!(ragdoll.joint(2), None);
        assert!(matches!(
            ragdoll.joint(1),
            Some(JointReading::SwingTwist { .. })
        ));
        let pose = ragdoll.pose();
        assert!(distance_squared(pose.position(1), RVec3::new(0.0, 1.0, 0.0)) < 1e-10);
        assert_eq!(world.body_count(), 2);
        world.remove_ragdoll(id).unwrap();
        assert_eq!(world.body_count(), 0);
    }
}
