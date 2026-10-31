//! Read and write access to one body.

use std::marker::PhantomData;
use std::ops::Deref;
use std::ptr::NonNull;

use oxijolt_sys::*;

use super::load::{
    length, point_torque, read_load, require, soft_body_force, sum, Load, PointTorque,
};
use super::{with_locked_body, with_read_locked_body, Activation, AllowedDofs, BodyId, MotionType};
use crate::limits::{
    self, is_angular_velocity, is_in_frame, is_linear_velocity, ANGULAR_VELOCITY_RULE,
    LINEAR_VELOCITY_RULE, POSITION_RULE,
};
use crate::math::ROTATION_RULE;
use crate::{BodyError, PhysicsWorld, Quat, RVec3, Vec3};

/// Read access to one body, borrowed from its world.
///
/// Each method takes Jolt's body lock for the duration of one call and returns a plain value.
/// Not `Send` or `Sync`; share the world instead.
///
/// Positions read back with the same bits they were set with when the shape's centre of mass
/// is at its origin (boxes, spheres, capsules, cylinders), except that the sign of a zero
/// component is not kept: `-0.0` may read back as `+0.0`, because Jolt stores
/// `position + rotation * centre_of_mass`. For other shapes, compounds in particular, the
/// position involves arithmetic.
pub struct BodyRef<'w> {
    pub(super) body_interface: NonNull<JPH_BodyInterface>,
    pub(super) body_lock_interface: NonNull<JPH_BodyLockInterface>,
    pub(super) id: BodyId,
    pub(super) _world: PhantomData<&'w PhysicsWorld>,
}

impl std::fmt::Debug for BodyRef<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BodyRef")
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}

impl BodyRef<'_> {
    /// The body's id.
    pub fn id(&self) -> BodyId {
        self.id
    }

    pub(super) fn interface(&self) -> *mut JPH_BodyInterface {
        self.body_interface.as_ptr()
    }

    /// Position of the body origin (not of the centre of mass) in metres.
    pub fn position(&self) -> RVec3 {
        let mut value = RVec3::ZERO.to_jph();
        // SAFETY: the body interface belongs to the world this view borrows, which holds the
        // body (`check`) and cannot change while the borrow lasts; `value` is a live local.
        unsafe { JPH_BodyInterface_GetPosition(self.interface(), self.id.raw, &mut value) };
        RVec3::from_jph(value)
    }

    /// Rotation.
    pub fn rotation(&self) -> Quat {
        let mut value = Quat::IDENTITY.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_BodyInterface_GetRotation(self.interface(), self.id.raw, &mut value) };
        Quat::from_jph(value)
    }

    /// Linear velocity of the centre of mass in m/s. For a soft body, Jolt reports the average
    /// of its vertex velocities.
    pub fn linear_velocity(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_BodyInterface_GetLinearVelocity(self.interface(), self.id.raw, &mut value) };
        Vec3::from_jph(value)
    }

    /// Angular velocity in rad/s. For a soft body, Jolt reports the average of `p × v` over its
    /// vertices (in m²/s, relative to the body origin), not a rigid rotation.
    pub fn angular_velocity(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_BodyInterface_GetAngularVelocity(self.interface(), self.id.raw, &mut value) };
        Vec3::from_jph(value)
    }

    /// How the body moves.
    pub fn motion_type(&self) -> MotionType {
        // SAFETY: as in `position`.
        MotionType::from_jph(unsafe {
            JPH_BodyInterface_GetMotionType(self.interface(), self.id.raw)
        })
    }

    /// Whether the body is awake. Static bodies are never active.
    pub fn is_active(&self) -> bool {
        // SAFETY: as in `position`.
        unsafe { JPH_BodyInterface_IsActive(self.interface(), self.id.raw) }
    }

    /// Whether a body that can move has fallen asleep. Static bodies never sleep.
    pub fn is_sleeping(&self) -> bool {
        self.motion_type() != MotionType::Static && !self.is_active()
    }

    /// Mass in kg of a dynamic body; `None` for static and kinematic bodies, whose mass is
    /// infinite. For the caller's own gravity `g` (m/s²) on a body created with
    /// [`gravity_factor(0.0)`](crate::BodySettings::gravity_factor), add the force `g * mass` every
    /// step with [`BodyMut::add_force`].
    ///
    /// For a soft body, the sum of the masses of its movable vertices; `None` while any vertex
    /// is kinematic, because Jolt then gives the whole body infinite mass.
    pub fn mass(&self) -> Option<f32> {
        let inverse_mass = with_read_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure. A dynamic body
            // has motion properties, so the unchecked getter reads a live member; the getters
            // only read.
            unsafe {
                JPH_Body_IsDynamic(body.as_ptr()).then(|| {
                    JPH_MotionProperties_GetInverseMassUnchecked(JPH_Body_GetMotionProperties(
                        body.as_ptr(),
                    ))
                })
            }
        })
        .flatten()?;
        (inverse_mass != 0.0).then(|| 1.0 / inverse_mass)
    }

    /// The inverse inertia tensor about the centre of mass in world coordinates, in kg⁻¹·m⁻²
    /// (Jolt `Body::GetInverseInertia`): the body's rotation is applied, and the rows and columns
    /// of world rotation axes locked by [`allowed_dofs`](Self::allowed_dofs) are zero, so a body
    /// with every rotation locked reads all zeros. Multiplying it by an angular impulse gives the
    /// change of angular velocity Jolt applies.
    ///
    /// In rows, as [`Inertia::Tensor`](crate::Inertia::Tensor): `tensor[i][j]` is the entry in
    /// row `i` and column `j`. `None` for a body that is not a dynamic rigid body (static,
    /// kinematic, soft) or that no longer exists.
    pub fn inverse_inertia(&self) -> Option<[[f32; 3]; 3]> {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure. Jolt's
            // `GetInverseInertia` reads the motion properties of a dynamic body (it asserts
            // `IsDynamic`), so it is called only for a dynamic rigid body; it is a const getter,
            // so the mutable pointer joltc takes is only read through. An all-zero `JPH_Mat4` is
            // valid (floats); joltc copies the whole matrix into it, a live local.
            unsafe {
                if !JPH_Body_IsRigidBody(body.as_ptr()) || !JPH_Body_IsDynamic(body.as_ptr()) {
                    return None;
                }
                let mut matrix: JPH_Mat4 = std::mem::zeroed();
                JPH_Body_GetInverseInertia(body.as_ptr(), &mut matrix);
                Some(rows(&matrix))
            }
        })
        .flatten()
    }

    /// The degrees of freedom the body may move in, as created
    /// ([`BodySettings::allowed_dofs`](crate::BodySettings::allowed_dofs)). A body without
    /// motion properties, a static body that cannot move, reports [`AllowedDofs::ALL`].
    pub fn allowed_dofs(&self) -> AllowedDofs {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure. A body that
            // can be kinematic or dynamic has motion properties, so the unchecked getter reads a
            // live member, without the assertion of the checked getter on static bodies; the
            // getters only read.
            unsafe {
                JPH_Body_CanBeKinematicOrDynamic(body.as_ptr()).then(|| {
                    AllowedDofs::from_jph(JPH_MotionProperties_GetAllowedDOFs(
                        JPH_Body_GetMotionPropertiesUnchecked(body.as_ptr()),
                    ))
                })
            }
        })
        .flatten()
        .unwrap_or(AllowedDofs::ALL)
    }

    /// Whether the body has motion properties, so that it can be kinematic or dynamic: it was
    /// created kinematic or dynamic, or static with
    /// [`BodySettings::allow_dynamic_or_kinematic`] (Jolt `Body::CanBeKinematicOrDynamic`).
    ///
    /// [`BodySettings::allow_dynamic_or_kinematic`]: crate::BodySettings::allow_dynamic_or_kinematic
    pub fn can_be_kinematic_or_dynamic(&self) -> bool {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure; the getter
            // only reads it.
            unsafe { JPH_Body_CanBeKinematicOrDynamic(body.as_ptr()) }
        })
        .unwrap_or(false)
    }

    /// Whether the body is a sensor, as created ([`BodySettings::sensor`]).
    ///
    /// [`BodySettings::sensor`]: crate::BodySettings::sensor
    pub fn is_sensor(&self) -> bool {
        // SAFETY: as in `position`.
        unsafe { JPH_BodyInterface_IsSensor(self.interface(), self.id.raw) }
    }

    /// The caller's value given at creation ([`BodySettings::user_data`]). A character's inner
    /// body carries the character's
    /// [`CharacterSettings::user_data`](crate::CharacterSettings::user_data), and a ragdoll part
    /// carries 0.
    ///
    /// [`BodySettings::user_data`]: crate::BodySettings::user_data
    pub fn user_data(&self) -> u64 {
        // SAFETY: as in `position`.
        unsafe { JPH_BodyInterface_GetUserData(self.interface(), self.id.raw) }
    }

    /// Whether the body is a soft body ([`PhysicsWorld::create_soft_body`]).
    pub fn is_soft_body(&self) -> bool {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure; the getter
            // only reads it.
            unsafe { JPH_Body_IsSoftBody(body.as_ptr()) }
        })
        .unwrap_or(false)
    }
}

/// The upper-left 3x3 block of Jolt's column-major `matrix`, in rows.
fn rows(matrix: &JPH_Mat4) -> [[f32; 3]; 3] {
    let [x, y, z, _] = matrix.column;
    [[x.x, y.x, z.x], [x.y, y.y, z.y], [x.z, y.z, z.z]]
}

/// Read and write access to one body, borrowed mutably from its world.
///
/// Dereferences to [`BodyRef`] for reads. Setters validate their input before it reaches
/// Jolt. Units: metres, m/s, rad/s, newtons and newton-metres. Static bodies ignore velocity
/// and force writes. Not `Send` or `Sync`.
///
/// The view borrows the whole world, so it ends before the world steps:
///
/// ```compile_fail,E0499
/// use oxijolt::*;
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let mut world = PhysicsWorld::new(WorldSettings::default())?;
/// let ball = world.create_body(&Shape::new_sphere(0.5)?, &BodySettings::new_dynamic())?;
/// let mut body = world.body_mut(ball)?;
/// let _ = world.step(1.0 / 60.0)?;
/// body.add_force(Vec3::new(0.0, 10.0, 0.0))?;
/// # Ok(())
/// # }
/// ```
pub struct BodyMut<'w> {
    pub(super) inner: BodyRef<'w>,
    pub(super) world: &'w mut PhysicsWorld,
}

impl std::fmt::Debug for BodyMut<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BodyMut")
            .field("id", &self.inner.id)
            .finish_non_exhaustive()
    }
}

impl<'w> Deref for BodyMut<'w> {
    type Target = BodyRef<'w>;

    fn deref(&self) -> &BodyRef<'w> {
        &self.inner
    }
}

impl BodyMut<'_> {
    /// Moves the body origin to `position`, every component at most [`limits::MAX_POSITION`] in
    /// absolute value.
    pub fn set_position(
        &mut self,
        position: RVec3,
        activation: Activation,
    ) -> Result<(), BodyError> {
        require(is_in_frame(position), POSITION_RULE)?;
        let mut position = position.to_jph();
        // SAFETY: the world is borrowed mutably through this view and holds the body; joltc only
        // reads `position`, a live local.
        unsafe {
            JPH_BodyInterface_SetPosition(
                self.interface(),
                self.id.raw,
                &mut position,
                activation.to_jph(),
            )
        };
        Ok(())
    }

    /// Sets the rotation, a finite unit quaternion.
    pub fn set_rotation(
        &mut self,
        rotation: Quat,
        activation: Activation,
    ) -> Result<(), BodyError> {
        require(rotation.is_valid_rotation(), ROTATION_RULE)?;
        let mut rotation = rotation.to_jph();
        // SAFETY: as in `set_position`.
        unsafe {
            JPH_BodyInterface_SetRotation(
                self.interface(),
                self.id.raw,
                &mut rotation,
                activation.to_jph(),
            )
        };
        Ok(())
    }

    /// Sets position and rotation together. The position follows the rule of
    /// [`set_position`](Self::set_position).
    pub fn set_position_and_rotation(
        &mut self,
        position: RVec3,
        rotation: Quat,
        activation: Activation,
    ) -> Result<(), BodyError> {
        require(is_in_frame(position), POSITION_RULE)?;
        require(rotation.is_valid_rotation(), ROTATION_RULE)?;
        let position = position.to_jph();
        let rotation = rotation.to_jph();
        // SAFETY: as in `set_position`.
        unsafe {
            JPH_BodyInterface_SetPositionAndRotation(
                self.interface(),
                self.id.raw,
                &position,
                &rotation,
                activation.to_jph(),
            )
        };
        Ok(())
    }

    /// Sets the linear velocity in m/s, finite and at most [`limits::MAX_LINEAR_VELOCITY`] long
    /// (Jolt would clamp a faster one; it is rejected, as at creation). Wakes the body when the
    /// velocity is not near zero.
    ///
    /// Fails with [`BodyError::NotRigidBody`] for a soft body, whose velocity is stored per vertex
    /// and which Jolt ignores this for (`Docs/Architecture.md:460`); use
    /// [`SoftBodyMut::set_vertex_velocity`](crate::SoftBodyMut::set_vertex_velocity).
    pub fn set_linear_velocity(&mut self, velocity: Vec3) -> Result<(), BodyError> {
        self.reject_soft_body()?;
        require(is_linear_velocity(velocity), LINEAR_VELOCITY_RULE)?;
        let velocity = velocity.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_BodyInterface_SetLinearVelocity(self.interface(), self.id.raw, &velocity) };
        Ok(())
    }

    /// Sets the angular velocity in rad/s, finite and at most [`limits::MAX_ANGULAR_VELOCITY`]
    /// long (Jolt would clamp a faster one; it is rejected, as at creation). Wakes the body when
    /// the velocity is not near zero.
    ///
    /// Fails with [`BodyError::NotRigidBody`] for a soft body, which Jolt ignores this for.
    pub fn set_angular_velocity(&mut self, velocity: Vec3) -> Result<(), BodyError> {
        self.reject_soft_body()?;
        require(is_angular_velocity(velocity), ANGULAR_VELOCITY_RULE)?;
        let mut velocity = velocity.to_jph();
        // SAFETY: as in `set_position`.
        unsafe {
            JPH_BodyInterface_SetAngularVelocity(self.interface(), self.id.raw, &mut velocity)
        };
        Ok(())
    }

    /// Adds a force in newtons at the centre of mass for the next step, and wakes the body.
    ///
    /// Jolt clears accumulated forces after every step; with
    /// [`PhysicsWorld::step_with_collision_steps`](crate::PhysicsWorld::step_with_collision_steps)
    /// a rigid body keeps the force for all collision steps of the call. A body that gets a force
    /// every step never falls asleep.
    ///
    /// The force must be finite, and on a dynamic body the force accumulated this step including
    /// this one may give the body at most [`limits::MAX_ACCELERATION`] (`|F| / mass`); otherwise
    /// [`BodyError::InvalidValue`] is returned and nothing changes. Static and kinematic bodies
    /// ignore forces.
    ///
    /// On a soft body Jolt spreads the force evenly over its vertices: a vertex of inverse mass
    /// `w` gains `F · w / N · dt` of velocity in a step, where `dt` is the first collision step:
    /// Jolt clears a soft body's force after each collision step. `N` is the number of vertices,
    /// and the accumulated force may give the lightest vertex at most
    /// [`limits::MAX_ACCELERATION`] and be at most `MAX_ACCELERATION · MAX_MASS` long (5e14 N)
    /// even when every vertex is pinned.
    /// Jolt adds the accumulated force to the vertices in the body's own frame, so oxijolt
    /// turns `force` into that frame with the body's current rotation; a rotation set later in
    /// the same step turns the force with the vertices.
    pub fn add_force(&mut self, force: Vec3) -> Result<(), BodyError> {
        require(force.is_finite(), "force must be finite")?;
        let force = self.check_load(force, None, Vec3::ZERO)?;
        let mut force = force.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_BodyInterface_AddForce(self.interface(), self.id.raw, &mut force) };
        Ok(())
    }

    /// Adds a force in newtons applied at a world-space `point`, which also adds the matching
    /// torque, and wakes the body.
    ///
    /// The force must be finite and the point within [`limits::MAX_POSITION`]. On a dynamic body
    /// the force accumulated this step including this one may give the body at most
    /// [`limits::MAX_ACCELERATION`], the torque at most [`limits::MAX_ANGULAR_ACCELERATION`]
    /// (`|τ|` times the largest principal inverse inertia, counting the rounding of Jolt's `f32`
    /// torque `(point - centre_of_mass) × force`), and that torque must not overflow; otherwise
    /// [`BodyError::InvalidValue`] is returned and nothing changes. [docs/limits.md#impulses]
    /// has the rounding bound.
    ///
    /// Fails with [`BodyError::NotRigidBody`] for a soft body: Jolt would add the torque as well,
    /// and a soft body never clears its torque (it resets only the force after a step), so the
    /// torque would stay in the body and in its saved state. Use [`add_force`](Self::add_force)
    /// or set vertex velocities instead.
    ///
    /// [docs/limits.md#impulses]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#impulses
    pub fn add_force_at_point(&mut self, force: Vec3, point: RVec3) -> Result<(), BodyError> {
        self.reject_soft_body()?;
        require(force.is_finite(), "force must be finite")?;
        require(
            is_in_frame(point),
            "point must be finite and within limits::MAX_POSITION",
        )?;
        self.check_load(force, Some(point), Vec3::ZERO)?;
        let mut force = force.to_jph();
        let mut point = point.to_jph();
        // SAFETY: as in `set_position`.
        unsafe {
            JPH_BodyInterface_AddForce2(self.interface(), self.id.raw, &mut force, &mut point)
        };
        Ok(())
    }

    /// Adds a torque in newton-metres for the next step, and wakes the body.
    ///
    /// The torque must be finite, and on a dynamic body the torque accumulated this step
    /// including this one may give the body at most [`limits::MAX_ANGULAR_ACCELERATION`] (`|τ|`
    /// times the largest principal inverse inertia); otherwise [`BodyError::InvalidValue`] is
    /// returned and nothing changes.
    ///
    /// Fails with [`BodyError::NotRigidBody`] for a soft body, which Jolt ignores torque for and
    /// never clears it on.
    pub fn add_torque(&mut self, torque: Vec3) -> Result<(), BodyError> {
        self.reject_soft_body()?;
        require(torque.is_finite(), "torque must be finite")?;
        self.check_load(Vec3::ZERO, None, torque)?;
        let mut torque = torque.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_BodyInterface_AddTorque(self.interface(), self.id.raw, &mut torque) };
        Ok(())
    }

    /// Fails with [`BodyError::NotRigidBody`] when the body is a soft body.
    pub(super) fn reject_soft_body(&self) -> Result<(), BodyError> {
        if self.is_soft_body() {
            Err(BodyError::NotRigidBody(self.id))
        } else {
            Ok(())
        }
    }

    /// Checks that adding `force` (at `point`, or at the centre of mass) and `torque` keeps this
    /// step's accumulated load within the acceleration bounds of [`limits`], and returns the
    /// force to give Jolt: `force` itself, or for a soft body `force` in the body frame. Reads
    /// the body under a read lock that is released before the caller adds the load: Jolt's body
    /// mutexes are not recursive.
    fn check_load(
        &self,
        force: Vec3,
        point: Option<RVec3>,
        torque: Vec3,
    ) -> Result<Vec3, BodyError> {
        let load = with_read_locked_body(self.inner.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure.
            unsafe { read_load(body) }
        })
        .ok_or(BodyError::NotFound(self.id))?;
        let state = match load {
            // Jolt ignores loads on static and kinematic bodies.
            Load::Ignored => return Ok(force),
            Load::Soft(state) => return soft_body_force(&state, force),
            Load::Rigid(state) => state,
        };
        let point_torque = match point {
            Some(point) => point_torque(force, point, state.center_of_mass)?,
            None => PointTorque::default(),
        };
        let new_force = sum(state.force, force, [0.0; 3]);
        let new_torque = point_torque.largest_with(sum(state.torque, torque, [0.0; 3]));
        let largest_inverse_inertia = state
            .inverse_inertia
            .x
            .max(state.inverse_inertia.y)
            .max(state.inverse_inertia.z);
        require(
            length(new_force) * f64::from(state.inverse_mass)
                <= f64::from(limits::MAX_ACCELERATION),
            "accumulated force would exceed limits::MAX_ACCELERATION for this body",
        )?;
        require(
            length(new_torque) * f64::from(largest_inverse_inertia)
                <= f64::from(limits::MAX_ANGULAR_ACCELERATION),
            "accumulated torque would exceed limits::MAX_ANGULAR_ACCELERATION for this body",
        )?;
        Ok(force)
    }

    /// Discards the force and torque added since the last step. Jolt clears them after every
    /// step anyway. Does nothing for static and kinematic bodies; works on soft bodies.
    pub fn reset_forces(&mut self) {
        with_locked_body(self.inner.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure. Jolt's
            // `ResetForce` and `ResetTorque` need motion properties, which only dynamic bodies
            // are guaranteed to have here, hence the `IsDynamic` check first.
            unsafe {
                if JPH_Body_IsDynamic(body.as_ptr()) {
                    JPH_Body_ResetForce(body.as_ptr());
                    JPH_Body_ResetTorque(body.as_ptr());
                }
            }
        });
    }
}
