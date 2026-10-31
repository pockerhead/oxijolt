//! Creating and removing soft bodies, and read and write access to their vertices.

use std::marker::PhantomData;
use std::ops::Deref;
use std::ptr::NonNull;

use oxijolt_sys::*;

use super::settings::creation_settings;
use super::shared::{vertex_mass, INERTIA_RULE, TOTAL_MASS_RULE, VERTEX_INVERSE_MASS_RULE};
use super::{SoftBodySettings, SoftBodySharedSettings};
use crate::body::{with_locked_body, with_read_locked_body};
use crate::limits::{
    self, is_in_frame, is_linear_velocity, is_soft_body_force, is_soft_body_inertia,
    is_soft_body_pressure, is_vertex_inverse_mass, SoftBodyMassDistribution, LINEAR_VELOCITY_RULE,
};
use crate::world::DELTA_TIME_RULE;
use crate::{BodyError, BodyId, PhysicsWorld, RVec3, Real, Vec3};

impl PhysicsWorld {
    /// Creates a soft body from `shared` and adds it to the world. The body keeps its own
    /// reference to the shared settings, so they may be dropped afterwards.
    ///
    /// The body is an ordinary body of the world (see [docs/soft-bodies.md]): it has
    /// a [`BodyId`], is removed with [`remove_body`](Self::remove_body) and read with
    /// [`body`](Self::body); [`soft_body`](Self::soft_body) and
    /// [`soft_body_mut`](Self::soft_body_mut) give access to its vertices. With
    /// [`SoftBodySettings::make_rotation_identity`] (the default) the vertices start at
    /// `position + rotation · vertex position`.
    ///
    /// A skinned body ([`SoftBodySharedSettingsBuilder::skinned`](crate::SoftBodySharedSettingsBuilder::skinned)) starts with its skin at the
    /// bind pose of its inverse binds and holds that pose until it is skinned again.
    ///
    /// Fails with [`BodyError::UnknownObjectLayer`] or [`BodyError::InvalidValue`] when a
    /// setting is out of range or the pressure is too high for the volume the faces of
    /// `shared` enclose (see [`SoftBodySettings::pressure`]), and with
    /// [`BodyError::TooManyBodies`] when the world is full; then nothing changes.
    ///
    /// Jolt computes the body's inertia in `f32` about the body origin from every vertex and
    /// decomposes it (unless a vertex is kinematic), which fails for vertices far from the
    /// origin compared with their spread, or on one line. Such a body is refused with
    /// [`BodyError::InvalidValue`] ([docs/limits.md#soft-body-inertia]). Give the vertices
    /// around the origin and place the body with [`SoftBodySettings::position`]. The check is
    /// conservative for thin bodies: a free ribbon narrower than about 1/70 of its length or a
    /// tube of radius below about 1/130 of its length is refused even centred on the origin,
    /// although Jolt decomposes it. Widen it or make a vertex kinematic, which skips the check.
    ///
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// // A 3 x 3 cloth hanging from two corners.
    /// let mut vertices = Vec::new();
    /// for z in 0..3 {
    ///     for x in 0..3 {
    ///         let position = Vec3::new(0.5 * x as f32, 0.0, 0.5 * z as f32);
    ///         vertices.push(if z == 0 && x != 1 {
    ///             SoftBodyVertex::kinematic(position)
    ///         } else {
    ///             SoftBodyVertex::new(position)
    ///         });
    ///     }
    /// }
    /// let mut faces = Vec::new();
    /// for z in 0..2 {
    ///     for x in 0..2 {
    ///         let i = z * 3 + x;
    ///         faces.push([i, i + 3, i + 4]);
    ///         faces.push([i, i + 4, i + 1]);
    ///     }
    /// }
    /// let cloth = SoftBodySharedSettings::builder(vertices, faces)
    ///     .create_constraints(SoftBodyBendType::Distance, SoftBodyVertexAttributes::default())
    ///     .build()?;
    /// let id = world.create_soft_body(
    ///     &cloth,
    ///     &SoftBodySettings::default().position(RVec3::new(0.0, 2.0, 0.0)),
    /// )?;
    /// for _ in 0..30 {
    ///     assert!(world.step(1.0 / 60.0)?.is_complete());
    /// }
    /// let vertices = world.soft_body(id)?.vertices();
    /// assert_eq!(vertices[0].inverse_mass, 0.0);
    /// assert!(vertices[8].position.y < 2.0);
    /// # Ok(())
    /// # }
    /// ```
    ///
    /// [docs/limits.md#soft-body-inertia]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-inertia
    /// [docs/soft-bodies.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/soft-bodies.md
    pub fn create_soft_body(
        &mut self,
        shared: &SoftBodySharedSettings,
        settings: &SoftBodySettings,
    ) -> Result<BodyId, BodyError> {
        settings.validate(self.object_layer_count)?;
        if !is_soft_body_pressure(settings.pressure, &shared.pressure_geometry) {
            // Faces around the body origin, wound counter-clockwise seen from outside, that
            // enclose a volume large enough for the pressure (`limits::is_soft_body_pressure`).
            return Err(BodyError::InvalidValue(
                "pressure needs counter-clockwise faces enclosing a large enough volume",
            ));
        }
        let baked_rotation = settings.make_rotation_identity.then_some(settings.rotation);
        if !is_soft_body_inertia(&shared.mass_distribution, baked_rotation) {
            return Err(BodyError::InvalidValue(INERTIA_RULE));
        }
        let creation = creation_settings(shared, settings)?;
        if !self.has_room_for_bodies(1) {
            return Err(BodyError::TooManyBodies);
        }
        let bind_pose = shared
            .skin
            .as_ref()
            .map(|skin| skin.bind_pose_matrices(baked_rotation));
        self.note_structure_change();
        // SAFETY: the body interface belongs to this live world, borrowed mutably; `creation` is
        // a fully set up settings object whose layer exists in this world.
        let body = unsafe {
            JPH_BodyInterface_CreateSoftBody(self.body_interface.as_ptr(), creation.as_ptr())
        };
        if body.is_null() {
            return Err(BodyError::TooManyBodies);
        }
        if let Some(joints) = &bind_pose {
            // Two calls make the previous and the current skin positions the bind positions,
            // so the first step neither jumps nor starts from Jolt's NaN skin state.
            for _ in 0..2 {
                // SAFETY: `body` was just created and is not added to the world yet, and the
                // world is borrowed mutably, so no step and no other thread reaches it. `joints`
                // holds one matrix per joint up to the largest bound one and lives for the call.
                let skinned = unsafe {
                    JPH_Body_SkinSoftBodyVertices(body, joints.as_ptr(), joints.len() as u32, false)
                };
                if !skinned {
                    // The settings have skinned constraints and `joints` covers every bound
                    // joint, the only refusals of the native call.
                    unreachable!("skinning a new soft body to its bind pose was refused");
                }
            }
        }
        // SAFETY: `body` is the live body created above; adding it is what Jolt's
        // `CreateAndAddSoftBody` does after `CreateSoftBody`.
        let raw = unsafe {
            let raw = JPH_Body_GetID(body);
            JPH_BodyInterface_AddBody(
                self.body_interface.as_ptr(),
                raw,
                settings.activation.to_jph(),
            );
            raw
        };
        Ok(BodyId::new(raw, self.tag))
    }

    /// Whether `id` names a soft body; `Ok` if so, [`BodyError::NotSoftBody`] for a rigid one.
    fn check_soft_body(&self, id: BodyId) -> Result<(), BodyError> {
        self.check(id)?;
        let is_soft = with_read_locked_body(self.body_lock_interface, id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure; the getter
            // only reads it.
            unsafe { JPH_Body_IsSoftBody(body.as_ptr()) }
        })
        .ok_or(BodyError::NotFound(id))?;
        if is_soft {
            Ok(())
        } else {
            Err(BodyError::NotSoftBody(id))
        }
    }

    /// Read access to the vertices of a soft body; [`BodyError::NotSoftBody`] for a rigid body.
    pub fn soft_body(&self, id: BodyId) -> Result<SoftBodyRef<'_>, BodyError> {
        self.check_soft_body(id)?;
        Ok(SoftBodyRef {
            body_interface: self.body_interface,
            body_lock_interface: self.body_lock_interface,
            id,
            _world: PhantomData,
        })
    }

    /// Read and write access to the vertices of a soft body; [`BodyError::NotSoftBody`] for a
    /// rigid body.
    pub fn soft_body_mut(&mut self, id: BodyId) -> Result<SoftBodyMut<'_>, BodyError> {
        self.check_soft_body(id)?;
        Ok(SoftBodyMut {
            inner: SoftBodyRef {
                body_interface: self.body_interface,
                body_lock_interface: self.body_lock_interface,
                id,
                _world: PhantomData,
            },
            _world: PhantomData,
        })
    }
}

/// One vertex of a soft body in the world, as [`SoftBodyRef::vertices`] reads it.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct SoftBodyVertexState {
    /// Position in world space, in metres.
    pub position: RVec3,
    /// Velocity in world space, in m/s. Jolt puts a soft body to sleep once the fastest vertex
    /// is slower than about 0.17 m/s (it compares the squared speed with
    /// `PhysicsSettings::mPointVelocitySleepThreshold`, 0.03) without zeroing the vertex
    /// velocities, so a sleeping body reads back such speeds and resumes them when it wakes.
    pub velocity: Vec3,
    /// Inverse mass in 1/kg; 0 for a kinematic vertex.
    pub inverse_mass: f32,
}

/// Read access to the vertices of one soft body, borrowed from its world.
///
/// Each method copies what it reads under Jolt's body read lock, so a readout is consistent.
/// The view borrows the world, so the world cannot step while it exists. Not `Send` or `Sync`;
/// share the world instead.
pub struct SoftBodyRef<'w> {
    pub(super) body_interface: NonNull<JPH_BodyInterface>,
    pub(super) body_lock_interface: NonNull<JPH_BodyLockInterface>,
    pub(super) id: BodyId,
    _world: PhantomData<&'w PhysicsWorld>,
}

impl std::fmt::Debug for SoftBodyRef<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SoftBodyRef")
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}

/// World positions, world velocities and inverse masses of a soft body's vertices.
type VertexArrays = (Vec<JPH_RVec3>, Vec<JPH_Vec3>, Vec<f32>);

impl SoftBodyRef<'_> {
    /// The body's id.
    pub fn id(&self) -> BodyId {
        self.id
    }

    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure; the getter
            // only reads it.
            unsafe { JPH_Body_GetSoftBodyVertexCount(body.as_ptr()) as usize }
        })
        .unwrap_or(0)
    }

    /// Every vertex in world space, in vertex order.
    pub fn vertices(&self) -> Vec<SoftBodyVertexState> {
        let mut vertices = Vec::new();
        self.vertices_into(&mut vertices);
        vertices
    }

    /// Like [`vertices`](Self::vertices), into `out`, which is cleared first; reusing it
    /// avoids an allocation per call.
    pub fn vertices_into(&self, out: &mut Vec<SoftBodyVertexState>) {
        out.clear();
        let (positions, velocities, inverse_masses) = self.arrays();
        out.extend(
            positions
                .into_iter()
                .zip(velocities)
                .zip(inverse_masses)
                .map(|((position, velocity), inverse_mass)| SoftBodyVertexState {
                    position: RVec3::from_jph(position),
                    velocity: Vec3::from_jph(velocity),
                    inverse_mass,
                }),
        );
    }

    /// Copies every vertex under one body read lock.
    fn arrays(&self) -> VertexArrays {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure. joltc writes
            // at most `count` elements to each output, and each holds exactly `count`.
            unsafe {
                let count = JPH_Body_GetSoftBodyVertexCount(body.as_ptr());
                let mut positions = vec![RVec3::ZERO.to_jph(); count as usize];
                let mut velocities = vec![Vec3::ZERO.to_jph(); count as usize];
                let mut inverse_masses = vec![0.0; count as usize];
                JPH_Body_GetSoftBodyVertices(
                    body.as_ptr(),
                    positions.as_mut_ptr(),
                    velocities.as_mut_ptr(),
                    inverse_masses.as_mut_ptr(),
                    count,
                );
                (positions, velocities, inverse_masses)
            }
        })
        .unwrap_or_default()
    }

    /// Every vertex position as Jolt stores it, relative to the body's centre of mass in the
    /// body frame, copied under one body read lock.
    fn local_positions(&self) -> Vec<Vec3> {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure. joltc writes
            // at most `count` elements to the output, which holds exactly `count`.
            unsafe {
                let count = JPH_Body_GetSoftBodyVertexCount(body.as_ptr());
                let mut positions = vec![Vec3::ZERO.to_jph(); count as usize];
                JPH_Body_GetSoftBodyVertexLocalPositions(
                    body.as_ptr(),
                    positions.as_mut_ptr(),
                    count,
                );
                positions.into_iter().map(Vec3::from_jph).collect()
            }
        })
        .unwrap_or_default()
    }
}

/// Read and write access to the vertices of one soft body, borrowed mutably from its world.
///
/// Dereferences to [`SoftBodyRef`] for reads. Every setter validates its input before it
/// reaches Jolt, changes nothing when it fails, and wakes the body when it succeeds.
///
/// Vertex inverse masses set here are configuration that a [`WorldState`](crate::WorldState)
/// does not save, and LRA anchors stay those chosen when the shared settings were built.
pub struct SoftBodyMut<'w> {
    inner: SoftBodyRef<'w>,
    _world: PhantomData<&'w mut PhysicsWorld>,
}

impl std::fmt::Debug for SoftBodyMut<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SoftBodyMut")
            .field("id", &self.inner.id)
            .finish_non_exhaustive()
    }
}

impl<'w> Deref for SoftBodyMut<'w> {
    type Target = SoftBodyRef<'w>;

    fn deref(&self) -> &SoftBodyRef<'w> {
        &self.inner
    }
}

impl SoftBodyMut<'_> {
    /// Sets the world-space velocity of vertex `index` in m/s, finite and at most
    /// [`limits::MAX_LINEAR_VELOCITY`] long. A kinematic vertex keeps this velocity until it is
    /// set again.
    pub fn set_vertex_velocity(&mut self, index: u32, velocity: Vec3) -> Result<(), BodyError> {
        self.check_index(index, self.vertex_count())?;
        require_body(is_linear_velocity(velocity), LINEAR_VELOCITY_RULE)?;
        self.write_velocity(index, velocity)
    }

    /// Sets the inverse mass of vertex `index` in 1/kg: 0 pins the vertex (it becomes
    /// kinematic), otherwise the inverse of a mass within
    /// [`limits::MIN_MASS`]`..=`[`limits::MAX_MASS`]. The masses of the movable vertices must
    /// still add up to at most [`limits::MAX_MASS`], and the force added to the body this step
    /// ([`BodyMut::add_force`](crate::BodyMut::add_force)) must stay within its bound for the
    /// new inverse masses, so unpinning a vertex cannot release a force that was accepted
    /// while every vertex was pinned. Jolt recomputes the body's mass and inertia from the
    /// vertices at their current positions; while any vertex is kinematic the body's mass is
    /// infinite ([`BodyRef::mass`](crate::BodyRef::mass) is `None`). Without a kinematic vertex
    /// the vertices must spread around the body origin for Jolt to decompose the inertia, as at
    /// creation ([`PhysicsWorld::create_soft_body`]): unpinning the last kinematic vertex of a
    /// body whose vertices drifted far from its origin
    /// ([`SoftBodySettings::update_position`]`(false)`) is refused.
    pub fn set_vertex_inverse_mass(
        &mut self,
        index: u32,
        inverse_mass: f32,
    ) -> Result<(), BodyError> {
        require_body(
            inverse_mass == 0.0 || is_vertex_inverse_mass(inverse_mass),
            VERTEX_INVERSE_MASS_RULE,
        )?;
        let (_, _, mut inverse_masses) = self.arrays();
        self.check_index(index, inverse_masses.len())?;
        inverse_masses[index as usize] = inverse_mass;
        let total_mass: f64 = inverse_masses
            .iter()
            .filter(|&&w| w > 0.0)
            .map(|&w| vertex_mass(w))
            .sum();
        require_body(total_mass <= f64::from(limits::MAX_MASS), TOTAL_MASS_RULE)?;
        let distribution = SoftBodyMassDistribution::new(
            self.local_positions()
                .into_iter()
                .zip(inverse_masses.iter().copied()),
        );
        require_body(is_soft_body_inertia(&distribution, None), INERTIA_RULE)?;
        let largest_inverse_mass = inverse_masses.iter().copied().fold(0.0, f32::max);
        let vertex_count = inverse_masses.len() as u32;
        let written = with_locked_body(self.body_lock_interface, self.id, |body| {
            let mut force = Vec3::ZERO.to_jph();
            // SAFETY: `body` is locked for writing for the duration of the closure. A soft body
            // is always dynamic, so it has the force accumulator the getter reads; `force` is a
            // live local.
            unsafe { JPH_Body_GetAccumulatedForce(body.as_ptr(), &mut force) };
            let force = Vec3::from_jph(force);
            let force = [force.x, force.y, force.z].map(f64::from);
            if !is_soft_body_force(force, largest_inverse_mass, vertex_count) {
                return false;
            }
            // SAFETY: as above; `index` names one of the body's vertices.
            unsafe { JPH_Body_SetSoftBodyVertexInvMass(body.as_ptr(), index, inverse_mass) };
            true
        })
        .ok_or(BodyError::NotFound(self.id))?;
        require_body(
            written,
            "force added this step exceeds the soft body force bound; step or reset forces first",
        )?;
        self.activate();
        Ok(())
    }

    /// Moves the kinematic vertex `index` to the world position `target` over the next step of
    /// `delta_time` seconds, by setting its velocity to `(target - position) / delta_time`.
    ///
    /// The vertex must be kinematic (inverse mass 0), `target` within
    /// [`limits::MAX_POSITION`], `delta_time` one that [`PhysicsWorld::step`] accepts, and the
    /// velocity at most [`limits::MAX_LINEAR_VELOCITY`] long. The velocity stays after the
    /// step, as for a kinematic body Jolt moves with `MoveKinematic`: the vertex keeps moving
    /// until it is moved again or stopped with
    /// [`set_vertex_velocity`](Self::set_vertex_velocity)`(index, Vec3::ZERO)`.
    pub fn move_kinematic_vertex(
        &mut self,
        index: u32,
        target: RVec3,
        delta_time: f32,
    ) -> Result<(), BodyError> {
        require_body(
            is_in_frame(target),
            "target must be finite and within limits::MAX_POSITION",
        )?;
        require_body(
            PhysicsWorld::is_valid_delta_time(delta_time),
            DELTA_TIME_RULE,
        )?;
        let (positions, _, inverse_masses) = self.arrays();
        self.check_index(index, positions.len())?;
        require_body(
            inverse_masses[index as usize] == 0.0,
            "only a kinematic vertex (inverse mass 0) can be moved",
        )?;
        let position = RVec3::from_jph(positions[index as usize]);
        let delta_time = Real::from(delta_time);
        // `Real` is `f32` without the `double-precision` feature, so the casts are no-ops there.
        #[allow(clippy::unnecessary_cast)]
        let velocity = Vec3::new(
            ((target.x - position.x) / delta_time) as f32,
            ((target.y - position.y) / delta_time) as f32,
            ((target.z - position.z) / delta_time) as f32,
        );
        require_body(
            is_linear_velocity(velocity),
            "the move needs a velocity above limits::MAX_LINEAR_VELOCITY",
        )?;
        self.write_velocity(index, velocity)
    }

    fn check_index(&self, index: u32, count: usize) -> Result<(), BodyError> {
        require_body(
            (index as usize) < count,
            "vertex index must name a vertex of the soft body",
        )
    }

    fn write_velocity(&mut self, index: u32, velocity: Vec3) -> Result<(), BodyError> {
        let velocity = velocity.to_jph();
        with_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure, `index` names
            // one of its vertices and `velocity` is a live local.
            unsafe { JPH_Body_SetSoftBodyVertexVelocity(body.as_ptr(), index, &velocity) }
        })
        .ok_or(BodyError::NotFound(self.id))?;
        self.activate();
        Ok(())
    }

    /// Wakes the body. Called after the write lock is released: `ActivateBody` locks the body
    /// again, and Jolt's body mutexes are not recursive.
    pub(super) fn activate(&mut self) {
        // SAFETY: the world is borrowed mutably through this view and holds the body; this
        // thread holds no body lock.
        unsafe { JPH_BodyInterface_ActivateBody(self.body_interface.as_ptr(), self.id.to_raw()) };
    }
}

pub(super) fn require_body(valid: bool, what: &'static str) -> Result<(), BodyError> {
    if valid {
        Ok(())
    } else {
        Err(BodyError::InvalidValue(what))
    }
}
