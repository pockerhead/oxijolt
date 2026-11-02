//! Structural changes of one body: its motion type and its shape, and the rules a body's shape
//! and mass meet at creation and after a change.

use oxijolt_sys::*;

use super::access::check_box;
use super::load::require;
use super::{
    has_finite_inverse, mass_properties, with_locked_body, with_read_locked_body, Activation,
    BodyMut, MassOverride, MotionType, INERTIA_RULE, KINEMATIC_MESH_MASS_RULE, MESH_DYNAMIC_RULE,
    SENSOR_SHAPE_RULE, STATIC_SHAPE_RULE,
};
use crate::limits::{is_mass, is_vertex_inverse_mass, MASS_RULE};
use crate::shape::static_only_leaves_are_meshes_of;
use crate::{BodyError, RVec3, Shape, Vec3};

/// How a body uses a shape: what decides which shapes and masses it may take.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ShapeUse {
    pub(crate) motion_type: MotionType,
    /// Whether the body has motion properties: it is not static, or it was created able to
    /// become kinematic or dynamic.
    pub(crate) can_move: bool,
    pub(crate) sensor: bool,
    /// The mass, and maybe the inertia, the body is given, or `None` for the shape's own.
    pub(crate) mass: Option<MassOverride>,
}

/// Checks that a body used as `using` describes may take `shape`, and returns the mass
/// properties Jolt gives it when it has motion properties (Jolt itself checks none of this).
///
/// A sensor may not use a shape that only static bodies may use (`Shape::MustBeStatic`). A body
/// that can move may use one only when every such leaf is a mesh, given a mass, and only when it
/// is not dynamic. Jolt cannot collide meshes, heightfields and planes with each other
/// ("Unsupported shape pair" in `CollisionDispatch`), and such pairs stay out of reach: only bodies that are not
/// dynamic carry meshes, and Jolt pairs a kinematic body with a static or kinematic one only
/// with `mCollideKinematicVsNonDynamic`, which this crate does not expose, or with a sensor
/// (`Body::sFindCollidingPairsCanCollide`), which takes no static-only shape; query, character
/// and ragdoll shapes refuse static-only shapes. Exposing that switch, or letting sensors take
/// such shapes, must revisit this rule.
///
/// A body that can move also needs mass properties with finite inverses
/// ([`has_finite_inverse`]), and a dynamic one a mass within `MIN_MASS..=MAX_MASS`.
pub(crate) fn check_shape_for(
    shape: &Shape,
    using: ShapeUse,
) -> Result<Option<JPH_MassProperties>, BodyError> {
    let static_only = shape.must_be_static();
    require(!(using.sensor && static_only), SENSOR_SHAPE_RULE)?;
    if !using.can_move {
        return Ok(None);
    }
    if static_only {
        require(shape.static_only_leaves_are_meshes(), STATIC_SHAPE_RULE)?;
        require(using.motion_type != MotionType::Dynamic, MESH_DYNAMIC_RULE)?;
        require(using.mass.is_some(), KINEMATIC_MESH_MASS_RULE)?;
    }
    let properties = mass_properties(shape, using.mass);
    require(has_finite_inverse(&properties), INERTIA_RULE)?;
    if using.motion_type == MotionType::Dynamic {
        require(is_mass(properties.mass), MASS_RULE)?;
    }
    Ok(Some(properties))
}

/// What [`BodyMut::set_shape`] reads of the body before it changes anything.
struct ShapeChange {
    motion_type: MotionType,
    can_move: bool,
    sensor: bool,
    /// Whether a dynamic body holds forces or torques added since the last step.
    pending_load: bool,
    bounds: JPH_AABox,
    /// The old shape's centre of mass in the body's space.
    center_of_mass: Vec3,
}

/// Which other bodies a shape change wakes.
enum NeighbourWake<'a> {
    /// Those near the box enclosing the body's old and new bounds.
    Enclosing,
    /// Those near any of these boxes `(min, max)`, already checked.
    Regions(&'a [(RVec3, RVec3)]),
}

/// An empty box for a getter to fill.
pub(super) const NO_BOUNDS: JPH_AABox = JPH_AABox {
    min: JPH_Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    },
    max: JPH_Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    },
};

/// Why a body cannot become dynamic or kinematic: its shape or mass.
enum MotionBlock {
    StaticShape,
    Mass,
}

impl BodyMut<'_> {
    /// Makes the body static, kinematic or dynamic (Jolt `BodyInterface::SetMotionType`).
    ///
    /// To static, Jolt puts an awake body to sleep ([`ActivationEvent::Deactivated`]) and zeroes
    /// its velocities; to kinematic or static, it drops the forces added since the last step;
    /// to kinematic or dynamic with [`Activation::Activate`], a sleeping body wakes
    /// ([`ActivationEvent::Activated`]). The body keeps its object layer, shape, velocities
    /// (unless made static) and degrees of freedom. The same motion type changes nothing.
    ///
    /// Jolt does not save motion types, so a successful change makes every earlier
    /// [`WorldState`](crate::WorldState) unrestorable ([`StateError::WorldChanged`]).
    ///
    /// Fails, changing nothing, with:
    /// - [`BodyError::OwnedByCharacter`], [`BodyError::UsedByVehicle`],
    ///   [`BodyError::OwnedByRagdoll`] or [`BodyError::UsedByConstraint`] for a body that a
    ///   character, vehicle, ragdoll or constraint holds, whose checks assume the motion type it
    ///   had, also when the motion type would not change;
    /// - [`BodyError::NotRigidBody`] for a soft body, which Jolt keeps dynamic;
    /// - [`BodyError::CannotMove`] for kinematic or dynamic on a body created static without
    ///   [`BodySettings::allow_dynamic_or_kinematic`];
    /// - [`BodyError::InvalidValue`] for dynamic when the shape contains a mesh, heightfield or plane,
    ///   or the mass is outside `MIN_MASS..=MAX_MASS` (a kinematic body's mass is not bounded
    ///   at creation), and for kinematic when the shape contains a static-only leaf that is not
    ///   a mesh.
    ///
    /// [`ActivationEvent::Deactivated`]: crate::ActivationEvent::Deactivated
    /// [`ActivationEvent::Activated`]: crate::ActivationEvent::Activated
    /// [`StateError::WorldChanged`]: crate::StateError::WorldChanged
    /// [`BodySettings::allow_dynamic_or_kinematic`]: crate::BodySettings::allow_dynamic_or_kinematic
    pub fn set_motion_type(
        &mut self,
        motion_type: MotionType,
        activation: Activation,
    ) -> Result<(), BodyError> {
        let id = self.inner.id;
        self.world.check_not_owned(id)?;
        self.reject_soft_body()?;
        if self.motion_type() == motion_type {
            return Ok(());
        }
        if motion_type != MotionType::Static {
            if !self.can_be_kinematic_or_dynamic() {
                return Err(BodyError::CannotMove(id));
            }
            match self.motion_block(motion_type) {
                Some(MotionBlock::StaticShape) => {
                    let rule = if motion_type == MotionType::Dynamic {
                        MESH_DYNAMIC_RULE
                    } else {
                        STATIC_SHAPE_RULE
                    };
                    return Err(BodyError::InvalidValue(rule));
                }
                Some(MotionBlock::Mass) => return Err(BodyError::InvalidValue(MASS_RULE)),
                None => {}
            }
        }
        // Jolt does not save motion types.
        self.world.note_structure_change();
        // SAFETY: the world is borrowed mutably through this view and holds the body, a rigid
        // body; a kinematic or dynamic target has motion properties (checked above), as Jolt
        // asserts. Jolt deactivates the body itself before making it static. This thread holds
        // no body lock.
        unsafe {
            JPH_BodyInterface_SetMotionType(
                self.interface(),
                id.raw,
                motion_type.to_jph(),
                activation.to_jph(),
            )
        };
        Ok(())
    }

    /// Gives the body a new shape (Jolt `BodyInterface::SetShape`). The body takes its own
    /// reference to `shape`, so the caller may drop it, and Jolt releases the old one.
    ///
    /// The body origin stays where it is and the centre of mass moves with the shape. Jolt
    /// stores the centre of mass position and derives the origin from it, so the origin read
    /// back can move by rounding ([docs/bodies.md#shape-and-motion-type-changes]). A body that
    /// can move gets the mass properties a body created with `shape` and `mass` would have
    /// ([`BodySettings::mass`]): `Some(m)` scales the shape's inertia to the mass `m`, `None`
    /// takes the shape's own mass and inertia. A static body that cannot move ignores `mass`,
    /// which is still checked. Velocities, forces, the motion type and the degrees of freedom
    /// are kept. With [`Activation::Activate`] the body wakes.
    ///
    /// Jolt does not wake the bodies around a body whose shape changes, so this wakes every
    /// other non-static body whose current bounds come within the speculative contact distance
    /// of the box enclosing the body's old and new bounds, in body-id order, as
    /// [`PhysicsWorld::remove_body`] does: nothing stays asleep
    /// floating above a shrunk floor or embedded in a grown one. When the old and new bounds lie
    /// apart, bodies between them that touch neither shape wake too.
    ///
    /// Jolt does not save shapes, so every successful call, also one with the body's current
    /// shape, makes every earlier [`WorldState`](crate::WorldState) unrestorable
    /// ([`StateError::WorldChanged`]).
    ///
    /// A static body may change shape while constraints use it. The constraints are told how
    /// far the centre of mass moved (Jolt `Constraint::NotifyShapeChanged`), so their attachment
    /// points stay where they are in world space up to rounding, also where a point no longer
    /// lies on the new shape: removing material does not break a weld. The joints keep their
    /// warm start, motor states and ids. Points given in [`ConstraintSpace::LocalToBodyCom`]
    /// referred to the old centre of mass; a constraint created afterwards uses the new one.
    ///
    /// A character touching the body keeps the sub-shape ids it
    /// read from the old shape until its next update, so
    /// [`CharacterRef::ground_compound_child`] and [`CharacterRef::contact_compound_child`] read
    /// them against the new shape meanwhile.
    ///
    /// Fails, changing nothing, with:
    /// - [`BodyError::OwnedByCharacter`], [`BodyError::UsedByVehicle`] or
    ///   [`BodyError::OwnedByRagdoll`] for a body that a character, vehicle or ragdoll holds;
    /// - [`BodyError::UsedByConstraint`] for a kinematic or dynamic body that a constraint
    ///   uses, whose creation checks used the body's mass, inertia and centre of mass;
    /// - [`BodyError::NotRigidBody`] for a soft body;
    /// - [`BodyError::InvalidValue`] when the body's motion type, movement capability or sensor
    ///   flag refuses the shape or mass, as [`PhysicsWorld::create_body`] would refuse them, or
    ///   when a dynamic body holds forces or torques added since the last step (call
    ///   [`reset_forces`](Self::reset_forces) first).
    ///
    /// [`BodySettings::mass`]: crate::BodySettings::mass
    /// [docs/bodies.md#shape-and-motion-type-changes]: https://github.com/pockerhead/oxijolt/blob/main/docs/bodies.md#shape-and-motion-type-changes
    /// [`PhysicsWorld::remove_body`]: crate::PhysicsWorld::remove_body
    /// [`PhysicsWorld::create_body`]: crate::PhysicsWorld::create_body
    /// [`StateError::WorldChanged`]: crate::StateError::WorldChanged
    /// [`ConstraintSpace::LocalToBodyCom`]: crate::ConstraintSpace::LocalToBodyCom
    /// [`CharacterRef::ground_compound_child`]: crate::CharacterRef::ground_compound_child
    /// [`CharacterRef::contact_compound_child`]: crate::CharacterRef::contact_compound_child
    pub fn set_shape(
        &mut self,
        shape: &Shape,
        mass: Option<f32>,
        activation: Activation,
    ) -> Result<(), BodyError> {
        self.change_shape(shape, mass, activation, NeighbourWake::Enclosing)
    }

    /// Gives the body a new shape as [`set_shape`](Self::set_shape) does, but wakes only the
    /// neighbours near `regions`, boxes `(min, max)` in world space (metres).
    ///
    /// Every other non-static body whose current bounds come within the speculative contact
    /// distance of any of the boxes wakes, each once, in body-id order; bodies elsewhere stay
    /// asleep, and an empty slice wakes no neighbours. Use it when a large static body changes in
    /// a small part, such as a compound that gains or loses a few children: `set_shape` would
    /// wake everything near the whole body. `activation` still decides whether the body itself
    /// wakes.
    ///
    /// The caller is responsible for the regions: a region that misses a changed part leaves
    /// bodies asleep floating above removed material or embedded in added material. Cover the
    /// old and the new bounds of every child that changed.
    ///
    /// Fails, changing nothing, as `set_shape` does, and with [`BodyError::InvalidValue`] when a
    /// region's corners are not within [`limits::MAX_POSITION`] or `min` exceeds `max` on some
    /// axis, as [`PhysicsWorld::activate_bodies_in_box`] requires.
    ///
    /// [`limits::MAX_POSITION`]: crate::limits::MAX_POSITION
    /// [`PhysicsWorld::activate_bodies_in_box`]: crate::PhysicsWorld::activate_bodies_in_box
    pub fn set_shape_waking_in(
        &mut self,
        shape: &Shape,
        mass: Option<f32>,
        activation: Activation,
        regions: &[(RVec3, RVec3)],
    ) -> Result<(), BodyError> {
        for &(min, max) in regions {
            check_box(min, max)?;
        }
        self.change_shape(shape, mass, activation, NeighbourWake::Regions(regions))
    }

    /// What [`set_shape`](Self::set_shape) and [`set_shape_waking_in`](Self::set_shape_waking_in)
    /// share: everything but which neighbours wake.
    fn change_shape(
        &mut self,
        shape: &Shape,
        mass: Option<f32>,
        activation: Activation,
        wake: NeighbourWake<'_>,
    ) -> Result<(), BodyError> {
        let id = self.inner.id;
        self.check_shape_owner()?;
        self.reject_soft_body()?;
        if let Some(mass) = mass {
            require(is_mass(mass), MASS_RULE)?;
        }
        let before = self.shape_change()?;
        require(
            !before.pending_load,
            "a body with pending forces cannot change shape; reset_forces first",
        )?;
        let properties = check_shape_for(
            shape,
            ShapeUse {
                motion_type: before.motion_type,
                can_move: before.can_move,
                sensor: before.sensor,
                mass: mass.map(MassOverride::Mass),
            },
        )?;

        // Jolt does not save shapes.
        self.world.note_structure_change();
        // SAFETY: the world is borrowed mutably through this view and holds the body, a rigid
        // body; `shape` is live for the call and the body takes its own reference to it. Jolt
        // keeps the old mass properties here; they are replaced below. This thread holds no
        // body lock.
        unsafe {
            JPH_BodyInterface_SetShape(
                self.interface(),
                id.raw,
                shape.as_ptr(),
                false,
                Activation::DontActivate.to_jph(),
            )
        };
        // Jolt's own subtraction (`Body::UpdateCenterOfMassInternal`), lane by lane in f32.
        let new = shape.center_of_mass();
        let old = before.center_of_mass;
        let delta = Vec3::new(new.x - old.x, new.y - old.y, new.z - old.z);
        self.world.notify_shape_changed(id, delta);
        let after = with_locked_body(self.inner.body_lock_interface, id, |body| {
            let body = body.as_ptr();
            let mut bounds = NO_BOUNDS;
            // SAFETY: `body` is locked for writing for the duration of the closure. A body
            // with mass properties to write has motion properties (`check_shape_for` returns
            // them only then), which the unchecked getter reads without the assertion of the
            // checked one on a static body that may move. The properties are those of a body
            // created with this shape and mass, checked to have finite inverses, and the
            // degrees of freedom are the body's own. `bounds` is a live local.
            unsafe {
                if let Some(properties) = &properties {
                    let motion = JPH_Body_GetMotionPropertiesUnchecked(body);
                    let dofs = JPH_MotionProperties_GetAllowedDOFs(motion);
                    JPH_MotionProperties_SetMassProperties(motion, dofs, properties);
                }
                JPH_Body_GetWorldSpaceBounds(body, &mut bounds);
            }
            bounds
        })
        .unwrap_or_else(|| unreachable!("`&mut` keeps the body in the world"));
        // Jolt's `SetShape` invalidated the contact cache of a changed shape itself and left it
        // alone for the same shape, whose mass may have changed. The pending request makes the
        // next simulating step invalidate it either way, also after a restore of a state saved
        // before that step.
        self.world.pending_cache_invalidations.insert(id.raw);
        if activation == Activation::Activate {
            self.activate();
        }
        match wake {
            NeighbourWake::Enclosing => self.wake_around(&before.bounds, &after),
            NeighbourWake::Regions(regions) => self.world.wake_bodies_near_boxes(regions, Some(id)),
        }
        Ok(())
    }

    /// Wakes the other bodies near the box enclosing the body's `old` and `new` bounds.
    fn wake_around(&mut self, old: &JPH_AABox, new: &JPH_AABox) {
        let enclosing = JPH_AABox {
            min: JPH_Vec3 {
                x: old.min.x.min(new.min.x),
                y: old.min.y.min(new.min.y),
                z: old.min.z.min(new.min.z),
            },
            max: JPH_Vec3 {
                x: old.max.x.max(new.max.x),
                y: old.max.y.max(new.max.y),
                z: old.max.z.max(new.max.z),
            },
        };
        self.world.wake_bodies_near(&enclosing, Some(self.inner.id));
    }

    /// Refuses a body whose shape a character, vehicle, ragdoll or constraint relies on
    /// (`PhysicsWorld::check_not_owned`), except a static body that only constraints use: a
    /// static body's mass never reaches them and they are told where its centre of mass moved.
    fn check_shape_owner(&self) -> Result<(), BodyError> {
        match self.world.check_not_owned(self.inner.id) {
            Err(BodyError::UsedByConstraint(_)) if self.motion_type() == MotionType::Static => {
                Ok(())
            }
            result => result,
        }
    }

    /// Reads what [`set_shape`](Self::set_shape) checks, under a read lock that is released
    /// before it returns.
    fn shape_change(&self) -> Result<ShapeChange, BodyError> {
        with_read_locked_body(self.inner.body_lock_interface, self.inner.id, |body| {
            let body = body.as_ptr();
            let mut bounds = NO_BOUNDS;
            let (mut force, mut torque) = (Vec3::ZERO.to_jph(), Vec3::ZERO.to_jph());
            let mut center_of_mass = Vec3::ZERO.to_jph();
            // SAFETY: `body` is locked for reading for the duration of the closure, which keeps
            // its shape alive; the getters only read and write the live locals. The accumulated
            // force and torque are read only from a dynamic body, which has motion properties.
            unsafe {
                let dynamic = JPH_Body_IsDynamic(body);
                if dynamic {
                    JPH_Body_GetAccumulatedForce(body, &mut force);
                    JPH_Body_GetAccumulatedTorque(body, &mut torque);
                }
                JPH_Body_GetWorldSpaceBounds(body, &mut bounds);
                JPH_Shape_GetCenterOfMass(JPH_Body_GetShape(body), &mut center_of_mass);
                ShapeChange {
                    motion_type: MotionType::from_jph(JPH_Body_GetMotionType(body)),
                    can_move: JPH_Body_CanBeKinematicOrDynamic(body),
                    sensor: JPH_Body_IsSensor(body),
                    pending_load: dynamic
                        && (Vec3::from_jph(force) != Vec3::ZERO
                            || Vec3::from_jph(torque) != Vec3::ZERO),
                    bounds,
                    center_of_mass: Vec3::from_jph(center_of_mass),
                }
            }
        })
        .ok_or(BodyError::NotFound(self.inner.id))
    }

    /// What keeps the body, which has motion properties, from becoming `motion_type`
    /// (kinematic or dynamic): a static-only leaf that is not a mesh, or for dynamic any
    /// static-only shape or a mass outside the dynamic range.
    fn motion_block(&self, motion_type: MotionType) -> Option<MotionBlock> {
        with_read_locked_body(self.inner.body_lock_interface, self.inner.id, |body| {
            let body = body.as_ptr();
            // SAFETY: `body` is locked for reading for the duration of the closure, which keeps
            // its shape alive. It has motion properties (the caller checked
            // `CanBeKinematicOrDynamic`), so the unchecked getter reads a live member without
            // the assertion of the checked getter on static bodies; the getters only read.
            unsafe {
                let shape = JPH_Body_GetShape(body);
                if JPH_Shape_MustBeStatic(shape) {
                    let meshes_only = static_only_leaves_are_meshes_of(shape);
                    if motion_type == MotionType::Dynamic || !meshes_only {
                        return Some(MotionBlock::StaticShape);
                    }
                }
                let properties = JPH_Body_GetMotionPropertiesUnchecked(body);
                let inverse_mass = JPH_MotionProperties_GetInverseMassUnchecked(properties);
                (motion_type == MotionType::Dynamic && !is_vertex_inverse_mass(inverse_mass))
                    .then_some(MotionBlock::Mass)
            }
        })
        .flatten()
    }
}
