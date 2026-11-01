//! Body properties Jolt can change on a live body: materials, gravity, damping, motion quality,
//! the collision filters (object layer and collision group) and the sensor flag.
//!
//! Jolt saves none of them in a [`WorldState`](crate::WorldState), and rereads them in every
//! step: a restore keeps the values set last, so a caller that rolls back sets the old values
//! back and replays ([docs/state.md#what-is-not-saved]). The sensor toggle is the exception: it
//! counts as a structural change.
//!
//! [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved

use oxijolt_sys::*;

use super::access::corners;
use super::load::require;
use super::structure::NO_BOUNDS;
use super::{
    check_object_layer, is_restitution, with_locked_body, with_read_locked_body, Activation,
    BodyMut, BodyRef, MotionQuality, ANGULAR_DAMPING_RULE, LINEAR_DAMPING_RULE, RESTITUTION_RULE,
    SENSOR_QUALITY_RULE, SENSOR_SHAPE_RULE,
};
use crate::collision_group::{same_group, NO_COLLISION_GROUP};
use crate::limits::{self, is_friction, is_gravity_factor};
use crate::math::is_finite_non_negative;
use crate::{BodyError, CollisionGroup, ObjectLayer};

/// Which bodies a motion property applies to.
#[derive(Clone, Copy, PartialEq)]
enum Bodies {
    /// Every body with motion properties, soft bodies too.
    Moving,
    /// Rigid bodies with motion properties: Jolt's soft body solver uses no angular damping and
    /// no motion quality.
    MovingRigid,
}

impl BodyRef<'_> {
    /// The friction coefficient, `0..=`[`limits::MAX_FRICTION`]
    /// ([`BodySettings::friction`](crate::BodySettings::friction),
    /// [`SoftBodySettings::friction`](crate::SoftBodySettings::friction)).
    pub fn friction(&self) -> f32 {
        // SAFETY: the body interface belongs to the world this view borrows, which holds the
        // body and cannot change while the borrow lasts.
        unsafe { JPH_BodyInterface_GetFriction(self.interface(), self.id.raw) }
    }

    /// The restitution, `0..=1` ([`BodySettings::restitution`](crate::BodySettings::restitution),
    /// [`SoftBodySettings::restitution`](crate::SoftBodySettings::restitution)).
    pub fn restitution(&self) -> f32 {
        // SAFETY: as in `friction`.
        unsafe { JPH_BodyInterface_GetRestitution(self.interface(), self.id.raw) }
    }

    /// The multiplier for the world's gravity
    /// ([`BodySettings::gravity_factor`](crate::BodySettings::gravity_factor)); `None` for a body
    /// without motion properties, a static body that cannot move.
    pub fn gravity_factor(&self) -> Option<f32> {
        self.can_be_kinematic_or_dynamic().then(|| {
            // SAFETY: as in `friction`; no body lock is held on this thread.
            unsafe { JPH_BodyInterface_GetGravityFactor(self.interface(), self.id.raw) }
        })
    }

    /// The linear damping, in 1/s
    /// ([`BodySettings::linear_damping`](crate::BodySettings::linear_damping),
    /// [`SoftBodySettings::linear_damping`](crate::SoftBodySettings::linear_damping)); `None`
    /// for a body without motion properties.
    pub fn linear_damping(&self) -> Option<f32> {
        self.read_motion_properties(Bodies::Moving, |properties| {
            // SAFETY: `properties` belongs to the body locked for reading around this call; the
            // getter only reads.
            unsafe { JPH_MotionProperties_GetLinearDamping(properties) }
        })
    }

    /// The angular damping, in 1/s
    /// ([`BodySettings::angular_damping`](crate::BodySettings::angular_damping)); `None` for a
    /// body without motion properties and for a soft body, whose solver uses none.
    pub fn angular_damping(&self) -> Option<f32> {
        self.read_motion_properties(Bodies::MovingRigid, |properties| {
            // SAFETY: as in `linear_damping`.
            unsafe { JPH_MotionProperties_GetAngularDamping(properties) }
        })
    }

    /// How the body is integrated
    /// ([`BodySettings::motion_quality`](crate::BodySettings::motion_quality)); `None` for a body
    /// without motion properties and for a soft body. A static body created with
    /// [`BodySettings::allow_dynamic_or_kinematic`](crate::BodySettings::allow_dynamic_or_kinematic)
    /// reports the quality it will move with.
    pub fn motion_quality(&self) -> Option<MotionQuality> {
        self.read_motion_properties(Bodies::MovingRigid, |properties| {
            // SAFETY: as in `linear_damping`.
            MotionQuality::from_jph(unsafe { JPH_MotionProperties_GetMotionQuality(properties) })
        })
    }

    /// The object layer ([`BodySettings::object_layer`](crate::BodySettings::object_layer),
    /// [`SoftBodySettings::object_layer`](crate::SoftBodySettings::object_layer)).
    pub fn object_layer(&self) -> ObjectLayer {
        // SAFETY: as in `friction`.
        ObjectLayer::new(unsafe { JPH_BodyInterface_GetObjectLayer(self.interface(), self.id.raw) })
    }

    /// Whether the body's collision group is `group`, or with `None` whether it has none
    /// ([`BodySettings::collision_group`](crate::BodySettings::collision_group)).
    ///
    /// Groups compare as Jolt compares them: the same table, the same group id and the same sub
    /// group. A clone of `group` is the same group; a group with equal ids in another table is
    /// not. A ragdoll part's group, which the ragdoll gives it, equals no group a caller builds.
    pub fn collision_group_is(&self, group: Option<&CollisionGroup>) -> bool {
        let wanted = group.map_or(NO_COLLISION_GROUP, CollisionGroup::to_jph);
        let mut current = NO_COLLISION_GROUP;
        // SAFETY: as in `friction`; `current` is a live local that joltc overwrites with a copy of
        // the body's group, whose table pointer is only compared, never dereferenced.
        unsafe { JPH_BodyInterface_GetCollisionGroup(self.interface(), self.id.raw, &mut current) };
        same_group(&current, &wanted)
    }

    /// Runs `read` on the motion properties of the body, locked for reading, when it is one of
    /// `bodies`; `None` otherwise.
    fn read_motion_properties<R>(
        &self,
        bodies: Bodies,
        read: impl FnOnce(*mut JPH_MotionProperties) -> R,
    ) -> Option<R> {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            let body = body.as_ptr();
            // SAFETY: `body` is locked for reading for the duration of the closure. A body that
            // can be kinematic or dynamic has motion properties, so the unchecked getter returns
            // a live member, without the assertion of the checked getter on a static body that
            // may move; joltc takes a mutable pointer but only reads it, and `read` only reads.
            unsafe {
                let rigid_only = bodies == Bodies::MovingRigid;
                if !JPH_Body_CanBeKinematicOrDynamic(body)
                    || (rigid_only && JPH_Body_IsSoftBody(body))
                {
                    return None;
                }
                Some(read(JPH_Body_GetMotionPropertiesUnchecked(body)))
            }
        })
        .flatten()
    }
}

impl BodyMut<'_> {
    /// Sets the friction coefficient, `0..=`[`limits::MAX_FRICTION`], on any body, rigid or soft,
    /// static too (Jolt `Body::SetFriction`).
    ///
    /// Jolt combines the friction of a contact again in every step from both bodies' values, so
    /// the new value acts from the next step on in every contact of an awake pair. It wakes
    /// nothing: a sleeping body uses it once something wakes it ([`activate`](Self::activate)).
    /// The value is not part of a [`WorldState`](crate::WorldState): a restore keeps it
    /// ([docs/state.md#what-is-not-saved]).
    ///
    /// Fails with [`BodyError::InvalidValue`] for a value outside the range, changing nothing.
    ///
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_friction(&mut self, friction: f32) -> Result<(), BodyError> {
        require(is_friction(friction), limits::FRICTION_RULE)?;
        // SAFETY: the world is borrowed mutably through this view and holds the body; this
        // thread holds no body lock.
        unsafe { JPH_BodyInterface_SetFriction(self.interface(), self.id.raw, friction) };
        Ok(())
    }

    /// Sets the restitution, `0..=1`, on any body, rigid or soft, static too (Jolt
    /// `Body::SetRestitution`).
    ///
    /// Jolt combines it in every step like the friction ([`set_friction`](Self::set_friction)),
    /// and as there it wakes nothing and a restore keeps it ([docs/state.md#what-is-not-saved]).
    ///
    /// Fails with [`BodyError::InvalidValue`] for a value outside the range, changing nothing.
    ///
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_restitution(&mut self, restitution: f32) -> Result<(), BodyError> {
        require(is_restitution(restitution), RESTITUTION_RULE)?;
        // SAFETY: as in `set_friction`.
        unsafe { JPH_BodyInterface_SetRestitution(self.interface(), self.id.raw, restitution) };
        Ok(())
    }

    /// Sets the multiplier for the world's gravity, at most [`limits::MAX_GRAVITY_FACTOR`] in
    /// absolute value, on a body with motion properties, soft bodies too (Jolt
    /// `MotionProperties::SetGravityFactor`). A kinematic body keeps it for when it becomes
    /// dynamic.
    ///
    /// It acts from the next step on and wakes nothing: a sleeping body keeps resting until
    /// something wakes it ([`activate`](Self::activate)). A restore keeps it
    /// ([docs/state.md#what-is-not-saved]). A vehicle whose gravity is set with
    /// [`VehicleMut::set_gravity`](crate::VehicleMut::set_gravity) writes 0 into its chassis'
    /// factor in every step in which the chassis is awake.
    ///
    /// Fails, changing nothing, with [`BodyError::CannotMove`] for a body created static without
    /// [`BodySettings::allow_dynamic_or_kinematic`](crate::BodySettings::allow_dynamic_or_kinematic),
    /// and with [`BodyError::InvalidValue`] for a value outside the range.
    ///
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_gravity_factor(&mut self, factor: f32) -> Result<(), BodyError> {
        self.require_motion_properties(Bodies::Moving)?;
        require(is_gravity_factor(factor), limits::GRAVITY_FACTOR_RULE)?;
        // SAFETY: as in `set_friction`. The body has motion properties (checked above), which
        // Jolt's setter checks again.
        unsafe { JPH_BodyInterface_SetGravityFactor(self.interface(), self.id.raw, factor) };
        Ok(())
    }

    /// Sets the linear damping in 1/s, finite and at least 0, on a body with motion properties,
    /// soft bodies too: Jolt scales the linear velocity (of a soft body, every vertex velocity
    /// in every sub-step) by `max(0, 1 - c * dt)`.
    ///
    /// It acts from the next step on and wakes nothing ([`activate`](Self::activate) wakes the
    /// body). A restore keeps it ([docs/state.md#what-is-not-saved]).
    ///
    /// Fails, changing nothing, with [`BodyError::CannotMove`] for a body created static without
    /// [`BodySettings::allow_dynamic_or_kinematic`](crate::BodySettings::allow_dynamic_or_kinematic),
    /// and with [`BodyError::InvalidValue`] for a value outside the range.
    ///
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_linear_damping(&mut self, damping: f32) -> Result<(), BodyError> {
        self.require_motion_properties(Bodies::Moving)?;
        require(is_finite_non_negative(damping), LINEAR_DAMPING_RULE)?;
        self.write_motion_properties(|properties| {
            // SAFETY: `properties` belongs to the body locked for writing around this call.
            unsafe { JPH_MotionProperties_SetLinearDamping(properties, damping) }
        });
        Ok(())
    }

    /// Sets the angular damping in 1/s, finite and at least 0, on a rigid body with motion
    /// properties: Jolt scales the angular velocity by `max(0, 1 - c * dt)` every step.
    ///
    /// It acts from the next step on and wakes nothing ([`activate`](Self::activate) wakes the
    /// body). A restore keeps it ([docs/state.md#what-is-not-saved]).
    ///
    /// Fails, changing nothing, with [`BodyError::NotRigidBody`] for a soft body, whose solver
    /// uses no angular damping, [`BodyError::CannotMove`] for a body created static without
    /// [`BodySettings::allow_dynamic_or_kinematic`](crate::BodySettings::allow_dynamic_or_kinematic),
    /// and [`BodyError::InvalidValue`] for a value outside the range.
    ///
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_angular_damping(&mut self, damping: f32) -> Result<(), BodyError> {
        self.require_motion_properties(Bodies::MovingRigid)?;
        require(is_finite_non_negative(damping), ANGULAR_DAMPING_RULE)?;
        self.write_motion_properties(|properties| {
            // SAFETY: as in `set_linear_damping`.
            unsafe { JPH_MotionProperties_SetAngularDamping(properties, damping) }
        });
        Ok(())
    }

    /// Sets how the body is integrated, on a rigid body with motion properties (Jolt
    /// `BodyInterface::SetMotionQuality`, which keeps Jolt's count of active continuous bodies
    /// right for awake and sleeping bodies alike). A static body that may move keeps it for when
    /// it moves.
    ///
    /// It acts from the next step on and wakes nothing ([`activate`](Self::activate) wakes the
    /// body). A restore keeps it ([docs/state.md#what-is-not-saved]).
    ///
    /// Fails, changing nothing, with [`BodyError::NotRigidBody`] for a soft body,
    /// [`BodyError::CannotMove`] for a body created static without
    /// [`BodySettings::allow_dynamic_or_kinematic`](crate::BodySettings::allow_dynamic_or_kinematic),
    /// and [`BodyError::InvalidValue`] for [`MotionQuality::LinearCast`] on a sensor, which Jolt
    /// integrates only discretely.
    ///
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_motion_quality(&mut self, quality: MotionQuality) -> Result<(), BodyError> {
        self.require_motion_properties(Bodies::MovingRigid)?;
        require(
            !(quality == MotionQuality::LinearCast && self.is_sensor()),
            SENSOR_QUALITY_RULE,
        )?;
        // SAFETY: as in `set_friction`. The body is rigid and has motion properties (checked
        // above); no step runs while the world is borrowed mutably, so Jolt's active body list
        // is not locked, as its setter asserts.
        unsafe {
            JPH_BodyInterface_SetMotionQuality(self.interface(), self.id.raw, quality.to_jph())
        };
        Ok(())
    }

    /// Moves the body to the object layer `layer` (Jolt `BodyInterface::SetObjectLayer`), and so
    /// to the broad-phase layer that `layer` maps to.
    ///
    /// Queries see the new layer at once. Pairs collide by the new layers from the next step in
    /// which one of their bodies is awake: a pair that no longer collides ends with
    /// [`ContactEvent::Removed`]. Jolt wakes nothing for a layer change, so this wakes the body
    /// with [`Activation::Activate`] and, with either activation, every other non-static body
    /// whose bounds come within the [speculative contact distance] of the body's, in body-id
    /// order: nothing stays asleep resting on a body it no longer collides with. A vehicle's wheels keep the layer of their collision tester, and a
    /// character collides by the new layer from its next update. The same layer changes
    /// nothing and wakes nothing.
    ///
    /// Jolt does not save layers, so a restore keeps the layer set last. To roll back across a
    /// change, set the old layer back before the restore, which then also puts back the sleep
    /// states that the set-back's wake changed, and drop the events queued before the restore
    /// ([docs/state.md#what-is-not-saved]).
    ///
    /// Fails, changing nothing, with [`BodyError::OwnedByCharacter`] for a character's inner
    /// body, whose layer the character's settings give it, and with
    /// [`BodyError::UnknownObjectLayer`] for a layer not in the world's
    /// [`CollisionLayers`](crate::CollisionLayers).
    ///
    /// [`ContactEvent::Removed`]: crate::ContactEvent::Removed
    /// [speculative contact distance]: crate::WorldSettings::speculative_contact_distance
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_object_layer(
        &mut self,
        layer: ObjectLayer,
        activation: Activation,
    ) -> Result<(), BodyError> {
        let id = self.inner.id;
        if self.world.is_inner_body(id) {
            return Err(BodyError::OwnedByCharacter(id));
        }
        check_object_layer(layer, self.world.object_layer_count)?;
        if self.object_layer() == layer {
            return Ok(());
        }
        // SAFETY: as in `set_friction`; the layer exists in the world's layer tables (checked
        // above), which Jolt indexes with it.
        unsafe { JPH_BodyInterface_SetObjectLayer(self.interface(), id.raw, layer.get()) };
        self.wake_after_filter_change(activation);
        Ok(())
    }

    /// Gives the body the collision group `group`, or no group for `None` (Jolt
    /// `Body::SetCollisionGroup`). The body takes its own reference to the group's table, so the
    /// caller may drop `group` and its [`GroupFilterTable`](crate::GroupFilterTable), and Jolt
    /// releases its reference to the old table.
    ///
    /// The solver checks the groups of every pair it collides, from the next step in which one
    /// of the bodies is awake: a pair the new group excludes ends with
    /// [`ContactEvent::Removed`]. Scene queries ignore groups. It wakes the body and the bodies
    /// around it as [`set_object_layer`](Self::set_object_layer) does. The same group
    /// ([`collision_group_is`](BodyRef::collision_group_is)) changes nothing and wakes nothing.
    ///
    /// Jolt does not save groups; roll back across a change as for
    /// [`set_object_layer`](Self::set_object_layer), setting the old group back before the
    /// restore ([docs/state.md#what-is-not-saved]).
    ///
    /// Fails, changing nothing, with [`BodyError::OwnedByRagdoll`] for a ragdoll part, whose
    /// group the ragdoll gives it to filter collisions between its parts.
    ///
    /// [`ContactEvent::Removed`]: crate::ContactEvent::Removed
    /// [docs/state.md#what-is-not-saved]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#what-is-not-saved
    pub fn set_collision_group(
        &mut self,
        group: Option<&CollisionGroup>,
        activation: Activation,
    ) -> Result<(), BodyError> {
        let id = self.inner.id;
        if self.world.is_ragdoll_body(id) {
            return Err(BodyError::OwnedByRagdoll(id));
        }
        if self.collision_group_is(group) {
            return Ok(());
        }
        let value = group.map_or(NO_COLLISION_GROUP, CollisionGroup::to_jph);
        // SAFETY: as in `set_friction`. `value` is a live local; its table, if any, is kept alive
        // by `group` for the call, and joltc's `CollisionGroup` adds the reference the body keeps.
        // Releasing the old table's reference may delete it, which no worker reads: no step runs
        // while the world is borrowed mutably.
        unsafe { JPH_BodyInterface_SetCollisionGroup(self.interface(), id.raw, &value) };
        self.wake_after_filter_change(activation);
        Ok(())
    }

    /// Makes the body a sensor or a solid body again (Jolt `Body::SetIsSensor`). A sensor
    /// detects other bodies without a collision response ([docs/events.md#sensors]).
    ///
    /// The next step that simulates collides the body's pairs afresh. A contact whose shapes
    /// overlap continues as [`ContactEvent::Persisted`] with the new
    /// [`ContactSettings::is_sensor`]. Jolt finds sensor contacts only between overlapping
    /// shapes, so a speculative contact, between shapes that are apart or just touch, ends with
    /// [`ContactEvent::Removed`] when the body becomes a sensor. A sensor made solid while it
    /// overlaps a body pushes that body out over the following steps. A sensor never
    /// falls asleep on its own; a body that stops being one can sleep again. A
    /// [`ContactListener`] still cannot make a contact of a sensor solid
    /// ([`ContactSettingsError::SensorBody`]). The change wakes the body and the bodies around
    /// it as [`set_object_layer`](Self::set_object_layer) does. The same flag changes nothing.
    ///
    /// A change counts as a structural change: every earlier [`WorldState`] is refused with
    /// [`StateError::WorldChanged`], because the toggle discards the body's cached contacts,
    /// which a restore cannot put back ([docs/state.md]).
    ///
    /// Fails, changing nothing, with [`BodyError::OwnedByCharacter`] for a character's inner
    /// body, [`BodyError::UsedByVehicle`] for a vehicle chassis, [`BodyError::OwnedByRagdoll`]
    /// for a ragdoll part, [`BodyError::NotRigidBody`] for a soft body, and, when making a
    /// sensor, with [`BodyError::InvalidValue`] for a body with [`MotionQuality::LinearCast`]
    /// (also one stored by a static body that may move) or a shape only static bodies may use,
    /// the rules of [`BodySettings::sensor`](crate::BodySettings::sensor).
    ///
    /// [`ContactEvent::Persisted`]: crate::ContactEvent::Persisted
    /// [`ContactEvent::Removed`]: crate::ContactEvent::Removed
    /// [`ContactSettings::is_sensor`]: crate::ContactSettings::is_sensor
    /// [`ContactListener`]: crate::ContactListener
    /// [`ContactSettingsError::SensorBody`]: crate::ContactSettingsError::SensorBody
    /// [`WorldState`]: crate::WorldState
    /// [`StateError::WorldChanged`]: crate::StateError::WorldChanged
    /// [docs/events.md#sensors]: https://github.com/pockerhead/oxijolt/blob/main/docs/events.md#sensors
    /// [docs/state.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/state.md#which-world-it-restores-into
    pub fn set_is_sensor(&mut self, sensor: bool, activation: Activation) -> Result<(), BodyError> {
        let id = self.inner.id;
        // A character moves its inner body as a solid kinematic body, a vehicle's checks assume
        // a solid chassis, and a ragdoll's parts collide with their surroundings. A constraint
        // body may change: the flag changes no mass, inertia or attachment.
        if self.world.is_inner_body(id) {
            return Err(BodyError::OwnedByCharacter(id));
        }
        if self.world.is_vehicle_body(id) {
            return Err(BodyError::UsedByVehicle(id));
        }
        if self.world.is_ragdoll_body(id) {
            return Err(BodyError::OwnedByRagdoll(id));
        }
        // Jolt asserts a rigid body.
        self.reject_soft_body()?;
        if self.is_sensor() == sensor {
            return Ok(());
        }
        if sensor {
            self.check_sensor_rules()?;
        }

        self.world.note_structure_change();
        // SAFETY: as in `set_friction`; the body is rigid (checked above), as Jolt asserts.
        unsafe { JPH_BodyInterface_SetIsSensor(self.interface(), id.raw, sensor) };
        // Jolt keeps a cached contact made with the other flag's separation distance and would
        // replay it. The pending request makes the next simulating step discard the cache.
        self.world.pending_cache_invalidations.insert(id.raw);
        self.wake_after_filter_change(activation);
        Ok(())
    }

    /// Checks what a body must satisfy to become a sensor: no [`MotionQuality::LinearCast`]
    /// stored in its motion properties and no shape that only static bodies may use.
    fn check_sensor_rules(&self) -> Result<(), BodyError> {
        let (linear_cast, static_only) =
            with_read_locked_body(self.inner.body_lock_interface, self.inner.id, |body| {
                let body = body.as_ptr();
                // SAFETY: `body` is locked for reading for the duration of the closure, which
                // keeps its shape alive; no shape pointer leaves it. A body that can be
                // kinematic or dynamic has motion properties, which the unchecked getter reads
                // without the assertion of the checked getter on a static body that may move.
                // The getters only read.
                unsafe {
                    let linear_cast = JPH_Body_CanBeKinematicOrDynamic(body)
                        && MotionQuality::from_jph(JPH_MotionProperties_GetMotionQuality(
                            JPH_Body_GetMotionPropertiesUnchecked(body),
                        )) == MotionQuality::LinearCast;
                    (linear_cast, JPH_Shape_MustBeStatic(JPH_Body_GetShape(body)))
                }
            })
            .unwrap_or_else(|| unreachable!("`&mut` keeps the body in the world"));
        require(!linear_cast, SENSOR_QUALITY_RULE)?;
        require(!static_only, SENSOR_SHAPE_RULE)
    }

    /// Wakes the body for [`Activation::Activate`] and, either way, every other non-static body
    /// whose bounds come within the speculative contact distance of it: Jolt collides a pair
    /// only when one of its bodies is awake, so a filter change would otherwise leave sleeping
    /// bodies floating or embedded.
    fn wake_after_filter_change(&mut self, activation: Activation) {
        let id = self.inner.id;
        let mut bounds = with_read_locked_body(self.inner.body_lock_interface, id, |body| {
            let mut bounds = NO_BOUNDS;
            // SAFETY: `body` is locked for reading for the duration of the closure; the getter
            // only reads it and writes the live local.
            unsafe { JPH_Body_GetWorldSpaceBounds(body.as_ptr(), &mut bounds) };
            bounds
        })
        .unwrap_or_else(|| unreachable!("`&mut` keeps the body in the world"));
        if activation == Activation::Activate {
            self.activate();
        }
        // Jolt pairs bodies whose bounds come within this distance (`QuadTree::FindCollidingPairs`
        // widens them in `f32` the same way), so a body resting on a speculative contact need not
        // touch the exact bounds.
        let margin = self.world.speculative_contact_distance();
        let widen = |v: JPH_Vec3, by: f32| JPH_Vec3 {
            x: v.x + by,
            y: v.y + by,
            z: v.z + by,
        };
        bounds.min = widen(bounds.min, -margin);
        bounds.max = widen(bounds.max, margin);
        let (min, max) = corners(&bounds);
        self.world.wake_bodies_overlapping(min, max, Some(id));
    }

    /// Fails with [`BodyError::NotRigidBody`] for a soft body when `bodies` asks for a rigid
    /// one, then with [`BodyError::CannotMove`] for a body without motion properties.
    fn require_motion_properties(&self, bodies: Bodies) -> Result<(), BodyError> {
        if bodies == Bodies::MovingRigid {
            self.reject_soft_body()?;
        }
        if !self.can_be_kinematic_or_dynamic() {
            return Err(BodyError::CannotMove(self.id));
        }
        Ok(())
    }

    /// Runs `write` on the motion properties of the body, locked for writing. The caller checked
    /// that the body has motion properties.
    fn write_motion_properties(&mut self, write: impl FnOnce(*mut JPH_MotionProperties)) {
        with_locked_body(self.inner.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure, and it has
            // motion properties (the caller checked `CanBeKinematicOrDynamic`, which the
            // mutable borrow of the world keeps true), so the unchecked getter returns a live
            // member.
            write(unsafe { JPH_Body_GetMotionPropertiesUnchecked(body.as_ptr()) })
        })
        .unwrap_or_else(|| unreachable!("`&mut` keeps the body in the world"));
    }
}
