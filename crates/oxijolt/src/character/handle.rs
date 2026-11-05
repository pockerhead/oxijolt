//! Read and write access to one character.

use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::ptr::{null, NonNull};

use oxijolt_sys::*;

use super::settings::{
    check_character_shape, check_inner_body_shape, check_supporting_volume, is_mass,
    is_max_slope_angle, is_recovery_speed, is_unit, HIT_REDUCTION_RULE, MASS_RULE,
    MAX_SLOPE_ANGLE_RULE, MAX_STRENGTH_RULE, RECOVERY_SPEED_RULE, SHAPE_OFFSET_RULE, UP_RULE,
};
use super::update::query_error;
use super::{
    CharacterContact, CharacterEntry, CharacterId, CharacterState, GroundState, INVALID_ID,
};
use crate::limits::{self, LINEAR_VELOCITY_RULE, MAX_NUM_HITS_RULE, POSITION_RULE};
use crate::math::{is_finite_non_negative, ROTATION_RULE};
use crate::owned::Owned;
use crate::{
    BodyId, CharacterError, CompoundSubShape, ObjectLayer, PhysicsWorld, Quat, QueryFilter, RVec3,
    Shape, SubShapeId, Vec3,
};

/// Read access to one character, borrowed from its world.
///
/// Each method reads the character's state as its last update, refresh or setter left it.
pub struct CharacterRef<'w> {
    pub(super) world: &'w PhysicsWorld,
    pub(super) id: CharacterId,
    pub(super) character: NonNull<JPH_CharacterVirtual>,
}

impl std::fmt::Debug for CharacterRef<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CharacterRef")
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}

impl CharacterRef<'_> {
    fn ptr(&self) -> *mut JPH_CharacterVirtual {
        self.character.as_ptr()
    }

    fn base(&self) -> *mut JPH_CharacterBase {
        self.character.as_ptr().cast()
    }

    /// The character's id.
    pub fn id(&self) -> CharacterId {
        self.id
    }

    /// Position in world space, metres. The shape sits at this position plus the rotated shape
    /// offset plus the padding along up.
    pub fn position(&self) -> RVec3 {
        let mut value = RVec3::ZERO.to_jph();
        // SAFETY: the world borrowed here owns the character, and changing it needs
        // `&mut PhysicsWorld`; the getter reads a member. `value` is a live local.
        unsafe { JPH_CharacterVirtual_GetPosition(self.ptr(), &mut value) };
        RVec3::from_jph(value)
    }

    /// Rotation.
    pub fn rotation(&self) -> Quat {
        let mut value = Quat::IDENTITY.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetRotation(self.ptr(), &mut value) };
        Quat::from_jph(value)
    }

    /// The up direction, a unit vector.
    pub fn up(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetUp(self.base(), &mut value) };
        Vec3::from_jph(value)
    }

    /// Linear velocity, m/s: what the caller set, as the last update changed it.
    pub fn linear_velocity(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetLinearVelocity(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// How the character stands.
    pub fn ground_state(&self) -> GroundState {
        // SAFETY: as in `position`.
        GroundState::from_jph(unsafe { JPH_CharacterBase_GetGroundState(self.base()) })
    }

    /// Normal of the ground contact, pointing toward the character; zero when there is none.
    pub fn ground_normal(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetGroundNormal(self.base(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The ground contact point in world space, metres.
    pub fn ground_position(&self) -> RVec3 {
        let mut value = RVec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetGroundPosition(self.base(), &mut value) };
        RVec3::from_jph(value)
    }

    /// Velocity of the ground under the character, m/s.
    pub fn ground_velocity(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetGroundVelocity(self.base(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The body the character stands on, or `None` (in the air, or on another character).
    pub fn ground_body(&self) -> Option<BodyId> {
        // SAFETY: as in `position`.
        let raw = unsafe { JPH_CharacterBase_GetGroundBodyId(self.base()) };
        (raw != INVALID_ID).then(|| BodyId::new(raw, self.world.tag))
    }

    /// The leaf of the ground body's shape the character stands on.
    pub fn ground_sub_shape_id(&self) -> SubShapeId {
        // SAFETY: as in `position`.
        SubShapeId::new(unsafe { JPH_CharacterBase_GetGroundSubShapeId(self.base()) })
    }

    /// The compound child the character stands on, with its user data (a collision group);
    /// `None` when the ground is not a compound child or no longer in the world.
    pub fn ground_compound_child(&self) -> Option<CompoundSubShape> {
        let body = self.ground_body()?;
        self.world
            .compound_sub_shape(body, self.ground_sub_shape_id())
            .ok()
            .flatten()
    }

    /// The contacts after the last update, refresh or fitting
    /// [`set_shape`](CharacterMut::set_shape), in Jolt's order (sorted, which is
    /// deterministic while [`max_hits_exceeded`](Self::max_hits_exceeded) is false).
    pub fn active_contacts(&self) -> Vec<CharacterContact> {
        // SAFETY: as in `position`.
        let count = unsafe { JPH_CharacterVirtual_GetNumActiveContacts(self.ptr()) };
        (0..count)
            .map(|index| {
                // SAFETY: an all-zero `JPH_CharacterContact` is valid: integers, floats,
                // `false`, null pointers and `JPH_MotionType_Static` (0).
                let mut contact: JPH_CharacterContact = unsafe { std::mem::zeroed() };
                // SAFETY: as in `position`; `index < count`, so joltc's `at` stays in range, and
                // `contact` is a live local that joltc overwrites.
                unsafe { JPH_CharacterVirtual_GetActiveContact(self.ptr(), index, &mut contact) };
                CharacterContact::from_jph(&contact, self.world.tag)
            })
            .collect()
    }

    /// The compound child `contact` touched, with its user data (a collision group); `None` for
    /// characters, bodies that are not compounds and bodies no longer in the world.
    pub fn contact_compound_child(&self, contact: &CharacterContact) -> Option<CompoundSubShape> {
        self.world
            .compound_sub_shape(contact.body?, contact.sub_shape_id)
            .ok()
            .flatten()
    }

    /// The object layer of the body `contact` touched; `None` for characters and bodies no
    /// longer in the world.
    pub fn contact_object_layer(&self, contact: &CharacterContact) -> Option<ObjectLayer> {
        let body = contact.body?;
        self.world
            .contains_body(body)
            .then(|| self.world.object_layer_of(body))
    }

    /// The character's inner body, if it has one.
    pub fn inner_body(&self) -> Option<BodyId> {
        self.world.characters[&self.id.raw].inner_body
    }

    /// Whether the last update found more contacts than
    /// [`CharacterSettings::max_num_hits`]. Jolt then drops contacts in an order that is not
    /// guaranteed to be deterministic.
    ///
    /// [`CharacterSettings::max_num_hits`]: crate::CharacterSettings::max_num_hits
    pub fn max_hits_exceeded(&self) -> bool {
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetMaxHitsExceeded(self.ptr()) }
    }

    /// Fraction in `[0, 1]` of a penetration the next update resolves: the creation setting
    /// ([`CharacterSettings::penetration_recovery_speed`]) or the last
    /// [`CharacterMut::set_penetration_recovery_speed`].
    ///
    /// [`CharacterSettings::penetration_recovery_speed`]: crate::CharacterSettings::penetration_recovery_speed
    pub fn penetration_recovery_speed(&self) -> f32 {
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetPenetrationRecoverySpeed(self.ptr()) }
    }

    /// Mass in kg with which the character presses on what it stands on.
    pub fn mass(&self) -> f32 {
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetMass(self.ptr()) }
    }

    /// Largest force in newtons with which the character pushes bodies.
    pub fn max_strength(&self) -> f32 {
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetMaxStrength(self.ptr()) }
    }

    /// Cosine of the maximum slope angle; Jolt keeps only the cosine.
    pub fn cos_max_slope_angle(&self) -> f32 {
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetCosMaxSlopeAngle(self.base()) }
    }

    /// The plane, in the character's local space, below which contacts can support the
    /// character: its normal and constant (see [`CharacterSettings::supporting_volume`]).
    ///
    /// [`CharacterSettings::supporting_volume`]: crate::CharacterSettings::supporting_volume
    pub fn supporting_volume(&self) -> (Vec3, f32) {
        let mut plane = JPH_Plane {
            normal: Vec3::ZERO.to_jph(),
            distance: 0.0,
        };
        // SAFETY: as in `position`; `plane` is a live local.
        unsafe { JPH_CharacterBase_GetSupportingVolume(self.base(), &mut plane) };
        (Vec3::from_jph(plane.normal), plane.distance)
    }

    /// Offset of the shape from the character position, in the character's local space,
    /// metres.
    pub fn shape_offset(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetShapeOffset(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// Cosine of the largest angle between two contact normals that Jolt merges into one.
    pub fn hit_reduction_cos_max_angle(&self) -> f32 {
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetHitReductionCosMaxAngle(self.ptr()) }
    }

    /// Whether Jolt removes ghost contacts with internal edges of triangle shapes.
    pub fn enhanced_internal_edge_removal(&self) -> bool {
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetEnhancedInternalEdgeRemoval(self.ptr()) }
    }

    /// Most contacts collected per query.
    pub fn max_num_hits(&self) -> u32 {
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetMaxNumHits(self.ptr()) }
    }

    /// The caller's value for this character.
    pub fn user_data(&self) -> u64 {
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetUserData(self.ptr()) }
    }

    /// The character's persistent state, to restore later with
    /// [`CharacterMut::restore_state`].
    pub fn save_state(&self) -> CharacterState {
        let mut state = CharacterState {
            jolt: Vec::new(),
            up: [0; 3],
        };
        self.save_state_into(&mut state);
        state
    }

    /// Writes the character's persistent state into `state`, reusing its memory.
    pub(crate) fn save_state_into(&self, state: &mut CharacterState) {
        // SAFETY: Jolt is initialised (the world exists). The handle takes over the recorder.
        let recorder = unsafe { Owned::from_raw(JPH_StateRecorder_Create()) }
            .unwrap_or_else(|| unreachable!("`new` does not return null"));
        // SAFETY: as in `position`; `SaveState` is const in Jolt, and the recorder is live and
        // used by this thread only.
        let size = unsafe {
            JPH_CharacterVirtual_SaveState(self.ptr(), recorder.as_ptr());
            JPH_StateRecorder_GetDataSize(recorder.as_ptr())
        };
        state.jolt.clear();
        state.jolt.resize(size, 0);
        // SAFETY: `state.jolt` holds exactly `size` writable bytes, which joltc copies at most.
        unsafe {
            JPH_StateRecorder_CopyData(recorder.as_ptr(), state.jolt.as_mut_ptr().cast(), size)
        };
        let up: [f32; 3] = self.up().into();
        state.up = up.map(f32::to_bits);
    }
}

/// Write access to one character, borrowed mutably from its world.
///
/// Read the character through [`PhysicsWorld::character`] once this borrow ends.
///
/// Each setter checks the value as [`CharacterSettings`](crate::CharacterSettings) does at
/// creation and changes nothing when it refuses it. The character padding, predictive contact
/// distance, collision tolerance, back-face collision, the collision and constraint iteration
/// counts and the minimum time remaining are fixed at creation: Jolt reads them from the
/// settings only. Whether the character collides with other characters is fixed at creation
/// too.
pub struct CharacterMut<'w> {
    pub(super) world: &'w mut PhysicsWorld,
    pub(super) id: CharacterId,
    pub(super) character: NonNull<JPH_CharacterVirtual>,
}

impl std::fmt::Debug for CharacterMut<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CharacterMut").finish_non_exhaustive()
    }
}

impl CharacterMut<'_> {
    fn ptr(&self) -> *mut JPH_CharacterVirtual {
        self.character.as_ptr()
    }

    fn base(&self) -> *mut JPH_CharacterBase {
        self.character.as_ptr().cast()
    }

    fn entry_mut(&mut self) -> &mut CharacterEntry {
        self.world
            .characters
            .get_mut(&self.id.to_raw())
            .unwrap_or_else(|| unreachable!("the borrow keeps the character in its world"))
    }

    /// Moves the character to `position` (metres, every component at most
    /// [`limits::MAX_POSITION`] in absolute value), without collision checks. Also moves the
    /// inner body.
    pub fn set_position(&mut self, position: RVec3) -> Result<(), CharacterError> {
        if !limits::is_in_frame(position) {
            return Err(CharacterError::InvalidValue(POSITION_RULE));
        }
        self.write_position(position);
        Ok(())
    }

    /// Writes a finite position without the frame bound of [`set_position`](Self::set_position),
    /// for state the world re-expresses ([`PhysicsWorld::rebase`]).
    pub(crate) fn write_position(&mut self, position: RVec3) {
        debug_assert!(position.is_finite());
        let position = position.to_jph();
        // SAFETY: the world is borrowed mutably through this view and owns the character. Jolt
        // moves the inner body through the locking body interface; this thread holds no body
        // lock. `position` is a live local.
        unsafe { JPH_CharacterVirtual_SetPosition(self.ptr(), &position) };
    }

    /// Sets the rotation, a finite unit quaternion. Also rotates the inner body.
    pub fn set_rotation(&mut self, rotation: Quat) -> Result<(), CharacterError> {
        if !rotation.is_valid_rotation() {
            return Err(CharacterError::InvalidValue(ROTATION_RULE));
        }
        let rotation = rotation.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_CharacterVirtual_SetRotation(self.ptr(), &rotation) };
        Ok(())
    }

    /// Sets the up direction, a finite unit vector, for the next updates.
    pub fn set_up(&mut self, up: Vec3) -> Result<(), CharacterError> {
        if !is_unit(up) {
            return Err(CharacterError::InvalidValue(UP_RULE));
        }
        let up = up.to_jph();
        // SAFETY: as in `set_position`; the setter writes a member.
        unsafe { JPH_CharacterBase_SetUp(self.ptr().cast(), &up) };
        Ok(())
    }

    /// Sets the linear velocity, m/s, that the next update moves the character with: finite and
    /// at most [`limits::MAX_LINEAR_VELOCITY`] long (Jolt's own length). Jolt does not clamp a
    /// character's velocity; it becomes the displacement the update casts.
    pub fn set_linear_velocity(&mut self, velocity: Vec3) -> Result<(), CharacterError> {
        if !limits::is_linear_velocity(velocity) {
            return Err(CharacterError::InvalidValue(LINEAR_VELOCITY_RULE));
        }
        self.write_linear_velocity(velocity);
        Ok(())
    }

    /// Sets the fraction of a penetration that each update resolves, from the next update on:
    /// 0 resolves nothing, 1 all of it at once. Must be finite and in `[0, 1]`; otherwise
    /// returns [`CharacterError::InvalidValue`] and changes nothing.
    ///
    /// The value is not part of [`CharacterState`] (Jolt's `CharacterVirtual::SaveState` does not
    /// write it), so [`restore_state`](Self::restore_state) keeps the current value; a replay
    /// sets it again.
    pub fn set_penetration_recovery_speed(&mut self, value: f32) -> Result<(), CharacterError> {
        if !is_recovery_speed(value) {
            return Err(CharacterError::InvalidValue(RECOVERY_SPEED_RULE));
        }
        // SAFETY: as in `set_up`; the setter writes a member.
        unsafe { JPH_CharacterVirtual_SetPenetrationRecoverySpeed(self.ptr(), value) };
        Ok(())
    }

    /// Writes a finite velocity without the bound of
    /// [`set_linear_velocity`](Self::set_linear_velocity), for state the world re-expresses
    /// ([`PhysicsWorld::rebase`]).
    pub(crate) fn write_linear_velocity(&mut self, velocity: Vec3) {
        debug_assert!(velocity.is_finite());
        let velocity = velocity.to_jph();
        // SAFETY: as in `set_up`.
        unsafe { JPH_CharacterVirtual_SetLinearVelocity(self.ptr(), &velocity) };
    }

    /// Sets the mass in kg, between 0 and [`limits::MAX_MASS`], with which the character presses
    /// on what it stands on, from the next update on, which also checks its weight impulse
    /// against it ([`PhysicsWorld::update_character`]).
    ///
    /// [`PhysicsWorld::update_character`]: crate::PhysicsWorld::update_character
    pub fn set_mass(&mut self, kg: f32) -> Result<(), CharacterError> {
        if !is_mass(kg) {
            return Err(CharacterError::InvalidValue(MASS_RULE));
        }
        // SAFETY: as in `set_up`; the setter writes a member.
        unsafe { JPH_CharacterVirtual_SetMass(self.ptr(), kg) };
        self.entry_mut().mass = kg;
        Ok(())
    }

    /// Sets the largest force in newtons, finite and at least 0, with which the character
    /// pushes bodies.
    pub fn set_max_strength(&mut self, newtons: f32) -> Result<(), CharacterError> {
        if !is_finite_non_negative(newtons) {
            return Err(CharacterError::InvalidValue(MAX_STRENGTH_RULE));
        }
        // SAFETY: as in `set_up`; the setter writes a member.
        unsafe { JPH_CharacterVirtual_SetMaxStrength(self.ptr(), newtons) };
        Ok(())
    }

    /// Sets the steepest ground the character can stand on, radians in `[0, π/2]`. Jolt keeps
    /// its cosine ([`CharacterRef::cos_max_slope_angle`]) and turns the limit off below about
    /// 0.81°, as [`CharacterSettings::max_slope_angle`] says. The ground state follows at the
    /// next update or refresh.
    ///
    /// [`CharacterSettings::max_slope_angle`]: crate::CharacterSettings::max_slope_angle
    pub fn set_max_slope_angle(&mut self, radians: f32) -> Result<(), CharacterError> {
        if !is_max_slope_angle(radians) {
            return Err(CharacterError::InvalidValue(MAX_SLOPE_ANGLE_RULE));
        }
        // SAFETY: as in `set_up`; the setter writes a member.
        unsafe { JPH_CharacterBase_SetMaxSlopeAngle(self.base(), radians) };
        Ok(())
    }

    /// Sets the plane, in the character's local space, below which contacts can support the
    /// character: `normal`, a finite unit vector, and a finite `constant`, as
    /// [`CharacterSettings::supporting_volume`] describes. The ground state follows at the next
    /// update or refresh.
    ///
    /// [`CharacterSettings::supporting_volume`]: crate::CharacterSettings::supporting_volume
    pub fn set_supporting_volume(
        &mut self,
        normal: Vec3,
        constant: f32,
    ) -> Result<(), CharacterError> {
        check_supporting_volume(normal, constant)?;
        let plane = JPH_Plane {
            normal: normal.to_jph(),
            distance: constant,
        };
        // SAFETY: as in `set_up`; the setter writes a member. `plane` is a live local.
        unsafe { JPH_CharacterBase_SetSupportingVolume(self.base(), &plane) };
        Ok(())
    }

    /// Sets the offset of the shape from the character position, in the character's local
    /// space, every component at most [`limits::MAX_SHAPE_EXTENT`] in absolute value. Jolt also
    /// moves the inner body to the new offset. Neither move checks for collisions.
    pub fn set_shape_offset(&mut self, offset: Vec3) -> Result<(), CharacterError> {
        if !limits::is_local_offset(offset) {
            return Err(CharacterError::InvalidValue(SHAPE_OFFSET_RULE));
        }
        let offset = offset.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_CharacterVirtual_SetShapeOffset(self.ptr(), &offset) };
        Ok(())
    }

    /// Sets the cosine of the largest angle between two contact normals that Jolt merges into
    /// one contact, finite; -1 turns merging off.
    pub fn set_hit_reduction_cos_max_angle(&mut self, cos: f32) -> Result<(), CharacterError> {
        if !cos.is_finite() {
            return Err(CharacterError::InvalidValue(HIT_REDUCTION_RULE));
        }
        // SAFETY: as in `set_up`; the setter writes a member.
        unsafe { JPH_CharacterVirtual_SetHitReductionCosMaxAngle(self.ptr(), cos) };
        Ok(())
    }

    /// Sets whether Jolt removes ghost contacts with internal edges of triangle shapes
    /// (heightfields, meshes), at extra CPU cost per contact.
    pub fn set_enhanced_internal_edge_removal(&mut self, value: bool) {
        // SAFETY: as in `set_up`; the setter writes a member.
        unsafe { JPH_CharacterVirtual_SetEnhancedInternalEdgeRemoval(self.ptr(), value) };
    }

    /// Sets the most contacts collected per query, from 1 to [`limits::MAX_CHARACTER_HITS`],
    /// from the next update or refresh on (see [`CharacterRef::max_hits_exceeded`]).
    pub fn set_max_num_hits(&mut self, value: u32) -> Result<(), CharacterError> {
        if !limits::is_max_num_hits(value) {
            return Err(CharacterError::InvalidValue(MAX_NUM_HITS_RULE));
        }
        // SAFETY: as in `set_up`; the setter writes a member.
        unsafe { JPH_CharacterVirtual_SetMaxNumHits(self.ptr(), value) };
        Ok(())
    }

    /// Sets the caller's value for this character. Jolt also gives it to the inner body as its
    /// body user data.
    pub fn set_user_data(&mut self, value: u64) {
        // SAFETY: as in `set_position`; Jolt writes the inner body's user data through the
        // locking body interface.
        unsafe { JPH_CharacterVirtual_SetUserData(self.ptr(), value) };
    }

    /// Recomputes the ground velocity from the ground body's current velocity (Jolt
    /// `CharacterVirtual::UpdateGroundVelocity`). An update computes it from the ground body's
    /// velocity at that time, so after a [`PhysicsWorld::step`] it is one step old; call this
    /// after the step and before computing the next velocity. Without a ground body, or when it
    /// has left the world, the ground velocity stays.
    ///
    /// The character contact listener's
    /// [`adjust_body_velocity`](crate::CharacterContactListener::adjust_body_velocity) is called
    /// for the ground body as in an update, and a panic in it resumes the same way.
    ///
    /// [`PhysicsWorld::step`]: crate::PhysicsWorld::step
    pub fn update_ground_velocity(&mut self) {
        let character = self.character;
        self.world
            .with_character_filters(self.id, character, &QueryFilter::new(), |_| {
                // SAFETY: as in `set_position`; Jolt reads the ground body under a read lock it
                // takes itself and skips an id that no longer resolves. It calls the attached
                // listener only, and changes no contact.
                unsafe { JPH_CharacterVirtual_UpdateGroundVelocity(character.as_ptr()) };
            })
            .unwrap_or_else(|_| unreachable!("an empty filter creates no native filter"));
    }

    /// Replaces the character's shape with `shape`, which must be convex as at creation, and
    /// returns whether it did.
    ///
    /// Jolt first collects the contacts of `shape` at the character's position, colliding with
    /// what `filter` selects. When one of them that is not a sensor penetrates deeper than
    /// `max_penetration_depth` metres (not negative; `f32::MAX` and `f32::INFINITY` skip the
    /// test and keep the contacts), the old shape stays and this returns `Ok(false)`; otherwise
    /// the collected contacts become the character's contacts, with the ground they give. Use it
    /// to stand up from a crouch only where there is room. The inner body keeps its shape; give it the new
    /// one with [`set_inner_body_shape`](Self::set_inner_body_shape).
    ///
    /// The character takes its own reference to `shape`. When the shape changed, earlier
    /// [`WorldState`](crate::WorldState)s become unrestorable
    /// ([`StateError::WorldChanged`](crate::StateError::WorldChanged)): Jolt does not save shapes.
    /// A shape that did not fit, or the current shape again, keeps them restorable.
    ///
    /// Fails as [`PhysicsWorld::refresh_character_contacts`] does for the filter; the character
    /// contact listener is called, and panics in filter and listener callbacks resume, the same
    /// way. Returns [`CharacterError::InvalidValue`] and changes nothing for a shape that is not
    /// convex or a negative or NaN depth.
    ///
    /// [`PhysicsWorld::refresh_character_contacts`]: crate::PhysicsWorld::refresh_character_contacts
    pub fn set_shape(
        &mut self,
        shape: &Shape,
        max_penetration_depth: f32,
        filter: &QueryFilter<'_>,
    ) -> Result<bool, CharacterError> {
        check_character_shape(shape)?;
        if max_penetration_depth.is_nan() || max_penetration_depth < 0.0 {
            return Err(CharacterError::InvalidValue(
                "max penetration depth must not be negative or NaN",
            ));
        }
        filter.validate(self.world).map_err(query_error)?;
        let character = self.character;
        let allocator = self.world.temp_allocator.as_ptr();
        let base = self.base();
        // SAFETY: as in `set_up`; the getter reads a member.
        let old_shape = unsafe { JPH_CharacterBase_GetShape(base) };
        let mut fits = false;
        // Only Rust frames unwind here (see `with_character_filters`), after the native call.
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            self.world
                .with_character_filters(self.id, character, filter, |raw| {
                    // SAFETY: as in `PhysicsWorld::refresh_character_contacts`, except that the
                    // character's own shape is replaced: only after its query, and the shape filter
                    // caches body root shapes only. `shape` is live for the call and the character
                    // takes its own reference to it.
                    fits = unsafe {
                        JPH_CharacterVirtual_SetShape2(
                            character.as_ptr(),
                            shape.as_ptr(),
                            max_penetration_depth,
                            null(),
                            raw.object_layer,
                            raw.body,
                            raw.shape,
                            allocator,
                        )
                    };
                })
        }));
        // SAFETY: as above.
        if unsafe { JPH_CharacterBase_GetShape(base) } != old_shape {
            self.world.note_structure_change();
        }
        match outcome {
            Ok(result) => result.map(|()| fits),
            Err(payload) => resume_unwind(payload),
        }
    }

    /// Gives the character's inner body `shape`, which must suit a kinematic body as
    /// [`InnerBody::shape`](crate::InnerBody::shape) says. The body keeps its mass properties,
    /// and neither it nor the bodies around it are woken (Jolt
    /// `CharacterVirtual::SetInnerBodyShape`). Earlier [`WorldState`](crate::WorldState)s become
    /// unrestorable, as after a body's shape change.
    ///
    /// Fails with [`CharacterError::NoInnerBody`] for a character without an inner body and with
    /// [`CharacterError::InvalidValue`] for an unsuitable shape, changing nothing.
    pub fn set_inner_body_shape(&mut self, shape: &Shape) -> Result<(), CharacterError> {
        if self.entry_mut().inner_body.is_none() {
            return Err(CharacterError::NoInnerBody(self.id));
        }
        check_inner_body_shape(shape)?;
        self.world.note_structure_change();
        // SAFETY: as in `set_position`; `shape` is live for the call and the body takes its own
        // reference to it. The character has an inner body, whose shape Jolt changes through the
        // locking body interface without updating its mass properties.
        unsafe { JPH_CharacterVirtual_SetInnerBodyShape(self.ptr(), shape.as_ptr()) };
        Ok(())
    }

    /// Restores a state saved with [`CharacterRef::save_state`]: pose, velocity, up, ground
    /// data and the contacts with collision.
    ///
    /// Any state from `save_state` may be restored into any character, also of another world.
    /// The result is meaningful when the worlds match: the same bodies with the same ids and the
    /// same character id and settings, as in a replay that rebuilds the world the same way. Ids
    /// in the state that do not resolve are harmless: Jolt checks every body id it looks up, and
    /// the readers here tolerate any sub-shape id. The inner body is moved to the restored pose,
    /// as [`set_position`](Self::set_position) moves it.
    ///
    /// Restored contacts carry Jolt's default material. The materials the contacts carried
    /// before stay alive until the character's next update, refresh or removal
    /// ([`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) releases them at
    /// once).
    ///
    /// # Errors
    /// [`CharacterError::RestoreFailed`] when Jolt could not read the state.
    pub fn restore_state(&mut self, state: &CharacterState) -> Result<(), CharacterError> {
        // SAFETY: Jolt is initialised (the world exists). The handle takes over the recorder.
        let recorder = unsafe { Owned::from_raw(JPH_StateRecorder_Create()) }
            .unwrap_or_else(|| unreachable!("`new` does not return null"));
        let up = Vec3::from(state.up.map(f32::from_bits)).to_jph();
        // SAFETY: the recorder is live and used by this thread only; `state.jolt` is readable
        // for its length. The bytes are a complete stream written by `SaveState` of this build
        // (`CharacterState` has no other constructor), so `RestoreState` reads exactly what was
        // written. The world is borrowed mutably and owns the character; `up` and `position` are
        // live locals. Jolt's `RestoreState` writes the pose members without moving the inner
        // body; setting the restored position again moves it there through the locking body
        // interface (`CharacterVirtual::SetPosition`), and this thread holds no body lock.
        let failed = unsafe {
            JPH_StateRecorder_WriteBytes(
                recorder.as_ptr(),
                state.jolt.as_ptr().cast(),
                state.jolt.len(),
            );
            JPH_StateRecorder_Rewind(recorder.as_ptr());
            JPH_CharacterVirtual_RestoreState(self.ptr(), recorder.as_ptr());
            JPH_CharacterBase_SetUp(self.ptr().cast(), &up);
            let mut position = RVec3::ZERO.to_jph();
            JPH_CharacterVirtual_GetPosition(self.ptr(), &mut position);
            JPH_CharacterVirtual_SetPosition(self.ptr(), &position);
            JPH_StateRecorder_IsFailed(recorder.as_ptr())
        };
        debug_assert!(!failed, "a saved character state failed to restore");
        if failed {
            return Err(CharacterError::RestoreFailed);
        }
        Ok(())
    }
}
