//! Floating-origin rebase: one rigid change of coordinates for the whole world.

use oxijolt_sys::*;

use crate::body::with_locked_body;
use crate::limits;
use crate::{AllowedDofs, BodyError, BodyId, MotionType, PhysicsWorld, Quat, RVec3, Vec3};

/// One body's pose and velocities in a frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct BodyFrameState {
    pub(super) position: RVec3,
    pub(super) rotation: Quat,
    pub(super) linear_velocity: Vec3,
    pub(super) angular_velocity: Vec3,
}

/// One character's pose, up and velocity in a frame.
#[derive(Clone, Copy, Debug, PartialEq)]
struct CharacterFrameState {
    position: RVec3,
    rotation: Quat,
    up: Vec3,
    linear_velocity: Vec3,
}

/// The rigid change of frame of a rebase: `p -> rotation * p + translation`.
#[derive(Clone, Copy, Debug)]
pub(super) struct FrameChange {
    pub(super) rotation: Quat,
    pub(super) translation: RVec3,
}

impl FrameChange {
    fn rotates(&self) -> bool {
        self.rotation != Quat::IDENTITY
    }

    fn is_noop(&self) -> bool {
        !self.rotates() && self.translation == RVec3::ZERO
    }

    /// A direction or velocity in the new frame; unchanged, bit for bit, without rotation.
    fn vector(&self, v: Vec3) -> Vec3 {
        if self.rotates() {
            self.rotation.rotate(v)
        } else {
            v
        }
    }

    fn point(&self, p: RVec3) -> RVec3 {
        let p = if self.rotates() {
            self.rotation.rotate_real(p)
        } else {
            p
        };
        RVec3::new(
            p.x + self.translation.x,
            p.y + self.translation.y,
            p.z + self.translation.z,
        )
    }

    /// `state` in the new frame, or `None` when a value would not be finite or the rotation
    /// not a valid unit quaternion: the rule every pose and velocity setter applies.
    pub(super) fn body(&self, state: BodyFrameState) -> Option<BodyFrameState> {
        let rotation = if self.rotates() {
            self.rotation.product(state.rotation).normalized()
        } else {
            state.rotation
        };
        let new = BodyFrameState {
            position: self.point(state.position),
            rotation,
            linear_velocity: self.vector(state.linear_velocity),
            angular_velocity: self.vector(state.angular_velocity),
        };
        let valid = new.position.is_finite()
            && new.rotation.is_valid_rotation()
            && new.linear_velocity.is_finite()
            && new.angular_velocity.is_finite();
        valid.then_some(new)
    }

    /// `state` in the new frame, or `None` when a value would not be finite or the rotation or
    /// up not of unit length: the rule every character setter applies.
    fn character(&self, state: CharacterFrameState) -> Option<CharacterFrameState> {
        let rotation = if self.rotates() {
            self.rotation.product(state.rotation).normalized()
        } else {
            state.rotation
        };
        let new = CharacterFrameState {
            position: self.point(state.position),
            rotation,
            up: self.vector(state.up),
            linear_velocity: self.vector(state.linear_velocity),
        };
        let unit_up = (new.up.dot(new.up) - 1.0).abs() <= 1.0e-5;
        let valid = new.position.is_finite()
            && new.rotation.is_valid_rotation()
            && new.up.is_finite()
            && unit_up
            && new.linear_velocity.is_finite();
        valid.then_some(new)
    }
}

impl PhysicsWorld {
    /// Moves the whole world into a new frame: one rigid change of coordinates, for a floating
    /// origin.
    ///
    /// Every body origin `p` becomes `rotation * p + translation` (metres), every body rotation
    /// `q` becomes `rotation * q`, linear and angular velocities `v` become `rotation * v`
    /// (m/s, rad/s), and so does the world's gravity. Characters, vehicle gravity overrides and
    /// collision testers, and pulleys move along; [docs/guide.md#floating-origin] says how.
    ///
    /// `bodies_in_key_order` must name every body of the world exactly once, in the caller's
    /// stable key order, which is the order the poses are written in; otherwise
    /// [`BodyError::InvalidValue`], [`BodyError::WrongWorld`] or [`BodyError::NotFound`] is
    /// returned. A detached body ([`create_body_detached`](Self::create_body_detached)) counts as
    /// a body of the world and fails with [`BodyError::Detached`], so add or destroy detached
    /// bodies before a rebase. `rotation` must be a finite unit quaternion and `translation` finite with every
    /// component at most `2 *` [`limits::MAX_POSITION`] in absolute value, and no new pose or
    /// velocity may overflow; otherwise [`BodyError::InvalidValue`] is returned. The new positions
    /// are only checked to be finite. A rotating rebase fails with [`BodyError::RestrictedDofs`]
    /// for the first listed body with fewer than six degrees of freedom: Jolt locks world axes,
    /// which a rotation would move. Every check runs before the first write, so an error leaves
    /// the world unchanged.
    ///
    /// No body is woken or put to sleep. An identity `rotation` leaves rotations, velocities
    /// and gravity untouched, bit for bit; an identity rotation with a zero translation changes
    /// nothing.
    ///
    /// Rebase between steps, before adding the tick's forces: forces and torques added since the
    /// last step are not rotated. After a rotation, call
    /// [`refresh_character_contacts`](Self::refresh_character_contacts) for every character
    /// before its next update.
    ///
    /// [docs/guide.md#floating-origin]: https://github.com/pockerhead/oxijolt/blob/main/docs/guide.md#floating-origin
    pub fn rebase(
        &mut self,
        bodies_in_key_order: &[BodyId],
        rotation: Quat,
        translation: RVec3,
    ) -> Result<(), BodyError> {
        let invalid = |what| Err(BodyError::InvalidValue(what));
        if !rotation.is_valid_rotation() {
            return invalid("rebase rotation must be a finite unit quaternion");
        }
        if !limits::is_frame_displacement(translation) {
            return invalid(
                "rebase translation must be finite and at most 2 * limits::MAX_POSITION per axis",
            );
        }
        let frame = FrameChange {
            rotation,
            translation,
        };

        let mut changes = Vec::with_capacity(bodies_in_key_order.len());
        for &id in bodies_in_key_order {
            let body = self.body(id)?;
            let old = BodyFrameState {
                position: body.position(),
                rotation: body.rotation(),
                linear_velocity: body.linear_velocity(),
                angular_velocity: body.angular_velocity(),
            };
            let Some(new) = frame.body(old) else {
                return invalid("rebase would give a body a non-finite pose or velocity");
            };
            changes.push((id, body.motion_type(), old, new));
        }
        let mut raw_ids: Vec<u32> = bodies_in_key_order.iter().map(|id| id.to_raw()).collect();
        raw_ids.sort_unstable();
        if raw_ids.windows(2).any(|pair| pair[0] == pair[1]) {
            return invalid("rebase body list names a body twice");
        }
        if bodies_in_key_order.len() != self.body_count() as usize {
            return invalid("rebase body list must name every body of the world");
        }
        // A rotation keeps gravity's length, which `set_gravity` bounds, so it stays finite.
        let gravity = frame.vector(self.gravity());
        if frame.is_noop() {
            return Ok(());
        }
        let pulleys = self
            .rebased_pulleys(|p| frame.point(p))
            .map_err(BodyError::InvalidValue)?;
        let mut characters = Vec::with_capacity(self.characters.len());
        for id in self.character_ids().collect::<Vec<_>>() {
            let character = self
                .character(id)
                .unwrap_or_else(|_| unreachable!("listed by the world"));
            let old = CharacterFrameState {
                position: character.position(),
                rotation: character.rotation(),
                up: character.up(),
                linear_velocity: character.linear_velocity(),
            };
            let Some(new) = frame.character(old) else {
                return invalid("rebase would give a character a non-finite pose or velocity");
            };
            characters.push((id, new));
        }
        let vehicles = if frame.rotates() {
            self.rotated_vehicles(|v| frame.vector(v))
                .map_err(BodyError::InvalidValue)?
        } else {
            Vec::new()
        };

        // Jolt's locked axes are world axes, so a rotation would turn a locked body out of its
        // plane, and the clamped velocity setters below would mask the rotated velocities.
        if frame.rotates() {
            for (id, ..) in &changes {
                if self.body(*id)?.allowed_dofs() != AllowedDofs::ALL {
                    return Err(BodyError::RestrictedDofs(*id));
                }
            }
        }

        // A rebase recreates pulleys, which a state saved before cannot match, and rotates the
        // vehicles' gravity overrides, which Jolt does not save.
        self.note_structure_change();
        for (id, motion_type, old, new) in changes {
            let position = new.position.to_jph();
            let rotation = new.rotation.to_jph();
            // SAFETY: the body interface belongs to this live world, borrowed mutably; the
            // checks above found `id` in this world and `&mut self` keeps it there. `position`
            // and `rotation` are live locals, finite and a unit quaternion. This thread holds no
            // body lock.
            unsafe {
                JPH_BodyInterface_SetPositionAndRotation(
                    self.body_interface.as_ptr(),
                    id.to_raw(),
                    &position,
                    &rotation,
                    JPH_Activation_DontActivate,
                )
            };
            let moving = old.linear_velocity != Vec3::ZERO || old.angular_velocity != Vec3::ZERO;
            if frame.rotates() && motion_type != MotionType::Static && moving {
                let linear = new.linear_velocity.to_jph();
                let angular = new.angular_velocity.to_jph();
                // `id` resolves (checked above), so the closure runs.
                with_locked_body(self.body_lock_interface, id, |body| {
                    // SAFETY: `body` is locked for writing for the duration of the closure and
                    // is not static, as Jolt's velocity setters assert. The clamped setters
                    // write the motion properties only and never activate the body.
                    // `linear` and `angular` are live locals.
                    unsafe {
                        if !JPH_Body_IsStatic(body.as_ptr()) {
                            JPH_Body_SetLinearVelocityClamped(body.as_ptr(), &linear);
                            JPH_Body_SetAngularVelocityClamped(body.as_ptr(), &angular);
                        }
                    }
                });
            }
        }
        // After the bodies: the character setters place each inner body absolutely at the
        // character's new pose, which is the pose the body loop gave it. Up goes first, because
        // the inner body's position includes the padding along up.
        for (id, new) in characters {
            let mut character = self
                .character_mut(id)
                .unwrap_or_else(|_| unreachable!("listed by the world"));
            // Position and velocity are re-expressed state, checked finite by
            // `FrameChange::character`; the frame and velocity bounds of the public setters apply
            // to caller input only.
            let written = character.set_up(new.up).and_then(|()| {
                character.write_position(new.position);
                character.set_rotation(new.rotation)
            });
            debug_assert_eq!(written, Ok(()), "checked by `FrameChange::character`");
            character.write_linear_velocity(new.linear_velocity);
        }
        self.apply_vehicle_rebase(vehicles);
        // After the bodies: Jolt computes each new pulley's world attachment points from the
        // bodies' new poses.
        self.apply_pulley_rebase(pulleys);
        if frame.rotates() {
            let gravity = gravity.to_jph();
            // SAFETY: the system is live and borrowed mutably; `gravity` is a live local.
            unsafe { JPH_PhysicsSystem_SetGravity(self.system.as_ptr(), &gravity) };
        }
        Ok(())
    }
}
