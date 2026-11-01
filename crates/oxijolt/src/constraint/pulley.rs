//! The pulley constraint: two bodies hang from fixed points on one rope.

use std::ptr::NonNull;

use oxijolt_sys::*;

use super::world::{sealed, ConstraintSettings};
use super::{
    point_anchors, solver::SolverOverrides, validate_point, within, ConstraintSpace, SpringSettings,
};
use crate::body::with_locked_bodies;
use crate::limits;
use crate::owned::Owned;
use crate::{
    ConstraintError, ConstraintMut, ConstraintRef, ConstraintType, PhysicsWorld, PulleyConstraint,
    RVec3,
};

/// The allowed rope length of a [`PulleyConstraintSettings`]: the length is
/// `|fixed_point1 - body_point1| + ratio · |fixed_point2 - body_point2|`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum PulleyLength {
    /// From 0 up to the length when the constraint is created: a rope that can go slack. The
    /// default (Jolt's `0, -1`).
    #[default]
    UpToCurrent,
    /// From `min` to `max` metres: `0 <= min <= max <= (1 + ratio) ·`
    /// [`limits::MAX_SHAPE_EXTENT`]. `min == max` is a rigid rod.
    Range {
        /// Shortest length, metres.
        min: f32,
        /// Longest length, metres.
        max: f32,
    },
}

/// What a pulley's length range must satisfy for `ratio`.
fn validate_pulley_length(min: f32, max: f32, ratio: f32) -> Result<(), &'static str> {
    let longest = (1.0 + ratio) * limits::MAX_SHAPE_EXTENT;
    if within(min, 0.0, longest) && within(max, 0.0, longest) && min <= max {
        Ok(())
    } else {
        Err("pulley lengths must be 0 <= min <= max <= (1 + ratio) * limits::MAX_SHAPE_EXTENT")
    }
}

/// A pulley (Jolt `PulleyConstraintSettings`): a rope from a point on body 1 over a fixed world
/// point and a second fixed world point to a point on body 2, so that `length1 + ratio ·
/// length2` stays within the allowed [`PulleyLength`]. A ratio of 2 makes body 2 move half as
/// far as body 1, a block and tackle.
///
/// The fixed points are world points whatever [`space`](Self::space) says;
/// [`PhysicsWorld::rebase`](crate::PhysicsWorld::rebase) moves them with the world. Both bodies
/// must be dynamic ([`ConstraintError::NotDynamic`]); the fixed points anchor the rope.
///
/// The default is Jolt's: every point at the origin, world space, ratio 1, a rope up to the
/// length at creation.
#[derive(Clone, Debug, PartialEq)]
pub struct PulleyConstraintSettings {
    space: ConstraintSpace,
    body_point1: RVec3,
    fixed_point1: RVec3,
    body_point2: RVec3,
    fixed_point2: RVec3,
    ratio: f32,
    length: PulleyLength,
    pub(super) solver: SolverOverrides,
}

impl Default for PulleyConstraintSettings {
    fn default() -> Self {
        Self::new(RVec3::ZERO, RVec3::ZERO, RVec3::ZERO, RVec3::ZERO)
    }
}

impl PulleyConstraintSettings {
    /// A rope from `body_point1` over `fixed_point1` and `fixed_point2` to `body_point2`, all in
    /// world space, with ratio 1 and up to the length it has when the constraint is created.
    pub fn new(
        body_point1: RVec3,
        fixed_point1: RVec3,
        body_point2: RVec3,
        fixed_point2: RVec3,
    ) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            body_point1,
            fixed_point1,
            body_point2,
            fixed_point2,
            ratio: 1.0,
            length: PulleyLength::UpToCurrent,
            solver: SolverOverrides::default(),
        }
    }

    /// The space the body points are given in; the fixed points are always world points.
    /// Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The ratio between the two rope segments, positive and within
    /// `1 /`[`limits::MAX_RATIO`]`..=`[`limits::MAX_RATIO`]. Default 1.
    #[must_use]
    pub fn ratio(mut self, value: f32) -> Self {
        self.ratio = value;
        self
    }

    /// The allowed rope length. Default [`PulleyLength::UpToCurrent`].
    #[must_use]
    pub fn length(mut self, value: PulleyLength) -> Self {
        self.length = value;
        self
    }

    fn to_jph(&self) -> JPH_PulleyConstraintSettings {
        let (min, max) = match self.length {
            PulleyLength::UpToCurrent => (0.0, -1.0),
            PulleyLength::Range { min, max } => (min, max),
        };
        JPH_PulleyConstraintSettings {
            base: self.solver.to_jph(),
            space: self.space.to_jph(),
            bodyPoint1: self.body_point1.to_jph(),
            fixedPoint1: self.fixed_point1.to_jph(),
            bodyPoint2: self.body_point2.to_jph(),
            fixedPoint2: self.fixed_point2.to_jph(),
            ratio: self.ratio,
            minLength: min,
            maxLength: max,
        }
    }
}

impl sealed::Settings for PulleyConstraintSettings {
    const NEEDS_DYNAMIC_BODIES: bool = true;

    fn validate(&self) -> Result<(), &'static str> {
        self.solver.validate()?;
        for point in [
            self.body_point1,
            self.fixed_point1,
            self.body_point2,
            self.fixed_point2,
        ] {
            validate_point(point)?;
        }
        if !(self.ratio > 0.0 && limits::is_ratio(self.ratio)) {
            return Err(
                "pulley ratio must be positive and within 1 / limits::MAX_RATIO..=limits::MAX_RATIO",
            );
        }
        if let PulleyLength::Range { min, max } = self.length {
            validate_pulley_length(min, max, self.ratio)?;
        }
        Ok(())
    }

    fn springs(&self) -> Vec<SpringSettings> {
        Vec::new()
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(point_anchors(
            self.space,
            self.body_point1,
            self.body_point2,
        ))
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = self.to_jph();
        // SAFETY: as in `FixedConstraintSettings::create`.
        unsafe { JPH_PulleyConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for PulleyConstraintSettings {
    type Kind = PulleyConstraint;
}

/// The settings of a live pulley as Jolt reports them: body points relative to the centres of
/// mass, world fixed points and the lengths resolved at creation.
///
/// # Safety
/// `pulley` is a live pulley constraint that nothing writes during the call.
unsafe fn pulley_settings(pulley: *const JPH_PulleyConstraint) -> JPH_PulleyConstraintSettings {
    // SAFETY: an all-zero settings value is valid (floats, integers, `false` and enums with a
    // zero value).
    let mut settings: JPH_PulleyConstraintSettings = unsafe { std::mem::zeroed() };
    // SAFETY: the caller's contract; Jolt's `GetConstraintSettings` is const and allocates a new
    // settings object, which joltc copies into `settings` and releases.
    unsafe { JPH_PulleyConstraint_GetSettings(pulley, &mut settings) };
    settings
}

impl ConstraintRef<'_, PulleyConstraint> {
    /// The current rope length, `length1 + ratio · length2`, metres, as of the last step or
    /// creation.
    pub fn current_length(&self) -> f32 {
        // SAFETY: the world borrowed here owns the constraint; the getter is a const member read.
        unsafe { JPH_PulleyConstraint_GetCurrentLength(self.ptr()) }
    }

    /// The shortest allowed length, metres.
    pub fn min_length(&self) -> f32 {
        // SAFETY: as in `current_length`.
        unsafe { JPH_PulleyConstraint_GetMinLength(self.ptr()) }
    }

    /// The longest allowed length, metres; the length at creation for
    /// [`PulleyLength::UpToCurrent`].
    pub fn max_length(&self) -> f32 {
        // SAFETY: as in `current_length`.
        unsafe { JPH_PulleyConstraint_GetMaxLength(self.ptr()) }
    }

    /// The ratio between the two rope segments.
    pub fn ratio(&self) -> f32 {
        // SAFETY: as in `current_length`; `pulley_settings` only reads the constraint.
        unsafe { pulley_settings(self.ptr()) }.ratio
    }

    /// The impulse in N·s the rope applied in the last collision step on body 1 along its rope,
    /// negative while the rope pulls; body 2 gets `ratio` times it along its rope.
    pub fn total_lambda_position(&self) -> f32 {
        // SAFETY: as in `current_length`.
        unsafe { JPH_PulleyConstraint_GetTotalLambdaPosition(self.ptr()) }
    }
}

impl ConstraintMut<'_, PulleyConstraint> {
    /// Sets the allowed rope length in metres: `0 <= min <= max <= (1 + ratio) ·`
    /// [`limits::MAX_SHAPE_EXTENT`]. Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_length(&mut self, min: f32, max: f32) -> Result<(), ConstraintError> {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs, and `pulley_settings` only reads it.
        let ratio = unsafe { pulley_settings(self.ptr()) }.ratio;
        validate_pulley_length(min, max, ratio).map_err(ConstraintError::InvalidValue)?;
        // SAFETY: as above; Jolt asserts `0 <= min <= max`, checked above.
        unsafe { JPH_PulleyConstraint_SetLength(self.ptr(), min, max) };
        self.wake_bodies();
        Ok(())
    }
}

/// A pulley's settings in the new frame of a [`PhysicsWorld::rebase`], computed before the
/// rebase writes anything.
pub(crate) struct PulleyRebase {
    raw: u32,
    settings: JPH_PulleyConstraintSettings,
}

impl PhysicsWorld {
    /// Every pulley's settings with its fixed points moved by `point`, in id order; an error
    /// names a fixed point that would not be finite.
    pub(crate) fn rebased_pulleys(
        &self,
        point: impl Fn(RVec3) -> RVec3,
    ) -> Result<Vec<PulleyRebase>, &'static str> {
        let mut rebased = Vec::new();
        for (&raw, entry) in &self.constraints {
            if entry.kind != ConstraintType::Pulley {
                continue;
            }
            // SAFETY: the world borrowed here owns the pulley; nothing writes it meanwhile.
            let mut settings = unsafe { pulley_settings(entry.constraint.as_ptr().cast()) };
            let fixed1 = point(RVec3::from_jph(settings.fixedPoint1));
            let fixed2 = point(RVec3::from_jph(settings.fixedPoint2));
            if !(fixed1.is_finite() && fixed2.is_finite()) {
                return Err("rebase would give a pulley a non-finite fixed point");
            }
            settings.fixedPoint1 = fixed1.to_jph();
            settings.fixedPoint2 = fixed2.to_jph();
            rebased.push(PulleyRebase { raw, settings });
        }
        Ok(rebased)
    }

    /// Replaces each pulley by one created from what [`rebased_pulleys`](Self::rebased_pulleys)
    /// computed, in id order, after the bodies have moved. The id, enabled state, ratio, lengths,
    /// priority and step overrides stay; the warm start goes, and Jolt's constraint order changes.
    pub(crate) fn apply_pulley_rebase(&mut self, rebased: Vec<PulleyRebase>) {
        for pulley in rebased {
            let entry = self
                .constraints
                .get(&pulley.raw)
                .unwrap_or_else(|| unreachable!("computed from this world's pulleys"));
            let replacement = with_locked_bodies(self.body_lock_interface, entry.bodies, |a, b| {
                // SAFETY: both bodies are live bodies of this world (the pulley keeps them),
                // locked for writing for the call; the settings are Jolt's own, with finite
                // fixed points. The handle takes over the one reference joltc returns.
                unsafe {
                    Owned::from_raw(
                        JPH_PulleyConstraint_Create(&pulley.settings, a.as_ptr(), b.as_ptr())
                            .cast::<JPH_Constraint>(),
                    )
                }
            })
            .unwrap_or_else(|| unreachable!("a constraint's bodies stay in the world"))
            .unwrap_or_else(|| unreachable!("joltc `new`s the constraint"));
            // SAFETY: the system and both constraints are live, the system is borrowed mutably
            // and no step runs; the old pulley was added by `create_constraint` or an earlier
            // rebase. The system takes its own reference to the new one.
            unsafe {
                JPH_PhysicsSystem_RemoveConstraint(self.system.as_ptr(), entry.constraint.as_ptr());
                JPH_PhysicsSystem_AddConstraint(self.system.as_ptr(), replacement.as_ptr());
            }
            let entry = self
                .constraints
                .get_mut(&pulley.raw)
                .unwrap_or_else(|| unreachable!("looked up above"));
            // Dropping the old handle releases the world's reference to the old pulley.
            entry.constraint = replacement;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::ensure_initialized;

    #[test]
    fn pulley_defaults_are_jolts() {
        assert!(ensure_initialized());
        // SAFETY: an all-zero settings value is valid (floats, integers, `false` and enums with a
        // zero value); joltc fills it with Jolt's defaults and allocates nothing.
        let mut jolt: JPH_PulleyConstraintSettings = unsafe { std::mem::zeroed() };
        // SAFETY: Jolt is initialised and `jolt` is a live local.
        unsafe { JPH_PulleyConstraintSettings_Init(&mut jolt) };
        let ours = PulleyConstraintSettings::default().to_jph();
        assert_eq!(ours.base.enabled, jolt.base.enabled);
        assert_eq!(ours.space, jolt.space);
        for (a, b) in [
            (ours.bodyPoint1, jolt.bodyPoint1),
            (ours.fixedPoint1, jolt.fixedPoint1),
            (ours.bodyPoint2, jolt.bodyPoint2),
            (ours.fixedPoint2, jolt.fixedPoint2),
        ] {
            assert_eq!(RVec3::from_jph(a), RVec3::from_jph(b));
        }
        assert_eq!(ours.ratio.to_bits(), jolt.ratio.to_bits());
        assert_eq!(ours.minLength.to_bits(), jolt.minLength.to_bits());
        assert_eq!(ours.maxLength.to_bits(), jolt.maxLength.to_bits());
    }
}
