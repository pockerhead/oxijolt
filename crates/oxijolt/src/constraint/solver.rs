//! How the solver treats one constraint: its priority and its velocity and position step
//! overrides (Jolt `ConstraintSettings::mConstraintPriority`, `mNumVelocityStepsOverride`,
//! `mNumPositionStepsOverride`), for the settings of every kind and on live constraints.

use oxijolt_sys::*;

use super::constraint_base;
use super::world::ConstraintKind;
use crate::limits;
use crate::{ConstraintError, ConstraintMut, ConstraintRef};

/// What a velocity or position step override must satisfy.
const STEPS_OVERRIDE_RULE: &str =
    "step overrides must be at most limits::MAX_SOLVER_STEPS_OVERRIDE";

/// The solver settings every constraint kind shares. The default is Jolt's: priority 0 and no
/// override (0), so the world's step counts apply.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct SolverOverrides {
    priority: u32,
    velocity_steps: u32,
    position_steps: u32,
}

impl SolverOverrides {
    /// Jolt asserts both overrides below 256 and keeps them in 8 bits (`Constraint.h:118-124`).
    pub(super) fn validate(&self) -> Result<(), &'static str> {
        if is_steps_override(self.velocity_steps) && is_steps_override(self.position_steps) {
            Ok(())
        } else {
            Err(STEPS_OVERRIDE_RULE)
        }
    }

    /// Jolt's `ConstraintSettings` with these values and Jolt's defaults for the rest.
    pub(super) fn to_jph(self) -> JPH_ConstraintSettings {
        JPH_ConstraintSettings {
            constraintPriority: self.priority,
            numVelocityStepsOverride: self.velocity_steps,
            numPositionStepsOverride: self.position_steps,
            ..constraint_base()
        }
    }
}

fn is_steps_override(value: u32) -> bool {
    value <= limits::MAX_SOLVER_STEPS_OVERRIDE
}

/// `Err(InvalidValue)` unless `value` is a step override Jolt accepts.
fn check_steps_override(value: u32) -> Result<(), ConstraintError> {
    if is_steps_override(value) {
        Ok(())
    } else {
        Err(ConstraintError::InvalidValue(STEPS_OVERRIDE_RULE))
    }
}

/// The builder methods of [`SolverOverrides`], the same on the settings of every kind.
macro_rules! solver_override_builders {
    ($($settings:ty),* $(,)?) => {
        $(
            impl $settings {
                /// The order in which the solver handles this constraint among the others of its
                /// island: Jolt sorts them by priority, lowest first, so a higher priority is
                /// solved later in each iteration and is more likely to end satisfied. Islands of
                /// 128 or more constraints and contacts keep this order only among constraints
                /// solved in the same parallel group. Default 0.
                /// See [docs/constraints.md#solver-overrides].
                ///
                /// [docs/constraints.md#solver-overrides]: https://github.com/pockerhead/oxijolt/blob/main/docs/constraints.md#solver-overrides
                #[must_use]
                pub fn constraint_priority(mut self, value: u32) -> Self {
                    self.solver.priority = value;
                    self
                }

                /// Velocity iterations for the island of this constraint while the constraint is
                /// active, at most [`limits::MAX_SOLVER_STEPS_OVERRIDE`]. Default 0, which asks
                /// for the world's
                /// [`WorldSettings::velocity_steps`](crate::WorldSettings::velocity_steps).
                ///
                /// Jolt solves the whole island, its contacts included, with the largest count
                /// that its active constraints and the dynamic bodies of its contacts ask for,
                /// and bodies ask for the world's count. A count above the world's makes the whole
                /// island pay for it; a count below it applies only to an island without contacts
                /// whose active constraints all ask for fewer. See
                /// [docs/constraints.md#solver-overrides].
                ///
                /// [docs/constraints.md#solver-overrides]: https://github.com/pockerhead/oxijolt/blob/main/docs/constraints.md#solver-overrides
                #[must_use]
                pub fn velocity_steps_override(mut self, value: u32) -> Self {
                    self.solver.velocity_steps = value;
                    self
                }

                /// Position iterations the island of this constraint needs while the constraint
                /// is active, at most [`limits::MAX_SOLVER_STEPS_OVERRIDE`]. Default 0, the
                /// world's [`WorldSettings::position_steps`](crate::WorldSettings::position_steps).
                /// Combined over the island like
                /// [`velocity_steps_override`](Self::velocity_steps_override).
                #[must_use]
                pub fn position_steps_override(mut self, value: u32) -> Self {
                    self.solver.position_steps = value;
                    self
                }
            }
        )*
    };
}

solver_override_builders!(
    crate::FixedConstraintSettings,
    crate::PointConstraintSettings,
    crate::DistanceConstraintSettings,
    crate::HingeConstraintSettings,
    crate::SliderConstraintSettings,
    crate::ConeConstraintSettings,
    crate::SwingTwistConstraintSettings,
    crate::SixDofConstraintSettings,
    crate::GearConstraintSettings,
    crate::RackAndPinionConstraintSettings,
    crate::PulleyConstraintSettings,
    crate::PathConstraintSettings,
);

impl<K: ConstraintKind> ConstraintRef<'_, K> {
    /// The constraint's solver priority
    /// ([`constraint_priority`](crate::HingeConstraintSettings::constraint_priority)).
    pub fn constraint_priority(&self) -> u32 {
        // SAFETY: the world borrowed here owns the constraint; the getter reads a member, and
        // writes need `&mut PhysicsWorld`.
        unsafe { JPH_Constraint_GetConstraintPriority(self.ptr()) }
    }

    /// The constraint's velocity step override, 0 for none
    /// ([`velocity_steps_override`](crate::HingeConstraintSettings::velocity_steps_override)).
    pub fn velocity_steps_override(&self) -> u32 {
        // SAFETY: as in `constraint_priority`.
        unsafe { JPH_Constraint_GetNumVelocityStepsOverride(self.ptr()) }
    }

    /// The constraint's position step override, 0 for none
    /// ([`position_steps_override`](crate::HingeConstraintSettings::position_steps_override)).
    pub fn position_steps_override(&self) -> u32 {
        // SAFETY: as in `constraint_priority`.
        unsafe { JPH_Constraint_GetNumPositionStepsOverride(self.ptr()) }
    }
}

impl<K: ConstraintKind> ConstraintMut<'_, K> {
    /// The constraint's solver priority.
    pub fn constraint_priority(&self) -> u32 {
        // SAFETY: the world is borrowed through this view and owns the constraint; the getter
        // reads a member.
        unsafe { JPH_Constraint_GetConstraintPriority(self.ptr()) }
    }

    /// The constraint's velocity step override, 0 for none.
    pub fn velocity_steps_override(&self) -> u32 {
        // SAFETY: as in `constraint_priority`.
        unsafe { JPH_Constraint_GetNumVelocityStepsOverride(self.ptr()) }
    }

    /// The constraint's position step override, 0 for none.
    pub fn position_steps_override(&self) -> u32 {
        // SAFETY: as in `constraint_priority`.
        unsafe { JPH_Constraint_GetNumPositionStepsOverride(self.ptr()) }
    }

    /// Sets the solver priority
    /// ([`constraint_priority`](crate::HingeConstraintSettings::constraint_priority)) and wakes
    /// the constraint's bodies. Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_constraint_priority(&mut self, priority: u32) {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. The setter writes a member.
        unsafe { JPH_Constraint_SetConstraintPriority(self.ptr(), priority) };
        self.wake_bodies();
    }

    /// Sets the velocity step override, at most [`limits::MAX_SOLVER_STEPS_OVERRIDE`] and 0 for
    /// none ([`velocity_steps_override`](crate::HingeConstraintSettings::velocity_steps_override)),
    /// and wakes the constraint's bodies. Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_velocity_steps_override(&mut self, steps: u32) -> Result<(), ConstraintError> {
        check_steps_override(steps)?;
        // SAFETY: as in `set_constraint_priority`; Jolt asserts `steps < 256`, checked above.
        unsafe { JPH_Constraint_SetNumVelocityStepsOverride(self.ptr(), steps) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the position step override, at most [`limits::MAX_SOLVER_STEPS_OVERRIDE`] and 0 for
    /// none ([`position_steps_override`](crate::HingeConstraintSettings::position_steps_override)),
    /// and wakes the constraint's bodies. Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_position_steps_override(&mut self, steps: u32) -> Result<(), ConstraintError> {
        check_steps_override(steps)?;
        // SAFETY: as in `set_velocity_steps_override`.
        unsafe { JPH_Constraint_SetNumPositionStepsOverride(self.ptr(), steps) };
        self.wake_bodies();
        Ok(())
    }
}
