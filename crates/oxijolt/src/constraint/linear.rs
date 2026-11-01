//! World constraints that hold points together or apart: fixed, point and distance.

use std::ptr::NonNull;

use oxijolt_sys::*;

use super::lever::check_spring;
use super::world::{sealed, ConstraintSettings};
use super::{
    point_anchors, solver::SolverOverrides, validate_frame, validate_point, within,
    ConstraintSpace, SpringSettings,
};
use crate::limits;
use crate::{
    ConstraintError, ConstraintMut, ConstraintRef, DistanceConstraint, FixedConstraint,
    PointConstraint, RVec3, Vec3,
};

/// A fixed constraint (Jolt `FixedConstraintSettings`): body 2 keeps its position and rotation
/// relative to body 1, as if the two were welded.
///
/// The default is Jolt's: both frames at the origin with axes +X and +Y, world space, no
/// automatic point.
#[derive(Clone, Debug, PartialEq)]
pub struct FixedConstraintSettings {
    space: ConstraintSpace,
    auto_detect_point: bool,
    point1: RVec3,
    axis_x1: Vec3,
    axis_y1: Vec3,
    point2: RVec3,
    axis_x2: Vec3,
    axis_y2: Vec3,
    pub(super) solver: SolverOverrides,
}

impl Default for FixedConstraintSettings {
    fn default() -> Self {
        Self::new(
            RVec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
    }
}

impl FixedConstraintSettings {
    /// A weld at `point` with the same frame on both bodies, in world space: `axis_x` and the
    /// perpendicular `axis_y` are unit vectors.
    pub fn new(point: RVec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            auto_detect_point: false,
            point1: point,
            axis_x1: axis_x,
            axis_y1: axis_y,
            point2: point,
            axis_x2: axis_x,
            axis_y2: axis_y,
            solver: SolverOverrides::default(),
        }
    }

    /// The space the frames are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The frame on body 1: a point and perpendicular unit X and Y axes.
    #[must_use]
    pub fn frame1(mut self, point: RVec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        self.point1 = point;
        self.axis_x1 = axis_x;
        self.axis_y1 = axis_y;
        self
    }

    /// The frame on body 2: a point and perpendicular unit X and Y axes.
    #[must_use]
    pub fn frame2(mut self, point: RVec3, axis_x: Vec3, axis_y: Vec3) -> Self {
        self.point2 = point;
        self.axis_x2 = axis_x;
        self.axis_y2 = axis_y;
        self
    }

    /// Whether Jolt places the weld point between the bodies as they are when the constraint is
    /// created, instead of the given points. Only in [`ConstraintSpace::WorldSpace`]. Default
    /// false.
    #[must_use]
    pub fn auto_detect_point(mut self, value: bool) -> Self {
        self.auto_detect_point = value;
        self
    }

    fn to_jph(&self) -> JPH_FixedConstraintSettings {
        JPH_FixedConstraintSettings {
            base: self.solver.to_jph(),
            space: self.space.to_jph(),
            autoDetectPoint: self.auto_detect_point,
            point1: self.point1.to_jph(),
            axisX1: self.axis_x1.to_jph(),
            axisY1: self.axis_y1.to_jph(),
            point2: self.point2.to_jph(),
            axisX2: self.axis_x2.to_jph(),
            axisY2: self.axis_y2.to_jph(),
        }
    }
}

impl sealed::Settings for FixedConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        self.solver.validate()?;
        validate_frame(self.point1, self.axis_x1, self.axis_y1)?;
        validate_frame(self.point2, self.axis_x2, self.axis_y2)?;
        // Jolt reads the bodies' world positions for the point, which only means something in
        // world space.
        if self.auto_detect_point && self.space != ConstraintSpace::WorldSpace {
            return Err("an automatic point needs ConstraintSpace::WorldSpace");
        }
        Ok(())
    }

    fn springs(&self) -> Vec<SpringSettings> {
        Vec::new()
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(if self.auto_detect_point {
            [sealed::Anchor::BetweenCentersOfMass; 2]
        } else {
            point_anchors(self.space, self.point1, self.point2)
        })
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = self.to_jph();
        // SAFETY: the caller locks both live bodies (trait contract); `settings` is a live,
        // validated local that joltc converts.
        unsafe { JPH_FixedConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for FixedConstraintSettings {
    type Kind = FixedConstraint;
}

impl ConstraintRef<'_, FixedConstraint> {
    /// The impulse in N·s the position part applied in the last collision step.
    pub fn total_lambda_position(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: the world borrowed here owns the constraint; the getter reads a member, and
        // `value` is a live local.
        unsafe { JPH_FixedConstraint_GetTotalLambdaPosition(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The angular impulse in N·m·s the rotation part applied in the last collision step.
    pub fn total_lambda_rotation(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `total_lambda_position`.
        unsafe { JPH_FixedConstraint_GetTotalLambdaRotation(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }
}

/// A point constraint (Jolt `PointConstraintSettings`): a ball joint that keeps a point of
/// body 1 on a point of body 2 and leaves the rotation free.
///
/// The default is Jolt's: both points at the origin, world space.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PointConstraintSettings {
    space: ConstraintSpace,
    point1: RVec3,
    point2: RVec3,
    pub(super) solver: SolverOverrides,
}

impl PointConstraintSettings {
    /// A ball joint at `point` on both bodies, in world space.
    pub fn new(point: RVec3) -> Self {
        Self {
            space: ConstraintSpace::WorldSpace,
            point1: point,
            point2: point,
            solver: SolverOverrides::default(),
        }
    }

    /// The space the points are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The point on body 1.
    #[must_use]
    pub fn point1(mut self, value: RVec3) -> Self {
        self.point1 = value;
        self
    }

    /// The point on body 2.
    #[must_use]
    pub fn point2(mut self, value: RVec3) -> Self {
        self.point2 = value;
        self
    }
}

impl sealed::Settings for PointConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        self.solver.validate()?;
        validate_point(self.point1)?;
        validate_point(self.point2)
    }

    fn springs(&self) -> Vec<SpringSettings> {
        Vec::new()
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(point_anchors(self.space, self.point1, self.point2))
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let settings = JPH_PointConstraintSettings {
            base: self.solver.to_jph(),
            space: self.space.to_jph(),
            point1: self.point1.to_jph(),
            point2: self.point2.to_jph(),
        };
        // SAFETY: as in `FixedConstraintSettings::create`.
        unsafe { JPH_PointConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for PointConstraintSettings {
    type Kind = PointConstraint;
}

impl ConstraintRef<'_, PointConstraint> {
    /// The impulse in N·s the constraint applied in the last collision step.
    pub fn total_lambda_position(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: the world borrowed here owns the constraint; the getter reads a member, and
        // `value` is a live local.
        unsafe { JPH_PointConstraint_GetTotalLambdaPosition(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }
}

/// The allowed distance range of a [`DistanceConstraintSettings`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum DistanceRange {
    /// Exactly the distance between the two points when the constraint is created, a rigid rod.
    /// The default (Jolt's `-1, -1`).
    #[default]
    Current,
    /// From `min` to `max` metres: `0 <= min <= max <=` [`limits::MAX_SHAPE_EXTENT`]. `min == 0`
    /// with a larger `max` is a rope.
    Range {
        /// Shortest distance, metres.
        min: f32,
        /// Longest distance, metres.
        max: f32,
    },
}

impl DistanceRange {
    fn limits(self) -> (f32, f32) {
        match self {
            Self::Current => (-1.0, -1.0),
            Self::Range { min, max } => (min, max),
        }
    }
}

/// What a distance range must satisfy.
fn validate_distance_range(min: f32, max: f32) -> Result<(), &'static str> {
    if within(min, 0.0, limits::MAX_SHAPE_EXTENT)
        && within(max, 0.0, limits::MAX_SHAPE_EXTENT)
        && min <= max
    {
        Ok(())
    } else {
        Err("distances must be 0 <= min <= max <= limits::MAX_SHAPE_EXTENT")
    }
}

/// A distance constraint (Jolt `DistanceConstraintSettings`): keeps the distance between a point
/// of body 1 and a point of body 2 within a range; the bodies rotate freely.
///
/// The default is Jolt's: both points at the origin, world space, the distance at creation, a
/// rigid limit.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DistanceConstraintSettings {
    space: ConstraintSpace,
    point1: RVec3,
    point2: RVec3,
    range: DistanceRange,
    limits_spring: SpringSettings,
    pub(super) solver: SolverOverrides,
}

impl DistanceConstraintSettings {
    /// A rod from `point1` on body 1 to `point2` on body 2, in world space, of the length it has
    /// when the constraint is created.
    pub fn new(point1: RVec3, point2: RVec3) -> Self {
        Self {
            point1,
            point2,
            ..Self::default()
        }
    }

    /// The space the points are given in. Default [`ConstraintSpace::WorldSpace`].
    #[must_use]
    pub fn space(mut self, value: ConstraintSpace) -> Self {
        self.space = value;
        self
    }

    /// The allowed distance. Default [`DistanceRange::Current`].
    #[must_use]
    pub fn range(mut self, value: DistanceRange) -> Self {
        self.range = value;
        self
    }

    /// Makes the range limits soft: a spring pulls the points back into the range. Default
    /// rigid.
    #[must_use]
    pub fn limits_spring(mut self, value: SpringSettings) -> Self {
        self.limits_spring = value;
        self
    }
}

impl sealed::Settings for DistanceConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        self.solver.validate()?;
        validate_point(self.point1)?;
        validate_point(self.point2)?;
        if let DistanceRange::Range { min, max } = self.range {
            validate_distance_range(min, max)?;
        }
        self.limits_spring.validate()
    }

    fn springs(&self) -> Vec<SpringSettings> {
        vec![self.limits_spring]
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        Some(point_anchors(self.space, self.point1, self.point2))
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let (min, max) = self.range.limits();
        let settings = JPH_DistanceConstraintSettings {
            base: self.solver.to_jph(),
            space: self.space.to_jph(),
            point1: self.point1.to_jph(),
            point2: self.point2.to_jph(),
            minDistance: min,
            maxDistance: max,
            limitsSpringSettings: self.limits_spring.to_jph(),
        };
        // SAFETY: as in `FixedConstraintSettings::create`.
        unsafe { JPH_DistanceConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for DistanceConstraintSettings {
    type Kind = DistanceConstraint;
}

impl ConstraintRef<'_, DistanceConstraint> {
    /// The shortest allowed distance, metres; the distance at creation for
    /// [`DistanceRange::Current`].
    pub fn min_distance(&self) -> f32 {
        // SAFETY: the world borrowed here owns the constraint; Jolt's `GetMinDistance` is a const
        // member read, joltc only declares the handle mutable.
        unsafe { JPH_DistanceConstraint_GetMinDistance(self.ptr()) }
    }

    /// The longest allowed distance, metres.
    pub fn max_distance(&self) -> f32 {
        // SAFETY: as in `min_distance`.
        unsafe { JPH_DistanceConstraint_GetMaxDistance(self.ptr()) }
    }

    /// The impulse in N·s the constraint applied along the line between the points in the last
    /// collision step.
    pub fn total_lambda_position(&self) -> f32 {
        // SAFETY: as in `min_distance`.
        unsafe { JPH_DistanceConstraint_GetTotalLambdaPosition(self.ptr()) }
    }
}

impl ConstraintMut<'_, DistanceConstraint> {
    /// Sets the allowed distance range in metres: `0 <= min <= max <=`
    /// [`limits::MAX_SHAPE_EXTENT`]. Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_distance(&mut self, min: f32, max: f32) -> Result<(), ConstraintError> {
        validate_distance_range(min, max).map_err(ConstraintError::InvalidValue)?;
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. Jolt asserts `min <= max`, checked above.
        unsafe { JPH_DistanceConstraint_SetDistance(self.ptr(), min, max) };
        self.wake_bodies();
        Ok(())
    }

    /// Replaces the spring of the range limits, bounded through the bodies' effective mass as
    /// at creation. Wakes the constraint's bodies.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_limits_spring(&mut self, spring: SpringSettings) -> Result<(), ConstraintError> {
        check_spring(spring, self.effective_mass_bound())?;
        let mut spring = spring.to_jph();
        // SAFETY: as in `set_distance`; `spring` is a live local that joltc copies.
        unsafe { JPH_DistanceConstraint_SetLimitsSpringSettings(self.ptr(), &mut spring) };
        self.wake_bodies();
        Ok(())
    }
}
