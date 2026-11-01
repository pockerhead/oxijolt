//! The path constraint: body 2 moves along a Hermite curve fixed to body 1.

use std::ptr::NonNull;

use oxijolt_sys::*;

use super::rotational::{check_friction, check_motor};
use super::world::{sealed, ConstraintSettings};
use super::{
    non_negative, solver::SolverOverrides, within, MotorSettings, SpringSettings, FRICTION_RULE,
    TARGET_VELOCITY_RULE,
};
use crate::limits;
use crate::math::is_unit;
use crate::owned::{JoltObject, Owned};
use crate::{
    ConstraintError, ConstraintMut, ConstraintRef, MotorState, PathConstraint, Quat, Vec3,
};

/// A Hermite path, of which the owner holds the one reference
/// `JPH_PathConstraintPathHermite_Create` returns; the path constraint settings and every path
/// constraint created from them take their own (`RefConst`).
impl JoltObject for JPH_PathConstraintPath {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), which `Release` drops.
        unsafe { JPH_PathConstraintPath_Destroy(ptr) };
    }
}

/// One point of a [`HermitePath`]: a position and the curve's tangent there, in path space.
/// The tangent's length is the speed of the curve parameter, not only its direction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HermitePathPoint {
    /// Position, metres, within [`limits::MAX_SHAPE_EXTENT`] per axis.
    pub position: Vec3,
    /// Tangent, metres per unit of path fraction, within [`limits::MAX_SHAPE_EXTENT`] per axis.
    pub tangent: Vec3,
}

/// The smallest chord between two consecutive path points, metres.
const MIN_CHORD: f64 = 1.0e-3;

/// What every segment of a path must satisfy: a chord of at least [`MIN_CHORD`], and tangents
/// that keep the curve moving along the chord and away from the normal.
const SEGMENT_RULE: &str =
    "each path segment needs a chord of at least 1 mm and tangents that follow it";

/// A planar Hermite path (Jolt `PathConstraintPathHermite`) for a [`PathConstraintSettings`]:
/// cubic Hermite segments through the points, one plane normal for all of them. Fraction `i`
/// is point `i`; a looping path also joins the last point back to the first.
///
/// The supported subset is planar paths whose tangent never turns towards the normal: the
/// constraint orients body 2 by the curve's tangent and the normal, and Jolt asserts that the
/// two are never parallel (`PathConstraintPathHermite::GetPointOnPath`). [`new`](Self::new)
/// checks every segment in closed form, without sampling: with the chord `c` from one point to
/// the next and `ĉ = c / |c|`, the curve's derivative along the chord is
/// `f(t) = (6t − 6t²)|c| + (3t² − 4t + 1)(M1·ĉ) + (3t² − 2t)(M2·ĉ)`, a quadratic whose minimum
/// over `[0, 1]` lies at an end or its vertex, and that minimum must be at least twice an upper
/// bound of the derivative along the normal, `1.5|c·n| + |M1·n| + |M2·n|`, and at least a
/// margin for `f32` rounding. The tangent then stays at least 60° away from the normal.
#[derive(Clone, Debug, PartialEq)]
pub struct HermitePath {
    normal: Vec3,
    points: Vec<HermitePathPoint>,
    looping: bool,
}

impl HermitePath {
    /// Most points a path may have. Jolt indexes points with `int` and measures the position
    /// along the path as an `f32` fraction, which with 4096 points still resolves about 5e-4 of
    /// a segment.
    pub const MAX_POINTS: usize = 4096;

    /// A path through `points` (at least 2, at most [`MAX_POINTS`](Self::MAX_POINTS)) in the
    /// plane of the unit `normal`; a `looping` path also joins the last point back to the first.
    /// Every position and tangent must be within [`limits::MAX_SHAPE_EXTENT`] per axis, and
    /// every segment must pass the check in the type's documentation; otherwise
    /// [`ConstraintError::InvalidValue`].
    pub fn new(
        normal: Vec3,
        points: Vec<HermitePathPoint>,
        looping: bool,
    ) -> Result<Self, ConstraintError> {
        let invalid = |what| Err(ConstraintError::InvalidValue(what));
        if !(2..=Self::MAX_POINTS).contains(&points.len()) {
            return invalid("a path needs between 2 and HermitePath::MAX_POINTS points");
        }
        if !is_unit(normal) {
            return invalid("the path normal must be a unit vector");
        }
        for point in &points {
            if !(limits::is_local_offset(point.position) && limits::is_local_offset(point.tangent))
            {
                return invalid(
                    "path positions and tangents must be within limits::MAX_SHAPE_EXTENT per axis",
                );
            }
        }
        let segments = if looping {
            points.len()
        } else {
            points.len() - 1
        };
        for i in 0..segments {
            let next = (i + 1) % points.len();
            if !segment_is_valid(normal, points[i], points[next]) {
                return invalid(SEGMENT_RULE);
            }
        }
        Ok(Self {
            normal,
            points,
            looping,
        })
    }

    /// The largest path fraction: the number of segments.
    pub fn max_fraction(&self) -> f32 {
        if self.looping {
            self.points.len() as f32
        } else {
            (self.points.len() - 1) as f32
        }
    }

    /// The plane normal.
    pub fn normal(&self) -> Vec3 {
        self.normal
    }

    /// The points.
    pub fn points(&self) -> &[HermitePathPoint] {
        &self.points
    }

    /// Whether the last point joins back to the first.
    pub fn is_looping(&self) -> bool {
        self.looping
    }

    /// The point at `fraction` (within `0..=max_fraction`) in path space, as Jolt's
    /// `PathConstraintPathHermite::GetPointOnPath` computes it.
    fn point_at(&self, fraction: f32) -> Vec3 {
        let count = self.points.len();
        // `fraction` is at least 0 and at most the point count, so it fits a `usize`.
        let mut index = fraction.trunc() as usize;
        let mut t = fraction - index as f32;
        if self.looping {
            index %= count;
        } else if index >= count - 1 {
            index = count - 2;
            t = 1.0;
        }
        let (p1, p2) = (self.points[index], self.points[(index + 1) % count]);
        let (t2, t3) = (t * t, t * t * t);
        let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        let h10 = t3 - 2.0 * t2 + t;
        let h01 = -2.0 * t3 + 3.0 * t2;
        let h11 = t3 - t2;
        let blend = |a: f32, b: f32, c: f32, d: f32| h00 * a + h10 * b + h01 * c + h11 * d;
        Vec3::new(
            blend(p1.position.x, p1.tangent.x, p2.position.x, p2.tangent.x),
            blend(p1.position.y, p1.tangent.y, p2.position.y, p2.tangent.y),
            blend(p1.position.z, p1.tangent.z, p2.position.z, p2.tangent.z),
        )
    }

    /// How far from the path's start any point of the path lies at most, metres: each segment
    /// stays within the hull of its Bézier control points `p0`, `p0 + t0 / 3`, `p1 - t1 / 3` and
    /// `p1`.
    fn reach(&self) -> f64 {
        self.points
            .iter()
            .map(|point| {
                limits::f64_length(point.position) + limits::f64_length(point.tangent) / 3.0
            })
            .fold(0.0, f64::max)
    }

    /// A new native path with these points, holding one reference.
    fn create(&self) -> Owned<JPH_PathConstraintPath> {
        // SAFETY: Jolt is initialised (a world exists to create the constraint in). The handle
        // takes over the one reference joltc returns.
        let path = unsafe { Owned::from_raw(JPH_PathConstraintPathHermite_Create()) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the path"));
        let normal = self.normal.to_jph();
        for point in &self.points {
            let position = point.position.to_jph();
            let tangent = point.tangent.to_jph();
            // SAFETY: `path` is a live Hermite path from `JPH_PathConstraintPathHermite_Create`,
            // owned here and used by no constraint yet; the vectors are live locals.
            unsafe {
                JPH_PathConstraintPathHermite_AddPoint(path.as_ptr(), &position, &tangent, &normal)
            };
        }
        // SAFETY: as above.
        unsafe { JPH_PathConstraintPath_SetIsLooping(path.as_ptr(), self.looping) };
        path
    }
}

fn to_f64(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z].map(f64::from)
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn length(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// Whether the Hermite segment from `p1` to `p2` keeps its derivative moving along the chord
/// and away from `normal`, with margins (see [`HermitePath`]).
fn segment_is_valid(normal: Vec3, p1: HermitePathPoint, p2: HermitePathPoint) -> bool {
    let n = to_f64(normal);
    let (a1, a2) = (to_f64(p1.position), to_f64(p2.position));
    let (m1, m2) = (to_f64(p1.tangent), to_f64(p2.tangent));
    let chord = [a2[0] - a1[0], a2[1] - a1[1], a2[2] - a1[2]];
    let chord_length = length(chord);
    if chord_length < MIN_CHORD {
        return false;
    }
    let unit_chord = chord.map(|c| c / chord_length);
    let (a, b) = (dot(m1, unit_chord), dot(m2, unit_chord));
    // f(t) = A t² + B t + C, with f(0) = a and f(1) = b.
    let quadratic = -6.0 * chord_length + 3.0 * a + 3.0 * b;
    let linear = 6.0 * chord_length - 4.0 * a - 2.0 * b;
    let mut minimum = a.min(b);
    if quadratic != 0.0 {
        let vertex = -linear / (2.0 * quadratic);
        if vertex > 0.0 && vertex < 1.0 {
            let at_vertex = (quadratic * vertex + linear) * vertex + a;
            minimum = minimum.min(at_vertex);
        }
    }
    // The derivative along the normal is at most 1.5|c·n| + |M1·n| + |M2·n| on [0, 1].
    let off_plane = 1.5 * dot(chord, n).abs() + dot(m1, n).abs() + dot(m2, n).abs();
    let scale = length(a1) + length(a2) + length(m1) + length(m2);
    let required = (1.0e-3 * chord_length)
        .max(1.0e-5 * scale)
        .max(2.0 * off_plane);
    minimum >= required
}

/// How a path constraint constrains the rotation of body 2 (Jolt `EPathRotationConstraintType`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum PathRotationConstraint {
    /// The rotation is free. The default.
    #[default]
    Free,
    /// Body 2 may only turn about the path's tangent.
    ConstrainAroundTangent,
    /// Body 2 may only turn about the path's normal.
    ConstrainAroundNormal,
    /// Body 2 may only turn about the path's binormal.
    ConstrainAroundBinormal,
    /// Body 2 follows the path's tangent and normal.
    ConstrainToPath,
    /// Body 2 keeps its rotation relative to body 1.
    FullyConstrained,
}

impl PathRotationConstraint {
    fn to_jph(self) -> JPH_PathRotationConstraintType {
        match self {
            Self::Free => JPH_PathRotationConstraintType_Free,
            Self::ConstrainAroundTangent => JPH_PathRotationConstraintType_ConstrainAroundTangent,
            Self::ConstrainAroundNormal => JPH_PathRotationConstraintType_ConstrainAroundNormal,
            Self::ConstrainAroundBinormal => JPH_PathRotationConstraintType_ConstrainAroundBinormal,
            Self::ConstrainToPath => JPH_PathRotationConstraintType_ConstrainToPath,
            Self::FullyConstrained => JPH_PathRotationConstraintType_FullyConstrained,
        }
    }
}

/// A path constraint (Jolt `PathConstraintSettings`): body 2 slides along a [`HermitePath`]
/// fixed to body 1, with friction, a position motor along the path and a choice of rotation
/// constraint.
///
/// The path's frame is placed by [`path_position`](Self::path_position) and
/// [`path_rotation`](Self::path_rotation) relative to body 1's origin and rotation (Jolt stores
/// it relative to body 1's centre of mass). Body 2 is attached where it is when the constraint
/// is created, at [`path_fraction`](Self::path_fraction) along the path.
///
/// Each creation builds its own native path, so one settings value serves any number of
/// constraints and worlds.
#[derive(Clone, Debug, PartialEq)]
pub struct PathConstraintSettings {
    path: HermitePath,
    path_position: Vec3,
    path_rotation: Quat,
    path_fraction: f32,
    max_friction_force: f32,
    rotation_constraint: PathRotationConstraint,
    position_motor: MotorSettings,
    pub(super) solver: SolverOverrides,
}

impl PathConstraintSettings {
    /// A constraint along `path`, placed at body 1's origin with its rotation, body 2 at
    /// fraction 0, no friction, free rotation and Jolt's default motor.
    pub fn new(path: HermitePath) -> Self {
        Self {
            path,
            path_position: Vec3::ZERO,
            path_rotation: Quat::IDENTITY,
            path_fraction: 0.0,
            max_friction_force: 0.0,
            rotation_constraint: PathRotationConstraint::Free,
            position_motor: MotorSettings::default(),
            solver: SolverOverrides::default(),
        }
    }

    /// Where the path starts relative to body 1's origin, in body 1's frame, within
    /// [`limits::MAX_SHAPE_EXTENT`] per axis. Default zero.
    #[must_use]
    pub fn path_position(mut self, value: Vec3) -> Self {
        self.path_position = value;
        self
    }

    /// The path's rotation relative to body 1, a unit quaternion. Default identity.
    #[must_use]
    pub fn path_rotation(mut self, value: Quat) -> Self {
        self.path_rotation = value;
        self
    }

    /// The fraction along the path where body 2 is attached, in `[0, max_fraction]`. Default 0.
    #[must_use]
    pub fn path_fraction(mut self, value: f32) -> Self {
        self.path_fraction = value;
        self
    }

    /// Force in N that friction applies along the path while the motor is off, at least 0.
    /// Default 0.
    #[must_use]
    pub fn max_friction_force(mut self, value: f32) -> Self {
        self.max_friction_force = value;
        self
    }

    /// How the rotation of body 2 is constrained. Default [`PathRotationConstraint::Free`].
    #[must_use]
    pub fn rotation_constraint(mut self, value: PathRotationConstraint) -> Self {
        self.rotation_constraint = value;
        self
    }

    /// The motor that drives body 2 along the path.
    #[must_use]
    pub fn position_motor(mut self, value: MotorSettings) -> Self {
        self.position_motor = value;
        self
    }
}

impl PathConstraintSettings {
    fn to_jph(&self, path: *const JPH_PathConstraintPath) -> JPH_PathConstraintSettings {
        JPH_PathConstraintSettings {
            base: self.solver.to_jph(),
            path,
            pathPosition: self.path_position.to_jph(),
            pathRotation: self.path_rotation.to_jph(),
            pathFraction: self.path_fraction,
            maxFrictionForce: self.max_friction_force,
            rotationConstraintType: self.rotation_constraint.to_jph(),
            positionMotorSettings: self.position_motor.to_jph(),
        }
    }
}

impl sealed::Settings for PathConstraintSettings {
    fn validate(&self) -> Result<(), &'static str> {
        self.solver.validate()?;
        if !limits::is_local_offset(self.path_position) {
            return Err("path position must be within limits::MAX_SHAPE_EXTENT per axis");
        }
        if !self.path_rotation.is_valid_rotation() {
            return Err("path rotation must be a finite unit quaternion");
        }
        if !within(self.path_fraction, 0.0, self.path.max_fraction()) {
            return Err("path fraction must be between 0 and the path's max fraction");
        }
        if !non_negative(self.max_friction_force) {
            return Err(FRICTION_RULE);
        }
        self.position_motor.validate()
    }

    fn springs(&self) -> Vec<SpringSettings> {
        vec![self.position_motor.spring]
    }

    fn anchors(&self) -> Option<[sealed::Anchor; 2]> {
        // Body 1 holds the path anywhere along it; body 2 holds the point where it is attached,
        // the path's point at the start fraction.
        let attached = self
            .path_rotation
            .rotate(self.path.point_at(self.path_fraction));
        Some([
            sealed::Anchor::OnBody1 {
                offset: self.path_position,
                radius: self.path.reach(),
            },
            sealed::Anchor::OnBody1 {
                offset: Vec3::new(
                    self.path_position.x + attached.x,
                    self.path_position.y + attached.y,
                    self.path_position.z + attached.z,
                ),
                radius: 0.0,
            },
        ])
    }

    unsafe fn create(
        &self,
        body1: NonNull<JPH_Body>,
        body2: NonNull<JPH_Body>,
    ) -> *mut JPH_Constraint {
        let path = self.path.create();
        let settings = self.to_jph(path.as_ptr());
        // SAFETY: the caller locks both live bodies (trait contract); `settings` is a live,
        // validated local and `path` a live path that the constraint takes its own reference
        // to. Dropping `path` afterwards releases ours.
        unsafe { JPH_PathConstraint_Create(&settings, body1.as_ptr(), body2.as_ptr()) }.cast()
    }
}

impl ConstraintSettings for PathConstraintSettings {
    type Kind = PathConstraint;
}

impl ConstraintRef<'_, PathConstraint> {
    /// The constraint's path, borrowed from the constraint.
    fn path(&self) -> *const JPH_PathConstraintPath {
        // SAFETY: the world borrowed here owns the constraint; the getter returns the member
        // path, which lives as long as the constraint.
        unsafe { JPH_PathConstraint_GetPath(self.ptr()) }
    }

    /// The largest fraction of the constraint's path.
    pub fn max_fraction(&self) -> f32 {
        // SAFETY: the path is live with the constraint; the getter is a const virtual call.
        unsafe { JPH_PathConstraintPath_GetPathMaxFraction(self.path()) }
    }

    /// Whether the constraint's path joins its last point back to the first.
    pub fn is_looping(&self) -> bool {
        // SAFETY: the path is live with the constraint; the getter reads a member.
        unsafe { JPH_PathConstraintPath_IsLooping(self.path()) }
    }

    /// Where body 2 is along the path, as of the last step or creation.
    pub fn path_fraction(&self) -> f32 {
        // SAFETY: the world borrowed here owns the constraint; the getter reads a member.
        unsafe { JPH_PathConstraint_GetPathFraction(self.ptr()) }
    }

    /// The fraction of the path point closest to `point`, given in path space (relative to the
    /// path's start and rotation) within [`limits::MAX_SHAPE_EXTENT`] per axis. `hint` is a
    /// finite fraction near the answer, which Jolt may use to search faster.
    pub fn closest_fraction(&self, point: Vec3, hint: f32) -> Result<f32, ConstraintError> {
        if !(limits::is_local_offset(point) && hint.is_finite()) {
            return Err(ConstraintError::InvalidValue(
                "the point must be within limits::MAX_SHAPE_EXTENT per axis and the hint finite",
            ));
        }
        let point = point.to_jph();
        // SAFETY: the path is live with the constraint; Jolt's `GetClosestPoint` is const and
        // reads the points only. `point` is a live local.
        Ok(unsafe { JPH_PathConstraintPath_GetClosestPoint(self.path(), &point, hint) })
    }

    /// The motor's state.
    pub fn motor_state(&self) -> MotorState {
        // SAFETY: as in `path_fraction`.
        MotorState::from_jph(unsafe { JPH_PathConstraint_GetPositionMotorState(self.ptr()) })
    }

    /// The fraction a position motor drives to.
    pub fn target_path_fraction(&self) -> f32 {
        // SAFETY: as in `path_fraction`.
        unsafe { JPH_PathConstraint_GetTargetPathFraction(self.ptr()) }
    }

    /// The velocity along the path a velocity motor drives to, m/s.
    pub fn target_velocity(&self) -> f32 {
        // SAFETY: as in `path_fraction`.
        unsafe { JPH_PathConstraint_GetTargetVelocity(self.ptr()) }
    }

    /// The friction force in N applied while the motor is off.
    pub fn max_friction_force(&self) -> f32 {
        // SAFETY: as in `path_fraction`.
        unsafe { JPH_PathConstraint_GetMaxFrictionForce(self.ptr()) }
    }

    /// The impulses in N·s that kept body 2 on the path in the last collision step, along the
    /// normal and the binormal.
    pub fn total_lambda_position(&self) -> [f32; 2] {
        let mut value = [0.0; 2];
        // SAFETY: as in `path_fraction`; `value` has room for the two values joltc writes.
        unsafe { JPH_PathConstraint_GetTotalLambdaPosition(self.ptr(), value.as_mut_ptr()) };
        value
    }

    /// The impulse in N·s the path's ends applied in the last collision step.
    pub fn total_lambda_position_limits(&self) -> f32 {
        // SAFETY: as in `path_fraction`.
        unsafe { JPH_PathConstraint_GetTotalLambdaPositionLimits(self.ptr()) }
    }

    /// The impulse in N·s the motor, or the friction while the motor is off, applied in the last
    /// collision step.
    pub fn total_lambda_motor(&self) -> f32 {
        // SAFETY: as in `path_fraction`.
        unsafe { JPH_PathConstraint_GetTotalLambdaMotor(self.ptr()) }
    }

    /// The motor's settings.
    pub fn position_motor_settings(&self) -> MotorSettings {
        // SAFETY: an all-zero `JPH_MotorSettings` is valid (floats and an enum with a zero
        // value).
        let mut motor: JPH_MotorSettings = unsafe { std::mem::zeroed() };
        // SAFETY: as in `path_fraction`; joltc copies the member into `motor`, a live local.
        unsafe { JPH_PathConstraint_GetPositionMotorSettings(self.ptr(), &mut motor) };
        MotorSettings::from_jph(motor)
    }

    /// The angular impulses in N·m·s that kept body 2 turning only about the free axis in the
    /// last collision step, for [`PathRotationConstraint::ConstrainAroundTangent`],
    /// `ConstrainAroundNormal` and `ConstrainAroundBinormal`; zero for the other rotation
    /// constraints.
    pub fn total_lambda_rotation_hinge(&self) -> [f32; 2] {
        let mut value = [0.0; 2];
        // SAFETY: as in `path_fraction`; `value` has room for the two values joltc writes.
        unsafe { JPH_PathConstraint_GetTotalLambdaRotationHinge(self.ptr(), value.as_mut_ptr()) };
        value
    }

    /// The angular impulse in N·m·s that held body 2's rotation in the last collision step, for
    /// [`PathRotationConstraint::ConstrainToPath`] and `FullyConstrained`; zero for the other
    /// rotation constraints.
    pub fn total_lambda_rotation(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `path_fraction`; `value` is a live local.
        unsafe { JPH_PathConstraint_GetTotalLambdaRotation(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }
}

impl ConstraintMut<'_, PathConstraint> {
    /// The largest fraction of the constraint's path.
    fn max_fraction(&self) -> f32 {
        // SAFETY: the world is borrowed through this view and owns the constraint; the path is a
        // member, and the getter is a const virtual call.
        unsafe { JPH_PathConstraintPath_GetPathMaxFraction(JPH_PathConstraint_GetPath(self.ptr())) }
    }

    /// Switches the motor and wakes the constraint's bodies.
    pub fn set_motor_state(&mut self, state: MotorState) {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. Jolt asserts valid motor settings for a running motor: every setting stored
        // was validated.
        unsafe { JPH_PathConstraint_SetPositionMotorState(self.ptr(), state.to_jph()) };
        self.wake_bodies();
    }

    /// Sets the fraction a position motor drives to, in `[0, max_fraction]` for every path (Jolt
    /// asserts the range for paths that do not loop). Wakes the constraint's bodies.
    pub fn set_target_path_fraction(&mut self, fraction: f32) -> Result<(), ConstraintError> {
        if !within(fraction, 0.0, self.max_fraction()) {
            return Err(ConstraintError::InvalidValue(
                "target path fraction must be between 0 and the path's max fraction",
            ));
        }
        // SAFETY: as in `set_motor_state`; the range Jolt asserts holds.
        unsafe { JPH_PathConstraint_SetTargetPathFraction(self.ptr(), fraction) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the velocity along the path a velocity motor drives to, m/s, at most
    /// [`limits::MAX_LINEAR_VELOCITY`] in magnitude. Wakes the constraint's bodies.
    pub fn set_target_velocity(&mut self, velocity: f32) -> Result<(), ConstraintError> {
        let speed = limits::MAX_LINEAR_VELOCITY;
        if !within(velocity, -speed, speed) {
            return Err(ConstraintError::InvalidValue(TARGET_VELOCITY_RULE));
        }
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_PathConstraint_SetTargetVelocity(self.ptr(), velocity) };
        self.wake_bodies();
        Ok(())
    }

    /// Replaces the motor settings, checked as at creation and bounded through the bodies'
    /// effective mass. Wakes the constraint's bodies.
    pub fn set_position_motor_settings(
        &mut self,
        motor: MotorSettings,
    ) -> Result<(), ConstraintError> {
        check_motor(motor, self.effective_mass_bound())?;
        let motor = motor.to_jph();
        // SAFETY: as in `set_motor_state`; `motor` is a live local that joltc copies.
        unsafe { JPH_PathConstraint_SetPositionMotorSettings(self.ptr(), &motor) };
        self.wake_bodies();
        Ok(())
    }

    /// Sets the friction force in N applied while the motor is off, finite and at least 0.
    /// Wakes the constraint's bodies.
    pub fn set_max_friction_force(&mut self, force: f32) -> Result<(), ConstraintError> {
        check_friction(force)?;
        // SAFETY: as in `set_motor_state`.
        unsafe { JPH_PathConstraint_SetMaxFrictionForce(self.ptr(), force) };
        self.wake_bodies();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::ensure_initialized;

    #[test]
    fn path_defaults_are_jolts() {
        assert!(ensure_initialized());
        // SAFETY: an all-zero settings value is valid (floats, integers, `false`, enums with a
        // zero value and a null path); joltc fills it with Jolt's defaults, a null path, and
        // allocates nothing.
        let mut jolt: JPH_PathConstraintSettings = unsafe { std::mem::zeroed() };
        // SAFETY: Jolt is initialised and `jolt` is a live local.
        unsafe { JPH_PathConstraintSettings_Init(&mut jolt) };
        let x = Vec3::new(1.0, 0.0, 0.0);
        let point = |position| HermitePathPoint {
            position,
            tangent: x,
        };
        let path = HermitePath::new(
            Vec3::new(0.0, 1.0, 0.0),
            vec![point(Vec3::ZERO), point(x)],
            false,
        )
        .unwrap();
        let ours = PathConstraintSettings::new(path).to_jph(std::ptr::null());
        assert!(jolt.path.is_null());
        assert_eq!(ours.base.enabled, jolt.base.enabled);
        assert_eq!(
            Vec3::from_jph(ours.pathPosition),
            Vec3::from_jph(jolt.pathPosition)
        );
        assert_eq!(
            Quat::from_jph(ours.pathRotation),
            Quat::from_jph(jolt.pathRotation)
        );
        assert_eq!(ours.pathFraction.to_bits(), jolt.pathFraction.to_bits());
        assert_eq!(
            ours.maxFrictionForce.to_bits(),
            jolt.maxFrictionForce.to_bits()
        );
        assert_eq!(ours.rotationConstraintType, jolt.rotationConstraintType);
        assert_eq!(
            MotorSettings::from_jph(ours.positionMotorSettings),
            MotorSettings::from_jph(jolt.positionMotorSettings)
        );
    }
}
