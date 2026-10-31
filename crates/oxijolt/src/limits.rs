//! The magnitudes the safe API accepts.
//!
//! Jolt checks most magnitudes only with debug assertions (the `asserts` feature) and otherwise
//! computes with whatever it is given, so a finite but huge input can overflow Jolt's `f32`
//! arithmetic inside a step. An input with a magnitude bound is checked against the constants of
//! this module before it reaches Jolt, and a value outside the bound is refused with the error of
//! the call. Other inputs only have to be finite, non-negative or ordered.
//!
//! # Frame and units
//! Positions are in metres in the world frame, which must keep every component of a caller-given
//! position within [`MAX_POSITION`]; [`PhysicsWorld::rebase`] moves the world into a new frame.
//! Shapes and local offsets are in metres around a shape's centre of mass, within
//! [`MAX_SHAPE_EXTENT`]. Velocities are in m/s and rad/s, accelerations in m/s² and rad/s², masses
//! in kg.
//!
//! [`MAX_LINEAR_VELOCITY`] and [`MAX_ANGULAR_VELOCITY`] are Jolt's own defaults. Every other
//! constant is crate policy, chosen from Jolt's documented ranges ("Conventions and Limits", "Big
//! Worlds" in Jolt's `Docs/Architecture.md`) or from the arithmetic it keeps finite.
//!
//! # Further reading
//! - [docs/limits.md] derives each bound from the Jolt arithmetic it protects.
//! - [docs/coverage.md] lists every input with its rule and boundary test, the paths that only
//!   tests cover, and what no bound here excludes.
//!
//! [docs/limits.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md
//! [docs/coverage.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/coverage.md

use crate::math::jolt_length;
use crate::{PhysicsWorld, Quat, RVec3, Real, Vec3};

/// Largest absolute value of each component of a caller-given world position, in metres:
/// 5 km with `f32` positions, 10 000 km with the `double-precision` feature.
///
/// See [docs/limits.md#frame-and-extent].
///
/// [docs/limits.md#frame-and-extent]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#frame-and-extent
pub const MAX_POSITION: Real = (if core::mem::size_of::<Real>() == 8 {
    1.0e7_f64
} else {
    5.0e3_f64
}) as Real;

/// Largest absolute value of each component of a shape's local bounds (around its centre of
/// mass), of a shape offset and of a local step distance, in metres.
///
/// See [docs/limits.md#frame-and-extent].
///
/// [docs/limits.md#frame-and-extent]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#frame-and-extent
pub const MAX_SHAPE_EXTENT: f32 = 2000.0;

/// Most shapes a compound may hold, counting itself, every child, decorator and leaf below it,
/// and a child shared by several parents at every use: 2^20. Jolt walks every child without
/// remembering the shapes it has seen when it builds a compound, computes its mass and steps
/// its body, so a compound that holds the level below twice at each of 31 levels would take
/// about 2^32 native calls per walk. At this bound a walk takes tens of milliseconds, as for a
/// compound of 2^20 distinct children.
///
/// See [docs/limits.md#expanded-compounds].
///
/// [docs/limits.md#expanded-compounds]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#expanded-compounds
pub const MAX_EXPANDED_SUB_SHAPES: u32 = 1 << 20;

/// Largest linear velocity a caller may give a body or character, in m/s: Jolt's default
/// `BodyCreationSettings::mMaxLinearVelocity`, to which Jolt also clamps a body's velocity every
/// step.
///
/// See [docs/limits.md#velocities-at-creation].
///
/// [docs/limits.md#velocities-at-creation]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#velocities-at-creation
pub const MAX_LINEAR_VELOCITY: f32 = 500.0;

/// Largest angular velocity a caller may give a body, in rad/s: Jolt's default
/// `BodyCreationSettings::mMaxAngularVelocity`, written as Jolt writes it so the `f32` bits match.
pub const MAX_ANGULAR_VELOCITY: f32 = 0.25 * core::f32::consts::PI * 60.0;

/// Largest length of a caller-given acceleration (world gravity, a character's or vehicle's
/// gravity, the acceleration a body's added forces give it), in m/s²: about 5e8,
/// [`MAX_LINEAR_VELOCITY`] over [`PhysicsWorld::MIN_DELTA_TIME`].
///
/// See [docs/limits.md#accelerations].
///
/// [docs/limits.md#accelerations]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#accelerations
pub const MAX_ACCELERATION: f32 = MAX_LINEAR_VELOCITY / PhysicsWorld::MIN_DELTA_TIME;

/// Largest angular acceleration a body's added torques may give it, in rad/s²: about 4.71e7,
/// [`MAX_ANGULAR_VELOCITY`] over [`PhysicsWorld::MIN_DELTA_TIME`].
///
/// See [docs/limits.md#accelerations].
///
/// [docs/limits.md#accelerations]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#accelerations
pub const MAX_ANGULAR_ACCELERATION: f32 = MAX_ANGULAR_VELOCITY / PhysicsWorld::MIN_DELTA_TIME;

/// Largest change of a body's linear velocity one impulse may cause, in m/s: twice
/// [`MAX_LINEAR_VELOCITY`], enough to reverse a body moving at that speed.
///
/// See [docs/limits.md#impulses].
///
/// [docs/limits.md#impulses]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#impulses
pub const MAX_VELOCITY_CHANGE: f32 = 2.0 * MAX_LINEAR_VELOCITY;

/// Largest change of a body's angular velocity one angular or point impulse may cause, in
/// rad/s: twice [`MAX_ANGULAR_VELOCITY`].
///
/// See [docs/limits.md#impulses].
///
/// [docs/limits.md#impulses]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#impulses
pub const MAX_ANGULAR_VELOCITY_CHANGE: f32 = 2.0 * MAX_ANGULAR_VELOCITY;

/// Largest absolute gravity factor of a body.
pub const MAX_GRAVITY_FACTOR: f32 = 1000.0;

/// Largest friction coefficient of a body or contact.
///
/// See [docs/limits.md#friction].
///
/// [docs/limits.md#friction]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#friction
pub const MAX_FRICTION: f32 = 1000.0;

/// Smallest mass of a dynamic body or ragdoll part, in kg: an inverse mass of at most 1000 per kg,
/// a 1 cm cube of water.
pub const MIN_MASS: f32 = 1.0e-3;

/// Smallest principal moment of an inertia given with
/// [`BodySettings::mass_and_inertia`](crate::BodySettings::mass_and_inertia), in kg·m²: Jolt
/// then never takes its near-zero fallback, and every principal inverse inertia is at most 1e6.
///
/// See [docs/limits.md#provided-inertia].
///
/// [docs/limits.md#provided-inertia]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#provided-inertia
pub const MIN_PROVIDED_INERTIA: f32 = 1.0e-6;

/// Largest inverse mass of a movable soft body vertex, in 1/kg: 1000, whose inverse Jolt
/// computes in `f32` as exactly [`MIN_MASS`]. `1.0 / MIN_MASS` itself rounds to 999.99994.
pub const MAX_VERTEX_INVERSE_MASS: f32 = 1000.0;

/// Largest mass of a dynamic body, ragdoll part or character, in kg: a 10 m cube of water, the
/// top of the dynamic object sizes Jolt documents. It also bounds the forces [`MAX_ACCELERATION`]
/// accepts (at most 5e14 N), contact effective masses, a vehicle's gravity force and a
/// character's weight impulse.
pub const MAX_MASS: f32 = 1.0e6;

/// Smallest positive factor a contact may put on a body's inverse mass or inverse inertia
/// ([`ContactSettings`](crate::ContactSettings), [`SoftBodyContactSettings`](crate::SoftBodyContactSettings)):
/// 1e-9, the ratio [`MIN_MASS`]` / `[`MAX_MASS`]. At this factor a body of [`MIN_MASS`] weighs as
/// much as one of [`MAX_MASS`] in the contact; a factor of exactly 0 makes the body immovable,
/// and a contact whose dynamic bodies all have 0 is dropped.
///
/// See [docs/limits.md#inverse-mass-and-inertia-scales].
///
/// [docs/limits.md#inverse-mass-and-inertia-scales]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#inverse-mass-and-inertia-scales
pub const MIN_CONTACT_SCALE: f32 = 1.0e-9;

/// Largest spring stiffness `k` and damping `c` Jolt may derive from a constraint spring.
///
/// See [docs/limits.md#springs].
///
/// [docs/limits.md#springs]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#springs
pub const MAX_SPRING_COEFFICIENT: f32 = 1.0e30;

/// Largest weight impulse a character may press on what it stands on during one update, in N·s:
/// its mass times the length of the update's gravity times the update's delta time. It allows a
/// character of [`MAX_MASS`] at 1000 m/s² with one-second updates.
///
/// See [docs/limits.md#character-weight-and-push].
///
/// [docs/limits.md#character-weight-and-push]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#character-weight-and-push
pub const MAX_WEIGHT_IMPULSE: f32 = 1.0e9;

/// Largest magnitude of a rack-and-pinion or pulley ratio; the smallest is its inverse. Gears
/// have their own, tighter range (see [`MAX_GEAR_RATIO`]).
///
/// See [docs/limits.md#coupling-ratios].
///
/// [docs/limits.md#coupling-ratios]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#coupling-ratios
pub const MAX_RATIO: f32 = 1.0e4;

/// Largest gear ratio; gear ratios are within `1..=MAX_GEAR_RATIO`
/// (see [`GearConstraintSettings`](crate::GearConstraintSettings)).
///
/// See [docs/limits.md#coupling-ratios].
///
/// [docs/limits.md#coupling-ratios]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#coupling-ratios
pub const MAX_GEAR_RATIO: f32 = 10.0;

/// Largest torque fraction of a vehicle engine's torque curve
/// ([`VehicleEngineSettings::normalized_torque`](crate::VehicleEngineSettings::normalized_torque)):
/// every point's y is within `0..=MAX_NORMALIZED_TORQUE`.
///
/// See [docs/limits.md#torque-curves].
///
/// [docs/limits.md#torque-curves]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#torque-curves
pub const MAX_NORMALIZED_TORQUE: f32 = 10.0;

/// Smallest x distance between neighbouring points of a vehicle engine's torque curve, whose x
/// are within `0..=1`, up to rounding x to `f32`.
///
/// See [docs/limits.md#torque-curves].
///
/// [docs/limits.md#torque-curves]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#torque-curves
pub const MIN_TORQUE_CURVE_SPACING: f32 = 1.0e-3;

/// Smallest moment of inertia of a tracked vehicle's track, kg·m².
///
/// See [docs/limits.md#track-drive-envelope].
///
/// [docs/limits.md#track-drive-envelope]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#track-drive-envelope
pub const MIN_TRACK_INERTIA: f32 = 1.0e-3;

/// Largest moment of inertia of a tracked vehicle's track, kg·m².
///
/// See [docs/limits.md#track-drive-envelope].
///
/// [docs/limits.md#track-drive-envelope]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#track-drive-envelope
pub const MAX_TRACK_INERTIA: f32 = 1.0e6;

/// Largest ratio between the moments of inertia of a tracked vehicle's two tracks.
///
/// See [docs/limits.md#track-inertia-ratio].
///
/// [docs/limits.md#track-inertia-ratio]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#track-inertia-ratio
pub const MAX_TRACK_INERTIA_RATIO: f32 = 100.0;

/// Largest track mass ratio of a tracked vehicle's wheel: its track's moment of inertia over
/// the wheel's radius squared (the track's mass as the ground feels it at that wheel), divided by
/// the smallest effective mass of the chassis where the wheel pushes it, along any direction in
/// the wheel's forward/up plane (tilted ground turns the push within that plane).
///
/// Jolt sizes each wheel's driving impulse for the track alone, as if the chassis did not move,
/// so a track heavy against the chassis makes the solver overshoot and diverge.
/// [`PhysicsWorld::create_tracked_vehicle`] computes the ratio from the chassis' mass, inertia
/// and centre of mass and refuses a vehicle with any wheel above this bound. Jolt's `TankTest`
/// tank has a ratio of 0.23; a narrow or flat hull, with little roll and pitch inertia, allows
/// much lighter tracks.
///
/// See [docs/limits.md#track-mass-ratio].
///
/// [docs/limits.md#track-mass-ratio]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#track-mass-ratio
pub const MAX_TRACK_MASS_RATIO: f32 = 0.25;

/// Largest lever-arm ratio a world constraint may give a dynamic body: how far the point where
/// the constraint holds the body lies from its centre of mass, measured against the body's own
/// size.
///
/// The lever-arm ratio of a body at a point `r` from its centre of mass is
/// `mass · trace([r]× I⁻¹ [r]×ᵀ)`, with `I⁻¹` the body's inverse inertia: summed over the body's
/// principal axes, the squared distance of the point from each axis divided by the squared
/// radius of gyration about it. A sphere or cube with radius of gyration `k` has
/// `2 · (|r| / k)²`, so the bound allows `|r|` up to about `22 · k`; a rod held at its end has a
/// ratio of 6 at any length.
/// [`PhysicsWorld::create_constraint`] says which points of a constraint it checks.
///
/// The bound allows a door on a hinge at its edge, a weld at the surface of a part, and a
/// pendulum bob of radius `a` on a point or hinge constraint up to about `14 · a` from the pivot;
/// a longer pendulum is a [`DistanceConstraintSettings`](crate::DistanceConstraintSettings) whose
/// points lie on the bodies.
///
/// See [docs/limits.md#lever-arm-ratio].
///
/// [docs/limits.md#lever-arm-ratio]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#lever-arm-ratio
pub const MAX_LEVER_ARM_RATIO: f32 = 1000.0;

/// Shortest distance between two soft body vertices that a face edge or an explicit edge
/// joins, in metres: 1 mm, measured as Jolt measures a rest length (an `f32` difference and an
/// `f32` length).
///
/// See [docs/limits.md#soft-body-edge-length].
///
/// [docs/limits.md#soft-body-edge-length]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-edge-length
pub const MIN_SOFT_BODY_EDGE_LENGTH: f32 = 1.0e-3;

/// Most joints a skinned soft body may use: every
/// [`SoftBodyInvBind::joint`](crate::SoftBodyInvBind::joint) is below it. It bounds the joint
/// array built for each skinning call.
///
/// See [docs/limits.md#soft-body-skinning].
///
/// [docs/limits.md#soft-body-skinning]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-skinning
pub const MAX_SKIN_JOINTS: u32 = 4096;

/// Shortest direction of a skeleton mapper chain, in metres: the ragdoll joints at its ends must
/// be exactly together or at least this far apart, and the animation chain between them at least
/// this long. A constant of the native mapper, not a setting; shorter chains give
/// [`RagdollError::DegenerateChain`](crate::RagdollError::DegenerateChain).
///
/// See [docs/limits.md#skeleton-mapper-chains].
///
/// [docs/limits.md#skeleton-mapper-chains]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#skeleton-mapper-chains
pub const MIN_MAPPED_CHAIN_LENGTH: f32 = 1.0e-3;

/// Largest compliance (inverse stiffness) of a soft body constraint, in the units of the
/// constraint's own equation; 0 is rigid. It keeps Jolt's compliance terms finite; it does not
/// make the solver stable at every compliance.
///
/// See [docs/limits.md#soft-body-compliance].
///
/// [docs/limits.md#soft-body-compliance]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-compliance
pub const MAX_COMPLIANCE: f32 = 1.0e20;

/// Largest pressure coefficient of a soft body (`n · R · T` in Jolt's terms, N·m). A new body
/// with pressure must also enclose enough volume for its faces; a body crushed to a tiny volume
/// later is not covered.
///
/// See [docs/limits.md#soft-body-pressure].
///
/// [docs/limits.md#soft-body-pressure]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-pressure
pub const MAX_SOFT_BODY_PRESSURE: f32 = 1.0e6;

/// Largest force in newtons that the pressure of a new soft body may give a vertex in Jolt's
/// formula: [`MAX_ACCELERATION`] for a vertex of [`MAX_VERTEX_INVERSE_MASS`], 5e5 N (see
/// [docs/limits.md#soft-body-pressure]).
///
/// [docs/limits.md#soft-body-pressure]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-pressure
const MAX_PRESSURE_VERTEX_FORCE: f64 = MAX_ACCELERATION as f64 / MAX_VERTEX_INVERSE_MASS as f64;

/// Unit roundoff of `f32`, `2^-24`.
const F32_UNIT_ROUNDOFF: f64 = f32::EPSILON as f64 / 2.0;

/// Bound on a product Jolt forms in `f32`, so that it and a sum of a few such products stay
/// finite (`f32::MAX` is about 3.4e38).
pub(crate) const F32_PRODUCT_HEADROOM: f64 = 1.0e37;

/// What [`is_soft_body_pressure`] needs to know of a soft body's vertices and faces, computed
/// once when its shared settings are built; the rule and its error bounds are derived in
/// [docs/limits.md#soft-body-pressure].
///
/// [docs/limits.md#soft-body-pressure]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-pressure
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SoftBodyPressureGeometry {
    /// Lower bound of the six-volume Jolt computes in `f32` before the first step, in m³.
    six_volume_low: f64,
    /// Largest sum, over the faces of one vertex, of the bound of `|x2 - x1| · |x3 - x1|` in
    /// Jolt's arithmetic, in m².
    largest_vertex_face_area: f64,
}

impl SoftBodyPressureGeometry {
    /// The geometry of `faces` (vertex indices, each valid) between vertices at `positions`
    /// about the body origin.
    pub(crate) fn new(positions: &[Vec3], faces: &[[u32; 3]]) -> Self {
        // The error terms of the derivation, in units of `u`, and the rotation's slack.
        const ROTATION_VOLUME: f64 = 4.0e-5;
        const ROTATION_LENGTH: f64 = 1.0001;
        const POSITION_ERROR: f64 = 15.0;
        const TERM_ERROR: f64 = 64.0;
        const UNDERFLOW_PER_FACE: f64 = 1.0e-36;
        let u = F32_UNIT_ROUNDOFF;
        let point = |index: u32| {
            let p = positions[index as usize];
            [p.x, p.y, p.z].map(f64::from)
        };
        let edge = |from: [f64; 3], to: [f64; 3]| {
            let e = [to[0] - from[0], to[1] - from[1], to[2] - from[2]];
            ROTATION_LENGTH * (norm(e) + POSITION_ERROR * u * (norm(from) + norm(to)))
        };
        let mut six_volume = 0.0;
        let mut partial_sums = 0.0;
        let mut length_products = 0.0;
        let mut vertex_face_area = vec![0.0_f64; positions.len()];
        for &face in faces {
            let [x1, x2, x3] = face.map(point);
            let c = [
                x1[1] * x2[2] - x1[2] * x2[1],
                x1[2] * x2[0] - x1[0] * x2[2],
                x1[0] * x2[1] - x1[1] * x2[0],
            ];
            six_volume += c[0] * x3[0] + c[1] * x3[1] + c[2] * x3[2];
            partial_sums += six_volume.abs();
            length_products += norm(x1) * norm(x2) * norm(x3);
            let area = edge(x1, x2) * edge(x1, x3);
            for index in face {
                vertex_face_area[index as usize] += area;
            }
        }
        let face_count = faces.len() as f64;
        let growth = (1.0 - u).powf(-face_count);
        let error = growth
            * (u * (TERM_ERROR * length_products + (1.0 + ROTATION_VOLUME) * partial_sums)
                + UNDERFLOW_PER_FACE * face_count);
        Self {
            six_volume_low: (1.0 - ROTATION_VOLUME) * six_volume - error,
            largest_vertex_face_area: vertex_face_area.into_iter().fold(0.0, f64::max),
        }
    }
}

/// The length of `v`.
fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Whether a soft body of `geometry` may be created with `pressure`, already checked to be
/// within `0..=MAX_SOFT_BODY_PRESSURE`: always without pressure, otherwise when the faces
/// enclose a positive volume large enough for the pressure (see
/// [docs/limits.md#soft-body-pressure]).
///
/// [docs/limits.md#soft-body-pressure]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-pressure
pub(crate) fn is_soft_body_pressure(pressure: f32, geometry: &SoftBodyPressureGeometry) -> bool {
    pressure == 0.0
        || (geometry.six_volume_low > 0.0
            && f64::from(pressure) * geometry.largest_vertex_face_area
                <= MAX_PRESSURE_VERTEX_FORCE * geometry.six_volume_low)
}

/// Whether a soft body with `vertex_count` vertices, the largest inverse mass among them
/// `largest_inverse_mass`, may hold the accumulated force `force` in newtons, computed in
/// `f64` (see [docs/limits.md#soft-body-forces]).
///
/// [docs/limits.md#soft-body-forces]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-forces
pub(crate) fn is_soft_body_force(
    force: [f64; 3],
    largest_inverse_mass: f32,
    vertex_count: u32,
) -> bool {
    let length = norm(force);
    let per_vertex = f64::from(largest_inverse_mass) / f64::from(vertex_count.max(1));
    length <= f64::from(MAX_ACCELERATION) * f64::from(MAX_MASS)
        && length * per_vertex <= f64::from(MAX_ACCELERATION)
}

/// Smallest principal moment of an inertia tensor Jolt decomposes, relative to the tensor's
/// Frobenius norm, that [`is_rigid_body_inertia`] and [`is_soft_body_inertia`] accept:
/// `1001 · 8 · u`, about 4.8e-4 (see [docs/limits.md#soft-body-inertia]).
///
/// [docs/limits.md#soft-body-inertia]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-inertia
const MIN_INERTIA_RATIO: f64 = 1001.0 * 8.0 * F32_UNIT_ROUNDOFF;

/// Whether Jolt decomposes the non-diagonal inertia `tensor` of a rigid body (Jolt's own `f32`
/// tensor, widened to `f64`) without an assertion and into positive principal moments (see
/// [docs/limits.md#rigid-body-inertia]).
///
/// The rule is scale-free: a tensor small enough for Jolt's unit-sphere fallback is decomposed
/// first all the same, so it is checked like any other.
///
/// [docs/limits.md#rigid-body-inertia]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#rigid-body-inertia
pub(crate) fn is_rigid_body_inertia(tensor: [[f64; 3]; 3]) -> bool {
    let [[a, b, c], [d, e, f], [g, h, i]] = tensor;
    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    let minors = (a * e - b * d) + (a * i - c * g) + (e * i - f * h);
    let frobenius = tensor.iter().flatten().map(|v| v * v).sum::<f64>().sqrt();
    // `det / minors` is at most the smallest principal moment (see `is_soft_body_inertia`).
    a + e + i > 0.0 && det > 0.0 && minors > 0.0 && det / minors >= MIN_INERTIA_RATIO * frobenius
}

/// The vertices of a soft body as Jolt's inertia computation sees them, reduced to what
/// [`is_soft_body_inertia`] needs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SoftBodyMassDistribution {
    /// `Σ m p pᵀ` over the vertices in `f64`, `p` the position about the body origin and `m` the
    /// mass as Jolt computes it (`1.0f / w` in `f32`).
    second_moment: [[f64; 3]; 3],
    vertex_count: usize,
    /// Whether a vertex is kinematic: Jolt then gives the body infinite mass and inertia
    /// without decomposing anything.
    has_kinematic_vertex: bool,
    /// Whether every position has at most one non-zero component, so that every off-diagonal
    /// term Jolt adds is exactly zero.
    on_axes: bool,
}

impl SoftBodyMassDistribution {
    /// The distribution of vertices given as (position about the body origin, inverse mass).
    pub(crate) fn new(vertices: impl IntoIterator<Item = (Vec3, f32)>) -> Self {
        let mut distribution = Self {
            second_moment: [[0.0; 3]; 3],
            vertex_count: 0,
            has_kinematic_vertex: false,
            on_axes: true,
        };
        for (position, inverse_mass) in vertices {
            distribution.vertex_count += 1;
            if inverse_mass == 0.0 {
                distribution.has_kinematic_vertex = true;
                continue;
            }
            let mass = f64::from(1.0 / inverse_mass);
            let p = [position.x, position.y, position.z].map(f64::from);
            for (i, row) in distribution.second_moment.iter_mut().enumerate() {
                for (j, entry) in row.iter_mut().enumerate() {
                    *entry += mass * p[i] * p[j];
                }
            }
            distribution.on_axes &= p.iter().filter(|&&c| c != 0.0).count() <= 1;
        }
        distribution
    }
}

/// Whether Jolt can decompose the inertia of a soft body whose vertices have `distribution`
/// about its origin, rotated by `rotation` when Jolt bakes a rotation into the vertices,
/// without an assertion and into a finite inverse inertia (see
/// [docs/limits.md#soft-body-inertia]).
///
/// [docs/limits.md#soft-body-inertia]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-inertia
pub(crate) fn is_soft_body_inertia(
    distribution: &SoftBodyMassDistribution,
    rotation: Option<Quat>,
) -> bool {
    if distribution.has_kinematic_vertex {
        return true;
    }
    let u = F32_UNIT_ROUNDOFF;
    let summed_terms = (distribution.vertex_count + 3) as f64 * u;
    if summed_terms >= 0.5 {
        return false;
    }
    // The identity quaternion gives Jolt the exact identity matrix.
    let rotation = rotation.filter(|q| !(q.x == 0.0 && q.y == 0.0 && q.z == 0.0));
    let c = match rotation {
        Some(q) => {
            let r = jolt_rotation_matrix(q);
            let rc = multiply(r, distribution.second_moment);
            multiply(rc, transpose(r))
        }
        None => distribution.second_moment,
    };
    let trace = c[0][0] + c[1][1] + c[2][2];
    let inertia: [[f64; 3]; 3] = std::array::from_fn(|i| {
        std::array::from_fn(|j| if i == j { trace - c[i][j] } else { -c[i][j] })
    });
    if rotation.is_none() && distribution.on_axes {
        // Jolt's tensor is exactly diagonal and its decomposition exact; as for rigid bodies,
        // a near-zero tensor gets the unit-sphere inertia and otherwise every moment is inverted.
        let diagonal = [inertia[0][0], inertia[1][1], inertia[2][2]];
        let length_sq: f64 = diagonal.iter().map(|d| d * d).sum();
        return length_sq <= 0.5e-12 || diagonal.iter().all(|&d| d >= 1.0e-30);
    }
    let position_error = if rotation.is_some() { 16.0 * u } else { 0.0 };
    let error = trace
        * ((1.0 + 3.0_f64.sqrt()) * (2.0 * position_error + position_error * position_error)
            + 3.0 * summed_terms / (1.0 - summed_terms) * (1.0 + position_error).powi(2));
    let frobenius = inertia.iter().flatten().map(|v| v * v).sum::<f64>().sqrt();
    let [[a, b, cc], [d, e, f], [g, h, i]] = inertia;
    let det = a * (e * i - f * h) - b * (d * i - f * g) + cc * (d * h - e * g);
    let minors = (a * e - b * d) + (a * i - cc * g) + (e * i - f * h);
    if !(det > 0.0 && minors > 0.0 && frobenius >= 1.0e-30) {
        return false;
    }
    // `det / minors` is at most the smallest principal moment: the sum of the principal
    // minors, the products of two moments, exceeds the product of the two larger ones.
    det / minors - error >= MIN_INERTIA_RATIO * (frobenius + error)
}

/// The matrix of Jolt's `Mat44::sRotation(q)`, in `f64`.
fn jolt_rotation_matrix(q: Quat) -> [[f64; 3]; 3] {
    let [x, y, z, w] = [q.x, q.y, q.z, q.w].map(f64::from);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn multiply(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}

fn transpose(a: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}

/// What a caller-given position must satisfy ([`is_in_frame`]).
pub(crate) const POSITION_RULE: &str = "position must be finite and within limits::MAX_POSITION";

/// Whether every component of `position` is at most [`MAX_POSITION`] in absolute value.
pub(crate) fn is_in_frame(position: RVec3) -> bool {
    [position.x, position.y, position.z]
        .iter()
        .all(|c| c.abs() <= MAX_POSITION)
}

/// Whether every component of `displacement` is at most `2 * MAX_POSITION` in absolute value:
/// the largest move between two positions in the frame.
pub(crate) fn is_frame_displacement(displacement: RVec3) -> bool {
    [displacement.x, displacement.y, displacement.z]
        .iter()
        .all(|c| c.abs() <= 2.0 * MAX_POSITION)
}

/// [`is_frame_displacement`] for an `f32` vector.
pub(crate) fn is_frame_span(span: Vec3) -> bool {
    [span.x, span.y, span.z]
        .iter()
        .all(|&c| Real::from(c).abs() <= 2.0 * MAX_POSITION)
}

/// Whether every component of `offset` is at most [`MAX_SHAPE_EXTENT`] in absolute value.
pub(crate) fn is_local_offset(offset: Vec3) -> bool {
    [offset.x, offset.y, offset.z]
        .iter()
        .all(|c| c.abs() <= MAX_SHAPE_EXTENT)
}

/// Whether `distance` is finite and within `0..=MAX_SHAPE_EXTENT`.
pub(crate) fn is_local_distance(distance: f32) -> bool {
    (0.0..=MAX_SHAPE_EXTENT).contains(&distance)
}

/// What a caller-given linear velocity must satisfy ([`is_linear_velocity`]).
pub(crate) const LINEAR_VELOCITY_RULE: &str =
    "linear velocity must be finite and at most limits::MAX_LINEAR_VELOCITY long";

/// Whether `velocity` is finite and Jolt's own length of it at most [`MAX_LINEAR_VELOCITY`].
pub(crate) fn is_linear_velocity(velocity: Vec3) -> bool {
    velocity.is_finite() && jolt_length(velocity) <= MAX_LINEAR_VELOCITY
}

/// What a caller-given angular velocity must satisfy ([`is_angular_velocity`]).
pub(crate) const ANGULAR_VELOCITY_RULE: &str =
    "angular velocity must be finite and at most limits::MAX_ANGULAR_VELOCITY long";

/// Whether `velocity` is finite and Jolt's own length of it at most [`MAX_ANGULAR_VELOCITY`].
pub(crate) fn is_angular_velocity(velocity: Vec3) -> bool {
    velocity.is_finite() && jolt_length(velocity) <= MAX_ANGULAR_VELOCITY
}

/// What a gravity vector must satisfy ([`is_acceleration`]).
pub(crate) const GRAVITY_RULE: &str =
    "gravity must be finite and at most limits::MAX_ACCELERATION long";

/// Whether `acceleration` is finite and its length, computed in `f64`, at most
/// [`MAX_ACCELERATION`].
pub(crate) fn is_acceleration(acceleration: Vec3) -> bool {
    acceleration.is_finite() && f64_length(acceleration) <= f64::from(MAX_ACCELERATION)
}

/// What a gravity factor must satisfy ([`is_gravity_factor`]).
pub(crate) const GRAVITY_FACTOR_RULE: &str =
    "gravity factor must be finite and within limits::MAX_GRAVITY_FACTOR";

/// Whether `factor` is finite and at most [`MAX_GRAVITY_FACTOR`] in absolute value.
pub(crate) fn is_gravity_factor(factor: f32) -> bool {
    factor.abs() <= MAX_GRAVITY_FACTOR
}

/// Whether `scale` is 0 or within `MIN_CONTACT_SCALE..=1`.
pub(crate) fn is_contact_scale(scale: f32) -> bool {
    scale == 0.0 || (MIN_CONTACT_SCALE..=1.0).contains(&scale)
}

/// What a body friction must satisfy ([`is_friction`]).
pub(crate) const FRICTION_RULE: &str =
    "friction must be finite and between 0 and limits::MAX_FRICTION";

/// Whether `friction` is finite and within `0..=MAX_FRICTION`.
pub(crate) fn is_friction(friction: f32) -> bool {
    (0.0..=MAX_FRICTION).contains(&friction)
}

/// Whether a character of `mass` updated with `gravity` for `delta_time` seconds presses with
/// a weight impulse of at most [`MAX_WEIGHT_IMPULSE`], computed in `f64` so that it cannot
/// overflow. The inputs are already checked to be finite and within their own bounds.
pub(crate) fn is_weight_impulse(mass: f32, gravity: Vec3, delta_time: f32) -> bool {
    f64::from(mass) * f64_length(gravity) * f64::from(delta_time) <= f64::from(MAX_WEIGHT_IMPULSE)
}

/// What an impulse on a dynamic body must satisfy ([`is_velocity_change`]).
pub(crate) const VELOCITY_CHANGE_RULE: &str =
    "impulse would change the velocity by more than limits::MAX_VELOCITY_CHANGE";

/// Whether an `impulse` (N·s, in `f64`) changes the velocity of a body with `inverse_mass` by at
/// most [`MAX_VELOCITY_CHANGE`]; false for NaN.
pub(crate) fn is_velocity_change(impulse: [f64; 3], inverse_mass: f32) -> bool {
    norm(impulse) * f64::from(inverse_mass) <= f64::from(MAX_VELOCITY_CHANGE)
}

/// What an angular or point impulse on a dynamic body must satisfy
/// ([`is_angular_velocity_change`]).
pub(crate) const ANGULAR_VELOCITY_CHANGE_RULE: &str =
    "impulse would change the angular velocity by more than limits::MAX_ANGULAR_VELOCITY_CHANGE";

/// Whether an `angular_impulse` (N·m·s, in `f64`) changes the angular velocity of a body whose
/// largest principal inverse inertia is `largest_inverse_inertia` by at most
/// [`MAX_ANGULAR_VELOCITY_CHANGE`]; false for NaN.
pub(crate) fn is_angular_velocity_change(
    angular_impulse: [f64; 3],
    largest_inverse_inertia: f32,
) -> bool {
    norm(angular_impulse) * f64::from(largest_inverse_inertia)
        <= f64::from(MAX_ANGULAR_VELOCITY_CHANGE)
}

/// What a motorcycle's lean spring must satisfy on its chassis ([`is_lean_spring`]).
pub(crate) const LEAN_SPRING_RULE: &str =
    "lean spring would exceed limits::MAX_ANGULAR_ACCELERATION for this chassis";

/// Whether a motorcycle lean spring of `constant` (N·m/rad) and `damping` (N·m·s/rad) keeps
/// the angular acceleration it gives a chassis whose largest principal inverse inertia is
/// `largest_inverse_inertia` within [`MAX_ANGULAR_ACCELERATION`], computed in `f64`: the
/// torque at a lean error of π and an angular velocity about forward of
/// [`MAX_ANGULAR_VELOCITY_CHANGE`], with headroom for a forward axis that is unit only within
/// `math::UNIT_TOLERANCE`. See docs/limits.md#motorcycle-lean.
pub(crate) fn is_lean_spring(constant: f32, damping: f32, largest_inverse_inertia: f32) -> bool {
    let torque = f64::from(constant) * std::f64::consts::PI
        + f64::from(damping) * f64::from(MAX_ANGULAR_VELOCITY_CHANGE);
    let headroom = 1.0 + 4.0 * f64::from(crate::math::UNIT_TOLERANCE);
    torque * f64::from(largest_inverse_inertia) * headroom <= f64::from(MAX_ANGULAR_ACCELERATION)
}

/// What a dynamic body's mass must satisfy ([`is_mass`]).
pub(crate) const MASS_RULE: &str = "mass must be between limits::MIN_MASS and limits::MAX_MASS";

/// Whether `mass` is finite and within `MIN_MASS..=MAX_MASS`.
pub(crate) fn is_mass(mass: f32) -> bool {
    (MIN_MASS..=MAX_MASS).contains(&mass)
}

/// What a provided inertia's entries must satisfy ([`check_provided_inertia`]).
pub(crate) const PROVIDED_INERTIA_VALUES_RULE: &str =
    "provided inertia must be finite and symmetric";

/// What a provided inertia's smallest principal moment must satisfy ([`check_provided_inertia`]).
pub(crate) const PROVIDED_INERTIA_FLOOR_RULE: &str =
    "provided inertia needs principal moments of at least limits::MIN_PROVIDED_INERTIA";

/// What a provided inertia's largest principal moment must satisfy ([`check_provided_inertia`]).
pub(crate) const PROVIDED_INERTIA_CEILING_RULE: &str =
    "provided inertia moments must be at most 6 * mass * limits::MAX_SHAPE_EXTENT^2";

/// What a provided inertia that is not diagonal must satisfy ([`check_provided_inertia`]).
pub(crate) const PROVIDED_INERTIA_CONDITION_RULE: &str =
    "provided inertia must be positive definite and well conditioned";

/// Checks the inertia `tensor` (in rows, kg·m²) provided for a body of `mass`, already within
/// `MIN_MASS..=MAX_MASS`, and returns the rule it breaks. Computed in `f64`; the rules are
/// derived in [docs/limits.md#provided-inertia].
///
/// [docs/limits.md#provided-inertia]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#provided-inertia
pub(crate) fn check_provided_inertia(mass: f32, tensor: [[f32; 3]; 3]) -> Result<(), &'static str> {
    let finite = tensor.iter().flatten().all(|value| value.is_finite());
    let symmetric = (0..3).all(|i| (0..3).all(|j| tensor[i][j] == tensor[j][i]));
    if !(finite && symmetric) {
        return Err(PROVIDED_INERTIA_VALUES_RULE);
    }
    let t = tensor.map(|row| row.map(f64::from));
    let extent = f64::from(MAX_SHAPE_EXTENT);
    let ceiling = 6.0 * f64::from(mass) * extent * extent;
    let floor = f64::from(MIN_PROVIDED_INERTIA);
    let diagonal = [t[0][0], t[1][1], t[2][2]];
    if t[0][1] == 0.0 && t[0][2] == 0.0 && t[1][2] == 0.0 {
        // Jolt decomposes a diagonal tensor exactly, so the diagonal is what it inverts.
        if !diagonal.iter().all(|&moment| moment >= floor) {
            return Err(PROVIDED_INERTIA_FLOOR_RULE);
        }
        if !diagonal.iter().all(|&moment| moment <= ceiling) {
            return Err(PROVIDED_INERTIA_CEILING_RULE);
        }
        return Ok(());
    }
    if !is_rigid_body_inertia(t) {
        return Err(PROVIDED_INERTIA_CONDITION_RULE);
    }
    let [[a, b, c], [d, e, f], [g, h, i]] = t;
    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    let minors = (a * e - b * d) + (a * i - c * g) + (e * i - f * h);
    let frobenius = t.iter().flatten().map(|v| v * v).sum::<f64>().sqrt();
    // `det / minors` is at most the smallest principal moment, and the moment Jolt's `f32`
    // decomposition returns is within the residual bound `8 u |I|_F` of it.
    if det / minors < floor + 8.0 * F32_UNIT_ROUNDOFF * frobenius {
        return Err(PROVIDED_INERTIA_FLOOR_RULE);
    }
    // The Frobenius norm is at least the largest principal moment.
    if frobenius > ceiling {
        return Err(PROVIDED_INERTIA_CEILING_RULE);
    }
    Ok(())
}

/// Whether `inverse_mass` is the inverse of a mass within `MIN_MASS..=MAX_MASS`: the range of
/// a movable soft body vertex, and of a rigid body that becomes dynamic.
pub(crate) fn is_vertex_inverse_mass(inverse_mass: f32) -> bool {
    (1.0 / MAX_MASS..=MAX_VERTEX_INVERSE_MASS).contains(&inverse_mass)
}

/// Whether `compliance` is finite and within `0..=MAX_COMPLIANCE`.
pub(crate) fn is_compliance(compliance: f32) -> bool {
    (0.0..=MAX_COMPLIANCE).contains(&compliance)
}

/// Whether `ratio` is finite and its magnitude within `1 / MAX_RATIO..=MAX_RATIO`.
pub(crate) fn is_ratio(ratio: f32) -> bool {
    (1.0 / MAX_RATIO..=MAX_RATIO).contains(&ratio.abs())
}

/// What an engine's torque curve must satisfy ([`is_torque_curve`]).
pub(crate) const TORQUE_CURVE_RULE: &str = "engine torque curve needs x in 0..=1, \
     limits::MIN_TORQUE_CURVE_SPACING apart, and y in 0..=limits::MAX_NORMALIZED_TORQUE";

/// Whether `points` make an engine torque curve: at least one point, x within `0..=1` and
/// increasing by at least [`MIN_TORQUE_CURVE_SPACING`], y within `0..=MAX_NORMALIZED_TORQUE`.
///
/// The spacing is measured on the exact values of the `f32` coordinates and may fall short by
/// one `f32` epsilon, which covers rounding x within `0..=1` to `f32`: points at `i / 1000` pass.
pub(crate) fn is_torque_curve(points: &[(f32, f32)]) -> bool {
    let spacing = f64::from(MIN_TORQUE_CURVE_SPACING) - f64::from(f32::EPSILON);
    !points.is_empty()
        && points
            .iter()
            .all(|&(x, y)| (0.0..=1.0).contains(&x) && (0.0..=MAX_NORMALIZED_TORQUE).contains(&y))
        && points
            .windows(2)
            .all(|pair| f64::from(pair[1].0) - f64::from(pair[0].0) >= spacing)
}

/// Whether `inertia` is within `MIN_TRACK_INERTIA..=MAX_TRACK_INERTIA`.
pub(crate) fn is_track_inertia(inertia: f32) -> bool {
    (MIN_TRACK_INERTIA..=MAX_TRACK_INERTIA).contains(&inertia)
}

/// Whether the larger of two positive track inertias is at most [`MAX_TRACK_INERTIA_RATIO`]
/// times the smaller, the product rounded to `f32`.
pub(crate) fn is_track_inertia_ratio(first: f32, second: f32) -> bool {
    first.max(second) <= MAX_TRACK_INERTIA_RATIO * first.min(second)
}

/// What a tracked vehicle's tracks must satisfy on their chassis ([`is_track_mass_ratio`]).
pub(crate) const TRACK_MASS_RATIO_RULE: &str =
    "a track is too heavy for its chassis: see limits::MAX_TRACK_MASS_RATIO";

/// Whether `ratio` is at most [`MAX_TRACK_MASS_RATIO`]; false for NaN.
pub(crate) fn is_track_mass_ratio(ratio: f64) -> bool {
    ratio <= f64::from(MAX_TRACK_MASS_RATIO)
}

/// The inverse mass and the principal inverse inertia of a dynamic body, in `f64`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PrincipalMass {
    pub(crate) inverse_mass: f64,
    pub(crate) inverse_inertia: [f64; 3],
}

impl PrincipalMass {
    /// The largest lever term of the body at `lever` along any unit direction `d` in the plane
    /// spanned by the orthonormal `first` and `second`, all in the principal frame: the largest
    /// `√((r × d)ᵀ I⁻¹ (r × d))`. An impulse of 1 along `d` at `r` changes the velocity along `d`
    /// at `r` by `inverse_mass + lever²`.
    pub(crate) fn lever_in_plane(&self, lever: [f64; 3], first: [f64; 3], second: [f64; 3]) -> f64 {
        let (a, b) = (cross(lever, first), cross(lever, second));
        let (aa, ab, bb) = (self.form(a, a), self.form(a, b), self.form(b, b));
        let half_gap = 0.5 * (aa - bb);
        (0.5 * (aa + bb) + (half_gap * half_gap + ab * ab).sqrt()).sqrt()
    }

    /// The largest lever term along any direction at any point within 1 m of the centre of
    /// mass: `√` of the largest principal inverse inertia.
    pub(crate) fn lever_per_metre(&self) -> f64 {
        self.inverse_inertia.into_iter().fold(0.0, f64::max).sqrt()
    }

    /// `aᵀ I⁻¹ b`.
    fn form(&self, a: [f64; 3], b: [f64; 3]) -> f64 {
        (0..3).map(|k| self.inverse_inertia[k] * a[k] * b[k]).sum()
    }
}

/// `a × b` in `f64`.
pub(crate) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// `v` scaled to unit length, in `f64`.
pub(crate) fn normalized(v: [f64; 3]) -> [f64; 3] {
    let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    v.map(|c| c / length)
}

/// The lever-arm ratio (see [`MAX_LEVER_ARM_RATIO`]) of a dynamic body with `inverse_mass` and
/// principal inverse inertia `inverse_inertia` at `lever`, given in the body's principal frame;
/// computed in `f64`.
pub(crate) fn lever_arm_ratio(inverse_mass: f32, inverse_inertia: Vec3, lever: Vec3) -> f64 {
    let d = [inverse_inertia.x, inverse_inertia.y, inverse_inertia.z].map(f64::from);
    let r = [lever.x, lever.y, lever.z].map(f64::from);
    let squared = r.iter().map(|c| c * c).sum::<f64>();
    let trace: f64 = (0..3).map(|k| d[k] * (squared - r[k] * r[k])).sum();
    trace / f64::from(inverse_mass)
}

/// The largest lever-arm ratio of the same body at any point within `distance` of its centre of
/// mass: the two largest principal inverse inertias times `distance²`, over the inverse mass.
pub(crate) fn lever_arm_ratio_within(
    inverse_mass: f32,
    inverse_inertia: Vec3,
    distance: f64,
) -> f64 {
    let mut d = [inverse_inertia.x, inverse_inertia.y, inverse_inertia.z].map(f64::from);
    d.sort_by(f64::total_cmp);
    (d[1] + d[2]) * distance * distance / f64::from(inverse_mass)
}

/// Whether `ratio` is at most [`MAX_LEVER_ARM_RATIO`]; false for NaN.
pub(crate) fn is_lever_arm_ratio(ratio: f64) -> bool {
    ratio <= f64::from(MAX_LEVER_ARM_RATIO)
}

/// The length of `v`, computed in `f64` so that it cannot overflow.
pub(crate) fn f64_length(v: Vec3) -> f64 {
    let [x, y, z] = [v.x, v.y, v.z].map(f64::from);
    (x * x + y * y + z * z).sqrt()
}

pub(crate) mod buoyancy;

#[cfg(test)]
mod tests;
