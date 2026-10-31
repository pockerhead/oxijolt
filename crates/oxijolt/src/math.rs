//! Plain value types for vectors and rotations.
//!
//! Jolt uses a right-handed coordinate system with Y up. Lengths are in metres, velocities in
//! metres per second and angular velocities in radians per second. Conversions to and from
//! Jolt copy the fields and never round, so values read back with the same bits.

use oxijolt_sys::{
    JPH_Mat4, JPH_Mat4_RotationTranslation, JPH_Quat, JPH_Quat_GetAxisAngle, JPH_Quat_Multiply,
    JPH_Quat_Rotate, JPH_RVec3, JPH_Vec3, JPH_Vec3_Length, JPH_Vec3_LengthSquared, JPH_Vec4,
};

/// Scalar type of world positions: `f64` with the `double-precision` feature, `f32` otherwise.
pub use oxijolt_sys::Real;

/// Whether `value` is finite and positive.
pub(crate) fn is_finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

/// Whether `value` is finite and not negative.
pub(crate) fn is_finite_non_negative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

/// The length of `v` as Jolt computes it (`Vec3::Length`), the expression Jolt compares with a
/// body's maximum velocity when it asserts at creation.
pub(crate) fn jolt_length(v: Vec3) -> f32 {
    let v = v.to_jph();
    // SAFETY: a pure function of a live local; it needs no initialization.
    unsafe { JPH_Vec3_Length(&v) }
}

/// `q * v` as Jolt computes it (`Quat::operator*(Vec3)`), so the result has Jolt's bits.
pub(crate) fn jolt_rotate(q: Quat, v: Vec3) -> Vec3 {
    let (q, v) = (q.to_jph(), v.to_jph());
    let mut result = Vec3::ZERO.to_jph();
    // SAFETY: a pure function of live locals that writes one live local; it needs no
    // initialization.
    unsafe { JPH_Quat_Rotate(&q, &v, &mut result) };
    Vec3::from_jph(result)
}

/// The product `a * b` as Jolt computes it (`Quat::operator*`), so the result has Jolt's bits.
pub(crate) fn jolt_product(a: Quat, b: Quat) -> Quat {
    let (a, b) = (a.to_jph(), b.to_jph());
    let mut result = Quat::IDENTITY.to_jph();
    // SAFETY: a pure function of live locals that writes one live local; it needs no
    // initialization.
    unsafe { JPH_Quat_Multiply(&a, &b, &mut result) };
    Quat::from_jph(result)
}

/// A matrix of zeros, to be overwritten by Jolt.
pub(crate) fn zero_matrix() -> JPH_Mat4 {
    let column = JPH_Vec4 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 0.0,
    };
    JPH_Mat4 {
        column: [column; 4],
    }
}

/// The matrix of a validated rigid transform. The rotation is normalised first: a rotation
/// within Rust's unit tolerance can be just outside Jolt's, which sums the squares in another
/// order.
pub(crate) fn matrix(translation: Vec3, rotation: Quat) -> JPH_Mat4 {
    let mut result = zero_matrix();
    // SAFETY: joltc only reads the two live locals and writes `result`; the rotation is a unit
    // quaternion within Jolt's tolerance, as `Mat44::sRotation` asserts.
    unsafe {
        JPH_Mat4_RotationTranslation(
            &mut result,
            &rotation.normalized().to_jph(),
            &translation.to_jph(),
        )
    };
    result
}

/// The inverse of the validated rigid transform `matrix(translation, rotation)`:
/// `matrix(-(q⁻¹ · t), q⁻¹)` with `q` the normalised rotation.
pub(crate) fn inverse_rigid_matrix(translation: Vec3, rotation: Quat) -> JPH_Mat4 {
    let inverse = rotation.normalized().conjugated();
    matrix(inverse.rotate(translation).scale(-1.0), inverse)
}

/// The angular velocity that turns by `rotation` in `delta_time` seconds, as Jolt's
/// `Quat::GetAngularVelocity` computes it (`Quat.inl:186-204`): the same branches, with Jolt's own
/// squared length and angle, so the result has Jolt's bits. `None` when `rotation` is not a
/// finite unit quaternion within Jolt's tolerance, for which Jolt asserts.
pub(crate) fn jolt_angular_velocity(rotation: Quat, delta_time: f32) -> Option<Vec3> {
    if !rotation.is_valid_rotation() {
        return None;
    }
    // Jolt's `EnsureWPositive` flips every sign when the sign bit of w is set.
    let q = if rotation.w.is_sign_negative() {
        Quat::from_xyzw(-rotation.x, -rotation.y, -rotation.z, -rotation.w)
    } else {
        rotation
    };
    let xyz = Vec3::new(q.x, q.y, q.z);
    let jph_xyz = xyz.to_jph();
    // SAFETY: a pure function of a live local; it needs no initialization.
    let length_squared = unsafe { JPH_Vec3_LengthSquared(&jph_xyz) };
    if length_squared < 4.0e-4 {
        return Some(xyz.scale(2.0 / delta_time));
    }
    let (jph_q, mut axis, mut angle) = (q.to_jph(), Vec3::ZERO.to_jph(), 0.0_f32);
    // SAFETY: reads one live local and writes two; `q` is normalized within Jolt's tolerance
    // (checked above), so Jolt's `IsNormalized` assertion holds. It needs no initialization.
    unsafe { JPH_Quat_GetAxisAngle(&jph_q, &mut axis, &mut angle) };
    let divisor = length_squared.sqrt() * delta_time;
    Some(Vec3::new(
        xyz.x / divisor * angle,
        xyz.y / divisor * angle,
        xyz.z / divisor * angle,
    ))
}

/// Tolerance of the unit-length checks: `|v·v − 1|` at most this, half of Jolt's
/// `Vec3::IsNormalized` tolerance 1e-6, so a rounding difference between the check here and
/// Jolt's cannot let a vector through that Jolt's assertion rejects.
pub(crate) const UNIT_TOLERANCE: f32 = 5.0e-7;

/// Whether `v` is finite and of unit length within [`UNIT_TOLERANCE`].
pub(crate) fn is_unit(v: Vec3) -> bool {
    v.is_finite() && (v.dot(v) - 1.0).abs() <= UNIT_TOLERANCE
}

/// A 3D vector of `f32`, used for directions, velocities, forces and extents.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    /// X component.
    pub x: f32,
    /// Y component (up).
    pub y: f32,
    /// Z component.
    pub z: f32,
}

impl Vec3 {
    /// The zero vector.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    /// Creates a vector from its components.
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub(crate) fn to_jph(self) -> JPH_Vec3 {
        JPH_Vec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }

    pub(crate) fn from_jph(value: JPH_Vec3) -> Self {
        Self::new(value.x, value.y, value.z)
    }

    pub(crate) fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    pub(crate) fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub(crate) fn length(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub(crate) fn scale(self, factor: f32) -> Self {
        Self::new(self.x * factor, self.y * factor, self.z * factor)
    }

    /// The unit vector along `self`, or zero when `self` is too short to have a direction.
    pub(crate) fn normalized_or_zero(self) -> Self {
        let length_squared = self.dot(self);
        if length_squared <= 1.0e-12 {
            Self::ZERO
        } else {
            self.scale(1.0 / length_squared.sqrt())
        }
    }

    fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl From<[f32; 3]> for Vec3 {
    fn from([x, y, z]: [f32; 3]) -> Self {
        Self::new(x, y, z)
    }
}

impl From<Vec3> for [f32; 3] {
    fn from(value: Vec3) -> Self {
        [value.x, value.y, value.z]
    }
}

/// A world-space position. Its components are [`Real`]: `f64` with the `double-precision`
/// feature, `f32` otherwise.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RVec3 {
    /// X component.
    pub x: Real,
    /// Y component (up).
    pub y: Real,
    /// Z component.
    pub z: Real,
}

impl RVec3 {
    /// The origin.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    /// Creates a position from its components.
    pub const fn new(x: Real, y: Real, z: Real) -> Self {
        Self { x, y, z }
    }

    pub(crate) fn to_jph(self) -> JPH_RVec3 {
        JPH_RVec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }

    pub(crate) fn from_jph(value: JPH_RVec3) -> Self {
        Self::new(value.x, value.y, value.z)
    }

    pub(crate) fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl From<[Real; 3]> for RVec3 {
    fn from([x, y, z]: [Real; 3]) -> Self {
        Self::new(x, y, z)
    }
}

impl From<RVec3> for [Real; 3] {
    fn from(value: RVec3) -> Self {
        [value.x, value.y, value.z]
    }
}

/// A rotation quaternion `(x, y, z, w)` with `w` the scalar part. Jolt expects unit length.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    /// X component of the vector part.
    pub x: f32,
    /// Y component of the vector part.
    pub y: f32,
    /// Z component of the vector part.
    pub z: f32,
    /// Scalar part.
    pub w: f32,
}

/// What a caller-given rotation must satisfy ([`Quat::is_valid_rotation`]).
pub(crate) const ROTATION_RULE: &str = "rotation must be a finite unit quaternion";

impl Quat {
    /// The rotation that does nothing.
    pub const IDENTITY: Self = Self::from_xyzw(0.0, 0.0, 0.0, 1.0);

    /// Creates a quaternion from its components; `w` is the scalar part.
    pub const fn from_xyzw(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    pub(crate) fn to_jph(self) -> JPH_Quat {
        JPH_Quat {
            x: self.x,
            y: self.y,
            z: self.z,
            w: self.w,
        }
    }

    pub(crate) fn from_jph(value: JPH_Quat) -> Self {
        Self::from_xyzw(value.x, value.y, value.z, value.w)
    }

    pub(crate) fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite() && self.w.is_finite()
    }

    /// Unit length within the tolerance of Jolt's `Quat::IsNormalized` (`|length² − 1| <= 1e-5`).
    pub(crate) fn is_normalized(&self) -> bool {
        let length_squared = self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w;
        (length_squared - 1.0).abs() <= 1.0e-5
    }

    /// Whether this is a finite unit quaternion, within Jolt's tolerance: what Jolt expects of
    /// every rotation.
    pub(crate) fn is_valid_rotation(&self) -> bool {
        self.is_finite() && self.is_normalized()
    }

    /// `v` rotated by this unit quaternion: `v + 2w (q x v) + 2 q x (q x v)`.
    pub(crate) fn rotate(self, v: Vec3) -> Vec3 {
        let q = Vec3::new(self.x, self.y, self.z);
        let t = q.cross(v).scale(2.0);
        v.add(t.scale(self.w)).add(q.cross(t))
    }

    /// The position `p` rotated by this unit quaternion, with the same formula as
    /// [`rotate`](Self::rotate) computed in [`Real`].
    pub(crate) fn rotate_real(self, p: RVec3) -> RVec3 {
        let [x, y, z, w] = [self.x, self.y, self.z, self.w].map(Real::from);
        let cross = |a: [Real; 3], b: [Real; 3]| {
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        };
        let q = [x, y, z];
        let v = [p.x, p.y, p.z];
        let t = cross(q, v).map(|value| 2.0 * value);
        let u = cross(q, t);
        RVec3::new(
            v[0] + w * t[0] + u[0],
            v[1] + w * t[1] + u[1],
            v[2] + w * t[2] + u[2],
        )
    }

    /// The Hamilton product `self * rhs`: the rotation that applies `rhs` first, then `self`
    /// (the order of Jolt's `Quat::operator*`).
    pub(crate) fn product(self, rhs: Quat) -> Quat {
        let (a, b) = (self, rhs);
        Quat::from_xyzw(
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        )
    }

    /// The conjugate: the vector part negated. For a unit quaternion, the inverse rotation.
    pub(crate) fn conjugated(self) -> Quat {
        Quat::from_xyzw(-self.x, -self.y, -self.z, self.w)
    }

    /// This quaternion divided by its length. Non-finite when the length is zero or not
    /// finite; callers check the result with `is_valid_rotation`.
    pub(crate) fn normalized(self) -> Quat {
        let length = (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt();
        Quat::from_xyzw(
            self.x / length,
            self.y / length,
            self.z / length,
            self.w / length,
        )
    }
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl From<[f32; 4]> for Quat {
    /// Takes the components in `[x, y, z, w]` order.
    fn from([x, y, z, w]: [f32; 4]) -> Self {
        Self::from_xyzw(x, y, z, w)
    }
}

impl From<Quat> for [f32; 4] {
    /// Returns the components in `[x, y, z, w]` order.
    fn from(value: Quat) -> Self {
        [value.x, value.y, value.z, value.w]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Values whose bits a lossy conversion would change.
    const AWKWARD: [f32; 4] = [
        -0.0,
        f32::MIN_POSITIVE / 2.0,
        f32::MAX,
        f32::from_bits(0x7fc0_1234),
    ];

    fn bits3(v: [f32; 3]) -> [u32; 3] {
        v.map(f32::to_bits)
    }

    #[test]
    fn vec3_round_trips_through_jolt_with_the_same_bits() {
        for value in AWKWARD {
            let v = Vec3::new(value, -1.5, value);
            let back = Vec3::from_jph(v.to_jph());
            assert_eq!(bits3(back.into()), bits3(v.into()));
        }
    }

    #[test]
    fn rvec3_round_trips_through_jolt_with_the_same_bits() {
        for value in AWKWARD {
            let v = RVec3::new(Real::from(value), 2.25, Real::from(value));
            let back: [Real; 3] = RVec3::from_jph(v.to_jph()).into();
            let original: [Real; 3] = v.into();
            assert_eq!(back.map(Real::to_bits), original.map(Real::to_bits));
        }
    }

    #[test]
    fn quat_round_trips_through_jolt_with_the_same_bits() {
        for value in AWKWARD {
            let q = Quat::from_xyzw(value, 0.5, value, -0.0);
            let back: [f32; 4] = Quat::from_jph(q.to_jph()).into();
            let original: [f32; 4] = q.into();
            assert_eq!(back.map(f32::to_bits), original.map(f32::to_bits));
        }
    }

    #[test]
    fn quat_defaults_to_identity() {
        assert_eq!(Quat::default(), Quat::IDENTITY);
    }

    #[test]
    fn is_normalized_uses_jolts_tolerance() {
        assert!(Quat::IDENTITY.is_normalized());
        let half = std::f32::consts::FRAC_1_SQRT_2;
        assert!(Quat::from_xyzw(0.0, half, 0.0, half).is_normalized());
        assert!(Quat::from_xyzw(0.5, 0.5, 0.5, 0.5).is_normalized());
        assert!(!Quat::from_xyzw(0.0, 0.0, 0.0, 2.0).is_normalized());
    }

    #[test]
    fn valid_rotations_are_finite_unit_quaternions() {
        assert!(Quat::IDENTITY.is_valid_rotation());
        assert!(!Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0).is_valid_rotation());
        assert!(!Quat::from_xyzw(0.0, 0.0, 0.0, 2.0).is_valid_rotation());
    }

    #[test]
    fn rotate_turns_x_into_minus_z_about_y() {
        let half = std::f32::consts::FRAC_1_SQRT_2;
        let quarter_turn_about_y = Quat::from_xyzw(0.0, half, 0.0, half);
        let v = quarter_turn_about_y.rotate(Vec3::new(1.0, 0.0, 0.0));
        assert!(v.x.abs() < 1e-6, "{v:?}");
        assert!(v.y.abs() < 1e-6, "{v:?}");
        assert!((v.z + 1.0).abs() < 1e-6, "{v:?}");
        let w = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(Quat::IDENTITY.rotate(w), w);
    }

    fn assert_quat_near(a: Quat, b: Quat, tolerance: f32) {
        let (a, b): ([f32; 4], [f32; 4]) = (a.into(), b.into());
        for (x, y) in a.into_iter().zip(b) {
            assert!((x - y).abs() <= tolerance, "{a:?} vs {b:?}");
        }
    }

    fn assert_vec_near(a: Vec3, b: Vec3, tolerance: f32) {
        let (a, b): ([f32; 3], [f32; 3]) = (a.into(), b.into());
        for (x, y) in a.into_iter().zip(b) {
            assert!((x - y).abs() <= tolerance, "{a:?} vs {b:?}");
        }
    }

    #[test]
    fn product_composes_rotations() {
        let half = std::f32::consts::FRAC_1_SQRT_2;
        let quarter_turn_about_y = Quat::from_xyzw(0.0, half, 0.0, half);
        let half_turn_about_y = Quat::from_xyzw(0.0, 1.0, 0.0, 0.0);
        assert_quat_near(
            quarter_turn_about_y.product(quarter_turn_about_y),
            half_turn_about_y,
            1e-6,
        );

        let a = Quat::from_xyzw(0.1, 0.2, 0.3, 0.9).normalized();
        let b = Quat::from_xyzw(-0.4, 0.1, 0.5, 0.7).normalized();
        let conjugate = Quat::from_xyzw(-a.x, -a.y, -a.z, a.w);
        assert_quat_near(a.product(conjugate), Quat::IDENTITY, 1e-6);

        let v = Vec3::new(1.5, -2.0, 0.25);
        assert_vec_near(a.product(b).rotate(v), a.rotate(b.rotate(v)), 1e-5);
    }

    #[test]
    fn conjugated_is_the_inverse_rotation() {
        let q = Quat::from_xyzw(0.1, -0.2, 0.3, 0.9).normalized();
        assert_quat_near(q.product(q.conjugated()), Quat::IDENTITY, 1e-6);
        assert_quat_near(q.conjugated().product(q), Quat::IDENTITY, 1e-6);
        assert_eq!(q.conjugated().w.to_bits(), q.w.to_bits());
    }

    #[test]
    fn rotate_real_matches_rotate() {
        let q = Quat::from_xyzw(0.1, 0.2, 0.3, 0.9).normalized();
        let v = Vec3::new(1.5, -2.0, 0.25);
        let p = q.rotate_real(RVec3::new(1.5, -2.0, 0.25));
        let expected = q.rotate(v);
        for (actual, expected) in [(p.x, expected.x), (p.y, expected.y), (p.z, expected.z)] {
            assert!(
                (actual - Real::from(expected)).abs() <= 1e-5,
                "{p:?} {expected:?}"
            );
        }
    }

    #[test]
    fn normalized_divides_by_the_length() {
        assert_eq!(
            Quat::from_xyzw(0.0, 0.0, 0.0, 2.0).normalized(),
            Quat::IDENTITY
        );
        assert!(!Quat::from_xyzw(0.0, 0.0, 0.0, 0.0)
            .normalized()
            .is_valid_rotation());
    }

    #[test]
    fn jolt_rotate_and_product_agree_with_ours() {
        let a = Quat::from_xyzw(0.1, 0.2, 0.3, 0.9).normalized();
        let b = Quat::from_xyzw(-0.4, 0.1, 0.5, 0.7).normalized();
        let v = Vec3::new(1.5, -2.0, 0.25);
        assert_vec_near(jolt_rotate(a, v), a.rotate(v), 1e-5);
        assert_quat_near(jolt_product(a, b), a.product(b), 1e-6);
        assert_eq!(jolt_rotate(Quat::IDENTITY, v), v);
    }

    #[test]
    fn jolt_angular_velocity_follows_both_branches() {
        let dt = 0.5;
        // Small angles: `(2 / dt) * xyz`, exactly.
        let small = Quat::from_xyzw(0.0, 0.01, 0.0, (1.0_f32 - 1.0e-4).sqrt());
        assert_eq!(
            jolt_angular_velocity(small, dt),
            Some(Vec3::new(0.0, 0.01 * (2.0 / dt), 0.0))
        );
        // A quarter turn about x in half a second, also with w negated (the same rotation).
        let half = std::f32::consts::FRAC_1_SQRT_2;
        for q in [
            Quat::from_xyzw(half, 0.0, 0.0, half),
            Quat::from_xyzw(-half, -0.0, -0.0, -half),
        ] {
            let w = jolt_angular_velocity(q, dt).unwrap();
            assert_vec_near(w, Vec3::new(std::f32::consts::PI, 0.0, 0.0), 1e-5);
        }
        assert_eq!(
            jolt_angular_velocity(Quat::from_xyzw(0.0, 0.0, 0.0, 2.0), dt),
            None
        );
        assert_eq!(jolt_angular_velocity(Quat::IDENTITY, dt), Some(Vec3::ZERO));
    }

    #[test]
    fn vector_helpers() {
        let v = Vec3::new(3.0, 0.0, 4.0);
        assert_eq!(v.dot(Vec3::new(1.0, 1.0, 1.0)), 7.0);
        assert_eq!(v.length(), 5.0);
        assert_eq!(jolt_length(v), 5.0);
        assert_eq!(jolt_length(Vec3::new(f32::MAX, 0.0, 0.0)), f32::INFINITY);
        assert!(jolt_length(Vec3::new(f32::NAN, 0.0, 0.0)).is_nan());
        assert_eq!(v.scale(2.0), Vec3::new(6.0, 0.0, 8.0));
        assert_eq!(v.normalized_or_zero(), Vec3::new(0.6, 0.0, 0.8));
        assert_eq!(Vec3::ZERO.normalized_or_zero(), Vec3::ZERO);
        assert_eq!(Vec3::new(1e-7, 0.0, 0.0).normalized_or_zero(), Vec3::ZERO);
    }
}
