//! The conversions for one glam version, which the parent module names `glam`.

use super::glam;
use crate::math::{Quat, RVec3, Real, Vec3};

mod scalar {
    use super::glam;

    /// glam's three-component vector of a scalar type.
    pub trait GlamScalar {
        /// The vector type.
        type Vec3;
    }

    impl GlamScalar for f32 {
        type Vec3 = glam::Vec3;
    }

    impl GlamScalar for f64 {
        type Vec3 = glam::DVec3;
    }
}

/// glam's vector of [`Real`]: `glam::DVec3` with the `double-precision` feature, `glam::Vec3`
/// otherwise. It follows the scalar type rather than a feature of this crate, so it also holds
/// when only `oxijolt-sys/double-precision` is enabled.
type GlamRVec3 = <Real as scalar::GlamScalar>::Vec3;

impl From<glam::Vec3> for Vec3 {
    fn from(value: glam::Vec3) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl From<Vec3> for glam::Vec3 {
    fn from(value: Vec3) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

/// From glam's vector of [`Real`]: `glam::DVec3` with the `double-precision` feature,
/// `glam::Vec3` otherwise.
impl From<GlamRVec3> for RVec3 {
    fn from(value: GlamRVec3) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

/// To glam's vector of [`Real`]: `glam::DVec3` with the `double-precision` feature,
/// `glam::Vec3` otherwise.
impl From<RVec3> for GlamRVec3 {
    fn from(value: RVec3) -> Self {
        GlamRVec3::new(value.x, value.y, value.z)
    }
}

impl From<glam::Quat> for Quat {
    fn from(value: glam::Quat) -> Self {
        Self::from_xyzw(value.x, value.y, value.z, value.w)
    }
}

impl From<Quat> for glam::Quat {
    fn from(value: Quat) -> Self {
        Self::from_xyzw(value.x, value.y, value.z, value.w)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `f32` values whose bits a lossy conversion would change: negative zero, a subnormal, the
    /// largest finite value and a quiet NaN with a payload.
    const AWKWARD: [f32; 4] = [
        -0.0,
        f32::MIN_POSITIVE / 2.0,
        f32::MAX,
        f32::from_bits(0x7fc0_1234),
    ];

    /// The same kinds of values in [`Real`].
    fn awkward_real() -> [Real; 4] {
        [
            -0.0,
            Real::MIN_POSITIVE / 2.0,
            Real::MAX,
            Real::from_bits(Real::NAN.to_bits() | 0x1234),
        ]
    }

    #[test]
    fn vec3_round_trips_through_glam_with_the_same_bits() {
        for value in AWKWARD {
            let v = Vec3::new(value, -1.5, value);
            let there = glam::Vec3::from(v);
            assert_eq!(
                there.to_array().map(f32::to_bits),
                <[f32; 3]>::from(v).map(f32::to_bits)
            );
            let back = Vec3::from(there);
            assert_eq!(
                <[f32; 3]>::from(back).map(f32::to_bits),
                <[f32; 3]>::from(v).map(f32::to_bits)
            );
        }
    }

    #[test]
    fn rvec3_round_trips_through_glam_with_the_same_bits() {
        for value in awkward_real() {
            let p = RVec3::new(value, 2.25, value);
            let there = GlamRVec3::from(p);
            assert_eq!(
                there.to_array().map(Real::to_bits),
                <[Real; 3]>::from(p).map(Real::to_bits)
            );
            let back = RVec3::from(there);
            assert_eq!(
                <[Real; 3]>::from(back).map(Real::to_bits),
                <[Real; 3]>::from(p).map(Real::to_bits)
            );
        }
    }

    #[test]
    fn rvec3_converts_to_glams_vector_of_real() {
        let there: GlamRVec3 = RVec3::new(1.0, 2.0, 3.0).into();
        let expected: [Real; 3] = [1.0, 2.0, 3.0];
        assert_eq!(there.to_array(), expected);
    }

    #[test]
    fn quat_round_trips_through_glam_with_the_same_bits() {
        for value in AWKWARD {
            let q = Quat::from_xyzw(value, 0.5, value, -0.0);
            let there = glam::Quat::from(q);
            assert_eq!(
                there.to_array().map(f32::to_bits),
                <[f32; 4]>::from(q).map(f32::to_bits)
            );
            let back = Quat::from(there);
            assert_eq!(
                <[f32; 4]>::from(back).map(f32::to_bits),
                <[f32; 4]>::from(q).map(f32::to_bits)
            );
        }
    }

    #[test]
    fn quat_conversion_does_not_normalize() {
        let q = Quat::from_xyzw(0.0, 0.0, 0.0, 2.0);
        assert_eq!(glam::Quat::from(q).to_array(), [0.0, 0.0, 0.0, 2.0]);
        assert_eq!(Quat::from(glam::Quat::from_xyzw(0.0, 0.0, 0.0, 2.0)), q);
    }
}
