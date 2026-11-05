//! An infinite plane cut to a square, for static ground.

use std::ptr::null;

use oxijolt_sys::*;

use super::{initialize, Shape, ShapeSettings};
use crate::math::is_unit;
use crate::{limits, PhysicsMaterial, ShapeError, Vec3};

/// The checks of [`Shape::new_plane`] that come before Jolt's bounds.
fn validate_plane(normal: Vec3, constant: f32, half_extent: f32) -> Result<(), ShapeError> {
    if !is_unit(normal) {
        return Err(ShapeError::InvalidValue(
            "plane normal must be a finite unit vector",
        ));
    }
    if !(constant.is_finite() && constant.abs() <= limits::MAX_SHAPE_EXTENT) {
        return Err(ShapeError::InvalidValue(
            "plane constant must be finite and within limits::MAX_SHAPE_EXTENT",
        ));
    }
    if !(half_extent > 0.0 && half_extent <= limits::MAX_SHAPE_EXTENT) {
        return Err(ShapeError::InvalidValue(
            "plane half extent must be positive and within limits::MAX_SHAPE_EXTENT",
        ));
    }
    Ok(())
}

impl Shape {
    /// A plane `normal · p + constant = 0` in the shape's frame (Jolt `PlaneShape`); everything
    /// on the side away from the normal, `normal · p + constant < 0`, is solid.
    ///
    /// The plane is only infinite in name: it ends at a square of `2 * half_extent` metres
    /// around the point `-constant * normal`, and its bounds reach `half_extent` metres behind
    /// it. Jolt does not collide anything outside these bounds and gives inconsistent contacts
    /// at their edge, so `half_extent` should leave room around where bodies will be (and stay
    /// small, for the broad phase).
    ///
    /// Only static bodies, and compounds or decorators on static bodies, may use a plane: Jolt
    /// marks it `MustBeStatic`, it has no volume or mass, and Jolt cannot collide it with meshes,
    /// heightfields or other planes. It collides with convex shapes (also as compound or decorated
    /// children), soft bodies and characters, and ray and shape casts hit it. It cannot be scaled
    /// ([`new_scaled`](Self::new_scaled) refuses it), and query, character and ragdoll shapes
    /// refuse it. A ray that starts behind the plane hits it at fraction 0 (solid, `<= 0`), while a
    /// point query reports only points strictly behind it (`< 0`).
    ///
    /// `normal` must be a finite unit vector, `constant` finite and at most
    /// [`limits::MAX_SHAPE_EXTENT`] in absolute value, `half_extent` positive and at most
    /// [`limits::MAX_SHAPE_EXTENT`], and the bounds within [`limits::MAX_SHAPE_EXTENT`] on
    /// every axis; otherwise [`ShapeError::InvalidValue`] or, for the normal,
    /// [`ShapeError::InvalidValue`]. For a normal along an axis the bounds reach
    /// `max(|constant|, |constant + half_extent|)` along it: with normal +Y a half extent of
    /// 2000 m fits for `constant = -1` (plane at y = 1) but not for `constant = 1`.
    ///
    /// ```
    /// # use oxijolt::*;
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// // Ground at y = 0, solid below, 500 m in every direction.
    /// let ground = Shape::new_plane(Vec3::new(0.0, 1.0, 0.0), 0.0, 500.0)?;
    /// world.create_body(&ground, &BodySettings::new_static())?;
    /// # Ok::<(), oxijolt::error::Error>(())
    /// ```
    pub fn new_plane(normal: Vec3, constant: f32, half_extent: f32) -> Result<Self, ShapeError> {
        Self::plane(normal, constant, half_extent, None)
    }

    /// [`new_plane`](Self::new_plane) made of `material`; the same rules apply.
    pub fn new_plane_with_material(
        normal: Vec3,
        constant: f32,
        half_extent: f32,
        material: &PhysicsMaterial,
    ) -> Result<Self, ShapeError> {
        Self::plane(normal, constant, half_extent, Some(material))
    }

    fn plane(
        normal: Vec3,
        constant: f32,
        half_extent: f32,
        material: Option<&PhysicsMaterial>,
    ) -> Result<Self, ShapeError> {
        validate_plane(normal, constant, half_extent)?;
        initialize()?;
        let plane = JPH_Plane {
            normal: normal.to_jph(),
            distance: constant,
        };
        let material = material.map_or(null(), PhysicsMaterial::as_ptr);
        // SAFETY: Jolt is initialised, `plane` is a live local and `material` null or live for
        // the call; the settings take their own reference to it. The returned settings hold one
        // reference, which the guard takes over.
        let settings = unsafe {
            ShapeSettings::from_raw(
                JPH_PlaneShapeSettings_Create(&plane, material, half_extent).cast(),
            )
        }?;
        settings.create()?.within_extent_bounds()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounds_reach_half_extent_around_the_plane_and_behind_it() {
        let up = Vec3::new(0.0, 1.0, 0.0);
        let (min, max) = Shape::new_plane(up, -1.0, 2000.0).unwrap().local_bounds();
        assert_eq!(
            (min, max),
            (
                Vec3::new(-2000.0, -1999.0, -2000.0),
                Vec3::new(2000.0, 1.0, 2000.0)
            )
        );
        let down = Vec3::new(0.0, -1.0, 0.0);
        let (min, max) = Shape::new_plane(down, 1.0, 1999.0).unwrap().local_bounds();
        assert_eq!((min.y, max.y), (1.0, 2000.0));

        let s = std::f32::consts::FRAC_1_SQRT_2;
        let half_extent = 2000.0 * s * (1.0 - 1.0e-5);
        let (min, max) = Shape::new_plane(Vec3::new(s, s, 0.0), 0.0, half_extent)
            .unwrap()
            .local_bounds();
        assert!(min.x < -1999.9 && min.y < -1999.9, "{min:?}");
        assert!(max.x <= 2000.0 && max.y <= 2000.0, "{max:?}");
        assert!((max.z - half_extent).abs() < 1.0e-3 && (min.z + half_extent).abs() < 1.0e-3);
    }
}
