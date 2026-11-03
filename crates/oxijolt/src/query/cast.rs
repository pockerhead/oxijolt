//! Shape casts: a shape moved along a direction until it hits an obstacle.

use std::ffi::c_void;
use std::fmt;
use std::mem::MaybeUninit;
use std::ptr::null;

use oxijolt_sys::*;

use super::{offset_from, rotation_translation, validate_pose, HitsSlot};
use crate::filter::with_query_filters;
use crate::limits;
use crate::math::is_finite_non_negative;
use crate::{
    BodyId, CompoundSubShape, ObjectLayer, PhysicsWorld, Quat, QueryError, QueryFilter, RVec3,
    Shape, SubShapeId, Vec3,
};

/// A shape moved from `position` along `direction`, for [`PhysicsWorld::cast_shape`] and
/// [`PhysicsWorld::cast_shape_all`].
///
/// The shape's origin starts at `position` with `rotation` and moves by `direction`, whose
/// length is the cast's length in metres. Hits report the fraction of `direction` covered.
#[derive(Clone, Copy)]
pub struct ShapeCast<'a> {
    shape: &'a Shape,
    position: RVec3,
    rotation: Quat,
    direction: Vec3,
    target_distance: f32,
    return_deepest_point: bool,
    back_faces: bool,
}

impl fmt::Debug for ShapeCast<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ShapeCast")
            .field("position", &self.position)
            .field("rotation", &self.rotation)
            .field("direction", &self.direction)
            .field("target_distance", &self.target_distance)
            .field("return_deepest_point", &self.return_deepest_point)
            .field("back_faces", &self.back_faces)
            .finish_non_exhaustive()
    }
}

impl<'a> ShapeCast<'a> {
    /// Largest [`target_distance`](Self::target_distance) accepted, in metres, inclusive.
    ///
    /// A oxijolt guard, not a Jolt limit: the reported
    /// [`ShapeCastHit::penetration_depth`] is Jolt's depth minus the target distance, which
    /// loses millimetre precision in `f32` well beyond a kilometre and overflows to infinity for
    /// huge values.
    pub const MAX_TARGET_DISTANCE: f32 = 1000.0;

    /// Casts `shape`, whose origin starts at `position` with `rotation`, along `direction`
    /// (metres). No target distance, no deepest point, back faces ignored.
    pub fn new(shape: &'a Shape, position: RVec3, rotation: Quat, direction: Vec3) -> Self {
        Self {
            shape,
            position,
            rotation,
            direction,
            target_distance: 0.0,
            return_deepest_point: false,
            back_faces: false,
        }
    }

    /// Stops the cast `metres` before the shape would touch an obstacle, finite, not negative and
    /// at most [`MAX_TARGET_DISTANCE`](Self::MAX_TARGET_DISTANCE). Default 0. Only sphere and
    /// capsule query shapes support a target distance above 0: the cast uses a copy of the shape
    /// whose radius is grown by `metres`, which allocates one temporary shape per cast.
    #[must_use]
    pub fn target_distance(mut self, metres: f32) -> Self {
        self.target_distance = metres;
        self
    }

    /// When the shape already overlaps an obstacle at the start, searches for the deepest
    /// penetration so depth and normal describe the real overlap (Jolt's
    /// `ShapeCastSettings::mReturnDeepestPoint`; costs extra time). Default `false`.
    #[must_use]
    pub fn return_deepest_point(mut self, value: bool) -> Self {
        self.return_deepest_point = value;
        self
    }

    /// Also reports back-facing hits: triangles hit from behind and convex obstacles the shape
    /// starts inside and moves out of. Default `false`.
    #[must_use]
    pub fn collide_with_back_faces(mut self, value: bool) -> Self {
        self.back_faces = value;
        self
    }
}

/// A hit of a [`PhysicsWorld::cast_shape`] or [`PhysicsWorld::cast_shape_all`].
///
/// `normal` is the outward surface normal of the obstacle in world space: a floor below the
/// cast gives `normal . up > 0`, a ceiling above it `normal . up < 0`.
///
/// `fraction == 0.0` means the shape at its start position already penetrates the obstacle or
/// lies within the target distance of it. Then `penetration_depth > 0` is a real overlap of the
/// shape (without target distance), and `-target_distance < penetration_depth <= 0` means the
/// shape is `-penetration_depth` metres from the obstacle. At `fraction > 0`,
/// `penetration_depth` is `-target_distance`. When the core of the shape (the segment of a
/// capsule, the centre of a sphere) overlaps the obstacle, depth and normal are reliable only
/// with [`ShapeCast::return_deepest_point`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct ShapeCastHit {
    /// The body that was hit.
    pub body: BodyId,
    /// Path from the body's shape to the leaf shape that was hit.
    pub sub_shape_id: SubShapeId,
    /// The object layer of the hit body.
    pub object_layer: ObjectLayer,
    /// The compound child that was hit, when the body's shape is a compound.
    pub compound_child: Option<CompoundSubShape>,
    /// Fraction of the cast's direction the shape moves before it stops, in `[0, 1]`.
    pub fraction: f32,
    /// Distance the shape moves before it stops, metres: `fraction` times the direction's
    /// length.
    pub distance: f32,
    /// Contact point on the obstacle in world space.
    pub point: RVec3,
    /// Outward surface normal of the obstacle in world space, unit length, or zero when Jolt
    /// reports no direction (exactly touching cores).
    pub normal: Vec3,
    /// How far the shape overlaps the obstacle at the hit, metres; see [`ShapeCastHit`].
    pub penetration_depth: f32,
}

/// Result callback of [`PhysicsWorld::cast_shape`] and [`PhysicsWorld::cast_shape_all`].
///
/// # Safety
/// Called only by joltc during a shape cast, with that call's live
/// `HitsSlot<JPH_ShapeCastResult>` as `user_data` and a live result.
unsafe extern "C" fn push_shape_cast_hit(
    user_data: *mut c_void,
    result: *const JPH_ShapeCastResult,
) {
    // SAFETY: guaranteed by the caller (function contract).
    unsafe { HitsSlot::<JPH_ShapeCastResult>::push(user_data, result) }
}

/// Jolt's measure of how close a shape cast hit is (`ShapeCastResult::GetEarlyOutFraction`):
/// the fraction, or at fraction 0 minus the penetration depth, so deeper start overlaps come
/// first.
fn early_out_fraction(hit: &JPH_ShapeCastResult) -> f32 {
    if hit.fraction > 0.0 {
        hit.fraction
    } else {
        -hit.penetrationDepth
    }
}

/// joltc's shape cast settings with Jolt's defaults.
fn default_cast_settings() -> JPH_ShapeCastSettings {
    let mut settings = MaybeUninit::<JPH_ShapeCastSettings>::uninit();
    // SAFETY: joltc writes every field of the settings it is given (it clears them first).
    // A null pointer would select `CollideWithAll` and back faces instead of Jolt's defaults.
    unsafe {
        JPH_ShapeCastSettings_Init(settings.as_mut_ptr());
        settings.assume_init()
    }
}

impl PhysicsWorld {
    /// The first obstacle among those `filter` selects that `cast`'s shape hits while moving
    /// along its direction, if any.
    ///
    /// The shape's origin moves `fraction * direction`. Of several obstacles the shape overlaps
    /// at the start, the deepest is returned. By default a cast that starts inside a convex
    /// obstacle and moves out of it does not hit it; see [`ShapeCast::collide_with_back_faces`].
    /// See [`ShapeCastHit`] for the normal and depth conventions.
    ///
    /// When several hits share the closest fraction (and at fraction 0 the depth), the one
    /// returned follows Jolt's traversal and may depend on the broad phase's history;
    /// [`cast_shape_all`](Self::cast_shape_all) orders such hits by body id and sub-shape id.
    ///
    /// The position must be finite and within [`limits::MAX_POSITION`], the rotation a finite unit
    /// quaternion, the direction finite, not zero and at most `2 *` [`limits::MAX_POSITION`] per
    /// component, the target distance finite, not negative and at most
    /// [`ShapeCast::MAX_TARGET_DISTANCE`] (and 0 unless the shape is a sphere or capsule), the
    /// shape not static-only (a mesh, heightfield or plane); otherwise
    /// [`QueryError::InvalidValue`] is returned.
    /// A filter that names an object layer this world does not have, or a body of another
    /// world, gives [`QueryError::UnknownObjectLayer`] or [`QueryError::WrongWorld`].
    pub fn cast_shape(
        &self,
        cast: &ShapeCast<'_>,
        filter: &QueryFilter<'_>,
    ) -> Result<Option<ShapeCastHit>, QueryError> {
        let hits = self.cast_shape_with(cast, filter, JPH_CollisionCollectorType_ClosestHit)?;
        Ok(hits.into_iter().next())
    }

    /// Every hit of `cast`'s shape on the bodies `filter` selects, in a fixed order that does not
    /// depend on the broad phase's history.
    ///
    /// Hits are sorted by fraction, then body id, then sub-shape id on the body, then the
    /// sub-shape id of the query shape's leaf (two leaves of a compound query shape can hit the
    /// same obstacle at the same fraction). Hits at fraction 0 (start overlaps) are ordered among
    /// themselves by penetration depth, deepest first, as Jolt's closest-hit collector ranks them
    /// (`ShapeCastResult::GetEarlyOutFraction`).
    ///
    /// A body is reported once per query-shape leaf and obstacle leaf that touch: a convex
    /// obstacle once per leaf of the query shape that hits it, a mesh or heightfield once per
    /// triangle the shape sweeps into. Hits follow the rules of [`cast_shape`](Self::cast_shape), and the
    /// validation and errors are the same.
    ///
    /// The first hit is `cast_shape`'s hit unless several share its fraction (and at fraction 0
    /// its depth); `cast_shape` then returns one of them.
    pub fn cast_shape_all(
        &self,
        cast: &ShapeCast<'_>,
        filter: &QueryFilter<'_>,
    ) -> Result<Vec<ShapeCastHit>, QueryError> {
        self.cast_shape_with(cast, filter, JPH_CollisionCollectorType_AllHit)
    }

    /// The hits of `cast` that joltc's `collector` reports, sorted as
    /// [`cast_shape_all`](Self::cast_shape_all) promises.
    fn cast_shape_with(
        &self,
        cast: &ShapeCast<'_>,
        filter: &QueryFilter<'_>,
        collector: JPH_CollisionCollectorType,
    ) -> Result<Vec<ShapeCastHit>, QueryError> {
        validate_pose(cast.shape, cast.position, cast.rotation)?;
        if !(limits::is_frame_span(cast.direction) && cast.direction != Vec3::ZERO) {
            return Err(QueryError::InvalidValue(
                "cast direction must be finite, not zero and at most 2 * limits::MAX_POSITION per axis",
            ));
        }
        let target_distance = cast.target_distance;
        if !is_finite_non_negative(target_distance) {
            return Err(QueryError::InvalidValue(
                "target distance must be finite and not negative",
            ));
        }
        if target_distance > ShapeCast::MAX_TARGET_DISTANCE {
            return Err(QueryError::InvalidValue(
                "target distance must be at most ShapeCast::MAX_TARGET_DISTANCE",
            ));
        }
        let inflated = if target_distance > 0.0 {
            let inflated = cast
                .shape
                .inflated(target_distance)
                .map_err(|_| QueryError::InvalidValue("target distance is too large"))?;
            Some(inflated.ok_or(QueryError::InvalidValue(
                "target distance needs a sphere or capsule query shape",
            ))?)
        } else {
            None
        };
        filter.validate(self)?;
        let query_shape = inflated.as_ref().unwrap_or(cast.shape);

        let mut settings = default_cast_settings();
        settings.returnDeepestPoint = cast.return_deepest_point;
        let back_face_mode = if cast.back_faces {
            JPH_BackFaceMode_CollideWithBackFaces
        } else {
            JPH_BackFaceMode_IgnoreBackFaces
        };
        settings.backFaceModeTriangles = back_face_mode;
        settings.backFaceModeConvex = back_face_mode;

        // joltc's `CastShape2` moves the transform to the shape's centre of mass itself
        // (`RShapeCast::sFromWorldTransform`).
        let transform = rotation_translation(cast.rotation, cast.position);
        let direction = cast.direction.to_jph();
        let mut base_offset = cast.position.to_jph();

        let mut hits = with_query_filters(self, filter, |raw, state| {
            let slot = HitsSlot::<JPH_ShapeCastResult>::new(state);
            // SAFETY: as in `cast_ray` for the query object and filters. `query_shape` is a live
            // shape kept alive by `cast` or `inflated` until after the call; `transform`,
            // `direction`, `settings`, `base_offset` and `slot` are live locals, and
            // `push_shape_cast_hit` matches the slot type and copies the result.
            unsafe {
                JPH_NarrowPhaseQuery_CastShape2(
                    self.narrow_phase_query.as_ptr(),
                    query_shape.as_ptr(),
                    &transform,
                    &direction,
                    &settings,
                    &mut base_offset,
                    collector,
                    Some(push_shape_cast_hit),
                    slot.as_user_data(),
                    null(),
                    raw.object_layer,
                    raw.body,
                    raw.shape,
                )
            };
            slot.into_hits()
        })?;
        hits.sort_unstable_by(|a, b| {
            early_out_fraction(a)
                .total_cmp(&early_out_fraction(b))
                .then(a.bodyID2.cmp(&b.bodyID2))
                .then(a.subShapeID2.cmp(&b.subShapeID2))
                .then(a.subShapeID1.cmp(&b.subShapeID1))
        });
        Ok(hits
            .into_iter()
            .map(|hit| self.shape_cast_hit(cast, hit))
            .collect())
    }

    /// The [`ShapeCastHit`] for a hit joltc reported for `cast`.
    fn shape_cast_hit(&self, cast: &ShapeCast<'_>, hit: JPH_ShapeCastResult) -> ShapeCastHit {
        let body = BodyId::new(hit.bodyID2, self.tag);
        let sub_shape_id = SubShapeId::new(hit.subShapeID2);
        ShapeCastHit {
            body,
            sub_shape_id,
            object_layer: self.object_layer_of(body),
            compound_child: self.compound_sub_shape(body, sub_shape_id).ok().flatten(),
            fraction: hit.fraction,
            distance: hit.fraction * cast.direction.length(),
            point: offset_from(cast.position, hit.contactPointOn2),
            normal: Vec3::from_jph(hit.penetrationAxis)
                .normalized_or_zero()
                .scale(-1.0),
            penetration_depth: hit.penetrationDepth - cast.target_distance,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joltc_cast_settings_init_gives_jolts_defaults() {
        let settings = default_cast_settings();
        assert_eq!(settings.base.collectFacesMode, JPH_CollectFacesMode_NoFaces);
        assert_eq!(
            settings.base.activeEdgeMode,
            JPH_ActiveEdgeMode_CollideOnlyWithActive
        );
        assert_eq!(settings.base.collisionTolerance, 1.0e-4);
        assert_eq!(settings.base.penetrationTolerance, 1.0e-4);
        assert_eq!(
            settings.backFaceModeTriangles,
            JPH_BackFaceMode_IgnoreBackFaces
        );
        assert_eq!(
            settings.backFaceModeConvex,
            JPH_BackFaceMode_IgnoreBackFaces
        );
        assert!(!settings.useShrunkenShapeAndConvexRadius);
        assert!(!settings.returnDeepestPoint);
    }
}
