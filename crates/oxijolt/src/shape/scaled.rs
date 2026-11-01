//! Scaled shapes.

use std::ptr::{null, null_mut};

use oxijolt_sys::*;

use super::geometry::{diagonal, mul, mul_vec, rotation, v3, M3, V3};
use super::mesh::{convex_extent_of, is_collidable};
use super::{initialize, Shape, ShapeSettings};
use crate::{limits, ShapeError, ThinTrianglesError, Vec3};

impl Shape {
    /// `shape` scaled by `scale` along its local axes, a Jolt `ScaledShape`. The new shape holds
    /// its own reference to `shape`, which may be dropped afterwards.
    ///
    /// Each component must be finite ([`ShapeError::InvalidValue`]) and the scale valid for
    /// the shape under Jolt's rules ([`ShapeError::InvalidValue`]): every component at least
    /// 1e-6 in absolute value; uniform for spheres, capsules and tapered capsules; uniform in X
    /// and Z for cylinders and tapered cylinders; for a compound, uniform unless every rotated
    /// child is turned so the scale maps onto its own axes. Negative components mirror the
    /// shape; a mirrored mesh's front faces follow the mirrored winding.
    ///
    /// Meshes and heightfields inside the shape must stay collidable: each stored triangle,
    /// scaled, must pass the rule [`Shape::new_mesh`] applies, for the
    /// [`MeshSettings::max_convex_extent`](crate::MeshSettings::max_convex_extent) the mesh was
    /// built with (the default for heightfields) and without its quantization term (the stored
    /// triangles are quantized already). Shrinking a mesh far below the size it was built at, or
    /// flattening it, is refused with [`ShapeError::ThinTriangles`], which names the scale and
    /// the extent ([docs/limits.md#scaled-shapes]). The check reads every stored triangle back
    /// from Jolt, so its cost grows with the triangle count.
    ///
    /// The scaled shape's local bounds must lie within [`limits::MAX_SHAPE_EXTENT`] on every
    /// axis, and so must its centre of mass, which moves with the scale
    /// ([`ShapeError::InvalidValue`]). Mass and inertia scale with the shape; a body made
    /// of it goes through the usual mass and inertia checks of
    /// [`PhysicsWorld::create_body`](crate::PhysicsWorld::create_body). A shape that only
    /// static bodies may use stays static-only, and a scaled mesh stays usable by kinematic
    /// bodies. Jolt scales the convex radius by the smallest absolute component, and contacts
    /// use at most 0.05 m of it. [`compound_sub_shape`](Self::compound_sub_shape) does not look
    /// through the decorator: it returns `None` for a scaled compound.
    ///
    /// [docs/limits.md#scaled-shapes]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#scaled-shapes
    pub fn new_scaled(shape: &Shape, scale: Vec3) -> Result<Self, ShapeError> {
        // Jolt's own check lets NaN and infinity through (`ScaleHelpers::IsZeroScale`).
        if !scale.is_finite() {
            return Err(ShapeError::InvalidValue("scale must be finite"));
        }
        initialize()?;
        let jolt_scale = scale.to_jph();
        // SAFETY: `shape` is live for the call and `jolt_scale` is a live local; the check only
        // reads them.
        if !unsafe { JPH_Shape_IsValidScale(shape.as_ptr(), &jolt_scale) } {
            return Err(ShapeError::InvalidValue(
                "scale is not valid for this shape",
            ));
        }
        check_scaled_triangles(shape, scale)?;
        // SAFETY: Jolt is initialised, `shape` is live (the settings take their own reference to
        // it) and `jolt_scale` is a live local. The returned settings hold one reference, which
        // the guard takes over.
        let settings = unsafe {
            ShapeSettings::from_raw(
                JPH_ScaledShapeSettings_Create2(shape.as_ptr(), &jolt_scale).cast(),
            )
        }?;
        let scaled = settings.create()?.within_extent_bounds()?;
        // Bounds are relative to the centre of mass, which the scale moves too.
        if !limits::is_local_offset(scaled.center_of_mass()) {
            return Err(ShapeError::InvalidValue(
                "scaled centre of mass must lie within limits::MAX_SHAPE_EXTENT",
            ));
        }
        Ok(scaled)
    }
}

/// The linear map from the coordinates of a shape inside a scaled shape to those Jolt rounds
/// when it collides that shape: the scales above it, turned by the rotations between them.
///
/// Jolt hands a mesh or heightfield its own stored coordinates times the accumulated scale and
/// folds every rotation and translation above it (compound child poses, rotated-translated
/// shapes, centre-of-mass offsets) into the transform it applies afterwards
/// (`CollisionDispatch::sCollideShapeVsShape` through `ScaledShape`, `RotatedTranslatedShape`,
/// `OffsetCenterOfMassShape` and `CompoundShape`). A translation moves a triangle without
/// changing its cross product, and a rotation changes neither its cross product nor its
/// distance from the origin, so the map leaves translations out.
type Placement = M3;

/// Checks that every stored triangle of the meshes and heightfields in `shape` stays collidable
/// once scaled by `scale`: [`is_collidable`] with no quantization (the stored triangles are
/// already quantized), against convex shapes up to the extent each mesh was built for.
fn check_scaled_triangles(shape: &Shape, scale: Vec3) -> Result<(), ShapeError> {
    let Some(leaves) = triangle_leaves(shape, scale) else {
        return Err(ShapeError::InvalidValue(
            "scale cannot be checked for this shape",
        ));
    };
    for (leaf, placement) in leaves {
        // SAFETY: `leaf` is a live mesh or heightfield part of `shape` (see `triangle_leaves`).
        let (max_convex_extent, triangles) =
            unsafe { (convex_extent_of(leaf), placed_triangles(leaf, &placement)) };
        if !triangles
            .into_iter()
            .all(|corners| is_collidable(corners, [0.0; 3], max_convex_extent))
        {
            return Err(ShapeError::ThinTriangles(ThinTrianglesError {
                scale,
                max_convex_extent,
            }));
        }
    }
    Ok(())
}

/// The meshes and heightfields in `shape` scaled by `scale`, with their placements; `None` when a
/// static-only part is of a kind whose triangles this walk cannot reach. The parts are kept alive
/// by `shape`.
fn triangle_leaves(shape: &Shape, scale: Vec3) -> Option<Vec<(*const JPH_Shape, Placement)>> {
    let mut leaves = Vec::new();
    let mut pending = vec![(shape.as_ptr(), diagonal(v3(scale)))];
    while let Some((part, placement)) = pending.pop() {
        // SAFETY: `part` is `shape`, kept alive by the borrow, or a part of a live shape below,
        // which its parent keeps alive; the getters only read it.
        let (static_only, sub_type) =
            unsafe { (JPH_Shape_MustBeStatic(part), JPH_Shape_GetSubType(part)) };
        // Only meshes and heightfields collide through their triangles. Planes, the other
        // static-only leaf, are not reached here and leave the shape unscalable.
        if !static_only {
            continue;
        }
        if sub_type == JPH_ShapeSubType_Mesh || sub_type == JPH_ShapeSubType_HeightField {
            leaves.push((part, placement));
        } else {
            // SAFETY: `part` is live (see above) and of `sub_type`.
            let known = unsafe { push_parts(part, sub_type, &placement, &mut pending) };
            if !known {
                return None;
            }
        }
    }
    Some(leaves)
}

/// Pushes the parts of the compound or decorated shape `shape` with their placements; `false`
/// for any other sub type, whose triangles this check cannot reach.
///
/// # Safety
/// `shape` is a live shape of sub type `sub_type`.
unsafe fn push_parts(
    shape: *const JPH_Shape,
    sub_type: JPH_ShapeSubType,
    placement: &Placement,
    pending: &mut Vec<(*const JPH_Shape, Placement)>,
) -> bool {
    let turned = |q: JPH_Quat| mul(placement, &rotation([q.x, q.y, q.z, q.w].map(f64::from)));
    let mut turn = JPH_Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };
    if sub_type == JPH_ShapeSubType_StaticCompound || sub_type == JPH_ShapeSubType_MutableCompound {
        let compound: *const JPH_CompoundShape = shape.cast();
        // SAFETY: `shape` is a live compound (contract); the getter only reads it.
        let count = unsafe { JPH_CompoundShape_GetNumSubShapes(compound) };
        for index in 0..count {
            let mut child = null();
            // SAFETY: as above, `index < count`, and every output is a live local or null; the
            // child is kept alive by the compound.
            unsafe {
                JPH_CompoundShape_GetSubShape(
                    compound,
                    index,
                    &mut child,
                    null_mut(),
                    &mut turn,
                    null_mut(),
                );
            }
            pending.push((child, turned(turn)));
        }
        return true;
    }
    let decorators = [
        JPH_ShapeSubType_Scaled,
        JPH_ShapeSubType_RotatedTranslated,
        JPH_ShapeSubType_OffsetCenterOfMass,
    ];
    if !decorators.contains(&sub_type) {
        return false;
    }
    let decorated: *const JPH_DecoratedShape = shape.cast();
    // SAFETY: `shape` is a live decorated shape (checked above); the inner shape is kept alive
    // by it.
    let inner = unsafe { JPH_DecoratedShape_GetInnerShape(decorated) };
    let inner_placement = if sub_type == JPH_ShapeSubType_Scaled {
        let mut inner_scale = Vec3::ZERO.to_jph();
        // SAFETY: `shape` is a live scaled shape; `inner_scale` is a live local.
        unsafe { JPH_ScaledShape_GetScale(shape.cast(), &mut inner_scale) };
        mul(placement, &diagonal(v3(Vec3::from_jph(inner_scale))))
    } else if sub_type == JPH_ShapeSubType_RotatedTranslated {
        // SAFETY: `shape` is a live rotated-translated shape; `turn` is a live local.
        unsafe { JPH_RotatedTranslatedShape_GetRotation(shape.cast(), &mut turn) };
        turned(turn)
    } else {
        // A centre-of-mass offset only translates.
        *placement
    };
    pending.push((inner, inner_placement));
    true
}

/// The stored triangles of the mesh or heightfield `shape`, placed by `placement`.
///
/// # Safety
/// `shape` is a live mesh or heightfield.
unsafe fn placed_triangles(shape: *const JPH_Shape, placement: &Placement) -> Vec<[V3; 3]> {
    // SAFETY: `shape` is a live leaf (contract); a null buffer with capacity 0 only counts.
    let count = unsafe { JPH_Shape_GetTriangles(shape, null_mut(), 0) };
    // Checked: on 32-bit targets `3 * count` can exceed `usize`, and a wrapped length would let
    // joltc write past the buffer.
    let vertex_count = (count as usize)
        .checked_mul(3)
        .expect("triangle count overflows usize");
    let mut vertices = vec![Vec3::ZERO.to_jph(); vertex_count];
    // SAFETY: as above; `vertices` holds 3 * count vertices.
    unsafe { JPH_Shape_GetTriangles(shape, vertices.as_mut_ptr(), count) };
    let placed: Vec<V3> = vertices
        .iter()
        .map(|&vertex| mul_vec(placement, v3(Vec3::from_jph(vertex))))
        .collect();
    placed.as_chunks::<3>().0.to_vec()
}

#[cfg(test)]
mod tests;
