//! Collision shapes.

use std::collections::BTreeMap;
use std::fmt;
use std::ptr::null;

use oxijolt_sys::*;

use crate::limits;
use crate::math::{is_finite_non_negative, is_finite_positive};
use crate::owned::{JoltObject, Owned};
use crate::world::ensure_initialized;

mod binary_state;
mod compound;
mod create;
mod geometry;
mod hull;
mod mass;
mod mesh;
mod mutable;
mod plane;
mod scaled;
mod static_only;
mod tapered;

use crate::material::height_field_materials;
use crate::{PhysicsMaterial, Quat, ShapeError, Vec3};
pub(crate) use compound::compound_sub_shape_of;
use compound::{build_compound, check_child_pose, sub_shape_ids, RawCompoundChild};
pub use mass::MassProperties;
pub use mesh::{DroppedTriangles, MeshBuildQuality, MeshSettings};
pub use mutable::MutableCompound;
pub(crate) use static_only::static_only_leaves_are_meshes_of;

/// A collision shape that bodies are created from.
///
/// Owns one Jolt reference. Every body created from it holds its own reference, so the shape
/// may be dropped while bodies use it. Jolt shapes cannot change after construction, so one
/// shape may serve any number of bodies in any number of worlds. To change a compound at run
/// time, edit a [`MutableCompound`] and install the shapes it publishes.
pub struct Shape(Owned<JPH_Shape>);

impl fmt::Debug for Shape {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shape").finish_non_exhaustive()
    }
}

// SAFETY: Jolt shapes are immutable after construction and `RefTarget` counts references
// atomically, so a shape may be used and released from any thread
// (https://jrouwe.github.io/JoltPhysicsDocs/5.6.0/index.html#memory-management).
unsafe impl Send for Shape {}
// SAFETY: as for `Send`; `&Shape` only lets bodies take further references and read the
// immutable shape.
unsafe impl Sync for Shape {}

/// Jolt's `SubShapeID`: the path from a body's root shape to the leaf shape that was hit (a
/// compound child, a heightfield triangle).
///
/// It is meaningful only together with the root shape it came from. Equal shapes built the same
/// way give equal ids, so ids may be hashed and compared.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct SubShapeId(u32);

impl SubShapeId {
    pub(crate) fn new(raw: u32) -> Self {
        Self(raw)
    }

    /// The id as Jolt stores it (`SubShapeID::GetValue`).
    pub fn to_raw(self) -> u32 {
        self.0
    }
}

/// One child of a compound shape, for [`Shape::new_compound`].
///
/// The pose is relative to the compound's origin (Jolt `CompoundShapeSettings::AddShape`).
/// `user_data` is free for the caller, for example a collision group. It belongs to the
/// compound child, not to the leaf shape (Jolt `CompoundShape::SubShape::mUserData`), so one
/// leaf [`Shape`] may serve several children with different values.
#[derive(Clone, Copy)]
pub struct CompoundChild<'a> {
    /// The child's shape.
    pub shape: &'a Shape,
    /// Position of the child's origin in the compound, metres; finite.
    pub position: Vec3,
    /// Rotation of the child in the compound; a finite unit quaternion.
    pub rotation: Quat,
    /// The caller's value for this child, readable from hits.
    pub user_data: u32,
}

impl fmt::Debug for CompoundChild<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CompoundChild")
            .field("position", &self.position)
            .field("rotation", &self.rotation)
            .field("user_data", &self.user_data)
            .finish_non_exhaustive()
    }
}

/// The compound child a [`SubShapeId`] leads to, from [`Shape::compound_sub_shape`] or
/// [`PhysicsWorld::compound_sub_shape`](crate::PhysicsWorld::compound_sub_shape).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CompoundSubShape {
    /// Position of the child in the `children` slice given to [`Shape::new_compound`], or in the
    /// [`MutableCompound`] when [`to_shape`](MutableCompound::to_shape) was called.
    pub index: u32,
    /// The child's [`CompoundChild::user_data`].
    pub user_data: u32,
}

/// Owns a `JPH_ShapeSettings`: the one reference every `JPH_*ShapeSettings_Create` returns
/// (joltc calls `AddRef` on the new settings).
///
/// Dropping it releases that reference, which deletes the settings and with them their
/// cached `ShapeResult` reference. A shape returned by a `*_CreateShape` or `*_Create` call
/// carries its own reference (joltc calls `AddRef` before returning it), so the caller keeps
/// exactly one reference to the shape.
pub(crate) struct ShapeSettings(Owned<JPH_ShapeSettings>);

/// Shape settings, of which the owner holds one Jolt reference.
impl JoltObject for JPH_ShapeSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference to the settings (trait contract), released here
        // once. Shapes created from them hold their own, and another Jolt object may still
        // hold further ones, so joltc only calls `Release`, never `delete`.
        unsafe { JPH_ShapeSettings_Destroy(ptr) };
    }
}

impl ShapeSettings {
    /// Takes over settings returned by a `JPH_*ShapeSettings_Create` call.
    ///
    /// # Safety
    /// `ptr` is null or points to live shape settings holding one reference that the caller
    /// hands over.
    pub(crate) unsafe fn from_raw(ptr: *mut JPH_ShapeSettings) -> Result<Self, ShapeError> {
        // SAFETY: the caller hands over one reference to live settings, or null.
        unsafe { Owned::from_raw(ptr) }
            .map(Self)
            .ok_or(ShapeError::AllocationFailed)
    }

    /// The settings as one of joltc's typed settings pointers. joltc's settings types are
    /// `reinterpret_cast`s of Jolt classes with single inheritance from `ShapeSettings`, the
    /// convention joltc itself uses, so the caller picks the type the settings were created as.
    pub(crate) fn as_ptr<T>(&self) -> *mut T {
        self.0.as_ptr().cast()
    }
}

/// Material indices, one per cell, and the live materials they index, for a heightfield.
pub(crate) type HeightFieldMaterials<'a> = (&'a [u8], &'a [*const JPH_PhysicsMaterial]);

/// Jolt heightfield settings holding `samples`, `settings` and, when given, `materials`.
///
/// # Safety
/// Jolt is initialised, `samples` holds `sample_count^2` finite values and `settings` passed
/// `validate_layout(sample_count)` and `validate_extents`. Material indices, when given, hold
/// `(sample_count - 1)^2` entries, and the material list is not empty and holds live materials.
unsafe fn height_field_settings(
    sample_count: u32,
    samples: &[f32],
    settings: &HeightFieldSettings,
    materials: Option<HeightFieldMaterials<'_>>,
) -> Result<ShapeSettings, ShapeError> {
    let offset = settings.offset.to_jph();
    let scale = settings.scale.to_jph();
    // SAFETY: the caller guarantees initialisation, that `samples` holds `sample_count^2`
    // floats and the index and material counts, all of which Jolt copies (the list takes its
    // own material references); `offset` and `scale` are live locals. Null material indices
    // are allowed without a list. The returned settings hold one reference, which the guard
    // takes over.
    let jolt_settings = unsafe {
        ShapeSettings::from_raw(
            match materials {
                None => JPH_HeightFieldShapeSettings_Create(
                    samples.as_ptr(),
                    &offset,
                    &scale,
                    sample_count,
                    null(),
                ),
                Some((indices, list)) => JPH_HeightFieldShapeSettings_Create2(
                    samples.as_ptr(),
                    &offset,
                    &scale,
                    sample_count,
                    indices.as_ptr(),
                    list.as_ptr(),
                    list.len() as u32,
                ),
            }
            .cast(),
        )
    }?;
    let ptr = jolt_settings.as_ptr();
    // SAFETY: the settings are live, owned by the guard and were created as heightfield
    // settings; the caller validated every value.
    unsafe {
        JPH_HeightFieldShapeSettings_SetBlockSize(ptr, settings.block_size);
        JPH_HeightFieldShapeSettings_SetBitsPerSample(ptr, settings.bits_per_sample);
        JPH_HeightFieldShapeSettings_SetActiveEdgeCosThresholdAngle(
            ptr,
            settings.active_edge_cos_threshold_angle,
        );
    }
    Ok(jolt_settings)
}

/// Runs `JPH_Init` once, mapping failure to [`ShapeError::InitFailed`].
pub(crate) fn initialize() -> Result<(), ShapeError> {
    if ensure_initialized() {
        Ok(())
    } else {
        Err(ShapeError::InitFailed)
    }
}

/// Settings of [`Shape::new_height_field`] other than the samples. The defaults are Jolt's
/// (`HeightFieldShapeSettings`): no materials, no offset, unit scale, block size 2, 8 bits per
/// sample and an active-edge threshold of 5 degrees.
#[derive(Clone, Debug)]
pub struct HeightFieldSettings<'a> {
    materials: Option<(&'a [&'a PhysicsMaterial], &'a [u8])>,
    offset: Vec3,
    scale: Vec3,
    block_size: u32,
    bits_per_sample: u32,
    active_edge_cos_threshold_angle: f32,
}

/// Equal when every value is equal and both name the same material objects with equal indices.
impl PartialEq for HeightFieldSettings<'_> {
    fn eq(&self, other: &Self) -> bool {
        let same_materials = match (self.materials, other.materials) {
            (None, None) => true,
            (Some((list, indices)), Some((other_list, other_indices))) => {
                indices == other_indices
                    && list.len() == other_list.len()
                    && list
                        .iter()
                        .zip(other_list)
                        .all(|(a, b)| std::ptr::eq(*a, *b))
            }
            _ => false,
        };
        same_materials
            && self.offset == other.offset
            && self.scale == other.scale
            && self.block_size == other.block_size
            && self.bits_per_sample == other.bits_per_sample
            && self.active_edge_cos_threshold_angle == other.active_edge_cos_threshold_angle
    }
}

impl Default for HeightFieldSettings<'_> {
    fn default() -> Self {
        Self {
            materials: None,
            offset: Vec3::ZERO,
            scale: Vec3::new(1.0, 1.0, 1.0),
            block_size: 2,
            bits_per_sample: 8,
            active_edge_cos_threshold_angle: 0.996195,
        }
    }
}

impl<'a> HeightFieldSettings<'a> {
    /// Gives cell `(x, y)`, the cell between samples `(x, y)` and `(x + 1, y + 1)`, the
    /// material `list[indices[y * (n - 1) + x]]`, `n` the sample count. `list` holds 1 to 256
    /// materials and `indices` one entry for each of the caller's `(n - 1)^2` cells, each naming
    /// a material of the list; [`Shape::new_height_field`] checks both
    /// ([`ShapeError::InvalidValue`]). The cells Jolt adds for padding (see
    /// [`Shape::new_height_field`] under "Block size") get material 0. Without materials every
    /// cell uses Jolt's default material. The shape holds its own reference to each material.
    #[must_use]
    pub fn materials(mut self, list: &'a [&'a PhysicsMaterial], indices: &'a [u8]) -> Self {
        self.materials = Some((list, indices));
        self
    }

    /// Shape-local position of sample (0, 0) at height 0, metres; finite. Default zero.
    #[must_use]
    pub fn offset(mut self, value: Vec3) -> Self {
        self.offset = value;
        self
    }

    /// Metres per sample step along X and Z, and the factor applied to the samples along Y;
    /// each finite and positive. Default `(1, 1, 1)`.
    #[must_use]
    pub fn scale(mut self, value: Vec3) -> Self {
        self.scale = value;
        self
    }

    /// Side length of the square blocks Jolt groups samples into, in `2..=8`. Larger blocks use
    /// less memory and make queries slower. Default 2.
    #[must_use]
    pub fn block_size(mut self, value: u32) -> Self {
        self.block_size = value;
        self
    }

    /// Bits Jolt stores per sample, in `1..=16`, each relative to the height range of its
    /// block. More bits follow the samples more closely and use more memory. Default 8.
    #[must_use]
    pub fn bits_per_sample(mut self, value: u32) -> Self {
        self.bits_per_sample = value;
        self
    }

    /// Cosine of the angle between two triangles above which their shared edge counts as
    /// active, in `[0, 1]`; concave edges are never active. Smaller values give more ghost
    /// collisions with edges, larger ones slower depenetration (Jolt's wording). Default
    /// `0.996195`, the cosine of 5 degrees.
    #[must_use]
    pub fn active_edge_cos_threshold_angle(mut self, value: f32) -> Self {
        self.active_edge_cos_threshold_angle = value;
        self
    }

    /// Checks the settings Jolt relies on before it looks at the samples and returns the
    /// padded sample count; `sample_count` is at least 2.
    fn validate_layout(&self, sample_count: u32) -> Result<u64, ShapeError> {
        let invalid = |what| Err(ShapeError::InvalidValue(what));
        if !(2..=8).contains(&self.block_size) {
            return invalid("block_size must be between 2 and 8");
        }
        if !(1..=16).contains(&self.bits_per_sample) {
            return invalid("bits_per_sample must be between 1 and 16");
        }
        let threshold = self.active_edge_cos_threshold_angle;
        if !(threshold.is_finite() && (0.0..=1.0).contains(&threshold)) {
            return invalid("active_edge_cos_threshold_angle must be between 0 and 1");
        }
        let padded = padded_sample_count(sample_count, self.block_size);
        if padded / u64::from(self.block_size) < 2 {
            return invalid("sample_count must be larger than block_size");
        }
        // Jolt needs `2 * bits(padded - 1) + 1` sub-shape id bits, at most 32.
        if padded > 32768 {
            return invalid("sample_count is too large");
        }
        Ok(padded)
    }

    /// Checks that offset, scale and the field's extents are finite, given the padded sample
    /// count and the range of the samples that are not holes.
    fn validate_extents(&self, padded: u64, heights: Option<(f32, f32)>) -> Result<(), ShapeError> {
        if !self.offset.is_finite() {
            return Err(ShapeError::InvalidValue(
                "height field offset must be finite",
            ));
        }
        let scale = [self.scale.x, self.scale.y, self.scale.z];
        if !scale.into_iter().all(is_finite_positive) {
            return Err(ShapeError::InvalidValue(
                "height field scale must be finite and positive",
            ));
        }
        let last = (padded - 1) as f32;
        let far_x = self.offset.x + self.scale.x * last;
        let far_z = self.offset.z + self.scale.z * last;
        if !(far_x.is_finite() && far_z.is_finite()) {
            return Err(ShapeError::InvalidValue(
                "height field extent along x or z must be finite",
            ));
        }
        if let Some((min, max)) = heights {
            let (offset, scale) = (self.offset.y, self.scale.y);
            if !((offset + scale * min).is_finite() && (offset + scale * max).is_finite()) {
                return Err(ShapeError::InvalidValue(
                    "height field extent along y must be finite",
                ));
            }
        }
        Ok(())
    }
}

/// The error of a shape that reaches beyond [`limits::MAX_SHAPE_EXTENT`].
const BEYOND_EXTENT: &str = "shape extends beyond limits::MAX_SHAPE_EXTENT";

/// Whether a positive dimension is at most [`limits::MAX_SHAPE_EXTENT`].
fn within_extent(dimension: f32) -> bool {
    dimension <= limits::MAX_SHAPE_EXTENT
}

/// `sample_count` rounded up to a multiple of `block_size`, as Jolt stores it.
fn padded_sample_count(sample_count: u32, block_size: u32) -> u64 {
    u64::from(sample_count).div_ceil(u64::from(block_size)) * u64::from(block_size)
}

/// Lowest and highest sample that is not a hole (`f32::MAX`, Jolt's `cNoCollisionValue`), or
/// `None` when every sample is a hole.
fn height_range(samples: &[f32]) -> Option<(f32, f32)> {
    samples
        .iter()
        .filter(|&&sample| sample != f32::MAX)
        .fold(None, |range, &sample| match range {
            None => Some((sample, sample)),
            Some((min, max)) => Some((sample.min(min), sample.max(max))),
        })
}

/// The checks of [`Shape::new_box_with_convex_radius`].
pub(crate) fn validate_box(half_extent: Vec3, convex_radius: f32) -> Result<(), ShapeError> {
    let components = [half_extent.x, half_extent.y, half_extent.z];
    if !components.into_iter().all(is_finite_positive) {
        return Err(ShapeError::InvalidValue(
            "box half extents must be finite and positive",
        ));
    }
    if !components.into_iter().all(within_extent) {
        return Err(ShapeError::InvalidValue(BEYOND_EXTENT));
    }
    validate_convex_radius(convex_radius)
}

fn validate_convex_radius(convex_radius: f32) -> Result<(), ShapeError> {
    if is_finite_non_negative(convex_radius) {
        Ok(())
    } else {
        Err(ShapeError::InvalidValue(
            "convex radius must be finite and not negative",
        ))
    }
}

/// The checks of [`Shape::new_sphere`].
pub(crate) fn validate_sphere(radius: f32) -> Result<(), ShapeError> {
    if radius > limits::MAX_SHAPE_EXTENT {
        return Err(ShapeError::InvalidValue(BEYOND_EXTENT));
    }
    validate_positive_sphere(radius)
}

fn validate_positive_sphere(radius: f32) -> Result<(), ShapeError> {
    if is_finite_positive(radius) {
        Ok(())
    } else {
        Err(ShapeError::InvalidValue(
            "sphere radius must be finite and positive",
        ))
    }
}

/// The checks of [`Shape::new_capsule`].
pub(crate) fn validate_capsule(
    half_height_of_cylinder: f32,
    radius: f32,
) -> Result<(), ShapeError> {
    if half_height_of_cylinder + radius > limits::MAX_SHAPE_EXTENT {
        return Err(ShapeError::InvalidValue(BEYOND_EXTENT));
    }
    validate_positive_capsule(half_height_of_cylinder, radius)
}

fn validate_positive_capsule(half_height_of_cylinder: f32, radius: f32) -> Result<(), ShapeError> {
    if is_finite_positive(half_height_of_cylinder) && is_finite_positive(radius) {
        Ok(())
    } else {
        Err(ShapeError::InvalidValue(
            "capsule half height and radius must be finite and positive",
        ))
    }
}

/// The checks of [`Shape::new_cylinder_with_convex_radius`].
pub(crate) fn validate_cylinder(
    half_height: f32,
    radius: f32,
    convex_radius: f32,
) -> Result<(), ShapeError> {
    if !(is_finite_positive(half_height) && is_finite_positive(radius)) {
        return Err(ShapeError::InvalidValue(
            "cylinder half height and radius must be finite and positive",
        ));
    }
    if !(within_extent(half_height) && within_extent(radius)) {
        return Err(ShapeError::InvalidValue(BEYOND_EXTENT));
    }
    validate_convex_radius(convex_radius)
}

/// The checks of [`Shape::new_height_field`] before Jolt sees anything.
pub(crate) fn validate_height_field(
    sample_count: u32,
    samples: &[f32],
    settings: &HeightFieldSettings,
) -> Result<(), ShapeError> {
    if sample_count < 2 {
        return Err(ShapeError::InvalidValue("sample_count must be at least 2"));
    }
    // Jolt divides by the block size before it checks it, so the settings are checked first.
    let padded = settings.validate_layout(sample_count)?;
    let count = sample_count as usize;
    if count.checked_mul(count) != Some(samples.len()) {
        return Err(ShapeError::InvalidValue(
            "samples must hold sample_count^2 values",
        ));
    }
    if !samples.iter().all(|sample| sample.is_finite()) {
        return Err(ShapeError::InvalidValue("height samples must be finite"));
    }
    let heights = height_range(samples);
    // Jolt quantises with `65534 / (max - min)`, so the range must be finite too.
    if heights.is_some_and(|(min, max)| !(max - min).is_finite()) {
        return Err(ShapeError::InvalidValue(
            "height sample range must be finite",
        ));
    }
    settings.validate_extents(padded, heights)
}

impl Shape {
    /// A box with the given half extents in metres (each finite, positive and at most
    /// [`limits::MAX_SHAPE_EXTENT`]) and Jolt's default convex radius of 0.05 m; see
    /// [`new_box_with_convex_radius`].
    ///
    /// [`new_box_with_convex_radius`]: Self::new_box_with_convex_radius
    pub fn new_box(half_extent: Vec3) -> Result<Self, ShapeError> {
        Self::new_box_with_convex_radius(half_extent, JPH_DEFAULT_CONVEX_RADIUS as f32)
    }

    /// A box with the given half extents in metres (each finite, positive and at most
    /// [`limits::MAX_SHAPE_EXTENT`]) and convex radius in metres (finite and not negative).
    ///
    /// Jolt shrinks the box by the convex radius and inflates it again, so the faces stay where
    /// they are while edges and corners are rounded for contacts and shape casts. Jolt clamps
    /// the radius to the smallest half extent (`BoxShape.h`), and contacts and shape casts use
    /// at most 0.05 m of it (`ScaleHelpers::ScaleConvexRadius` caps it at Jolt's default), so a
    /// larger radius collides like 0.05. A radius of 0 gives sharp edges; collision detection
    /// is then somewhat slower, because Jolt falls back to EPA more often. Ray casts always see
    /// the sharp box, whatever the radius (`BoxShape::CastRay` tests the half extents only).
    pub fn new_box_with_convex_radius(
        half_extent: Vec3,
        convex_radius: f32,
    ) -> Result<Self, ShapeError> {
        validate_box(half_extent, convex_radius)?;
        initialize()?;
        let half_extent = half_extent.to_jph();
        // SAFETY: Jolt is initialised, `half_extent` is a live local and both inputs were
        // checked against Jolt's assertions. The returned box holds one reference, which
        // `Self` takes over.
        unsafe { Self::from_raw(JPH_BoxShape_Create(&half_extent, convex_radius).cast()) }
    }

    /// A sphere with the given radius in metres (finite, positive and at most
    /// [`limits::MAX_SHAPE_EXTENT`]).
    pub fn new_sphere(radius: f32) -> Result<Self, ShapeError> {
        validate_sphere(radius)?;
        Self::sphere(radius)
    }

    /// A sphere of any finite positive radius, also beyond the extent bound: what
    /// [`inflated`](Self::inflated) needs for a query shape.
    fn sphere(radius: f32) -> Result<Self, ShapeError> {
        validate_positive_sphere(radius)?;
        initialize()?;
        // SAFETY: Jolt is initialised. The returned sphere holds one reference, which `Self`
        // takes over.
        unsafe { Self::from_raw(JPH_SphereShape_Create(radius).cast()) }
    }

    /// A cylinder along the local Y axis, centred on the origin, `2 * half_height` metres high,
    /// with Jolt's default convex radius of 0.05 m; see
    /// [`new_cylinder_with_convex_radius`].
    ///
    /// [`new_cylinder_with_convex_radius`]: Self::new_cylinder_with_convex_radius
    pub fn new_cylinder(half_height: f32, radius: f32) -> Result<Self, ShapeError> {
        Self::new_cylinder_with_convex_radius(half_height, radius, JPH_DEFAULT_CONVEX_RADIUS as f32)
    }

    /// A cylinder along the local Y axis, centred on the origin, `2 * half_height` metres high.
    ///
    /// Half height and radius must be finite, positive and at most [`limits::MAX_SHAPE_EXTENT`],
    /// the convex radius finite and not negative. Like a box's, the convex radius rounds the edges
    /// for contacts and shape casts; Jolt clamps it to `min(half_height, radius)`
    /// (`CylinderShape.cpp`) and uses at most 0.05 m of it there
    /// (`ScaleHelpers::ScaleConvexRadius`).
    pub fn new_cylinder_with_convex_radius(
        half_height: f32,
        radius: f32,
        convex_radius: f32,
    ) -> Result<Self, ShapeError> {
        validate_cylinder(half_height, radius, convex_radius)?;
        initialize()?;
        // `JPH_CylinderShape_Create` would ignore the convex radius (joltc passes 0), so the
        // cylinder is built through its settings.
        // SAFETY: Jolt is initialised; the returned settings hold one reference, which the
        // guard takes over.
        let settings = unsafe {
            ShapeSettings::from_raw(
                JPH_CylinderShapeSettings_Create(half_height, radius, convex_radius).cast(),
            )
        }?;
        settings.create()
    }

    /// A capsule along the local Y axis, centred on the origin: a cylinder
    /// `2 * half_height_of_cylinder` metres high with a hemisphere of `radius` at each end, so
    /// `2 * (half_height_of_cylinder + radius)` metres high in total. Both values must be finite
    /// and positive, and `half_height_of_cylinder + radius` at most
    /// [`limits::MAX_SHAPE_EXTENT`].
    pub fn new_capsule(half_height_of_cylinder: f32, radius: f32) -> Result<Self, ShapeError> {
        validate_capsule(half_height_of_cylinder, radius)?;
        Self::capsule(half_height_of_cylinder, radius)
    }

    /// A capsule of any finite positive size, also beyond the extent bound: what
    /// [`inflated`](Self::inflated) needs for a query shape.
    fn capsule(half_height_of_cylinder: f32, radius: f32) -> Result<Self, ShapeError> {
        validate_positive_capsule(half_height_of_cylinder, radius)?;
        initialize()?;
        // SAFETY: Jolt is initialised and both values are positive, as Jolt asserts
        // (`CapsuleShape.h`). The returned capsule holds one reference, which `Self` takes
        // over.
        unsafe { Self::from_raw(JPH_CapsuleShape_Create(half_height_of_cylinder, radius).cast()) }
    }

    /// A heightfield of `sample_count` x `sample_count` height samples, in metres before
    /// scaling.
    ///
    /// # Layout
    /// The surface passes through `offset + scale * (x, samples[y * n + x], y)` for `x, y` in
    /// `0..n`, `n = sample_count`. `y` runs along +Z, so row `y` of `samples` holds the heights
    /// at `z = offset.z + y * scale.z` (Jolt's row-major order). A column-major source
    /// `heights[x * n + z]` must be transposed first:
    ///
    /// ```text
    /// samples[z * n + x] = heights[x * n + z]
    /// ```
    ///
    /// Each cell is split into two triangles along the diagonal from sample `(x, y)` to sample
    /// `(x + 1, y + 1)`. A sample of `f32::MAX` is a hole: the cells touching it have no
    /// collision. Every other sample must be finite.
    ///
    /// # Block size
    /// Jolt rounds `n` up to a multiple of the block size and fills the extra rows and columns
    /// with holes, so the cells touching them have no collision. With the default block size 2,
    /// n = 33 is stored as 34 x 34 and the surface covers exactly the 32 x 32 cells given.
    /// `n / block_size`, rounded up, must be at least 2.
    ///
    /// # Precision
    /// Jolt first quantises the heights to 16 bits over the height range of the whole field,
    /// then to [`HeightFieldSettings::bits_per_sample`] bits within the height range of each
    /// block. [`height_field_position`](Self::height_field_position) reads back the stored
    /// heights.
    ///
    /// # Static only
    /// [`PhysicsWorld::create_body`](crate::PhysicsWorld::create_body) refuses heightfields for
    /// dynamic and kinematic bodies.
    ///
    /// # Extent
    /// The shape's local bounds must lie within [`limits::MAX_SHAPE_EXTENT`] on every axis;
    /// otherwise [`ShapeError::InvalidValue`] is returned. A field of holes only has empty
    /// bounds and passes.
    pub fn new_height_field(
        sample_count: u32,
        samples: &[f32],
        settings: &HeightFieldSettings<'_>,
    ) -> Result<Self, ShapeError> {
        validate_height_field(sample_count, samples, settings)?;
        let Some((list, indices)) = settings.materials else {
            // SAFETY: `samples` and `settings` were validated above; there are no materials.
            return unsafe { Self::height_field(sample_count, samples, settings, None) };
        };
        let list = height_field_materials(sample_count, list, indices)?;
        // SAFETY: `samples` and `settings` were validated above; the index count is
        // `(sample_count - 1)^2` and the list holds 1 to 256 live materials, which `settings`
        // keeps alive for the call.
        unsafe { Self::height_field(sample_count, samples, settings, Some((indices, &list))) }
    }

    /// The heightfield of [`new_height_field`](Self::new_height_field), with `materials` when
    /// given.
    ///
    /// # Safety
    /// `samples` and `settings` passed [`validate_height_field`]; `materials`, when given,
    /// meets the contract of `height_field_settings`.
    pub(crate) unsafe fn height_field(
        sample_count: u32,
        samples: &[f32],
        settings: &HeightFieldSettings,
        materials: Option<HeightFieldMaterials<'_>>,
    ) -> Result<Self, ShapeError> {
        initialize()?;
        // SAFETY: Jolt is initialised, the caller validated `samples` and `settings` and
        // guarantees the material contract.
        let jolt_settings =
            unsafe { height_field_settings(sample_count, samples, settings, materials) }?;
        jolt_settings.create()?.within_extent_bounds()
    }

    /// The stored surface point of heightfield sample `(x, y)` in shape-local space, after
    /// Jolt's quantisation. `None` when the shape is not a heightfield, when `(x, y)` lies
    /// outside the stored (padded) grid, or when the sample is a hole, including the padding
    /// Jolt adds.
    pub fn height_field_position(&self, x: u32, y: u32) -> Option<Vec3> {
        if self.sub_type() != JPH_ShapeSubType_HeightField {
            return None;
        }
        let shape: *const JPH_HeightFieldShape = self.as_ptr().cast();
        // SAFETY: the shape is live and a heightfield (checked above); the getter only reads it.
        let sample_count = unsafe { JPH_HeightFieldShape_GetSampleCount(shape) };
        if x >= sample_count || y >= sample_count {
            return None;
        }
        // SAFETY: as above, and `x` and `y` are inside the stored grid, as Jolt asserts.
        if unsafe { JPH_HeightFieldShape_IsNoCollision(shape, x, y) } {
            return None;
        }
        let mut position = Vec3::ZERO.to_jph();
        // SAFETY: as above; `position` is a live local.
        unsafe { JPH_HeightFieldShape_GetPosition(shape, x, y, &mut position) };
        Some(Vec3::from_jph(position))
    }

    /// A compound of `children`, each with its own pose and user data.
    ///
    /// The children may be dropped afterwards: the compound holds its own references. Child
    /// order is part of the shape and so of a deterministic state. Jolt moves the compound's
    /// centre of mass to the children's mass-weighted centre, so body positions of compounds
    /// read back through arithmetic.
    ///
    /// Two or more children make a Jolt `StaticCompoundShape`. A single child makes a
    /// `MutableCompoundShape`, because Jolt's static compound replaces a lone child by the
    /// child itself (or a `RotatedTranslatedShape`) and drops its user data
    /// (`StaticCompoundShape.cpp`); oxijolt never changes it after construction.
    ///
    /// Every child position must be finite with each component at most
    /// [`limits::MAX_SHAPE_EXTENT`] in absolute value ([`ShapeError::InvalidValue`]), and the
    /// compound's local bounds must lie within [`limits::MAX_SHAPE_EXTENT`] on every axis
    /// ([`ShapeError::InvalidValue`]). A hierarchy whose sub-shape ids Jolt cannot form
    /// gives [`ShapeError::Rejected`] when it needs more than 32 bits and
    /// [`ShapeError::InvalidValue`] when a one-child compound would start at bit 32. A
    /// compound of more than [`limits::MAX_EXPANDED_SUB_SHAPES`] shapes, counting a child shared
    /// by several parents at every use, gives [`ShapeError::TooManySubShapes`].
    pub fn new_compound(children: &[CompoundChild<'_>]) -> Result<Self, ShapeError> {
        if children.is_empty() {
            return Err(ShapeError::InvalidValue(compound::EMPTY_COMPOUND_RULE));
        }
        if u32::try_from(children.len()).is_err() {
            return Err(ShapeError::InvalidValue(
                "a compound has at most u32::MAX children",
            ));
        }
        for child in children {
            check_child_pose(child.position, child.rotation)?;
        }
        let mut memo = BTreeMap::new();
        let raw: Vec<_> = children
            .iter()
            .map(|child| RawCompoundChild {
                shape: child.shape.as_ptr(),
                position: child.position,
                rotation: child.rotation,
                user_data: child.user_data,
                // SAFETY: `children` borrows every shape for the call.
                ids: unsafe { sub_shape_ids(child.shape.as_ptr(), &mut memo) },
            })
            .collect();
        // SAFETY: `children` borrows every shape for the call, the poses were checked above and
        // the count fits in `u32`.
        unsafe { build_compound(&raw) }
    }

    /// `shape` with its centre of mass moved by `offset` (shape space, metres, each component at
    /// most [`limits::MAX_SHAPE_EXTENT`] in absolute value), a Jolt `OffsetCenterOfMassShape`.
    /// The new shape's local bounds, which are relative to the new centre of mass, must lie
    /// within [`limits::MAX_SHAPE_EXTENT`] on every axis.
    ///
    /// Only the centre of mass moves: the body origin and the collision surface stay where
    /// `shape` puts them, and mass and inertia are computed about the new centre. A vehicle
    /// chassis gets a low centre of mass this way. The new shape holds its own reference to
    /// `shape`, which may be dropped afterwards. A decorated shape that only static bodies may
    /// use (a heightfield, mesh or plane) stays static-only.
    /// [`compound_sub_shape`](Self::compound_sub_shape) does not look through the decorator: it
    /// returns `None` for an offset compound.
    pub fn new_offset_center_of_mass(shape: &Shape, offset: Vec3) -> Result<Self, ShapeError> {
        if !limits::is_local_offset(offset) {
            return Err(ShapeError::InvalidValue(
                "centre of mass offset must be finite and within limits::MAX_SHAPE_EXTENT",
            ));
        }
        initialize()?;
        let offset = offset.to_jph();
        // SAFETY: Jolt is initialised, `offset` is a live local and `shape` is live for the
        // call; the decorator takes its own reference to it. The returned shape holds one
        // reference, which `Self` takes over.
        unsafe {
            Self::from_raw(JPH_OffsetCenterOfMassShape_Create(&offset, shape.as_ptr()).cast())
        }?
        .within_extent_bounds()
    }

    /// `self` when its local bounds lie within [`limits::MAX_SHAPE_EXTENT`] on every axis or are
    /// empty (a field of holes); otherwise an error, and the shape is released.
    pub(crate) fn within_extent_bounds(self) -> Result<Self, ShapeError> {
        let mut bounds = JPH_AABox {
            min: Vec3::ZERO.to_jph(),
            max: Vec3::ZERO.to_jph(),
        };
        // SAFETY: the shape is live for the call; the getter only reads it, and `bounds` is a
        // live local.
        unsafe { JPH_Shape_GetLocalBounds(self.as_ptr(), &mut bounds) };
        let (min, max) = (Vec3::from_jph(bounds.min), Vec3::from_jph(bounds.max));
        let empty = min.x > max.x || min.y > max.y || min.z > max.z;
        if empty || (limits::is_local_offset(min) && limits::is_local_offset(max)) {
            Ok(self)
        } else {
            Err(ShapeError::InvalidValue(BEYOND_EXTENT))
        }
    }

    /// The child of this compound that `id` leads to.
    ///
    /// `None` for shapes that are not compounds. Only the root level is decoded, so `id` may be
    /// a hit's full path or a partial path below this shape. An id that came from another
    /// shape gives `None` or an arbitrary valid child: the bits cannot prove where they came
    /// from.
    pub fn compound_sub_shape(&self, id: SubShapeId) -> Option<CompoundSubShape> {
        // SAFETY: `self` keeps the shape alive for the call.
        unsafe { compound_sub_shape_of(self.0.as_non_null(), id) }
    }

    /// Takes over a shape returned by a shape `Create` call, where null means joltc could not
    /// create it.
    ///
    /// # Safety
    /// `ptr` is null or a live shape holding one reference that the caller hands over.
    unsafe fn from_raw(ptr: *mut JPH_Shape) -> Result<Self, ShapeError> {
        // SAFETY: the caller hands over one reference to a live shape, or null.
        unsafe { Owned::from_raw(ptr) }
            .map(Self)
            .ok_or(ShapeError::AllocationFailed)
    }

    pub(crate) fn as_ptr(&self) -> *const JPH_Shape {
        self.0.as_ptr()
    }

    /// Jolt's concrete shape type.
    pub(crate) fn sub_type(&self) -> JPH_ShapeSubType {
        // SAFETY: the shape is live for the call; the getter only reads it.
        unsafe { JPH_Shape_GetSubType(self.as_ptr()) }
    }

    /// Whether Jolt allows this shape only on static bodies (meshes, heightfields, planes and
    /// compound or decorated shapes that contain one).
    pub(crate) fn must_be_static(&self) -> bool {
        // SAFETY: the shape is live for the call; the getter only reads it.
        unsafe { JPH_Shape_MustBeStatic(self.as_ptr()) }
    }

    /// The centre of mass in the shape's own space, in metres, relative to the shape's origin
    /// (Jolt `Shape::GetCenterOfMass`). Zero for boxes, spheres, capsules and cylinders; a
    /// compound's is its children's mass-weighted centre, and
    /// [`new_offset_center_of_mass`](Self::new_offset_center_of_mass) adds its offset. A body
    /// with this shape reads it in world space with
    /// [`BodyRef::center_of_mass_position`](crate::BodyRef::center_of_mass_position).
    pub fn center_of_mass(&self) -> Vec3 {
        let mut center = Vec3::ZERO.to_jph();
        // SAFETY: the shape is live for the call; the getter only reads it, and `center` is a
        // live local.
        unsafe { JPH_Shape_GetCenterOfMass(self.as_ptr(), &mut center) };
        Vec3::from_jph(center)
    }

    /// This sphere or capsule with its radius grown by `by` metres; `Ok(None)` for every other
    /// shape type.
    ///
    /// For spheres and capsules Jolt's default support function is the core (a point or a
    /// segment) plus a convex radius equal to the radius (`SphereShape.cpp`, `CapsuleShape.cpp`
    /// `GetSupportFunction`), and a shape cast adds `ShapeCastSettings::mExtraConvexRadius` to
    /// that same radius (`ConvexShape.cpp`, `CastSphereVsTriangles.cpp`,
    /// `CastConvexVsTriangles.cpp`). Casting the grown shape is therefore the cast Jolt does
    /// with an extra convex radius of `by`.
    pub(crate) fn inflated(&self, by: f32) -> Result<Option<Shape>, ShapeError> {
        let sub_type = self.sub_type();
        if sub_type == JPH_ShapeSubType_Sphere {
            // SAFETY: the shape is live and a sphere (checked above); the getter only reads it.
            let radius = unsafe { JPH_SphereShape_GetRadius(self.as_ptr().cast()) };
            Shape::sphere(radius + by).map(Some)
        } else if sub_type == JPH_ShapeSubType_Capsule {
            let capsule: *const JPH_CapsuleShape = self.as_ptr().cast();
            // SAFETY: the shape is live and a capsule (checked above); the getters only read it.
            let (half_height, radius) = unsafe {
                (
                    JPH_CapsuleShape_GetHalfHeightOfCylinder(capsule),
                    JPH_CapsuleShape_GetRadius(capsule),
                )
            };
            Shape::capsule(half_height, radius + by).map(Some)
        } else {
            Ok(None)
        }
    }
}

/// A shape, of which the owner holds one Jolt reference: joltc returns created shapes holding
/// one reference, and `JPH_Shape_Destroy` releases it. Bodies, body creation settings and
/// compounds hold their own.
impl JoltObject for JPH_Shape {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference to the shape (trait contract), released here
        // once. Bodies, creation settings and compounds keep their own.
        unsafe { JPH_Shape_Destroy(ptr) };
    }
}

#[cfg(test)]
mod tests;
