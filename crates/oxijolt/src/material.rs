//! Physics materials with the caller's user data, and the shapes that carry them.

use std::fmt;

use oxijolt_sys::*;

use crate::owned::{JoltObject, Owned};
use crate::shape::{
    initialize, validate_box, validate_capsule, validate_cylinder, validate_sphere, ShapeSettings,
};
use crate::{Shape, ShapeError, SubShapeId, Vec3};

/// Jolt's `Color::sGrey`, the debug colour of every material made here.
const GREY: u32 = 0xFF80_8080;

/// Most materials Jolt accepts for one heightfield (`HeightFieldShape.cpp`).
const MAX_HEIGHT_FIELD_MATERIALS: usize = 256;

/// A surface identity for shapes: a Jolt physics material that carries a `u64` of the caller's.
///
/// A material says which surface a shape is made of, for example to pick footstep sounds or
/// the friction of a contact. Jolt keeps no friction or restitution in a material; those stay
/// on the bodies. Contact events report the user data of the material on each side
/// ([`ContactManifold::materials`](crate::ContactManifold::materials)).
///
/// Owns one Jolt reference. Every shape made with the material holds its own, so the material
/// may be dropped while shapes use it. Materials made here cannot be serialized by Jolt's
/// `ObjectStream`.
pub struct PhysicsMaterial {
    material: Owned<JPH_PhysicsMaterial>,
    user_data: u64,
}

/// A material, of which the owner holds one Jolt reference.
impl JoltObject for JPH_PhysicsMaterial {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference to the material (trait contract), released here
        // once; shapes hold their own, so joltc only calls `Release`.
        unsafe { JPH_PhysicsMaterial_Destroy(ptr) };
    }
}

// SAFETY: a material is immutable after construction and Jolt's `RefTarget` counts references
// atomically, so it may be used and released from any thread.
unsafe impl Send for PhysicsMaterial {}
// SAFETY: as for `Send`; `&PhysicsMaterial` only lets shapes take references and read the
// immutable user data.
unsafe impl Sync for PhysicsMaterial {}

impl fmt::Debug for PhysicsMaterial {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PhysicsMaterial")
            .field("user_data", &self.user_data())
            .finish()
    }
}

impl PhysicsMaterial {
    /// A material that carries `user_data`.
    pub fn new(user_data: u64) -> Result<Self, ShapeError> {
        initialize()?;
        // SAFETY: Jolt is initialised and the name is a NUL-terminated literal, which Jolt
        // copies. The returned material holds one reference, which `Self` takes over.
        let material = unsafe { JPH_PhysicsMaterial_Create2(c"".as_ptr(), GREY, user_data) };
        // SAFETY: as above.
        let material = unsafe { Owned::from_raw(material) }.ok_or(ShapeError::AllocationFailed)?;
        Ok(Self {
            material,
            user_data,
        })
    }

    /// The value given to [`new`](Self::new).
    pub fn user_data(&self) -> u64 {
        self.user_data
    }

    /// Jolt's reference count of the material: this value's reference plus every shape's and
    /// character's.
    #[cfg(test)]
    pub(crate) fn reference_count(&self) -> u32 {
        // SAFETY: `self` keeps the material alive; the getter reads its count.
        unsafe { JPH_PhysicsMaterial_GetRefCount(self.as_ptr()) }
    }

    pub(crate) fn as_ptr(&self) -> *const JPH_PhysicsMaterial {
        self.material.as_ptr()
    }
}

/// The user data of the [`PhysicsMaterial`] of the leaf of `shape` that `id` leads to, or
/// `None` when that leaf has no such material (Jolt's default material, or one made elsewhere).
///
/// # Safety
/// `shape` is live, and `id` is a sub-shape id Jolt reported for this shape: Jolt decodes it
/// without range checks.
pub(crate) unsafe fn shape_material(shape: *const JPH_Shape, id: SubShapeId) -> Option<u64> {
    let mut user_data = 0;
    // SAFETY: the shape is live and `id` is valid for it (contract). Jolt's `GetMaterial` only
    // reads the shape and returns a material the shape keeps alive, which the extension reads
    // at once; `user_data` is a live local.
    unsafe {
        let material = JPH_Shape_GetMaterial(shape, id.to_raw());
        JPH_PhysicsMaterial_GetUserData(material, &mut user_data).then_some(user_data)
    }
}

impl Shape {
    /// [`new_box_with_convex_radius`](Self::new_box_with_convex_radius) made of `material`;
    /// the same rules apply.
    pub fn new_box_with_material(
        half_extent: Vec3,
        convex_radius: f32,
        material: &PhysicsMaterial,
    ) -> Result<Self, ShapeError> {
        validate_box(half_extent, convex_radius)?;
        initialize()?;
        let half_extent = half_extent.to_jph();
        // SAFETY: Jolt is initialised and `half_extent` is a live local. The returned settings
        // hold one reference, which the guard takes over.
        let settings = unsafe {
            ShapeSettings::from_raw(JPH_BoxShapeSettings_Create(&half_extent, convex_radius).cast())
        }?;
        // SAFETY: the settings are live, owned by the guard, have not created a shape yet and
        // were created as box settings, which derive from convex settings.
        unsafe { Self::attach_material(&settings, material) };
        settings.create()
    }

    /// [`new_sphere`](Self::new_sphere) made of `material`; the same rules apply.
    pub fn new_sphere_with_material(
        radius: f32,
        material: &PhysicsMaterial,
    ) -> Result<Self, ShapeError> {
        validate_sphere(radius)?;
        initialize()?;
        // SAFETY: Jolt is initialised. The returned settings hold one reference, which the guard
        // takes over.
        let settings =
            unsafe { ShapeSettings::from_raw(JPH_SphereShapeSettings_Create(radius).cast()) }?;
        // SAFETY: the settings are live, owned by the guard, have not created a shape yet and
        // were created as sphere settings, which derive from convex settings.
        unsafe { Self::attach_material(&settings, material) };
        settings.create()
    }

    /// [`new_capsule`](Self::new_capsule) made of `material`; the same rules apply.
    pub fn new_capsule_with_material(
        half_height_of_cylinder: f32,
        radius: f32,
        material: &PhysicsMaterial,
    ) -> Result<Self, ShapeError> {
        validate_capsule(half_height_of_cylinder, radius)?;
        initialize()?;
        // SAFETY: Jolt is initialised. The returned settings hold one reference, which the guard
        // takes over.
        let settings = unsafe {
            ShapeSettings::from_raw(
                JPH_CapsuleShapeSettings_Create(half_height_of_cylinder, radius).cast(),
            )
        }?;
        // SAFETY: the settings are live, owned by the guard, have not created a shape yet and
        // were created as capsule settings, which derive from convex settings.
        unsafe { Self::attach_material(&settings, material) };
        settings.create()
    }

    /// [`new_cylinder_with_convex_radius`](Self::new_cylinder_with_convex_radius) made of
    /// `material`; the same rules apply.
    pub fn new_cylinder_with_material(
        half_height: f32,
        radius: f32,
        convex_radius: f32,
        material: &PhysicsMaterial,
    ) -> Result<Self, ShapeError> {
        validate_cylinder(half_height, radius, convex_radius)?;
        initialize()?;
        // SAFETY: Jolt is initialised. The returned settings hold one reference, which the guard
        // takes over.
        let settings = unsafe {
            ShapeSettings::from_raw(
                JPH_CylinderShapeSettings_Create(half_height, radius, convex_radius).cast(),
            )
        }?;
        // SAFETY: the settings are live, owned by the guard, have not created a shape yet and
        // were created as cylinder settings, which derive from convex settings.
        unsafe { Self::attach_material(&settings, material) };
        settings.create()
    }

    /// Sets `material` on convex shape `settings`, which keep their own reference to it.
    ///
    /// # Safety
    /// `settings` are live convex shape settings (single inheritance from
    /// `ConvexShapeSettings`) that have not created a shape yet.
    pub(crate) unsafe fn attach_material(settings: &ShapeSettings, material: &PhysicsMaterial) {
        // SAFETY: the settings are live and convex (contract); the material is live.
        unsafe { JPH_ConvexShapeSettings_SetMaterial(settings.as_ptr(), material.as_ptr()) };
    }
}

/// Checks a heightfield's material list and cell indices for `sample_count` samples per side
/// and returns the native list, whose pointers stay valid while `list` borrows the materials.
pub(crate) fn height_field_materials(
    sample_count: u32,
    list: &[&PhysicsMaterial],
    indices: &[u8],
) -> Result<Vec<*const JPH_PhysicsMaterial>, ShapeError> {
    if !(1..=MAX_HEIGHT_FIELD_MATERIALS).contains(&list.len()) {
        return Err(ShapeError::InvalidValue(
            "material list must hold 1 to 256 materials",
        ));
    }
    let cells = (sample_count as usize - 1).checked_mul(sample_count as usize - 1);
    if cells != Some(indices.len()) {
        return Err(ShapeError::InvalidValue(
            "material indices must hold (sample_count - 1)^2 values",
        ));
    }
    if indices
        .iter()
        .any(|&index| usize::from(index) >= list.len())
    {
        return Err(ShapeError::InvalidValue(
            "material indices must name a material of the list",
        ));
    }
    Ok(list.iter().map(|material| material.as_ptr()).collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BodySettings, CompoundChild, PhysicsWorld, Quat, QueryFilter, RVec3, RayCast, WorldSettings,
    };

    /// The user data of the material Jolt finds for `shape` at sub-shape `id`.
    fn material_at(shape: &Shape, id: u32) -> Option<u64> {
        // SAFETY: the shape is live and `id` is its root (no sub-shapes) or an id Jolt reported
        // for it.
        unsafe { shape_material(shape.as_ptr(), SubShapeId::new(id)) }
    }

    /// The shape's local bounds as `[min, max]`.
    fn bounds(shape: &Shape) -> [[f32; 3]; 2] {
        let (min, max) = shape.local_bounds();
        [min.into(), max.into()]
    }

    const ROOT: u32 = u32::MAX;

    #[test]
    fn user_data_round_trips() {
        for value in [0, 42, u64::MAX] {
            let material = PhysicsMaterial::new(value).unwrap();
            assert_eq!(material.user_data(), value);
            assert_eq!(
                format!("{material:?}"),
                format!("PhysicsMaterial {{ user_data: {value} }}")
            );
        }
    }

    #[test]
    fn convex_constructors_keep_the_plain_shape_and_carry_the_material() {
        let material = PhysicsMaterial::new(7).unwrap();
        let half = Vec3::new(0.5, 1.0, 1.5);
        let pairs = [
            (
                Shape::new_box_with_convex_radius(half, 0.1).unwrap(),
                Shape::new_box_with_material(half, 0.1, &material).unwrap(),
            ),
            (
                Shape::new_sphere(0.7).unwrap(),
                Shape::new_sphere_with_material(0.7, &material).unwrap(),
            ),
            (
                Shape::new_capsule(0.8, 0.3).unwrap(),
                Shape::new_capsule_with_material(0.8, 0.3, &material).unwrap(),
            ),
            (
                Shape::new_cylinder_with_convex_radius(0.8, 0.3, 0.02).unwrap(),
                Shape::new_cylinder_with_material(0.8, 0.3, 0.02, &material).unwrap(),
            ),
        ];
        drop(material);
        for (plain, with_material) in &pairs {
            assert_eq!(plain.sub_type(), with_material.sub_type());
            assert_eq!(bounds(plain), bounds(with_material));
            assert_eq!(material_at(plain, ROOT), None);
            // The shape holds its own reference after the material was dropped.
            assert_eq!(material_at(with_material, ROOT), Some(7));
        }
    }

    #[test]
    fn convex_constructors_apply_the_plain_rules() {
        let material = PhysicsMaterial::new(1).unwrap();
        let too_big = crate::limits::MAX_SHAPE_EXTENT * 2.0;
        let errors = [
            Shape::new_box_with_material(Vec3::new(0.0, 1.0, 1.0), 0.0, &material).err(),
            Shape::new_box_with_material(Vec3::new(too_big, 1.0, 1.0), 0.0, &material).err(),
            Shape::new_box_with_material(Vec3::new(1.0, 1.0, 1.0), -0.1, &material).err(),
            Shape::new_sphere_with_material(f32::NAN, &material).err(),
            Shape::new_sphere_with_material(too_big, &material).err(),
            Shape::new_capsule_with_material(1.0, 0.0, &material).err(),
            Shape::new_capsule_with_material(too_big, 1.0, &material).err(),
            Shape::new_cylinder_with_material(1.0, -1.0, 0.0, &material).err(),
            Shape::new_cylinder_with_material(too_big, 1.0, 0.0, &material).err(),
            Shape::new_cylinder_with_material(1.0, 1.0, f32::INFINITY, &material).err(),
        ];
        for error in errors {
            assert!(
                matches!(error, Some(ShapeError::InvalidValue(_))),
                "{error:?}"
            );
        }
    }

    #[test]
    fn compound_children_resolve_their_own_materials() {
        let (a, b) = (
            PhysicsMaterial::new(10).unwrap(),
            PhysicsMaterial::new(20).unwrap(),
        );
        let unit = Vec3::new(0.5, 0.5, 0.5);
        let leaves = [
            Shape::new_box_with_material(unit, 0.05, &a).unwrap(),
            Shape::new_box_with_material(unit, 0.05, &b).unwrap(),
            Shape::new_box(unit).unwrap(),
        ];
        let children: Vec<_> = leaves
            .iter()
            .enumerate()
            .map(|(i, shape)| CompoundChild {
                shape,
                position: Vec3::new(3.0 * i as f32, 0.0, 0.0),
                rotation: Quat::IDENTITY,
                user_data: i as u32,
            })
            .collect();
        let compound = Shape::new_compound(&children).unwrap();
        let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
        world
            .create_body(&compound, &BodySettings::new_static())
            .unwrap();
        for (i, expected) in [Some(10), Some(20), None].into_iter().enumerate() {
            let ray = RayCast::new(
                RVec3::new(3.0 * i as crate::Real, 5.0, 0.0),
                Vec3::new(0.0, -10.0, 0.0),
            );
            let hit = world.cast_ray(&ray, &QueryFilter::new()).unwrap().unwrap();
            assert_eq!(material_at(&compound, hit.sub_shape_id.to_raw()), expected);
        }
    }

    /// A flat `n` x `n` field with `materials` and cell `(x, y)` given index `(x + y) % len`.
    fn checker_field(n: u32, materials: &[&PhysicsMaterial]) -> Shape {
        let samples = vec![0.0; (n * n) as usize];
        let cells = (n - 1) as usize;
        let indices: Vec<u8> = (0..cells * cells)
            .map(|i| ((i % cells + i / cells) % materials.len()) as u8)
            .collect();
        let settings = crate::HeightFieldSettings::default().materials(materials, &indices);
        Shape::new_height_field(n, &samples, &settings).unwrap()
    }

    fn cell_material(shape: &Shape, x: u32, y: u32) -> Option<u64> {
        let mut user_data = 0;
        // SAFETY: the shape is a live heightfield and `(x, y)` a cell of its stored grid.
        unsafe {
            let material = JPH_HeightFieldShape_GetMaterial(shape.as_ptr().cast(), x, y);
            JPH_PhysicsMaterial_GetUserData(material, &mut user_data).then_some(user_data)
        }
    }

    #[test]
    fn padded_height_field_cells_resolve_their_materials() {
        let list: Vec<PhysicsMaterial> = (0..3)
            .map(|i| PhysicsMaterial::new(100 + i).unwrap())
            .collect();
        let refs: Vec<&PhysicsMaterial> = list.iter().collect();
        // n = 33 is stored as 34 x 34 with the default block size 2.
        let field = checker_field(33, &refs);
        drop(refs);
        drop(list);
        for y in 0..32 {
            for x in 0..32 {
                let expected = 100 + u64::from((x + y) % 3);
                assert_eq!(
                    cell_material(&field, x, y),
                    Some(expected),
                    "cell ({x}, {y})"
                );
            }
        }
        // The padding cells Jolt adds get material 0.
        assert_eq!(cell_material(&field, 32, 5), Some(100));
        assert_eq!(cell_material(&field, 5, 32), Some(100));
    }

    #[test]
    fn single_material_height_field_resolves_everywhere() {
        let material = PhysicsMaterial::new(9).unwrap();
        let field = checker_field(4, &[&material]);
        assert_eq!(cell_material(&field, 0, 0), Some(9));
        assert_eq!(cell_material(&field, 2, 2), Some(9));
    }
}
