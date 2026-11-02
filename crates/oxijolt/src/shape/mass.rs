//! A shape's own mass and inertia.

use crate::body::mass_properties;
use crate::Shape;

/// The mass and inertia of a shape, from [`Shape::mass_properties`].
///
/// The values are those of a body made from the shape without a mass override, in the form
/// [`BodySettings::mass_and_inertia`](crate::BodySettings::mass_and_inertia) takes:
/// `mass_and_inertia(p.mass, Inertia::Tensor(p.inertia))` provides what the body would
/// otherwise get from the shape, under the rules of [`Inertia`](crate::Inertia).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct MassProperties {
    /// The mass in kg.
    pub mass: f32,
    /// The inertia tensor about the shape's centre of mass ([`Shape::center_of_mass`]) in the
    /// shape's axes, in kg·m². In rows, as [`Inertia::Tensor`](crate::Inertia::Tensor):
    /// `inertia[i][j]` is the entry in row `i` and column `j`, a moment on the diagonal and the
    /// negated product of inertia off it. Exactly symmetric.
    pub inertia: [[f32; 3]; 3],
}

impl Shape {
    /// The shape's mass and its inertia tensor about its centre of mass (Jolt
    /// `Shape::GetMassProperties`), computed from the shape's volume at Jolt's density of
    /// 1000 kg/m³: a box of half extents `(a, b, c)` has mass `8abc · 1000` kg and the moment
    /// `mass · (b² + c²) / 3` about x. A compound sums its children, each rotated and moved
    /// to the compound's centre of mass; a scaled shape scales its inner shape's.
    ///
    /// Meshes, heightfields and planes have no volume Jolt can compute: they give mass 0 and an
    /// all-zero tensor, and contribute nothing to a compound that contains them. A kinematic
    /// body with a mesh takes its mass from [`BodySettings::mass`].
    ///
    /// Jolt adds rotated children in `f32`, so its tensor can differ across the diagonal in the
    /// last bits. Jolt's decomposition of the tensor reads only the entries above the diagonal
    /// (`EigenValueSymmetric`); the tensor returned here copies them below it, so it is
    /// symmetric and what a body's inertia is derived from.
    ///
    /// The centre of mass is [`center_of_mass`](Self::center_of_mass).
    ///
    /// [`BodySettings::mass`]: crate::BodySettings::mass
    pub fn mass_properties(&self) -> MassProperties {
        let properties = mass_properties(self, None);
        // Column `j` of Jolt's matrix is column `j` of the tensor; rows `i < j` are kept.
        let [x, y, z, _] = properties.inertia.column;
        MassProperties {
            mass: properties.mass,
            inertia: [[x.x, y.x, z.x], [y.x, y.y, z.y], [z.x, z.y, z.z]],
        }
    }
}
