//! Shared soft-body settings: vertices, edges, faces and volumes, built and checked once.

use std::collections::BTreeMap;

use oxijolt_sys::*;

use super::skin::{validate_skin, SkinBinding};
use super::{
    SoftBodyBendType, SoftBodyInvBind, SoftBodySkinned, SoftBodyVertex, SoftBodyVertexAttributes,
};
use crate::limits::{
    self, is_compliance, is_linear_velocity, is_local_offset, is_vertex_inverse_mass,
    SoftBodyMassDistribution, SoftBodyPressureGeometry, LINEAR_VELOCITY_RULE,
};
use crate::math::{inverse_rigid_matrix, jolt_length};
use crate::owned::{JoltObject, Owned};
use crate::world::ensure_initialized;
use crate::{SoftBodyError, Vec3};

/// What a constraint compliance must satisfy.
pub(super) const COMPLIANCE_RULE: &str =
    "a compliance must be finite and within 0..=limits::MAX_COMPLIANCE";

/// What [`SoftBodySharedSettingsBuilder::create_constraints`] or
/// [`create_constraints_per_vertex`](SoftBodySharedSettingsBuilder::create_constraints_per_vertex)
/// asked for.
#[derive(Clone, Debug)]
enum GeneratedConstraints {
    Uniform(SoftBodyBendType, SoftBodyVertexAttributes),
    PerVertex(SoftBodyBendType, Vec<SoftBodyVertexAttributes>),
}

/// An edge constraint the caller adds explicitly (Jolt `SoftBodySharedSettings::Edge`): keeps
/// two vertices at their rest distance, measured when the settings are built.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyEdge {
    /// The two vertices, different, at least [`limits::MIN_SOFT_BODY_EDGE_LENGTH`] apart.
    pub vertices: [u32; 2],
    /// Compliance in m/N, `0..=`[`limits::MAX_COMPLIANCE`]; 0 is rigid.
    pub compliance: f32,
}

/// A dihedral bend constraint the caller adds explicitly (Jolt
/// `SoftBodySharedSettings::DihedralBend`): keeps the angle between two triangles that share an
/// edge at its rest angle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyDihedralBend {
    /// Four different vertices; the first two are the shared edge, at least
    /// [`limits::MIN_SOFT_BODY_EDGE_LENGTH`] long, the last two the vertices opposite it.
    pub vertices: [u32; 4],
    /// Compliance (inverse stiffness of the angle constraint), `0..=`[`limits::MAX_COMPLIANCE`];
    /// 0 is rigid.
    pub compliance: f32,
}

/// A volume constraint the caller adds explicitly (Jolt `SoftBodySharedSettings::Volume`):
/// keeps the volume of a tetrahedron at its rest volume.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyVolume {
    /// Four different vertices that span a tetrahedron with a volume.
    pub vertices: [u32; 4],
    /// Compliance (inverse stiffness of the volume constraint), `0..=`[`limits::MAX_COMPLIANCE`];
    /// 0 is rigid.
    pub compliance: f32,
}

/// The particles of a soft body and the constraints between them (Jolt
/// `SoftBodySharedSettings`), built and checked by [`SoftBodySharedSettings::builder`].
///
/// Owns one Jolt reference. Every soft body created from the settings holds its own, so the
/// settings may be dropped while bodies use them. They cannot change after they are built, and
/// one value may serve many bodies in many worlds on many threads.
pub struct SoftBodySharedSettings {
    settings: Owned<JPH_SoftBodySharedSettings>,
    pub(super) pressure_geometry: SoftBodyPressureGeometry,
    pub(super) mass_distribution: SoftBodyMassDistribution,
    pub(super) skin: Option<SkinBinding>,
}

impl std::fmt::Debug for SoftBodySharedSettings {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SoftBodySharedSettings")
            .field("vertex_count", &self.vertex_count())
            .field("face_count", &self.face_count())
            .finish_non_exhaustive()
    }
}

// SAFETY: the settings are never changed after `build` returns (no method takes `&mut self`, and
// Jolt requires shared settings to stay constant while bodies use them, `Docs/Architecture.md:427`),
// no Jolt member of `SoftBodySharedSettings` is `mutable`, and `RefTarget` counts references
// atomically, so the settings may be used and released from any thread.
unsafe impl Send for SoftBodySharedSettings {}
// SAFETY: as for `Send`; `&SoftBodySharedSettings` only lets bodies take further references and
// read the immutable settings.
unsafe impl Sync for SoftBodySharedSettings {}

/// Shared settings, of which the owner holds one reference.
impl JoltObject for JPH_SoftBodySharedSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds the one reference `JPH_SoftBodySharedSettings_Create` added
        // (trait contract), released here once; creation settings and bodies keep their own
        // (`SoftBodyCreationSettings::mSettings` is a `RefConst`).
        unsafe { JPH_SoftBodySharedSettings_Destroy(ptr) };
    }
}

impl SoftBodySharedSettings {
    /// Starts settings with `vertices` and the triangle `faces` between them, given as vertex
    /// indices. Faces are what other bodies collide with and what queries hit; without a call
    /// to [`create_constraints`](SoftBodySharedSettingsBuilder::create_constraints) or explicit
    /// constraints ([`edge`](SoftBodySharedSettingsBuilder::edge),
    /// [`dihedral_bend`](SoftBodySharedSettingsBuilder::dihedral_bend),
    /// [`volume`](SoftBodySharedSettingsBuilder::volume)) the particles are not tied together.
    ///
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// // A square of two triangles, pinned at one corner.
    /// let vertices = vec![
    ///     SoftBodyVertex::kinematic(Vec3::new(0.0, 0.0, 0.0)),
    ///     SoftBodyVertex::new(Vec3::new(1.0, 0.0, 0.0)),
    ///     SoftBodyVertex::new(Vec3::new(0.0, 0.0, 1.0)),
    ///     SoftBodyVertex::new(Vec3::new(1.0, 0.0, 1.0)),
    /// ];
    /// let settings = SoftBodySharedSettings::builder(vertices, vec![[0, 2, 3], [0, 3, 1]])
    ///     .create_constraints(
    ///         SoftBodyBendType::Distance,
    ///         SoftBodyVertexAttributes::default().bend_compliance(Some(1.0e-4)),
    ///     )
    ///     .build()?;
    /// assert_eq!(settings.vertex_count(), 4);
    /// assert!(settings.edge_constraint_count() >= 5);
    /// # Ok(())
    /// # }
    /// ```
    pub fn builder(
        vertices: Vec<SoftBodyVertex>,
        faces: Vec<[u32; 3]>,
    ) -> SoftBodySharedSettingsBuilder {
        SoftBodySharedSettingsBuilder {
            vertices,
            faces,
            generated: None,
            edges: Vec::new(),
            dihedral_bends: Vec::new(),
            volumes: Vec::new(),
            inv_binds: Vec::new(),
            skinned: Vec::new(),
        }
    }

    pub(crate) fn as_ptr(&self) -> *const JPH_SoftBodySharedSettings {
        self.settings.as_ptr()
    }

    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        // SAFETY: the settings are live and immutable; the getter only reads them.
        unsafe { JPH_SoftBodySharedSettings_GetVertexCount(self.as_ptr()) as usize }
    }

    /// Number of faces.
    pub fn face_count(&self) -> usize {
        // SAFETY: as in `vertex_count`.
        unsafe { JPH_SoftBodySharedSettings_GetFaceCount(self.as_ptr()) as usize }
    }

    /// Number of edge constraints, the generated edges, shear edges and distance bends
    /// included.
    pub fn edge_constraint_count(&self) -> usize {
        // SAFETY: as in `vertex_count`.
        unsafe { JPH_SoftBodySharedSettings_GetEdgeConstraintCount(self.as_ptr()) as usize }
    }

    /// Number of dihedral bend constraints.
    pub fn dihedral_bend_constraint_count(&self) -> usize {
        // SAFETY: as in `vertex_count`.
        unsafe { JPH_SoftBodySharedSettings_GetDihedralBendConstraintCount(self.as_ptr()) as usize }
    }

    /// Number of volume constraints.
    pub fn volume_constraint_count(&self) -> usize {
        // SAFETY: as in `vertex_count`.
        unsafe { JPH_SoftBodySharedSettings_GetVolumeConstraintCount(self.as_ptr()) as usize }
    }

    /// Number of long range attachment constraints.
    pub fn lra_constraint_count(&self) -> usize {
        // SAFETY: as in `vertex_count`.
        unsafe { JPH_SoftBodySharedSettings_GetLRAConstraintCount(self.as_ptr()) as usize }
    }

    /// Number of inverse binds.
    pub fn inv_bind_count(&self) -> usize {
        // SAFETY: as in `vertex_count`.
        unsafe { JPH_SoftBodySharedSettings_GetInvBindCount(self.as_ptr()) as usize }
    }

    /// Number of skinned constraints.
    pub fn skinned_constraint_count(&self) -> usize {
        // SAFETY: as in `vertex_count`.
        unsafe { JPH_SoftBodySharedSettings_GetSkinnedConstraintCount(self.as_ptr()) as usize }
    }

    /// How many joint transforms a skinning call needs: 1 + the largest joint of the inverse
    /// binds, 0 for settings without skinning.
    pub fn skin_joint_count(&self) -> u32 {
        self.skin.as_ref().map_or(0, |skin| skin.joint_count)
    }
}

/// Collects the vertices, faces and constraints of [`SoftBodySharedSettings`]; made by
/// [`SoftBodySharedSettings::builder`].
#[derive(Clone, Debug)]
#[must_use]
pub struct SoftBodySharedSettingsBuilder {
    vertices: Vec<SoftBodyVertex>,
    faces: Vec<[u32; 3]>,
    generated: Option<GeneratedConstraints>,
    edges: Vec<SoftBodyEdge>,
    dihedral_bends: Vec<SoftBodyDihedralBend>,
    volumes: Vec<SoftBodyVolume>,
    inv_binds: Vec<SoftBodyInvBind>,
    skinned: Vec<SoftBodySkinned>,
}

impl SoftBodySharedSettingsBuilder {
    /// Creates edge, shear, bend and LRA constraints from the faces, every vertex with the
    /// same `attributes` (Jolt `SoftBodySharedSettings::CreateConstraints`, with its default
    /// angle tolerance of 8° for detecting quads). Replaces an earlier call.
    pub fn create_constraints(
        mut self,
        bend: SoftBodyBendType,
        attributes: SoftBodyVertexAttributes,
    ) -> Self {
        self.generated = Some(GeneratedConstraints::Uniform(bend, attributes));
        self
    }

    /// Like [`create_constraints`](Self::create_constraints) with one attribute set per vertex,
    /// in vertex order; there must be exactly as many as vertices.
    pub fn create_constraints_per_vertex(
        mut self,
        bend: SoftBodyBendType,
        attributes: Vec<SoftBodyVertexAttributes>,
    ) -> Self {
        self.generated = Some(GeneratedConstraints::PerVertex(bend, attributes));
        self
    }

    /// Adds an edge constraint, kept in addition to the generated ones. Long range attachments
    /// follow the generated edges only.
    pub fn edge(mut self, edge: SoftBodyEdge) -> Self {
        self.edges.push(edge);
        self
    }

    /// Adds a dihedral bend constraint, kept in addition to the generated ones.
    pub fn dihedral_bend(mut self, bend: SoftBodyDihedralBend) -> Self {
        self.dihedral_bends.push(bend);
        self
    }

    /// Adds a volume constraint, for example one per tetrahedron of a solid soft body.
    pub fn volume(mut self, volume: SoftBodyVolume) -> Self {
        self.volumes.push(volume);
        self
    }

    /// Adds the bind pose of a joint for skinning; skinned constraints name binds by the order
    /// in which they were added. See [docs/soft-bodies.md].
    ///
    /// [docs/soft-bodies.md]: https://github.com/pockerhead/oxijolt/blob/main/docs/soft-bodies.md
    pub fn inv_bind(mut self, bind: SoftBodyInvBind) -> Self {
        self.inv_binds.push(bind);
        self
    }

    /// Adds several binds as [`inv_bind`](Self::inv_bind) does, for example those of
    /// [`SoftBodyInvBind::from_pose`].
    pub fn inv_binds(mut self, binds: impl IntoIterator<Item = SoftBodyInvBind>) -> Self {
        self.inv_binds.extend(binds);
        self
    }

    /// Skins a vertex to the joints of the inverse binds.
    pub fn skinned(mut self, skinned: SoftBodySkinned) -> Self {
        self.skinned.push(skinned);
        self
    }

    /// Checks everything and builds the settings; nothing is allocated when a check fails.
    ///
    /// Fails with [`SoftBodyError::InvalidValue`] when:
    /// - there is no vertex;
    /// - a vertex position is beyond [`limits::MAX_SHAPE_EXTENT`], a velocity beyond
    ///   [`limits::MAX_LINEAR_VELOCITY`], or an inverse mass neither 0 nor the inverse of a mass
    ///   within [`limits::MIN_MASS`]`..=`[`limits::MAX_MASS`];
    /// - the masses of the movable vertices add up to more than [`limits::MAX_MASS`] (Jolt gives
    ///   the body their sum as its mass);
    /// - a face names a vertex that does not exist or one vertex twice, has no area, or has an
    ///   edge shorter than [`limits::MIN_SOFT_BODY_EDGE_LENGTH`];
    /// - with [`SoftBodyBendType::Distance`], the vertices opposite a shared edge are closer than
    ///   that length (the bend edge between them would have no length);
    /// - an attribute is out of range, or a per-vertex attribute does not have one value per
    ///   vertex;
    /// - an explicit constraint names a vertex that does not exist or one vertex twice, has a
    ///   compliance out of range or an edge (or a bend's shared edge) shorter than
    ///   [`limits::MIN_SOFT_BODY_EDGE_LENGTH`], or is a tetrahedron without volume;
    /// - there are inverse binds without skinned constraints or the reverse; an inverse bind's
    ///   joint is not below [`limits::MAX_SKIN_JOINTS`] or has another bind, or its position is
    ///   beyond [`limits::MAX_SHAPE_EXTENT`] or its rotation not a unit quaternion;
    /// - a skinned constraint names a vertex that does not exist or has another skinned
    ///   constraint; a weight is negative or not finite, none is positive, a positive one names
    ///   no inverse bind or rounds to 0 once normalised; the max distance is beyond
    ///   [`limits::MAX_SHAPE_EXTENT`]; the back stop's distance is beyond it or not below the
    ///   max distance, its radius not within `(0, MAX_SHAPE_EXTENT]`, or no face around the
    ///   vertex has three skinned vertices;
    /// - a skinned vertex touches 256 faces or more whose vertices are all skinned, or these
    ///   counts add up to 2^24 or more (Jolt packs them into 8 and 24 bits; see
    ///   [docs/limits.md#soft-body-skinning]).
    ///
    /// [docs/limits.md#soft-body-skinning]: https://github.com/pockerhead/oxijolt/blob/main/docs/limits.md#soft-body-skinning
    pub fn build(self) -> Result<SoftBodySharedSettings, SoftBodyError> {
        if !ensure_initialized() {
            return Err(SoftBodyError::InitFailed);
        }
        self.validate()?;
        let skin = validate_skin(
            self.vertices.len(),
            &self.faces,
            &self.inv_binds,
            &self.skinned,
        )?;
        let positions: Vec<Vec3> = self.vertices.iter().map(|v| v.position).collect();
        let pressure_geometry = SoftBodyPressureGeometry::new(&positions, &self.faces);
        let mass_distribution = SoftBodyMassDistribution::new(
            self.vertices.iter().map(|v| (v.position, v.inverse_mass)),
        );
        // SAFETY: Jolt is initialised. The handle takes over the one reference joltc's
        // `_Create` adds.
        let owned = unsafe { Owned::from_raw(JPH_SoftBodySharedSettings_Create()) }
            .unwrap_or_else(|| unreachable!("`new` does not return null"));
        let settings = SoftBodySharedSettings {
            settings: owned,
            pressure_geometry,
            mass_distribution,
            skin: skin.as_ref().map(|skin| skin.binding.clone()),
        };
        let ptr = settings.settings.as_ptr();
        let vertices: Vec<JPH_SoftVertex> = self.vertices.iter().map(|v| v.to_jph()).collect();
        let faces: Vec<JPH_SoftFace> = self
            .faces
            .iter()
            .map(|&[vertex1, vertex2, vertex3]| JPH_SoftFace {
                vertex1,
                vertex2,
                vertex3,
                // joltc's `_Create` adds Jolt's default material at index 0.
                materialIndex: 0,
            })
            .collect();
        // SAFETY: `ptr` is the live settings object owned by `settings`, which nothing else
        // uses yet. Both arrays are live for the calls and hold the counts passed, which
        // `validate` checked to fit `u32`; every face index names a vertex.
        unsafe {
            JPH_SoftBodySharedSettings_AddVertices(ptr, vertices.as_ptr(), vertices.len() as u32);
            JPH_SoftBodySharedSettings_AddFaces(ptr, faces.as_ptr(), faces.len() as u32);
        }
        if let Some(generated) = &self.generated {
            let (bend, attributes) = match generated {
                GeneratedConstraints::Uniform(bend, attributes) => {
                    (bend, vec![attributes.to_jph()])
                }
                GeneratedConstraints::PerVertex(bend, attributes) => {
                    (bend, attributes.iter().map(|a| a.to_jph()).collect())
                }
            };
            // SAFETY: `ptr` is live and holds the checked vertices and faces; `attributes` holds
            // at least one set (`validate`) and lives for the call.
            unsafe {
                JPH_SoftBodySharedSettings_CreateConstraints2(
                    ptr,
                    attributes.as_ptr(),
                    attributes.len() as u32,
                    bend.to_jph(),
                );
            }
        }
        // Generating constraints replaces the edges, so explicit ones come after it.
        self.add_explicit_constraints(ptr);
        if let Some(skin) = &skin {
            add_skin(ptr, &skin.binding, &skin.constraints);
        }
        // SAFETY: `ptr` is live; Jolt reorders the constraints for its solver, the last change
        // the settings get.
        unsafe { JPH_SoftBodySharedSettings_Optimize(ptr) };
        Ok(settings)
    }

    /// Adds the explicit constraints to `ptr` and computes their rest values.
    fn add_explicit_constraints(&self, ptr: *mut JPH_SoftBodySharedSettings) {
        // SAFETY: `ptr` is the live settings object `build` owns, which holds the checked
        // vertices; `validate` checked every index and the geometry the rest values need.
        unsafe {
            for edge in &self.edges {
                let [a, b] = edge.vertices;
                JPH_SoftBodySharedSettings_AddEdgeConstraint(ptr, a, b, edge.compliance);
            }
            for bend in &self.dihedral_bends {
                let [a, b, c, d] = bend.vertices;
                JPH_SoftBodySharedSettings_AddDihedralBendConstraint(
                    ptr,
                    a,
                    b,
                    c,
                    d,
                    bend.compliance,
                );
            }
            for volume in &self.volumes {
                let [a, b, c, d] = volume.vertices;
                JPH_SoftBodySharedSettings_AddVolumeConstraint(ptr, a, b, c, d, volume.compliance);
            }
            if !self.edges.is_empty() {
                JPH_SoftBodySharedSettings_CalculateEdgeLengths(ptr);
            }
            if !self.dihedral_bends.is_empty() {
                JPH_SoftBodySharedSettings_CalculateBendConstraintConstants(ptr);
            }
            if !self.volumes.is_empty() {
                JPH_SoftBodySharedSettings_CalculateVolumeConstraintVolumes(ptr);
            }
        }
    }

    fn validate(&self) -> Result<(), SoftBodyError> {
        require(
            !self.vertices.is_empty(),
            "a soft body needs at least one vertex",
        )?;
        require(
            u32::try_from(self.vertices.len()).is_ok() && u32::try_from(self.faces.len()).is_ok(),
            "vertex and face counts must fit u32",
        )?;
        let mut total_mass = 0.0_f64;
        for vertex in &self.vertices {
            require(
                is_local_offset(vertex.position),
                "a vertex position must be finite and within limits::MAX_SHAPE_EXTENT",
            )?;
            require(is_linear_velocity(vertex.velocity), LINEAR_VELOCITY_RULE)?;
            if vertex.inverse_mass != 0.0 {
                require(
                    is_vertex_inverse_mass(vertex.inverse_mass),
                    VERTEX_INVERSE_MASS_RULE,
                )?;
                total_mass += vertex_mass(vertex.inverse_mass);
            }
        }
        require(total_mass <= f64::from(limits::MAX_MASS), TOTAL_MASS_RULE)?;
        for &face in &self.faces {
            self.validate_face(face)?;
        }
        if let Some(generated) = &self.generated {
            let bend = match generated {
                GeneratedConstraints::Uniform(bend, attributes) => {
                    attributes.validate()?;
                    *bend
                }
                GeneratedConstraints::PerVertex(bend, attributes) => {
                    require(
                        attributes.len() == self.vertices.len(),
                        "per-vertex attributes need one set per vertex",
                    )?;
                    for attributes in attributes {
                        attributes.validate()?;
                    }
                    *bend
                }
            };
            if bend == SoftBodyBendType::Distance {
                self.validate_distance_bends()?;
            }
        }
        self.validate_explicit_constraints()
    }

    /// Whether `indices` name different vertices.
    fn names_different_vertices(&self, indices: &[u32]) -> Result<(), SoftBodyError> {
        let count = self.vertices.len();
        require(
            indices.iter().all(|&index| (index as usize) < count),
            "a constraint index must name a vertex",
        )?;
        let distinct = indices
            .iter()
            .enumerate()
            .all(|(i, a)| indices[i + 1..].iter().all(|b| a != b));
        require(distinct, "a constraint must name different vertices")
    }

    fn validate_explicit_constraints(&self) -> Result<(), SoftBodyError> {
        for edge in &self.edges {
            self.names_different_vertices(&edge.vertices)?;
            require(is_compliance(edge.compliance), COMPLIANCE_RULE)?;
            let [a, b] = edge.vertices.map(|index| self.position(index));
            require(is_edge_length(a, b), EDGE_LENGTH_RULE)?;
        }
        for bend in &self.dihedral_bends {
            self.names_different_vertices(&bend.vertices)?;
            require(is_compliance(bend.compliance), COMPLIANCE_RULE)?;
            let [a, b, _, _] = bend.vertices.map(|index| self.position(index));
            require(
                is_edge_length(a, b),
                "the shared edge of a bend must be at least limits::MIN_SOFT_BODY_EDGE_LENGTH long",
            )?;
        }
        for volume in &self.volumes {
            self.names_different_vertices(&volume.vertices)?;
            require(is_compliance(volume.compliance), COMPLIANCE_RULE)?;
            let [x1, x2, x3, x4] = volume.vertices.map(|index| self.position(index));
            // Jolt's six times the rest volume, in f32 (`CalculateVolumeConstraintVolumes`).
            let six_volume = dot(
                cross(difference(x1, x2), difference(x1, x3)),
                difference(x1, x4),
            );
            require(
                six_volume.is_finite() && six_volume != 0.0,
                "a volume constraint must span a tetrahedron with a volume",
            )?;
        }
        Ok(())
    }

    fn position(&self, index: u32) -> Vec3 {
        self.vertices[index as usize].position
    }

    fn validate_face(&self, face: [u32; 3]) -> Result<(), SoftBodyError> {
        let count = self.vertices.len();
        require(
            face.iter().all(|&index| (index as usize) < count),
            "a face index must name a vertex",
        )?;
        let [a, b, c] = face;
        require(
            a != b && b != c && a != c,
            "a face must name three different vertices",
        )?;
        let [pa, pb, pc] = face.map(|index| self.position(index));
        for (from, to) in [(pa, pb), (pb, pc), (pc, pa)] {
            require(is_edge_length(from, to), EDGE_LENGTH_RULE)?;
        }
        let area = jolt_length(cross(difference(pa, pb), difference(pa, pc)));
        require(area.is_finite() && area > 0.0, "a face must have an area")
    }

    /// With distance bends, Jolt joins the two vertices opposite each shared edge by an edge
    /// (`SoftBodySharedSettings.cpp:295-302`), which must have a length like every edge.
    fn validate_distance_bends(&self) -> Result<(), SoftBodyError> {
        let mut opposite: BTreeMap<(u32, u32), Vec<u32>> = BTreeMap::new();
        for face in &self.faces {
            for i in 0..3 {
                let (v0, v1) = (face[i], face[(i + 1) % 3]);
                opposite
                    .entry((v0.min(v1), v0.max(v1)))
                    .or_default()
                    .push(face[(i + 2) % 3]);
            }
        }
        for vertices in opposite.values() {
            for (i, &first) in vertices.iter().enumerate() {
                for &second in &vertices[i + 1..] {
                    if first != second {
                        // With distance bends, Jolt keeps these two vertices apart.
                        require(
                            is_edge_length(self.position(first), self.position(second)),
                            "vertices opposite a shared edge must be at least \
                             limits::MIN_SOFT_BODY_EDGE_LENGTH apart",
                        )?;
                    }
                }
            }
        }
        Ok(())
    }
}

/// Adds the inverse binds and the skinned constraints to `ptr` and computes the skinned
/// normals, which must come before `Optimize` reorders the constraints.
fn add_skin(
    ptr: *mut JPH_SoftBodySharedSettings,
    binding: &SkinBinding,
    constraints: &[JPH_SoftBodySkinnedConstraint],
) {
    for &(joint, position, rotation) in &binding.binds {
        let inverse = inverse_rigid_matrix(position, rotation);
        // SAFETY: `ptr` is the live settings object `build` owns; `inverse` is a live local
        // that joltc copies.
        let added = unsafe { JPH_SoftBodySharedSettings_AddInvBind(ptr, joint, &inverse) };
        if !added {
            // `validate_skin` kept every joint below `MAX_SKIN_JOINTS`, so none is the refused
            // `u32::MAX`.
            unreachable!("a validated inverse bind was refused");
        }
    }
    for constraint in constraints {
        // SAFETY: `ptr` is live and holds the vertices and every inverse bind; `constraint` is
        // a live element that joltc copies.
        let added = unsafe { JPH_SoftBodySharedSettings_AddSkinnedConstraint(ptr, constraint) };
        if !added {
            // `validate_skin` checked the vertex and the inverse bind of every positive weight,
            // the only refusals of the native call.
            unreachable!("a validated skinned constraint was refused");
        }
    }
    // SAFETY: `ptr` is live; `validate_skin` kept the per-vertex face counts below 256 and
    // their total below 2^24, as Jolt asserts.
    unsafe { JPH_SoftBodySharedSettings_CalculateSkinnedConstraintNormals(ptr) };
}

/// What the inverse mass of a soft body vertex must satisfy: 0 (kinematic), or the inverse
/// of a mass within [`limits::MIN_MASS`]`..=`[`limits::MAX_MASS`]
/// ([`limits::is_vertex_inverse_mass`]).
pub(super) const VERTEX_INVERSE_MASS_RULE: &str =
    "vertex inverse mass must be 0 or within 1 / limits::MAX_MASS..=limits::MAX_VERTEX_INVERSE_MASS";
/// What the vertex masses of a soft body must satisfy together.
pub(super) const TOTAL_MASS_RULE: &str =
    "the masses of the movable vertices must add up to at most limits::MAX_MASS";
/// What the movable vertices of a soft body without a kinematic vertex must satisfy: they
/// spread around the body origin so that Jolt can decompose their inertia (see [`limits`]).
pub(super) const INERTIA_RULE: &str =
    "without a kinematic vertex, the vertices must spread around the body origin";
/// What an edge between two vertices must satisfy.
pub(super) const EDGE_LENGTH_RULE: &str =
    "an edge must be at least limits::MIN_SOFT_BODY_EDGE_LENGTH long";

/// The mass of a movable vertex as Jolt computes it (`1.0f / mInvMass`,
/// `SoftBodyMotionProperties.cpp:33`), for summing in `f64`.
pub(super) fn vertex_mass(inverse_mass: f32) -> f64 {
    f64::from(1.0 / inverse_mass)
}

pub(super) fn require(valid: bool, what: &'static str) -> Result<(), SoftBodyError> {
    if valid {
        Ok(())
    } else {
        Err(SoftBodyError::InvalidValue(what))
    }
}

/// `to - from` in `f32`, as Jolt subtracts vertex positions.
fn difference(from: Vec3, to: Vec3) -> Vec3 {
    Vec3::new(to.x - from.x, to.y - from.y, to.z - from.z)
}

fn dot(a: Vec3, b: Vec3) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

/// Whether `from` and `to` are at least [`limits::MIN_SOFT_BODY_EDGE_LENGTH`] apart, measured
/// as Jolt measures a rest length: an `f32` difference and Jolt's own `f32` length.
pub(super) fn is_edge_length(from: Vec3, to: Vec3) -> bool {
    jolt_length(difference(from, to)) >= limits::MIN_SOFT_BODY_EDGE_LENGTH
}
