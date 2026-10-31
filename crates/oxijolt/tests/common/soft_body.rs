//! Soft body scenes shared by the integration tests.

use oxijolt::*;

/// A square cloth of `side` x `side` vertices in the XZ plane, centred on the origin, with
/// `spacing` metres between neighbours and two triangles per cell whose faces point up (+Y).
/// Vertex `z * side + x` sits at column `x`, row `z`.
pub struct Cloth {
    pub side: u32,
    pub spacing: f32,
    pub vertices: Vec<SoftBodyVertex>,
    pub faces: Vec<[u32; 3]>,
}

impl Cloth {
    pub fn new(side: u32, spacing: f32) -> Self {
        let half = 0.5 * spacing * (side - 1) as f32;
        let mut vertices = Vec::new();
        for z in 0..side {
            for x in 0..side {
                let position = Vec3::new(x as f32 * spacing - half, 0.0, z as f32 * spacing - half);
                vertices.push(SoftBodyVertex::new(position));
            }
        }
        let mut faces = Vec::new();
        for z in 0..side - 1 {
            for x in 0..side - 1 {
                let v00 = z * side + x;
                let (v10, v01, v11) = (v00 + 1, v00 + side, v00 + side + 1);
                faces.push([v00, v01, v11]);
                faces.push([v00, v11, v10]);
            }
        }
        Self {
            side,
            spacing,
            vertices,
            faces,
        }
    }

    pub fn index(&self, x: u32, z: u32) -> u32 {
        z * self.side + x
    }

    /// Makes the listed vertices kinematic.
    pub fn pin(mut self, pinned: &[u32]) -> Self {
        for &index in pinned {
            self.vertices[index as usize].inverse_mass = 0.0;
        }
        self
    }

    /// The two corners of the row `z = 0`.
    pub fn first_row_corners(&self) -> [u32; 2] {
        [self.index(0, 0), self.index(self.side - 1, 0)]
    }

    /// The grid edges (rows and columns, not diagonals) as vertex index pairs.
    pub fn grid_edges(&self) -> Vec<[u32; 2]> {
        let mut edges = Vec::new();
        for z in 0..self.side {
            for x in 0..self.side {
                if x + 1 < self.side {
                    edges.push([self.index(x, z), self.index(x + 1, z)]);
                }
                if z + 1 < self.side {
                    edges.push([self.index(x, z), self.index(x, z + 1)]);
                }
            }
        }
        edges
    }

    pub fn builder(&self) -> SoftBodySharedSettingsBuilder {
        SoftBodySharedSettings::builder(self.vertices.clone(), self.faces.clone())
    }

    /// The cloth without constraints, its rows `z < rows.len()` skinned to joint 0 (bound at
    /// the origin, unturned), row `z` with `rows[z]` as its max distance.
    pub fn skinned(&self, rows: &[Option<f32>]) -> SoftBodySharedSettingsBuilder {
        let mut builder =
            self.builder()
                .inv_bind(SoftBodyInvBind::new(0, Vec3::ZERO, Quat::IDENTITY));
        let only = |inv_bind| {
            let mut weights = [SoftBodySkinWeight::default(); 4];
            weights[0] = SoftBodySkinWeight::new(inv_bind, 1.0);
            weights
        };
        for (z, &max_distance) in rows.iter().enumerate() {
            for x in 0..self.side {
                builder = builder.skinned(SoftBodySkinned {
                    max_distance,
                    ..SoftBodySkinned::new(self.index(x, z as u32), only(0))
                });
            }
        }
        builder
    }

    /// Shared settings with dihedral bends and geodesic long range attachments.
    pub fn settings(&self) -> SoftBodySharedSettings {
        self.builder()
            .create_constraints(SoftBodyBendType::Dihedral, cloth_attributes())
            .build()
            .unwrap()
    }
}

/// Rigid edges, soft bends and geodesic LRA constraints that allow 5 % stretch.
pub fn cloth_attributes() -> SoftBodyVertexAttributes {
    SoftBodyVertexAttributes::default()
        .bend_compliance(Some(1.0))
        .long_range_attachment(LongRangeAttachment::GeodesicDistance, 1.05)
}

/// A closed sphere of `radius` metres: a UV sphere with `rings` rings and `segments` segments,
/// faces wound counter-clockwise seen from outside.
pub fn sphere(radius: f32, rings: u32, segments: u32) -> (Vec<SoftBodyVertex>, Vec<[u32; 3]>) {
    let mut vertices = vec![SoftBodyVertex::new(Vec3::new(0.0, radius, 0.0))];
    for ring in 1..rings {
        let polar = std::f32::consts::PI * ring as f32 / rings as f32;
        for segment in 0..segments {
            let azimuth = 2.0 * std::f32::consts::PI * segment as f32 / segments as f32;
            vertices.push(SoftBodyVertex::new(Vec3::new(
                radius * polar.sin() * azimuth.cos(),
                radius * polar.cos(),
                radius * polar.sin() * azimuth.sin(),
            )));
        }
    }
    let bottom = vertices.len() as u32;
    vertices.push(SoftBodyVertex::new(Vec3::new(0.0, -radius, 0.0)));
    let at = |ring: u32, segment: u32| 1 + (ring - 1) * segments + segment % segments;
    let mut faces = Vec::new();
    for segment in 0..segments {
        faces.push([0, at(1, segment + 1), at(1, segment)]);
        faces.push([bottom, at(rings - 1, segment), at(rings - 1, segment + 1)]);
    }
    for ring in 1..rings - 1 {
        for segment in 0..segments {
            let (a, b) = (at(ring, segment), at(ring, segment + 1));
            let (c, d) = (at(ring + 1, segment), at(ring + 1, segment + 1));
            faces.push([a, b, d]);
            faces.push([a, d, c]);
        }
    }
    (vertices, faces)
}

/// The largest vertex speed of soft body `id`, in m/s.
pub fn max_vertex_speed(world: &PhysicsWorld, id: BodyId) -> f32 {
    world
        .soft_body(id)
        .unwrap()
        .vertices()
        .iter()
        .map(|v| super::length(v.velocity))
        .fold(0.0, f32::max)
}

/// The distance between two world positions, in metres.
pub fn distance(a: RVec3, b: RVec3) -> f64 {
    let d = [a.x - b.x, a.y - b.y, a.z - b.z].map(f64::from);
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// The six tetrahedra of a cube around its diagonal from corner 0 to corner 7, corner
/// `x + 2y + 4z` at `(x, y, z)`.
pub const CUBE_TETRAHEDRA: [[u32; 4]; 6] = [
    [0, 1, 3, 7],
    [0, 3, 2, 7],
    [0, 2, 6, 7],
    [0, 6, 4, 7],
    [0, 4, 5, 7],
    [0, 5, 1, 7],
];

/// The twelve surface triangles of the same cube, wound counter-clockwise seen from outside.
pub const CUBE_FACES: [[u32; 3]; 12] = [
    [0, 4, 6],
    [0, 6, 2],
    [1, 3, 7],
    [1, 7, 5],
    [0, 1, 5],
    [0, 5, 4],
    [2, 6, 7],
    [2, 7, 3],
    [0, 2, 3],
    [0, 3, 1],
    [4, 5, 7],
    [4, 7, 6],
];

/// A solid cube of `side` metres centred on the origin: the surface faces for collisions, an
/// explicit edge along every tetrahedron edge and a volume constraint per tetrahedron.
pub fn tetrahedral_cube(side: f32) -> SoftBodySharedSettingsBuilder {
    let vertices = (0..8)
        .map(|corner: u32| {
            let at = |bit: u32| side * ((corner >> bit) & 1) as f32 - 0.5 * side;
            SoftBodyVertex::new(Vec3::new(at(0), at(1), at(2)))
        })
        .collect();
    let mut builder = SoftBodySharedSettings::builder(vertices, CUBE_FACES.to_vec());
    let mut edges = std::collections::BTreeSet::new();
    for tetrahedron in CUBE_TETRAHEDRA {
        for i in 0..4 {
            for j in i + 1..4 {
                let (a, b) = (tetrahedron[i], tetrahedron[j]);
                edges.insert((a.min(b), a.max(b)));
            }
        }
        builder = builder.volume(SoftBodyVolume {
            vertices: tetrahedron,
            compliance: 0.0,
        });
    }
    for (a, b) in edges {
        builder = builder.edge(SoftBodyEdge {
            vertices: [a, b],
            compliance: 1.0e-4,
        });
    }
    builder
}

/// The volume a closed soft body's faces enclose, from its vertices' world positions.
pub fn enclosed_volume(world: &PhysicsWorld, id: BodyId, faces: &[[u32; 3]]) -> f64 {
    let vertices = world.soft_body(id).unwrap().vertices();
    let p = |index: u32| {
        let v = vertices[index as usize].position;
        [v.x, v.y, v.z].map(f64::from)
    };
    faces
        .iter()
        .map(|&[a, b, c]| {
            let (a, b, c) = (p(a), p(b), p(c));
            let cross = [
                b[1] * c[2] - b[2] * c[1],
                b[2] * c[0] - b[0] * c[2],
                b[0] * c[1] - b[1] * c[0],
            ];
            (a[0] * cross[0] + a[1] * cross[1] + a[2] * cross[2]) / 6.0
        })
        .sum()
}
