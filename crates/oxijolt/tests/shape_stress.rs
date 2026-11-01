//! Seeded stress of the shape constructors and the compound editor: random inputs per shape
//! family, a named expectation where the outcome is defined, and collision use of a share of the
//! accepted shapes.
//!
//! The cases run in a child process (the ignored test `shape_stress_child`), so a Jolt assertion
//! (asserts builds abort on it) or a crash fails the parent test, which names the last case the
//! child started. Every case is independent of the worker count and of the other cases' outcomes.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use std::io::Write as _;
use std::process::Command;

use common::*;
use oxijolt::*;

#[path = "shape_stress/mutable.rs"]
mod mutable;
#[path = "shape_stress/scaled.rs"]
mod scaled;
#[path = "shape_stress/tapered.rs"]
mod tapered;

const CHILD: &str = "shape_stress_child";

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn shapes_survive_seeded_stress() {
    let output = Command::new(std::env::current_exe().unwrap())
        .args([
            CHILD,
            "--exact",
            "--ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .output()
        .unwrap();
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        let last = stderr
            .lines()
            .rev()
            .find(|line| line.starts_with("case "))
            .unwrap_or("before the first case");
        // With --nocapture the child's panic message is on stderr between the case lines.
        let messages: Vec<&str> = stderr
            .lines()
            .filter(|line| !line.starts_with("case "))
            .collect();
        let messages = messages.join("\n");
        let tail = &messages[messages.len().saturating_sub(4000)..];
        panic!(
            "the stress child failed ({}) at {last}\nstderr:\n{tail}",
            output.status
        );
    }
}

#[test]
#[ignore = "run in its own process by shapes_survive_seeded_stress"]
fn shape_stress_child() {
    let mut arena = Arena::new();
    hull_family(&mut arena);
    mesh_family(&mut arena);
    scaled::scaled_family(&mut arena);
    tapered::tapered_family(&mut arena);
    mutable::mutable_family(&mut arena);
}

/// Announces a case on stderr before it runs, so a crash can be traced to it.
fn announce(family: &str, index: usize) {
    eprintln!("case {family} {index}");
    std::io::stderr().flush().unwrap();
}

/// SplitMix64: a fixed-seed generator, so every run sees the same cases.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.unit()
    }

    /// Log-uniform in `[low, high)`, both positive.
    fn log_range(&mut self, low: f64, high: f64) -> f64 {
        self.range(low.ln(), high.ln()).exp()
    }

    /// Uniform in `low..high`.
    fn below(&mut self, low: usize, high: usize) -> usize {
        low + (self.next_u64() % (high - low) as u64) as usize
    }

    /// A uniformly random rotation (Shoemake's method) as a row-major matrix.
    fn rotation(&mut self) -> [[f64; 3]; 3] {
        let (u1, u2, u3) = (self.unit(), self.unit(), self.unit());
        let tau = std::f64::consts::TAU;
        let (a, b) = ((1.0 - u1).sqrt(), u1.sqrt());
        let (x, y, z, w) = (
            a * (tau * u2).sin(),
            a * (tau * u2).cos(),
            b * (tau * u3).sin(),
            b * (tau * u3).cos(),
        );
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    }
}

fn rotate(m: &[[f64; 3]; 3], p: [f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|row| m[row][0] * p[0] + m[row][1] * p[1] + m[row][2] * p[2])
}

// `Real` is `f32` without the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn real(value: f32) -> Real {
    Real::from(value)
}

fn to_vec3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

/// Where a shape's input geometry lies in shape space: lowest and highest y and the centroid.
#[derive(Clone, Copy, Debug)]
struct Extent {
    min_y: f32,
    max_y: f32,
    centroid: Vec3,
}

impl Extent {
    fn of(points: &[Vec3]) -> Self {
        let count = points.len() as f64;
        let sum = points.iter().fold([0.0f64; 3], |s, p| {
            [
                s[0] + f64::from(p.x),
                s[1] + f64::from(p.y),
                s[2] + f64::from(p.z),
            ]
        });
        Self {
            min_y: points.iter().map(|p| p.y).fold(f32::INFINITY, f32::min),
            max_y: points.iter().map(|p| p.y).fold(f32::NEG_INFINITY, f32::max),
            centroid: to_vec3(sum.map(|c| c / count)),
        }
    }

    fn height(&self) -> f32 {
        self.max_y - self.min_y
    }
}

/// One world reused by every case, with a static floor whose top face is at y = 0.
struct Arena {
    world: PhysicsWorld,
    /// The dynamic shapes dropped onto static shapes under test.
    probes: Vec<Shape>,
    /// A box as large as the convex shapes a mesh takes by default.
    large_probe: Shape,
}

impl Arena {
    fn new() -> Self {
        let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
        let floor = Shape::new_box(Vec3::new(2000.0, 1.0, 2000.0)).unwrap();
        world
            .create_body(
                &floor,
                &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)),
            )
            .unwrap();
        let probes = vec![
            Shape::new_sphere(0.3).unwrap(),
            Shape::new_box(Vec3::new(0.3, 0.3, 0.3)).unwrap(),
            Shape::new_capsule(0.3, 0.2).unwrap(),
            Shape::new_convex_hull_with_convex_radius(
                &[
                    Vec3::new(-0.3, -0.2, -0.3),
                    Vec3::new(0.3, -0.25, -0.2),
                    Vec3::new(0.0, -0.3, 0.35),
                    Vec3::new(0.05, 0.35, 0.0),
                    Vec3::new(-0.2, 0.1, 0.2),
                ],
                0.05,
            )
            .unwrap(),
        ];
        let half = MeshSettings::DEFAULT_MAX_CONVEX_EXTENT;
        let large_probe = Shape::new_box(Vec3::new(half, 1.0, half)).unwrap();
        Self {
            world,
            probes,
            large_probe,
        }
    }

    /// Drops `shape` as a dynamic body with its lowest point 0.5 m above the floor, when the
    /// world accepts it as dynamic, and steps `ticks` times; the body must stay finite. Returns
    /// whether the world accepted the shape as dynamic.
    fn drop_dynamic(&mut self, shape: &Shape, extent: Extent, ticks: usize, what: &str) -> bool {
        let position = RVec3::new(
            -real(extent.centroid.x),
            real(0.5 - extent.min_y),
            -real(extent.centroid.z),
        );
        let settings = BodySettings::new_dynamic().position(position);
        let Ok(id) = self.world.create_body(shape, &settings) else {
            return false;
        };
        for _ in 0..ticks {
            assert!(self.world.step(DT).unwrap().is_complete());
        }
        assert_finite_body(&self.world, id, what);
        self.world.remove_body(id).unwrap();
        true
    }

    /// Adds `shape` as a static body at the origin and runs a down-ray and a capsule overlap
    /// through the centroid of its input geometry; both must report finite results.
    fn query_static(&mut self, shape: &Shape, extent: Extent, what: &str) {
        let id = self
            .world
            .create_body(shape, &BodySettings::new_static())
            .unwrap();
        let c = extent.centroid;
        let top = real(extent.max_y) + 1.0;
        let ray = RayCast::new(
            RVec3::new(real(c.x), top, real(c.z)),
            Vec3::new(0.0, -(extent.height() + 2.0), 0.0),
        );
        if let Some(hit) = self.world.cast_ray(&ray, &QueryFilter::new()).unwrap() {
            let y = ray.point_at(hit.fraction).y;
            assert!(
                y.is_finite() && hit.normal.y.is_finite(),
                "{what}: ray hit {hit:?}"
            );
        }
        let capsule = Shape::new_capsule(0.5, 0.2).unwrap();
        let centre = RVec3::new(real(c.x), real(c.y), real(c.z));
        let query = CollideShape::new(&capsule, centre, Quat::IDENTITY);
        for hit in self
            .world
            .collide_shape(&query, &QueryFilter::new())
            .unwrap()
        {
            assert!(
                hit.penetration_depth.is_finite() && hit.normal.x.is_finite(),
                "{what}: overlap {hit:?}"
            );
        }
        self.world.remove_body(id).unwrap();
    }
}

impl Arena {
    /// Adds `shape` as a static body at the origin, drops a dynamic sphere, box, capsule and
    /// hull onto it for `ticks` steps, then runs a down-ray and a sphere cast at its centroid
    /// and overlaps [`Arena::large_probe`] with it; every body and result must stay finite.
    fn drop_probes_on(&mut self, shape: &Shape, extent: Extent, ticks: usize, what: &str) {
        let ground = self
            .world
            .create_body(shape, &BodySettings::new_static())
            .unwrap();
        let c = extent.centroid;
        let top = real(extent.max_y) + 1.0;
        let mut probes = Vec::new();
        for (i, probe) in self.probes.iter().enumerate() {
            let position = RVec3::new(real(c.x) + 1.2 * i as Real - 1.8, top, real(c.z));
            let settings = BodySettings::new_dynamic().position(position);
            probes.push(self.world.create_body(probe, &settings).unwrap());
        }
        for _ in 0..ticks {
            assert!(self.world.step(DT).unwrap().is_complete());
        }
        for &id in &probes {
            assert_finite_body(&self.world, id, what);
            self.world.remove_body(id).unwrap();
        }
        let ray = RayCast::new(
            RVec3::new(real(c.x), top, real(c.z)),
            Vec3::new(0.0, -(extent.height() + 2.0), 0.0),
        );
        if let Some(hit) = self.world.cast_ray(&ray, &QueryFilter::new()).unwrap() {
            assert!(ray.point_at(hit.fraction).y.is_finite(), "{what}: {hit:?}");
        }
        let sphere = Shape::new_sphere(0.3).unwrap();
        let cast = ShapeCast::new(
            &sphere,
            RVec3::new(real(c.x), top, real(c.z)),
            Quat::IDENTITY,
            Vec3::new(0.0, -(extent.height() + 2.0), 0.0),
        );
        if let Some(hit) = self.world.cast_shape(&cast, &QueryFilter::new()).unwrap() {
            assert!(
                hit.distance.is_finite() && hit.normal.y.is_finite(),
                "{what}: {hit:?}"
            );
        }
        // The centroid lies near a bottom corner of the large box, where Jolt's coordinates
        // of the triangles in the box's space are largest.
        let reach = real(MeshSettings::DEFAULT_MAX_CONVEX_EXTENT - 1.0);
        let centre = RVec3::new(
            real(c.x) + reach,
            real(extent.max_y) + 0.9,
            real(c.z) + reach,
        );
        let query = CollideShape::new(&self.large_probe, centre, Quat::IDENTITY);
        for hit in self
            .world
            .collide_shape(&query, &QueryFilter::new())
            .unwrap()
        {
            assert!(
                hit.penetration_depth.is_finite() && hit.normal.x.is_finite(),
                "{what}: large box {hit:?}"
            );
        }
        self.world.remove_body(ground).unwrap();
    }
}

fn assert_finite_body(world: &PhysicsWorld, id: BodyId, what: &str) {
    let body = world.body(id).unwrap();
    let position: [Real; 3] = body.position().into();
    let rotation: [f32; 4] = body.rotation().into();
    let linear: [f32; 3] = body.linear_velocity().into();
    let angular: [f32; 3] = body.angular_velocity().into();
    assert!(
        position.iter().all(|v| v.is_finite())
            && rotation
                .iter()
                .chain(&linear)
                .chain(&angular)
                .all(|v| v.is_finite()),
        "{what}: body state is not finite"
    );
}

/// The kinds of point clouds the hull family draws from.
///
/// The stress runs with the `asserts` feature too, so it leaves out the clouds on which Jolt's
/// hull builder asserts although they pass the hull rules, a documented limit of that feature
/// (docs/limits.md#convex-hulls): faces sampled densely and moved a few coplanar distances off
/// their planes, and dense flat cones and domes. The clouds here are sparse: at most 300 points
/// spread through a volume or a slab or over a sphere, and at most 40 on a cap.
/// `hull_shapes.rs` covers the limit's clouds without asserts.
#[derive(Clone, Copy, Debug)]
enum Cloud {
    /// Fewer than four points.
    TooFew,
    /// A box of random aspect ratios.
    Box,
    /// Points on a sphere surface, more than Jolt keeps as hull vertices.
    Sphere,
    /// Points in an axis plane, exactly.
    ExactPlane,
    /// Points close to a plane.
    NearPlane,
    /// Points on an axis line, exactly.
    ExactLine,
    /// Points close to a line.
    NearLine,
    /// A few points repeated many times.
    Duplicates,
    /// A well-spread cloud with a spike of nearly collinear points sticking out of it.
    Spur,
    /// A well-spread cloud with a nearly flat cap of points just above one face.
    Cap,
    /// A well-spread cloud whose extreme points each come again, slightly moved.
    NearTies,
}

const CLOUDS: [Cloud; 10] = [
    Cloud::Box,
    Cloud::Sphere,
    Cloud::ExactPlane,
    Cloud::NearPlane,
    Cloud::ExactLine,
    Cloud::NearLine,
    Cloud::Duplicates,
    Cloud::Spur,
    Cloud::Cap,
    Cloud::NearTies,
];

/// A hull point cloud of `kind`, in shape space.
fn cloud(rng: &mut Rng, kind: Cloud) -> Vec<Vec3> {
    let size = rng.log_range(1.0e-4, 1.0e3);
    let translation = [0; 3].map(|_| rng.range(-1000.0, 1000.0));
    let rotation = rng.rotation();
    let place = |p: [f64; 3]| {
        let r = rotate(&rotation, p);
        to_vec3([0, 1, 2].map(|i| r[i] * size + translation[i]))
    };
    match kind {
        Cloud::TooFew => (0..rng.below(0, 4))
            .map(|_| place([0; 3].map(|_| rng.range(-1.0, 1.0))))
            .collect(),
        Cloud::Box => {
            let aspect = [0; 3].map(|_| rng.log_range(1.0e-4, 1.0));
            (0..rng.below(4, 301))
                .map(|_| place([0, 1, 2].map(|i| aspect[i] * rng.range(-1.0, 1.0))))
                .collect()
        }
        Cloud::Sphere => (0..rng.below(260, 301))
            .map(|_| {
                let z = rng.range(-1.0, 1.0);
                let angle = rng.range(0.0, std::f64::consts::TAU);
                let r = (1.0 - z * z).sqrt();
                place([r * angle.cos(), r * angle.sin(), z])
            })
            .collect(),
        Cloud::ExactPlane => {
            // No rotation or scaling across the plane, so the y values stay exactly equal.
            let y = rng.range(-1000.0, 1000.0) as f32;
            let size = size as f32;
            (0..rng.below(4, 60))
                .map(|_| {
                    Vec3::new(
                        size * rng.range(-1.0, 1.0) as f32,
                        y,
                        size * rng.range(-1.0, 1.0) as f32,
                    )
                })
                .collect()
        }
        Cloud::NearPlane => {
            let offset = rng.log_range(1.0e-7, 1.0e-2);
            (0..rng.below(4, 120))
                .map(|_| {
                    place([
                        rng.range(-1.0, 1.0),
                        offset * rng.range(-1.0, 1.0),
                        rng.range(-1.0, 1.0),
                    ])
                })
                .collect()
        }
        Cloud::ExactLine => {
            let (y, z) = (
                rng.range(-1000.0, 1000.0) as f32,
                rng.range(-1000.0, 1000.0) as f32,
            );
            let size = size as f32;
            (0..rng.below(4, 40))
                .map(|_| Vec3::new(size * rng.range(-1.0, 1.0) as f32, y, z))
                .collect()
        }
        Cloud::NearLine => {
            let offset = rng.log_range(1.0e-9, 1.0e-3);
            (0..rng.below(4, 60))
                .map(|_| {
                    place([
                        rng.range(-1.0, 1.0),
                        offset * rng.range(-1.0, 1.0),
                        offset * rng.range(-1.0, 1.0),
                    ])
                })
                .collect()
        }
        Cloud::Duplicates => {
            let distinct: Vec<[f64; 3]> = (0..rng.below(1, 7))
                .map(|_| [0; 3].map(|_| rng.range(-1.0, 1.0)))
                .collect();
            (0..rng.below(4, 80))
                .map(|_| place(distinct[rng.below(0, distinct.len())]))
                .collect()
        }
        Cloud::Spur | Cloud::Cap | Cloud::NearTies => {
            let mut points: Vec<[f64; 3]> = (0..rng.below(4, 40))
                .map(|_| [0; 3].map(|_| rng.range(-1.0, 1.0)))
                .collect();
            let jitter = rng.log_range(1.0e-9, 1.0e-3);
            match kind {
                Cloud::Spur => {
                    let length = rng.range(1.5, 4.0);
                    for _ in 0..rng.below(2, 20) {
                        let t = rng.range(1.0, length);
                        points.push([0; 3].map(|_| t + jitter * rng.range(-1.0, 1.0)));
                    }
                }
                Cloud::Cap => {
                    let lift = rng.log_range(1.0e-9, 1.0e-2);
                    for _ in 0..rng.below(3, 40) {
                        let (x, z) = (rng.range(-1.0, 1.0), rng.range(-1.0, 1.0));
                        points.push([x, 1.0 + lift + jitter * rng.range(-1.0, 1.0), z]);
                    }
                }
                _ => {
                    for axis in 0..3 {
                        for extreme in [f64::min, f64::max] {
                            let far = points
                                .iter()
                                .copied()
                                .reduce(|a, b| {
                                    if extreme(a[axis], b[axis]) == a[axis] {
                                        a
                                    } else {
                                        b
                                    }
                                })
                                .unwrap();
                            points.push(far.map(|c| c * (1.0 + jitter * rng.range(-1.0, 1.0))));
                        }
                    }
                }
            }
            // Jolt's hull builder starts from points picked by index order and extremes.
            for i in (1..points.len()).rev() {
                points.swap(i, rng.below(0, i + 1));
            }
            points.into_iter().map(place).collect()
        }
    }
}

/// The outcome the hull rules define for `kind`, when they define one.
fn expected_hull_error(kind: Cloud, points: &[Vec3]) -> Option<&'static [ConvexHullError]> {
    let within = points.iter().all(|p| {
        [p.x, p.y, p.z]
            .iter()
            .all(|c| c.abs() <= limits::MAX_SHAPE_EXTENT)
    });
    match kind {
        Cloud::TooFew => Some(&[ConvexHullError::TooFewPoints]),
        Cloud::ExactLine if within => Some(&[ConvexHullError::Degenerate]),
        // A tiny plane is also smaller than Jolt's minimum initial triangle.
        Cloud::ExactPlane if within => {
            Some(&[ConvexHullError::Coplanar, ConvexHullError::Degenerate])
        }
        _ => None,
    }
}

const HULL_CASES: usize = 3000;

fn hull_family(arena: &mut Arena) {
    let mut rng = Rng::new(0x5EED_0001);
    let mut accepted = 0;
    for index in 0..HULL_CASES {
        announce("hull", index);
        let kind = if index % 50 == 0 {
            Cloud::TooFew
        } else {
            CLOUDS[index % CLOUDS.len()]
        };
        let points = cloud(&mut rng, kind);
        let radius = if index % 3 == 0 { 0.0 } else { 0.05 };
        let what = format!("hull {index} ({kind:?}, {} points)", points.len());
        let result = Shape::new_convex_hull_with_convex_radius(&points, radius);
        if let Some(allowed) = expected_hull_error(kind, &points) {
            assert!(
                matches!(result, Err(ShapeError::ConvexHull(error)) if allowed.contains(&error)),
                "{what}: {:?}, expected one of {allowed:?}",
                result.err()
            );
            continue;
        }
        let shape = match result {
            Ok(shape) => shape,
            Err(error) => {
                eprintln!("{what}: {error}");
                continue;
            }
        };
        accepted += 1;
        if accepted % 4 == 0 {
            let extent = Extent::of(&points);
            arena.drop_dynamic(&shape, extent, 30, &what);
            arena.query_static(&shape, extent, &what);
        }
    }
    eprintln!("hull: {accepted} of {HULL_CASES} accepted");
    assert!(
        accepted > HULL_CASES / 10,
        "only {accepted} hulls were accepted"
    );
}

/// The kinds of triangle meshes the mesh family draws from.
#[derive(Clone, Copy, Debug)]
enum Soup {
    /// A height grid.
    Grid,
    /// Random triangles, some near-degenerate, some duplicated.
    Random,
    /// One large triangle and slivers that collapse under Jolt's vertex quantization.
    Collapsing,
    /// Only degenerate triangles: repeated indices and collinear vertices.
    Degenerate,
    /// A level-like ground with thin strips on it, sometimes next to far triangles.
    Thin,
}

const SOUPS: [Soup; 5] = [
    Soup::Grid,
    Soup::Random,
    Soup::Collapsing,
    Soup::Degenerate,
    Soup::Thin,
];

/// A mesh of `kind`, in shape space.
fn soup(rng: &mut Rng, kind: Soup) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let size = rng.log_range(1.0e-2, 500.0);
    let translation = [0; 3].map(|_| rng.range(-500.0, 500.0));
    let rotation = rng.rotation();
    let place = |p: [f64; 3]| {
        let r = rotate(&rotation, p);
        to_vec3([0, 1, 2].map(|i| r[i] * size + translation[i]))
    };
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    match kind {
        Soup::Grid => {
            let cells = rng.below(1, 51) as u32;
            let roughness = rng.log_range(1.0e-4, 0.5);
            let side = cells + 1;
            for k in 0..side {
                for i in 0..side {
                    let (x, z) = (
                        f64::from(i) / f64::from(cells) - 0.5,
                        f64::from(k) / f64::from(cells) - 0.5,
                    );
                    vertices.push(place([x, roughness * rng.range(-1.0, 1.0), z]));
                }
            }
            for k in 0..cells {
                for i in 0..cells {
                    let v = k * side + i;
                    triangles.push([v, v + side, v + side + 1]);
                    triangles.push([v, v + side + 1, v + 1]);
                }
            }
        }
        Soup::Random => {
            for _ in 0..rng.below(1, 2001) {
                let base = vertices.len() as u32;
                let a = [0; 3].map(|_| rng.range(-0.5, 0.5));
                let b = [0; 3].map(|_| rng.range(-0.5, 0.5));
                // Every fifth triangle has a third vertex close to the first.
                let c = if rng.below(0, 5) == 0 {
                    let gap = rng.log_range(1.0e-7, 1.0e-3);
                    [0, 1, 2].map(|i| a[i] + gap * rng.range(-1.0, 1.0))
                } else {
                    [0; 3].map(|_| rng.range(-0.5, 0.5))
                };
                vertices.extend([a, b, c].map(place));
                triangles.push([base, base + 1, base + 2]);
                // Every tenth triangle is repeated, with its indices rotated.
                if rng.below(0, 10) == 0 {
                    triangles.push([base + 1, base + 2, base]);
                }
            }
        }
        Soup::Collapsing => {
            vertices.extend([[-0.5, 0.0, -0.5], [-0.5, 0.0, 0.5], [0.5, 0.0, -0.5]].map(place));
            triangles.push([0, 1, 2]);
            for _ in 0..rng.below(1, 200) {
                let base = vertices.len() as u32;
                let at = [0; 3].map(|_| rng.range(-0.4, 0.4));
                let tiny = rng.log_range(1.0e-9, 1.0e-6);
                let corners = [
                    at,
                    [at[0] + tiny, at[1], at[2]],
                    [at[0], at[1] + 0.1, at[2]],
                ];
                vertices.extend(corners.map(place));
                triangles.push([base, base + 1, base + 2]);
            }
        }
        Soup::Degenerate => {
            for _ in 0..rng.below(1, 100) {
                let base = vertices.len() as u32;
                if rng.below(0, 2) == 0 {
                    let corners = [0; 3].map(|_| [0; 3].map(|_| rng.range(-0.5, 0.5)));
                    vertices.extend(corners.map(place));
                    let repeated = rng.below(0, 3) as u32;
                    triangles.push([base + repeated, base + repeated, base + 2 - repeated]);
                } else {
                    // Collinear along x: y and z are exactly equal, so f32 keeps them collinear.
                    let (y, z) = (
                        rng.range(-500.0, 500.0) as f32,
                        rng.range(-500.0, 500.0) as f32,
                    );
                    let size = size as f32;
                    vertices.extend(
                        [0; 3].map(|_| Vec3::new(size * rng.range(-0.5, 0.5) as f32, y, z)),
                    );
                    triangles.push([base, base + 1, base + 2]);
                }
            }
        }
        Soup::Thin => thin_strips(rng, &mut vertices, &mut triangles),
    }
    (vertices, triangles)
}

/// A ground grid in metres, 1 m above the arena floor and up to 500 m from the origin (at the
/// origin, where the rule keeps the thinnest strips, in one case of four), with strips 1 um to
/// 1 cm wide lying on it where the probes land, and, in three cases of four, far triangles that
/// stretch the mesh's bounds (and so its quantization step) along x, along z or down along y.
fn thin_strips(rng: &mut Rng, vertices: &mut Vec<Vec3>, triangles: &mut Vec<[u32; 3]>) {
    let centre = if rng.below(0, 4) == 0 {
        [0.0, 1.0, 0.0]
    } else {
        [rng.range(-500.0, 500.0), 1.0, rng.range(-500.0, 500.0)]
    };
    let at = |p: [f64; 3]| to_vec3([0, 1, 2].map(|i| centre[i] + p[i]));
    let side = rng.log_range(4.0, 400.0);
    let cells = rng.below(1, 9) as u32;
    let rows = cells + 1;
    for k in 0..rows {
        for i in 0..rows {
            let (x, z) = (
                f64::from(i) / f64::from(cells) - 0.5,
                f64::from(k) / f64::from(cells) - 0.5,
            );
            vertices.push(at([x * side, 0.0, z * side]));
        }
    }
    for k in 0..cells {
        for i in 0..cells {
            let v = k * rows + i;
            triangles.extend([[v, v + rows, v + rows + 1], [v, v + rows + 1, v + 1]]);
        }
    }
    for _ in 0..rng.below(1, 100) {
        let start = [
            rng.range(-2.0, 2.0),
            rng.range(1.0e-4, 2.0e-3),
            rng.range(-0.5, 0.5),
        ];
        let (length, width) = (rng.log_range(0.05, 2.0), rng.log_range(1.0e-6, 1.0e-2));
        let yaw = rng.range(0.0, std::f64::consts::TAU);
        let (cos, sin) = (yaw.cos(), yaw.sin());
        let base = vertices.len() as u32;
        vertices.extend(
            [
                start,
                [start[0] + cos * length, start[1], start[2] + sin * length],
                [start[0] - sin * width, start[1], start[2] + cos * width],
            ]
            .map(at),
        );
        // Counter-clockwise seen from above.
        triangles.push([base, base + 2, base + 1]);
    }
    let far: &[[f64; 3]] = match rng.below(0, 4) {
        0 => &[],
        1 => &[[1000.0, 0.0, 0.0], [-1000.0, 0.0, 0.0]],
        2 => &[[0.0, 0.0, 1000.0], [0.0, 0.0, -1000.0]],
        _ => &[[0.0, -1000.0, 0.0]],
    };
    for offset in far {
        let base = vertices.len() as u32;
        vertices.extend(
            [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]
                .map(|p| at([0, 1, 2].map(|i| offset[i] + p[i]))),
        );
        triangles.push([base, base + 1, base + 2]);
    }
}

const MESH_CASES: usize = 600;

fn mesh_family(arena: &mut Arena) {
    let mut rng = Rng::new(0x5EED_0002);
    let materials: Vec<PhysicsMaterial> = (0..32)
        .map(|i| PhysicsMaterial::new(1000 + i).unwrap())
        .collect();
    let refs: Vec<&PhysicsMaterial> = materials.iter().collect();
    let mut accepted = 0;
    let (mut strips, mut dropped_strips) = (0, 0);
    for index in 0..MESH_CASES {
        announce("mesh", index);
        let kind = SOUPS[index % SOUPS.len()];
        let (vertices, triangles) = soup(&mut rng, kind);
        let list = &refs[..rng.below(1, refs.len() + 1)];
        let indices: Vec<u8> = triangles
            .iter()
            .map(|_| rng.below(0, list.len()) as u8)
            .collect();
        let settings = if index % 2 == 0 {
            MeshSettings::default().materials(list, &indices)
        } else {
            MeshSettings::default().build_quality(MeshBuildQuality::FavorBuildSpeed)
        };
        let what = format!("mesh {index} ({kind:?}, {} triangles)", triangles.len());
        let result = Shape::new_mesh_with_settings(&vertices, &triangles, &settings);
        if let (Soup::Thin, Ok((_, dropped))) = (kind, &result) {
            strips += triangles.len() - dropped.count();
            dropped_strips += dropped.count();
        }
        if let Soup::Degenerate = kind {
            assert!(
                matches!(result, Err(ShapeError::Mesh(MeshError::NoTriangles))),
                "{what}: {:?}",
                result.err()
            );
            continue;
        }
        if let Soup::Collapsing | Soup::Thin = kind {
            // The large triangles always survive.
            assert!(result.is_ok(), "{what}: {:?}", result.err());
        }
        let shape = match result {
            Ok((shape, _)) => shape,
            Err(error) => {
                eprintln!("{what}: {error}");
                continue;
            }
        };
        accepted += 1;
        if accepted % 4 == 0 || matches!(kind, Soup::Thin) {
            arena.drop_probes_on(&shape, Extent::of(&vertices), 40, &what);
        }
    }
    eprintln!("mesh: {accepted} of {MESH_CASES} accepted");
    eprintln!("thin: {strips} triangles kept, {dropped_strips} dropped");
    assert!(
        accepted > MESH_CASES / 2,
        "only {accepted} meshes were accepted"
    );
}
