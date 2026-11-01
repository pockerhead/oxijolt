//! Shape binary state: every shape kind restored from its saved bytes answers rays, carries mass
//! and materials like the original, saves to the same bytes again, in another process too, and a
//! world on a restored mesh steps exactly like one on the original.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use std::process::Command;

use common::meshes::*;
use common::*;
use oxijolt::*;

/// Set in the child process of [`saves_in_two_processes_are_equal`]: where to write the bytes.
const CHILD_ENV: &str = "OXIJOLT_SHAPE_STATE_CHILD";

fn restore(bytes: &[u8]) -> Shape {
    // SAFETY: every test restores bytes this build saved in this or a child process of the same
    // binary, unchanged.
    unsafe { Shape::restore_binary_state(bytes) }.unwrap()
}

/// `shape` saved and restored; the restored shape saves to the same bytes.
fn round_trip(shape: &Shape) -> Shape {
    let bytes = shape.save_binary_state().unwrap();
    let restored = restore(&bytes);
    assert_eq!(
        restored.save_binary_state().unwrap(),
        bytes,
        "save, restore and save gives the first bytes"
    );
    restored
}

// `Real` is `f32` without the `double-precision` feature.
#[allow(clippy::unnecessary_cast)]
fn real(value: f32) -> Real {
    value as Real
}

/// What a ray reports: fraction, normal and sub-shape id as bits, and the compound child.
type RayReading = Option<([u32; 4], u32, Option<CompoundSubShape>)>;

/// The readings of a 32 x 32 grid of rays down onto `shape` and of one across it along +x,
/// with the shape on a static body at the origin; the rays cover `[-half, half]^2`.
fn ray_grid(shape: &Shape, half: f32) -> Vec<RayReading> {
    let mut world = world(Vec3::ZERO, 1);
    world
        .create_body(shape, &BodySettings::new_static())
        .unwrap();
    let reach = 4.0 * half + 2.0;
    let at = |i: usize| -half + (i as f32 + 0.5) * 2.0 * half / 32.0;
    let mut readings = Vec::new();
    for i in 0..32 {
        for j in 0..32 {
            let (u, v) = (at(i), at(j));
            for ray in [
                RayCast::new(
                    RVec3::new(real(u), real(2.0 * half + 1.0), real(v)),
                    Vec3::new(0.0, -reach, 0.0),
                ),
                RayCast::new(
                    RVec3::new(real(-2.0 * half - 1.0), real(u), real(v)),
                    Vec3::new(reach, 0.0, 0.0),
                ),
            ] {
                let hit = world.cast_ray(&ray, &QueryFilter::new()).unwrap();
                readings.push(hit.map(|hit| {
                    (
                        [hit.fraction, hit.normal.x, hit.normal.y, hit.normal.z].map(f32::to_bits),
                        hit.sub_shape_id.to_raw(),
                        hit.compound_child,
                    )
                }));
            }
        }
    }
    readings
}

/// The mass of a dynamic body of `shape`, as bits.
fn mass_bits(shape: &Shape) -> u32 {
    let mut world = world(Vec3::ZERO, 1);
    let id = world
        .create_body(shape, &BodySettings::new_dynamic())
        .unwrap();
    world.body(id).unwrap().mass().unwrap().to_bits()
}

fn child(shape: &Shape, position: Vec3, user_data: u32) -> CompoundChild<'_> {
    CompoundChild {
        shape,
        position,
        // A turn of about 0.3 rad about +y.
        rotation: Quat::from_xyzw(0.0, 0.149_438_13, 0.0, 0.988_771_1),
        user_data,
    }
}

fn bumpy_mesh() -> Shape {
    let (vertices, triangles) = grid(16, 0.5, |x, z| 0.3 * (0.7 * x).sin() * (0.5 * z).cos());
    Shape::new_mesh(&vertices, &triangles).unwrap().0
}

fn height_field() -> Shape {
    let n = 17;
    let samples: Vec<f32> = (0..n * n)
        .map(|i| 0.2 * ((i % n) as f32 * 0.4).sin() + 0.1 * ((i / n) as f32 * 0.3).cos())
        .collect();
    let settings = HeightFieldSettings::default()
        .offset(Vec3::new(-4.0, 0.0, -4.0))
        .scale(Vec3::new(0.5, 1.0, 0.5));
    Shape::new_height_field(n, &samples, &settings).unwrap()
}

/// Every kind this crate builds, with the half width its rays cover and whether a dynamic body
/// may use it.
fn every_kind() -> Vec<(&'static str, Shape, f32, bool)> {
    let block = Shape::new_box(Vec3::new(0.5, 1.0, 1.5)).unwrap();
    let ball = Shape::new_sphere(0.7).unwrap();
    let hull = Shape::new_convex_hull(&irregular_points()).unwrap();
    let mesh = bumpy_mesh();
    let compound = Shape::new_compound(&[
        child(&block, Vec3::new(-1.0, 0.0, 0.0), 11),
        child(&ball, Vec3::new(1.0, 0.5, 0.0), 12),
        child(&hull, Vec3::new(0.0, 0.0, 1.5), 13),
        child(&block, Vec3::new(0.0, 1.0, -1.5), 14),
    ])
    .unwrap();
    let mut mutable = MutableCompound::from_children(&[
        child(&ball, Vec3::new(0.0, 0.0, 0.0), 21),
        child(&hull, Vec3::new(1.5, 0.0, 0.0), 22),
    ])
    .unwrap();
    mutable
        .add_shape(&child(&block, Vec3::new(-1.5, 0.0, 0.0), 23))
        .unwrap();
    vec![
        (
            "box",
            Shape::new_box(Vec3::new(0.5, 1.0, 1.5)).unwrap(),
            2.0,
            true,
        ),
        ("sphere", Shape::new_sphere(0.7).unwrap(), 1.0, true),
        ("capsule", Shape::new_capsule(0.5, 0.3).unwrap(), 1.0, true),
        (
            "tapered capsule",
            Shape::new_tapered_capsule(0.5, 0.2, 0.4).unwrap(),
            1.0,
            true,
        ),
        (
            "cylinder",
            Shape::new_cylinder(0.5, 0.4).unwrap(),
            1.0,
            true,
        ),
        (
            "tapered cylinder",
            Shape::new_tapered_cylinder(0.5, 0.1, 0.4).unwrap(),
            1.0,
            true,
        ),
        (
            "convex hull",
            Shape::new_convex_hull(&irregular_points()).unwrap(),
            1.5,
            true,
        ),
        (
            "plane",
            Shape::new_plane(Vec3::new(0.0, 1.0, 0.0), 0.2, 5.0).unwrap(),
            4.0,
            false,
        ),
        (
            "offset centre of mass",
            Shape::new_offset_center_of_mass(&block, Vec3::new(0.1, -0.2, 0.3)).unwrap(),
            2.0,
            true,
        ),
        (
            "scaled",
            Shape::new_scaled(&hull, Vec3::new(1.0, 2.0, 0.5)).unwrap(),
            1.0,
            true,
        ),
        ("compound", compound, 3.0, true),
        (
            "one-child compound",
            Shape::new_compound(&[child(&block, Vec3::new(0.5, 0.0, 0.0), 31)]).unwrap(),
            2.0,
            true,
        ),
        ("mutable compound", mutable.to_shape().unwrap(), 3.0, true),
        ("mesh", bumpy_mesh(), 4.0, false),
        (
            "scaled mesh",
            Shape::new_scaled(&mesh, Vec3::new(1.5, 2.0, 1.5)).unwrap(),
            6.0,
            false,
        ),
        ("heightfield", height_field(), 4.0, false),
    ]
}

#[test]
fn every_shape_kind_round_trips() {
    for (name, shape, half, dynamic) in every_kind() {
        let restored = round_trip(&shape);
        let (original_rays, restored_rays) = (ray_grid(&shape, half), ray_grid(&restored, half));
        let hits = original_rays.iter().filter(|hit| hit.is_some()).count();
        assert!(hits > 100, "{name}: the rays hit the shape ({hits} hits)");
        assert_eq!(restored_rays, original_rays, "{name}: ray readings");
        if dynamic {
            assert_eq!(mass_bits(&restored), mass_bits(&shape), "{name}: mass");
        }
    }
}

/// The material user data of the contacts that cubes dropped at `drops` make with `ground`,
/// per cube.
fn contact_materials(ground: &Shape, drops: &[(f32, f32)]) -> Vec<Vec<Option<u64>>> {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    world
        .create_body(ground, &BodySettings::new_static())
        .unwrap();
    let cubes: Vec<BodyId> = drops
        .iter()
        .map(|&(x, z)| add_cube(&mut world, RVec3::new(real(x), 1.2, real(z))))
        .collect();
    let mut seen = vec![Vec::new(); cubes.len()];
    for _ in 0..60 {
        step(&mut world, 1);
        for event in world.take_events().contacts {
            if let ContactEvent::Added { manifold, .. } = event {
                if let Some(cube) = cubes.iter().position(|&id| id == manifold.pair.body2) {
                    seen[cube].push(manifold.materials[0]);
                }
            }
        }
    }
    seen
}

#[test]
fn materials_round_trip_with_their_user_data() {
    let (west, east) = (
        PhysicsMaterial::new(71).unwrap(),
        PhysicsMaterial::new(72).unwrap(),
    );
    let (vertices, triangles) = grid(8, 1.0, |_, _| 0.0);
    let indices: Vec<u8> = triangles
        .iter()
        .map(|triangle| u8::from(vertices[triangle[1] as usize].x >= 0.5))
        .collect();
    let list = [&west, &east];
    let (mesh, _) = Shape::new_mesh_with_settings(
        &vertices,
        &triangles,
        &MeshSettings::default().materials(&list, &indices),
    )
    .unwrap();
    let n = 9;
    let field_indices: Vec<u8> = (0..(n - 1) * (n - 1))
        .map(|i| u8::from(i % 8 >= 4))
        .collect();
    let field = Shape::new_height_field(
        n,
        &vec![0.0; (n * n) as usize],
        &HeightFieldSettings::default()
            .offset(Vec3::new(-4.0, 0.0, -4.0))
            .materials(&list, &field_indices),
    )
    .unwrap();
    let ball = Shape::new_sphere_with_material(6.0, &east).unwrap();
    // A large ball whose top is at the origin, so the cubes land on it.
    let dome = Shape::new_compound(&[CompoundChild {
        shape: &ball,
        position: Vec3::new(0.0, -6.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data: 0,
    }])
    .unwrap();
    drop((west, east));

    let drops = [(-2.3, 0.4), (2.3, -0.4)];
    for (name, ground, expected) in [
        ("mesh", mesh, [71, 72]),
        ("heightfield", field, [71, 72]),
        ("sphere", dome, [72, 72]),
    ] {
        let original = contact_materials(&ground, &drops);
        let restored = contact_materials(&round_trip(&ground), &drops);
        assert_eq!(restored, original, "{name}");
        for (cube, value) in original.iter().zip(expected) {
            assert!(
                !cube.is_empty() && cube.iter().all(|&m| m == Some(value)),
                "{name}: {original:?}"
            );
        }
    }
}

/// A heightfield with no or one material stores no material indices, and Jolt reads that empty
/// array with a null destination and a length of 0. The joltc extension's input stream returns
/// before `memcpy` then, and under `asserts` it asserts that every other read has a destination,
/// so this test aborts in the asserts build if the early return goes.
#[test]
fn heightfields_without_material_indices_round_trip() {
    let only = PhysicsMaterial::new(73).unwrap();
    let list = [&only];
    let n = 9;
    let indices = vec![0; ((n - 1) * (n - 1)) as usize];
    let samples = vec![0.0; (n * n) as usize];
    let plain = HeightFieldSettings::default().offset(Vec3::new(-4.0, 0.0, -4.0));
    let one_material = plain.clone().materials(&list, &indices);
    let drops = [(-2.3, 0.4), (2.3, -0.4)];
    for (name, settings, single) in [
        ("no material", plain, false),
        ("one material", one_material, true),
    ] {
        let field = Shape::new_height_field(n, &samples, &settings).unwrap();
        let restored = round_trip(&field);
        assert_eq!(ray_grid(&restored, 3.5), ray_grid(&field, 3.5), "{name}");
        let seen = contact_materials(&restored, &drops);
        assert_eq!(seen, contact_materials(&field, &drops), "{name}");
        assert!(seen.iter().all(|cube| !cube.is_empty()), "{name}: {seen:?}");
        if single {
            assert!(seen.iter().flatten().all(|&m| m == Some(73)), "{seen:?}");
        }
    }
}

#[test]
fn a_restored_mesh_keeps_its_convex_extent() {
    let sliver = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(2.0e-5, 0.0, 0.5),
    ];
    let settings = MeshSettings::default().max_convex_extent(1.0);
    let (mesh, dropped) = Shape::new_mesh_with_settings(&sliver, &[[0, 1, 2]], &settings).unwrap();
    assert!(dropped.is_empty());
    let restored = round_trip(&mesh);
    let shrunk = Vec3::new(0.2, 0.2, 0.2);
    let refused = Shape::new_scaled(&mesh, shrunk).err();
    assert!(matches!(
        refused,
        Some(ShapeError::ThinTriangles(error)) if error.max_convex_extent == 1.0
    ));
    assert_eq!(Shape::new_scaled(&restored, shrunk).err(), refused);
    assert!(Shape::new_scaled(&restored, Vec3::new(2.0, 2.0, 2.0)).is_ok());
}

/// The bits of a world coordinate in either precision.
// `Real` is `f64` with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn real_bits(value: Real) -> u64 {
    f64::from(value).to_bits()
}

/// The per-tick poses and velocities, as bits, of 20 spheres and boxes dropped onto `floor`
/// for 300 ticks with `threads` workers.
fn drop_on(floor: &Shape, threads: u32) -> Vec<Vec<[u64; 13]>> {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), threads);
    world
        .create_body(floor, &BodySettings::new_static())
        .unwrap();
    let ball = Shape::new_sphere(0.25).unwrap();
    let block = Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap();
    let ids: Vec<BodyId> = (0..20)
        .map(|i| {
            let shape = if i % 2 == 0 { &ball } else { &block };
            let position = RVec3::new(
                (i % 5) as Real * 1.3 - 2.6,
                1.0 + (i / 5) as Real * 0.7,
                (i % 3) as Real * 1.1 - 1.1,
            );
            world
                .create_body(shape, &BodySettings::new_dynamic().position(position))
                .unwrap()
        })
        .collect();
    (0..300)
        .map(|_| {
            step(&mut world, 1);
            ids.iter()
                .map(|&id| {
                    let body = world.body(id).unwrap();
                    let (p, r) = (body.position(), body.rotation());
                    let (v, w) = (body.linear_velocity(), body.angular_velocity());
                    [
                        real_bits(p.x),
                        real_bits(p.y),
                        real_bits(p.z),
                        u64::from(r.x.to_bits()),
                        u64::from(r.y.to_bits()),
                        u64::from(r.z.to_bits()),
                        u64::from(r.w.to_bits()),
                        u64::from(v.x.to_bits()),
                        u64::from(v.y.to_bits()),
                        u64::from(v.z.to_bits()),
                        u64::from(w.x.to_bits()),
                        u64::from(w.y.to_bits()),
                        u64::from(w.z.to_bits()),
                    ]
                })
                .collect()
        })
        .collect()
}

#[test]
fn a_world_on_a_restored_mesh_steps_like_one_on_the_original() {
    let mesh = bumpy_mesh();
    let restored = round_trip(&mesh);
    for threads in [1, 4] {
        assert_eq!(
            drop_on(&restored, threads),
            drop_on(&mesh, threads),
            "{threads} workers"
        );
    }
}

/// The bytes the child process writes: the compound and the mesh of [`every_kind`].
fn child_bytes() -> Vec<u8> {
    every_kind()
        .iter()
        .filter(|(name, ..)| matches!(*name, "compound" | "mesh" | "heightfield"))
        .flat_map(|(_, shape, ..)| shape.save_binary_state().unwrap())
        .collect()
}

#[test]
fn write_bytes_in_child() {
    if let Some(path) = std::env::var_os(CHILD_ENV) {
        std::fs::write(path, child_bytes()).unwrap();
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn saves_in_two_processes_are_equal() {
    let path = std::env::temp_dir().join(format!("oxijolt-shape-state-{}.bin", std::process::id()));
    let status = Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "write_bytes_in_child", "--test-threads=1"])
        .env(CHILD_ENV, &path)
        .status()
        .unwrap();
    assert!(status.success());
    let theirs = std::fs::read(&path).unwrap();
    std::fs::remove_file(&path).unwrap();
    assert_eq!(theirs, child_bytes());
    assert!(theirs.len() > 1000);
}
