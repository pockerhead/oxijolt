//! The large real models that `scripts/fetch_models.py` downloads: Keenan Crane's spot (5856
//! triangles) and the Khronos ScatteringSkull (188 871 triangles). The tests are ignored by
//! default; run them with `OXIJOLT_MODELS=<dir> cargo test --test real_meshes_downloaded --
//! --include-ignored`. They fail when a file of a model is missing or its SHA-256 differs from
//! `assets/models/models.tsv`. Results are in `docs/real-meshes.md`.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

#[path = "real_meshes_checks/mod.rs"]
mod checks;
mod common;

use std::path::PathBuf;

use checks::*;
use oxijolt::*;

/// The convex extent the skull at its own size (0.25 m) is built for, metres: it meets
/// hand-sized bodies only.
const SKULL_CONVEX_EXTENT: f32 = 2.0;
/// Largest share of the real-size skull's area its mesh may drop at [`SKULL_CONVEX_EXTENT`]:
/// 0.47 % of it is triangles at or below Jolt's own limit of 1e-6 m² for twice the area.
const SKULL_MAX_DROPPED_SHARE: f64 = 0.005;

#[test]
#[ignore = "needs scripts/fetch_models.py"]
fn spot() {
    let model = Model::load("spot");
    let built = build(&model);
    rain_bodies(&model, &built.mesh, SMALL_DROP_SIZES);
}

/// The skull at its own size, built for hand-sized bodies, with such bodies dropped on it; at
/// the default extent it loses more of its fine triangles, and the test prints how much.
#[test]
#[ignore = "needs scripts/fetch_models.py"]
fn scattering_skull() {
    let model = Model::load("ScatteringSkull");
    let settings = MeshSettings::default().max_convex_extent(SKULL_CONVEX_EXTENT);
    let built = build_with(&model, &settings, SKULL_MAX_DROPPED_SHARE);
    rain_bodies(&model, &built.mesh, SMALL_DROP_SIZES);
    let (_, dropped) = Shape::new_mesh(&model.vertices, &model.triangles).unwrap();
    let share = f64::from(dropped.area()) / model.surface_area();
    println!(
        "ScatteringSkull at the default extent: {} dropped, {:.4} % of the area",
        dropped.count(),
        100.0 * share
    );
    assert!(share < 0.05, "{share}");
}

/// The skull as a 2.5 m statue at the default extent, for bodies of 0.1 m.
#[test]
#[ignore = "needs scripts/fetch_models.py"]
fn scattering_skull_statue() {
    let model = Model::load("ScatteringSkull").scaled(10.0);
    let built = build(&model);
    rain_bodies(&model, &built.mesh, DROP_SIZES);
}

/// Byte offset in `ScatteringSkull_binary.bin` of the first NORMAL value (buffer view 1 of
/// `ScatteringSkull.gltf`), which the importer does not read.
const SKULL_NORMAL_BYTE: usize = 1_174_560;

/// A changed byte of the skull's glTF buffer is refused by its SHA-256 before import, also
/// when the importer would not see it.
#[test]
#[cfg(not(target_arch = "wasm32"))]
#[ignore = "needs scripts/fetch_models.py"]
fn a_changed_skull_buffer_is_refused() {
    let models = PathBuf::from(std::env::var_os(MODELS_ENV).unwrap());
    let copy = std::env::temp_dir().join(format!("oxijolt-skull-{}", std::process::id()));
    std::fs::create_dir_all(&copy).unwrap();
    for file in ["ScatteringSkull.gltf", "ScatteringSkull_binary.bin"] {
        std::fs::copy(models.join(file), copy.join(file)).unwrap();
    }
    let buffer = copy.join("ScatteringSkull_binary.bin");
    let mut bytes = std::fs::read(&buffer).unwrap();
    bytes[SKULL_NORMAL_BYTE] ^= 1;
    std::fs::write(&buffer, bytes).unwrap();

    let gltf = |dir: &PathBuf| mesh_import::load(dir.join("ScatteringSkull.gltf")).unwrap();
    let (original, changed) = (gltf(&models), gltf(&copy));
    let result = Model::try_load("ScatteringSkull", Some(&copy)).map(|_| ());
    std::fs::remove_dir_all(&copy).unwrap();
    assert!(
        original.vertices == changed.vertices && original.triangles == changed.triangles,
        "the changed byte is outside the geometry"
    );
    let error = result.unwrap_err();
    assert!(
        error.contains("ScatteringSkull_binary.bin: SHA-256 differs"),
        "{error}"
    );
}
