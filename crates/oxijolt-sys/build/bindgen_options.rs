//! The bindgen settings of the raw bindings, shared by `build.rs` (with the `bindgen` feature)
//! and the `xtask` regenerator of the committed files through `#[path]`.

use std::path::Path;

use bindgen::{EnumVariation, Formatter, RustEdition, RustTarget};

/// joltc functions left out of the bindings.
///
/// They `reinterpret_cast` a `JPH_Mat4*` (4-aligned) to a `JPH::Mat44*`
/// (16-aligned), which is undefined behaviour for most caller-provided arrays.
/// They stay unbound: ragdoll poses go through per-body transforms instead, and the skeleton
/// mapper uses the aligned `JPH_SkeletonMapper_*2` entry points of `joltc_ext`.
pub const EXCLUDED_FUNCTIONS: &[&str] = &[
    "JPH_RagdollSettings_DisableParentChildCollisions",
    "JPH_Ragdoll_SetPose2",
    "JPH_Ragdoll_GetPose2",
    "JPH_SkeletonMapper_Initialize",
    "JPH_SkeletonMapper_LockAllTranslations",
    "JPH_SkeletonMapper_LockTranslations",
    "JPH_SkeletonMapper_Map",
    "JPH_SkeletonMapper_MapReverse",
];

/// joltc functions that joltc.cpp defines only under `JPH_DEBUG_RENDERER`; left out of the
/// bindings without the `debug-renderer` feature, so no Rust code can reference a missing symbol.
pub const DEBUG_RENDERER_FUNCTIONS: &[&str] = &[
    "JPH_Shape_Draw",
    "JPH_PhysicsSystem_Draw.*",
    "JPH_BodyDrawFilter_.*",
    "JPH_DebugRenderer_.*",
];

/// A bindgen builder for `header` (`joltc_ext.h`, which includes `joltc.h` from `include_dir`)
/// and the Rust target triple `target`.
///
/// The output depends only on the headers, these settings and libclang:
/// - Rust target 1.82 and edition 2021 are pinned, so the invoking rustc does not change it;
/// - Prettyplease formats it, so the installed rustfmt does not;
/// - header comments are dropped, because their text varies between libclang versions;
/// - `-ffreestanding` with an explicit `--target=` parses against Clang's own headers, so one
///   host generates the bindings of every target;
/// - `-fvisibility=default` keeps the declarations visible on wasm32, whose Clang driver hides
///   them by default and bindgen then emits no functions; other targets already default to it.
pub fn builder(
    header: &Path,
    include_dir: &Path,
    target: &str,
    double_precision: bool,
    debug_renderer: bool,
) -> bindgen::Builder {
    let mut builder = bindgen::Builder::default()
        .header(header.display().to_string())
        .clang_arg(format!("-I{}", include_dir.display()))
        .clang_arg("-ffreestanding")
        .clang_arg(format!("--target={target}"))
        .clang_arg("-fvisibility=default")
        .allowlist_item("JPH_.*")
        .allowlist_item("JobSystemThreadPoolConfig")
        .default_enum_style(EnumVariation::Consts)
        .prepend_enum_name(false)
        .rust_target(
            RustTarget::stable(82, 0).expect("1.82 is a stable Rust version bindgen knows"),
        )
        .rust_edition(RustEdition::Edition2021)
        .formatter(Formatter::Prettyplease)
        .generate_comments(false);

    // The header's only ABI switch. JPH_DEBUG_RENDERER does not change the header; it only
    // decides which functions joltc defines, which DEBUG_RENDERER_FUNCTIONS handles.
    if double_precision {
        builder = builder.clang_arg("-DJPH_DOUBLE_PRECISION");
    }
    for function in EXCLUDED_FUNCTIONS {
        builder = builder.blocklist_function(function);
    }
    if !debug_renderer {
        for function in DEBUG_RENDERER_FUNCTIONS {
            builder = builder.blocklist_function(function);
        }
    }
    builder
}
