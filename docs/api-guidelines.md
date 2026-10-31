# API guidelines

The rules the public API of `oxijolt` follows from 1.0 on. New code follows them; a change that
breaks one needs a reason written next to it. The Rust API Guidelines
(https://rust-lang.github.io/api-guidelines/checklist.html) apply where these rules say nothing.

## R1. Vocabulary

- Type and method names keep Jolt's words in Rust case. `ID` becomes `Id` and `DOF` becomes `Dof`
  (`BodyId`, `SubShapeId`, `SixDofConstraint`, `AllowedDofs`).
- Jolt's C++ prefixes `E`, `s` and `m` are dropped.
- Abbreviations follow Jolt, spelled out where Jolt spells them out: `inverse_mass`, not `inv_mass`.
- A name leaves Jolt's only to avoid a clash or to name a Rust concept (`WorldState`).

## R2. Construction

- Values are built with `Type::new(..)` or `Type::new_<variant>(..)`. Shapes are always
  `Shape::new_<kind>`, decorators included (`new_scaled`, `new_offset_center_of_mass`).
- Variants of a constructor add suffixes in this order: `_with_convex_radius`, then
  `_with_material` (one `&PhysicsMaterial`, last argument), then `_with_settings` (a settings
  struct by reference, last argument). A constructor without a suffix uses Jolt's default for
  what the suffix would add.
- Objects the world owns are made with `PhysicsWorld::create_<object>(..)`, which returns a typed
  id, and are removed with `remove_<object>`.

## R3. Settings

- `<Thing>Settings` has private fields, a `Default` equal to Jolt's defaults, and consuming
  `#[must_use]` setters named after the field. A flag setter takes a `bool`.
- Settings that need an input with no useful default (a character's shape, a wheel's position,
  a vehicle's wheels and collision tester, a path, the centre of debug lines) take it in
  `new(..)` and have no `Default`. The settings a listener receives (`ContactSettings`,
  `CharacterContactSettings`, `SoftBodyContactSettings`) come from Jolt with the call.
- Presets are named constructors (`humanoid`, `car`, `bike`).
- Plain-Rust settings are checked by the call that consumes them and fail with that call's error.
  Jolt-backed immutable objects are checked when they are built.
- Optional per-element data, such as materials, is a settings setter, never a positional argument.

## R4. Access

- `world.<object>(id)` returns `Result<<Object>Ref, <Area>Error>` and `world.<object>_mut(id)`
  returns `Result<<Object>Mut, <Area>Error>`.
- Getters have no `get_` prefix. Boolean getters start with `is_`, `has_` or `can_`.
- Setters on live objects are `set_<field>` and return `Result` when they check their input.
- A world collection lists its ids with `<object>_ids()`, which returns an iterator
  (`impl Iterator<Item = <Object>Id>`), counts them with `<object>_count()` and tests one with
  `contains_<object>(id)`: bodies, characters, vehicles, ragdolls and constraints.

## R5. Conversions and buffers

- `as_` borrows and costs nothing, `to_` allocates, `into_` consumes.
- A call that fills a caller's buffer is `<name>_into(&self, .., out: &mut T)`, next to an
  allocating `<name>`.
- No other out-parameters: return values, iterators or slices.

## R6. Errors

- One `<Area>Error` per area, in `error.rs`. Each is `#[non_exhaustive]` and
  `Clone + Copy + Debug + PartialEq + Eq`. `Copy` is a 1.0 commitment: payloads are
  `&'static str`, ids, numbers or `JoltMessage`.
- A value out of its range is `InvalidValue(&'static str)` in every area.
- A native allocation that fails is `AllocationFailed`; a refusal Jolt reports is
  `Rejected(JoltMessage)`.
- A body problem found by another area is `<Area>Error::Body(BodyError)`. An error that wraps
  another returns it from `source()`.
- The umbrella `error::Error` is transparent: its `Display` and `source()` are those of the area
  error it holds, so an error report does not print that error twice. Get the area error with a
  `match`, not from `source()`.
- `Display` is lowercase, one clause, without a trailing period.
- Fallible public calls return `Result` and do not panic on caller input; `expect` is for internal
  invariants only.

## R7. Derives

- Every public type implements `Debug` (the crate sets `#![warn(missing_debug_implementations)]`).
  Handles show their id; wrappers of native objects use `finish_non_exhaustive`.
- Ids are `Clone + Copy + PartialEq + Eq + PartialOrd + Ord + Hash`.
- Readouts, events and settings are `Clone + PartialEq`, and `Copy` when they hold no heap data and
  no Jolt reference. Settings are `Default` unless R3 says otherwise.
- Settings that borrow a `Shape` or a `PhysicsMaterial` (`CharacterSettings`, `MeshSettings`) or
  hold the caller's job system (`WorldSettings`) are not `PartialEq`: shapes, materials and job
  systems have no equality. `RagdollSettings` and `SoftBodySharedSettings` are built Jolt objects,
  not settings in this sense.
- `tests/api_traits.rs` checks these traits at compile time.

## R8. `#[non_exhaustive]`

- On every error enum, every pub-field readout or event struct the crate produces, and every enum
  Jolt may extend (sub-types, results).
- Enums that mirror a closed Jolt enum stay exhaustive: `MotionType`, `MotionQuality`,
  `Activation`.
- Pub-field structs and enums the caller writes as literals stay exhaustive (`CompoundChild`,
  `ContactPoint`, input vertices, `RagdollJoint`).

## R9. Integers

- Counts of world objects and of Jolt indices (bodies, constraints, wheels, parts, joints,
  sub-shapes) are `u32`, Jolt's own index type.
- Counts and indices of caller-indexed buffers (vertices, faces, triangles,
  `DroppedTriangles::count`) are `usize`.

## R10. Constants

- Physical magnitudes live in `limits`.
- Per-type capacities and Jolt defaults are associated consts of their type
  (`WorldSettings::MAX_BODIES`, `WheelSettings::DEFAULT_LATERAL_FRICTION`).
- Nothing else sits at the crate root.

## R11. Layout

- The root is flat: modules are private except `error`, `limits` and `prelude` (with
  `prelude::math`).
- The root names nothing that common globs export besides the math types (`Vec3`, `Quat`, `Real`,
  `RVec3`); `Error` and `Result` stay in `error`.
- The prelude holds the types most programs name (world, bodies, shapes, layers, queries, events,
  ids, the settings of the main objects, every area error) and no math type or `Result`, so it can
  be glob-imported next to `bevy::prelude::*`. `tests/prelude.rs` and `tests/root_glob.rs` guard
  this.

## R12. Features

- Features are additive, except `double-precision`, which changes `Real` in every signature. Only
  the final application enables it; libraries built on `oxijolt` forward it.
- `bindgen` is a build switch outside the semver promise.
- A feature for a public dependency carries the dependency's version in its name (`glam032`,
  `glam033`), so a later version can be added next to it in a minor release. `mint` (0.5) keeps its name.

## R13. Minimum supported Rust version

- `rust-version` in both crates' `Cargo.toml` is the MSRV; CI checks the workspace with that
  toolchain.
- The MSRV is at least six months old. Raising it is a minor release, noted in the changelog.
- Edition 2021.

## R14. Documentation

- `#![warn(missing_docs)]`; CI turns warnings into errors.
- Every public item states its units, conventions and errors (`# Errors`). Every `unsafe fn` has a
  `# Safety` section.
- Measured numbers live in `docs/`, linked from the rustdoc.
