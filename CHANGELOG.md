# Changelog

All notable changes to this fork. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## Unreleased

- `BodyRef::inverse_inertia` reads a dynamic rigid body's inverse inertia tensor about its centre
  of mass in world coordinates (Jolt `Body::GetInverseInertia`), in rows like `Inertia::Tensor`,
  with the body's rotation applied and locked rotation axes zeroed; `None` for static, kinematic
  and soft bodies. The idea comes from pull request #3 by @def9a2a4.
- The `glam033` feature adds the `glam032` conversions for glam 0.33, the version Bevy 0.20 uses:
  `Vec3`, `Quat` and the vector of `Real` (`DVec3` in double precision), both ways, bit for bit.
  Both glam features can be enabled together. Refs #4.
- Soft body skinning (Jolt's skinned constraints): `SoftBodySharedSettingsBuilder::inv_bind`,
  `inv_binds` and `skinned` take the joints' bind pose (`SoftBodyInvBind`, with
  `SoftBodyInvBind::from_pose` for a world pose) and the skinned vertices (`SoftBodySkinned` with
  up to four `SoftBodySkinWeight`s, a max distance and a `SoftBodyBackStop`), checked by `build`
  and bounded by the new `limits::MAX_SKIN_JOINTS`. `SoftBodyMut::skin_vertices` and
  `hard_skin_vertices` move the skin to a world `SkeletonPose` such as `RagdollRef::pose` returns,
  and `set_skinned_max_distance_multiplier` with `SoftBodyRef::skinned_max_distance_multiplier`
  scale the max distances. A new skinned body starts at its bind pose. The C wrapper extension
  gains `JPH_SoftBodySharedSettings_AddInvBind`, `AddSkinnedConstraint`,
  `CalculateSkinnedConstraintNormals`, `GetInvBindCount`, `GetSkinnedConstraintCount` and
  `JPH_Body_GetSoftBodySkinJointCount`, `SkinSoftBodyVertices`,
  `Get`/`SetSoftBodySkinnedMaxDistanceMultiplier` ([guide](docs/soft-bodies.md#skinning)). Refs #6.

## 1.4.0 — 2026-10-28

- `BodySettings::mass_and_inertia(mass, inertia)` gives a body its mass and inertia, and Jolt
  uses neither the shape's mass nor its inertia (`EOverrideMassProperties::MassAndInertiaProvided`);
  the centre of mass stays the shape's. `Inertia::Principal` takes the moments about the body's
  axes, `Inertia::Tensor` the full tensor in Jolt's convention, both about the centre of mass in
  body space. `create_body` refuses an inertia that is not finite or symmetric, has a principal
  moment below the new `limits::MIN_PROVIDED_INERTIA` (1e-6 kg·m²) or above
  `6 · mass · limits::MAX_SHAPE_EXTENT²`, or is not diagonal and not positive definite and well
  conditioned ([limits](docs/limits.md#provided-inertia)). The last of `mass` and
  `mass_and_inertia` wins; `BodyMut::set_shape` computes the inertia from the new shape again;
  `RagdollSettings::new_stabilized` refuses parts with a provided inertia.
- The engine and transmission of every vehicle kind change while it drives:
  `VehicleRef::engine_settings`, `transmission_settings` and `VehicleMut::set_engine`,
  `set_transmission`. A change is checked as at creation and refused whole; the rpm, gear, clutch
  and shift timers carry over. The number of forward and reverse gears and the clutch release
  time are fixed at creation, since Jolt's state saves the gear and the clutch countdown but not
  what they are read against; a change of them is the new `VehicleError::FixedAtCreation`. A
  tank's track drive envelope also covers the track speeds earlier configurations allowed
  ([limits](docs/limits.md#live-changes)).
- Wheeled vehicles and motorcycles change their differentials and the limited slip ratio between
  them (`VehicleMut::set_differentials`, read with `VehicleRef::differentials` and
  `differential_limited_slip_ratio`), bounded by the new sealed trait `WheeledControllerKind`.
  Tanks change both tracks' inertia, damping, brake torque and differential ratio
  (`VehicleMut::set_tracks`, read with `VehicleRef::track_settings`), checked against the
  chassis' track mass ratio; a track's wheels and driven wheel are fixed at creation.
- A per-vehicle tire max impulse callback for wheeled vehicles and motorcycles
  (`VehicleMut::set_tire_max_impulse_callback`, `clear_tire_max_impulse_callback`,
  `VehicleRef::has_tire_max_impulse_callback`): a `Fn(&TireMaxImpulseInput) -> TireFriction`
  closure replaces the friction coefficients that Jolt multiplies by the suspension impulse to cap
  each wheel's tire impulses; each coefficient is clamped to `0..=limits::MAX_FRICTION` and NaN
  counts as 0. It runs on the solver's threads, so it is `Send + Sync`, and its results do not
  depend on the thread count as long as they depend only on the input and on data that does not
  change during the step; a panic in it gives the vehicle Jolt's friction for the rest of the step
  and is resumed by `step`. The callback is dropped with its vehicle and is not part of
  `WorldState`.
- None of the live settings above is part of `WorldState`: a restore keeps the current ones
  ([state](docs/state.md#what-is-not-saved)).
- The joltc extension adds `JPH_WheeledVehicleController_SetEngine`, `_SetTransmission`,
  `_SetDifferentials`, `_SetDifferentialLimitedSlipRatio`, `_ResetTireMaxImpulseCallback` and
  `JPH_TrackedVehicleController_SetEngine`, `_SetTransmission`, `_SetTrack`. `JOLTC_EXT_REVISION`
  is 25: a `JOLTC_LIB_DIR` prefix built before this change is refused, the 1.3.0 release archives
  no longer match and the build falls back to source until the next release, and shape binary
  states saved by an earlier build are refused with `BinaryStateError::OtherBuild`; save them
  again.

## 1.3.0 — 2026-10-27

- `WorldSettings::penetration_slop` and `speculative_contact_distance` set Jolt's penetration
  slop and speculative contact distance per world, in metres (default 0.02 m, Jolt's;
  `0..=limits::MAX_SHAPE_EXTENT`, other values refused with `WorldError::InvalidValue` before any
  native call). They are set when the world is created and read back with
  `PhysicsWorld::penetration_slop` and `speculative_contact_distance`. The slop is the overlap
  Jolt leaves uncorrected, not a depth bodies are pushed to
  ([limits](docs/limits.md#contact-distances)).
- CI builds and tests macOS on Apple silicon (`aarch64-apple-darwin`, Apple clang) in the same
  five configurations as Windows and Linux, together with the playground and the real-mesh
  checks. The engine comparison stays on Windows and Linux, where its harness reads CPU time and
  memory.
- Prebuilt native libraries for macOS on Apple silicon (`aarch64-apple-darwin`), built for macOS
  11.0 and recorded as `macos=` in `prebuilt.txt`; the build script refuses them for an older
  deployment target (`MACOSX_DEPLOYMENT_TARGET`). Intel Macs build from source.
- The published `oxijolt` package leaves out the tests that build only inside the repository
  (the real-mesh checks and the character study), so `cargo test --no-run` builds the tests of
  the unpacked crate. The doctests of the README and the guides, which the package does not
  ship, run only inside the repository, so `cargo test` passes the doctests of the unpacked
  crate. The tests still run in the repository and in CI.

## 1.2.0 — 2026-10-21

- `PhysicsWorld::step_with_collision_steps(delta_time, collision_steps)` advances the world by
  `delta_time` in `collision_steps` collision steps inside one Jolt update, the way to sub-step a
  fast scene; `step(dt)` is the same call with one collision step. The count is
  `1..=PhysicsWorld::MAX_COLLISION_STEPS` (4, so that one update fits Jolt's job pool,
  [limits](docs/limits.md#collision-steps)); outside it the call returns the new
  `StepError::InvalidValue`. `StepError::InvalidDeltaTime` now also covers a collision step
  shorter than `MIN_DELTA_TIME`. The determinism gates run stacks, a fleet and the event scene
  with 4 collision steps per call.
- `WorldSettings::single_threaded` runs a world's jobs on Jolt's `JobSystemSingleThreaded`: on the
  stepping thread, with concurrency 1, without threads. The determinism gates compare it bit for
  bit with Jolt's thread pool.
- A vehicle's gravity override (`VehicleMut::set_gravity`) acts once per collision step. Jolt's
  `VehicleConstraint` adds the override force in every collision step, and the force accumulator
  clears only after the last one, so with several collision steps the override would grow step by
  step; vehicles now run on the extension's `JPH_VehicleStepListener`, which keeps it to one force.
  With one collision step nothing changes.
- The joltc extension adds `JPH_JobSystemSingleThreaded_Create` and `JPH_VehicleStepListener_Create`,
  `_Destroy` and `_AsPhysicsStepListener`. `JOLTC_EXT_REVISION` is 24: a
  `JOLTC_LIB_DIR` prefix built before this change is refused, the 1.1.0 release archives no longer
  match and the build falls back to source until the next release, and shape binary states saved
  by an earlier build are refused with `BinaryStateError::OtherBuild`; save them again.
- Constraint impulse readouts and `BodyMut::add_force` say how collision steps change them: a
  readout is the last collision step's (divide by `delta_time / collision_steps`), a rigid body's
  force acts over the whole call and a soft body's only in the first collision step
  ([docs/constraints.md](docs/constraints.md#breakable-constraints)).
- Docs: `WorldSettings::max_bodies` says that `max_body_pairs` and `max_contact_constraints` do not
  grow with it; `StepReport::is_complete` says that an incomplete step leaves a valid world and
  names the limit to raise; the README notes `f32` positions, `double-precision` and the floating
  origin.

## 1.1.0 — 2026-10-11

- Prebuilt native libraries: on `x86_64-pc-windows-msvc` and `x86_64-unknown-linux-gnu` the build
  script of `oxijolt-sys` downloads the GitHub release archive of its own version that matches the
  target, C runtime and features, checks it against the sha256 recorded in the packaged
  `prebuilt.txt`, and links it without CMake or a C++ compiler. It needs `curl`.
  It builds from source with one warning when no archive fits: other targets or features
  (`asserts`), `crt-static`, native sources that differ from the release, an older MSVC toolset or
  glibc than the archive's, a linker override or cross build, offline (`CARGO_NET_OFFLINE`),
  vendored or Nix builds, or a failed download. `JOLTC_PREBUILT=off` skips the archives,
  `JOLTC_PREBUILT=require` makes a fallback an error, and `JOLTC_PREBUILT_URL` names a mirror.
  `JOLTC_LIB_DIR` and docs.rs work as before ([docs/building.md](docs/building.md#prebuilt-native-libraries)).
- The new default feature `prebuilt` of both crates turns the download on; `oxijolt` depends on
  `oxijolt-sys` with `default-features = false` and forwards it, so `default-features = false` on
  `oxijolt` builds from source.
- Release archives for Linux are built with GCC 12 in Ubuntu 22.04 and need glibc 2.35 (was GCC 13,
  Ubuntu 24.04, glibc 2.39).
- Releases are driven by the tag: the Release workflow builds the archives, records their list on
  `release/v<version>`, attaches them to the GitHub release and publishes both crates from that
  commit ([docs/building.md](docs/building.md#releases)).
- The build script refuses a native manifest (`JOLTC_LIB_DIR` or an archive) with a repeated key.
- Constraint impulse readouts: `ConstraintRef<HingeConstraint>::total_lambda_rotation` and
  `ConstraintRef<SliderConstraint>::total_lambda_position`, so every constraint impulse readout of
  the C layer has a safe method. The readouts' docs state their units, that they are the impulse of
  the last step the constraint was solved in, their frame and how they map to each body; a disabled
  or sleeping constraint keeps its last values. The constraints guide has a section on breakable
  constraints ([docs/constraints.md](docs/constraints.md#breakable-constraints)).
- Playground: a `bonds` scene, a brick wall of separate bodies joined by fixed constraints that
  break when their impulse readouts pass a force or torque limit; lowering the wall's prop cracks
  it at its pier ([docs/playground.md](docs/playground.md#-breakable-bonds-bonds)).
- A comparison with Rapier 0.36.0 and Avian 0.7.0 on ten of Rapier's stress scenes
  ([docs/comparison.md](docs/comparison.md)): speed at 1 to 16 threads, quality bounds,
  determinism across thread counts and features, with the harness (`benches/comparison`, not
  published), its scripts and the raw results of a 16-core Linux machine. Where oxijolt loses is
  stated there.
- `WorldSettings::velocity_steps` and `position_steps`: the solver's velocity and position
  iterations per step, set when the world is created (Jolt's defaults 10 and 2; velocity
  `2..=255`, position `0..=255`, [limits](docs/limits.md#solver-step-counts)), read back with
  `PhysicsWorld::velocity_steps` and `position_steps`.
- Playground: in `destruction`, a wall piece that lands hard on the ground or on another piece
  breaks at the contact point that hit hardest, also when it tips over an edge it stands on.

## 1.0.1 — 2026-10-07

### Fixed

- Linux and other non-MSVC, non-Apple builds compile Jolt with its x86-64 instruction-set flags
  (`-mavx2 -mfma` and the rest, and Jolt's matching `JPH_USE_*` definitions): joltc caches
  `CMAKE_OSX_ARCHITECTURES=x86_64;arm64` on every platform, which Jolt read as an ARM build, so
  GCC and Clang built it for plain x86-64. The native library now needs a CPU with AVX2 and FMA
  on those platforms too, as it already did with MSVC; CI checks the instructions on Linux.

## 1.0.0 — 2026-10-06

- The public API follows one set of rules, written down in
  [docs/api-guidelines.md](docs/api-guidelines.md).
- Both crates declare `rust-version = "1.88"`, the first Rust with `<[T]>::as_chunks`, and CI
  checks the workspace with that toolchain. Raising it will be a minor release.
- `PhysicsWorld::save_state_into(bodies, &mut state)`: saves into an existing `WorldState` and
  reuses its memory, so a rollback ring allocates nothing on the Rust heap per save;
  `WorldState::new` (and `Default`) is an empty state of no world, and `WorldState::data_size`
  the length of Jolt's stream ([state guide](docs/state.md#size) lists what each object adds).
- `BodySelection` (`All`, `Movable`, `Only`) for `save_state_of`, `save_state_into` and the new
  `PhysicsWorld::restore_state_of`, which restores only the selected bodies and leaves every
  other body in its current state.
- Shape cooking ([guide](docs/shape-cooking.md)): `Shape::save_binary_state` and `unsafe
  Shape::restore_binary_state` save a built shape with its children and materials to bytes and
  restore it, so level meshes are built once at asset-build time. The bytes carry a versioned,
  checksummed header that names the build (Jolt and joltc commits, extension revision, precision,
  determinism mode, byte order); bytes of another build, cut-short bytes and most damage are
  refused with `ShapeError::BinaryState(BinaryStateError)`. Restoring is `unsafe` because Jolt does
  not validate the inside of its records: the bytes must be the unchanged output of a save by the
  same build.
- joltc extension: `JPH_Shape_SaveBinaryState`, `JPH_Shape_RestoreBinaryState`,
  `JPH_ShapeBinaryState_GetSize`, `_CopyData`, `_Destroy` and `JPH_Shape_GetBinaryStateVersion`,
  which walk the shape graph themselves and check every record's envelope before Jolt reads it.
  `JOLTC_EXT_REVISION` is 22: a `JOLTC_LIB_DIR` prefix built before this change is refused until
  the next release's archives. `oxijolt-sys` exports `JOLT_COMMIT`, `JOLTC_COMMIT`,
  `JOLTC_EXT_REVISION` and `CROSS_PLATFORM_DETERMINISTIC_ENABLED`.
- Real meshes from open sources tested in CI ([report](docs/real-meshes.md)): CC0 and
  public-domain models (Kenney props and level pieces, Crane's oloid and spot, the Khronos
  ScatteringSkull) with their sources, licences and SHA-256 in `assets/models/models.tsv`,
  `scripts/fetch_models.py` for the large ones, and a `real-meshes` CI job with and without
  assertions. The skull is tested at its own size (0.25 m, built for convex shapes up to 2 m, which
  drops 0.48 % of its area) and as a 2.5 m statue.
- Playground: the scenes `meshes` (the real models with bodies raining onto them) and `model`
  (any OBJ or glTF file given with `--model PATH`); `--models DIR` adds the downloaded models.
  The wall of the `destruction` scene is a dynamic body that splits into pieces of touching
  bricks, so a piece without support topples or falls.
- Thin dynamic hulls on a floor ([limits](docs/limits.md#thin-dynamic-hulls-on-a-floor)): the
  sinking reported earlier is the impact of a wide thin hull that tips over and slaps down faster
  than Jolt's discrete step resolves, not its hull or convex radius. It sinks deep into box floors
  and can pass through mesh floors; `LinearCast` and shorter steps reduce it.
- Every public type implements `Debug`. `WorldSettings::MAX_BODIES` (2^23),
  `CollisionLayers::MAX_OBJECT_LAYERS` (65535), `WheelSettings::DEFAULT_SUSPENSION_SPRING`,
  `RayCast::origin`/`direction`, `PhysicsWorld::debug_lines` (allocating),
  `SliderConstraint`'s `remove_limits`, `SixDofAxis::default()` (`Free`),
  `QueryError::{WrongWorld, UnknownObjectLayer, AllocationFailed}`,
  `CharacterError::{Query, RestoreFailed}` and `CollisionGroupError` in the prelude are new, as are
  `character_count`, `contains_character`, `vehicle_count`, `contains_vehicle`, `ragdoll_count`,
  `contains_ragdoll` and `contains_constraint`; `BodySelection` and `BinaryStateError` are in the
  prelude, and `DebugLineSettings` is `PartialEq`.
- Changed (breaking): `save_state_of` takes a `BodySelection` instead of `&[BodyId]` and returns
  `StateError`; a foreign or removed id is `StateError::Body` (was `BodyError`).
- Changed (breaking): `MeshSettings::DEFAULT_MAX_CONVEX_EXTENT` is 300 m (was 1100 m), so meshes
  keep thinner triangles, such as the 1 mm bevels of small props. A mesh that meets convex shapes
  larger than 300 m is built with a larger `max_convex_extent`
  ([limits](docs/limits.md#convex-shapes-against-meshes)).
- Changed: mesh triangles are kept from twice the area of 1e-6 m², Jolt's own limit, plus the
  rounding margin (was 1e-5 m²), so a right triangle with legs of 1.2 mm is kept at the default
  extent (was 3.8 mm) ([limits](docs/limits.md#triangle-meshes)).
- Changed (breaking), renames: `RayHit` to `RayCastHit`, `PointHit` to `CollidePointHit`,
  `HullError` to `ConvexHullError`, `BodyError::SoftBody` to `NotRigidBody`, `VehicleSettings`
  and `create_vehicle` to `WheeledVehicleSettings` and `create_wheeled_vehicle`,
  `SuspensionSpring` to `SpringSettings` (whose `Default` is 0 Hz and no damping, Jolt's spring
  default; a wheel's 1.5 Hz and 0.5 is `WheelSettings::DEFAULT_SUSPENSION_SPRING`),
  `Shape::scaled` to `Shape::new_scaled`,
  `PhysicsWorld::contains` to `contains_body`, `CharacterState::as_bytes` to `to_bytes`,
  `ContactPoint::{on1, on2}` to `point_on1`, `point_on2`, the `inv_*` contact scales to
  `inverse_*`, and the feature `glam` to `glam032`. The root `DEFAULT_*` vehicle curves are
  associated consts of `WheelSettings` and `VehicleEngineSettings`.
- Changed (breaking), call shapes: `RayCast` has private fields and `cast_ray` borrows it;
  `body_ids` and `constraints_of_body` return iterators and `constraint_count` a `u32`;
  `debug_lines` returns the lines and `debug_lines_into` fills a buffer;
  `new_convex_hull(points)` and `new_tapered_cylinder(half_height, top, bottom)` take Jolt's
  0.05 m convex radius and their `_with_convex_radius` forms take one; height-field materials
  are set with `HeightFieldSettings::materials`; `auto_detect_point(bool)`; the gear's
  `constraints(..)`; the slider's `set_limits(min, max)` and `remove_limits()`;
  `collision_tester()` returns a copy.
- Changed (breaking), errors: out-of-range values are `InvalidValue` in every area (was
  `InvalidSettings` or `InvalidDimensions`); a query filter's body of another world, unknown
  object layer or failed native object is `QueryError::WrongWorld`, `UnknownObjectLayer` or
  `AllocationFailed` (was `InvalidValue`); a character update wraps a filter error in
  `CharacterError::Query`, and a failed character restore is `CharacterError::RestoreFailed`;
  area errors that wrap another one return it from `source()`, and the umbrella `error::Error`
  shows the area error it holds as its own `Display` and `source()`.
- Changed (breaking): the readouts `DebugLine`, `ContactManifold`, `ContactSettingsRejection`,
  `SoftBodyVertexState`, `SoftBodyContacts`, `SoftBodyVertexContact` and `SoftBodyValidation` are
  `#[non_exhaustive]`.

## 0.7.0 — 2026-10-06

- `CharacterSettings::humanoid(height, radius)`: settings that own a capsule `height` tall standing
  on the character position, with Jolt's defaults for the rest; their clones share the capsule.
- `VehicleSettings::car(front_left_wheel, wheel_radius, tester)`: a front-wheel-drive car of four
  mirrored wheels with steering, hand brake, anti-roll bars and Jolt's default engine and
  transmission, validated by `create_vehicle` like any settings.
- `MotorcycleSettings::bike(front_wheel, wheel_radius, tester)`: the motorcycle of Jolt's
  `MotorcycleTest` sample (raked front suspension, rear drive, six gears), validated by
  `create_motorcycle`.
- `VehicleRef::wheel_world_transform(index)`: a wheel's centre and rotation in world space (Jolt's
  `GetWheelWorldTransform`) for a wheel model whose axle is its local Y, so a renderer can draw
  wheels from what the binding reports.
- `PhysicsWorld::body_ids()`: every body of the world in ascending id order (Jolt's
  `PhysicsSystem::GetBodies`), soft bodies, ragdoll parts and characters' inner bodies included.
- Playground (`examples/playground`, [guide](docs/playground.md)): ten scenes in a window that show
  the binding's features, a headless mode that runs every scene's script and prints a digest
  (compared with 1 and 4 worker threads in CI), and a record mode that makes the GIFs of the
  playground guide in `docs/media`. The playground is a workspace member outside the default
  members.

## 0.6.0 — 2026-10-06

- Skeleton mapper ([guide](docs/guide.md#ragdolls)): `SkeletonMapper` (Jolt's `SkeletonMapper`)
  between a ragdoll skeleton and a detailed animation skeleton matched by name, with
  `MappedSkeleton` and `TranslationLocks`. `map` turns a ragdoll pose and the animation's local
  pose into an animation pose, `map_reverse` an animation pose into a ragdoll pose for `set_pose`
  and the drives. New `RagdollError::UnmappedJoint`, `HierarchyMismatch` and `DegenerateChain`;
  `limits::MIN_MAPPED_CHAIN_LENGTH` is the shortest chain direction
  ([limits](docs/limits.md#skeleton-mapper-chains)).
- joltc extension: `JPH_SkeletonMapper_Initialize2`, `_LockAllTranslations2`, `_LockTranslations2`,
  `_Map2` and `_MapReverse2`, which copy matrix arrays through aligned storage (joltc's own five
  cast 4-aligned arrays to 16-aligned `Mat44` and stay unbound) and refuse inputs that would make
  Jolt assert or read out of bounds. `JOLTC_EXT_REVISION` is 20: a `JOLTC_LIB_DIR` prefix built
  before this change is refused until the next release's archives.
- Compound edits at run time ([guide](docs/bodies.md#compound-edits)): `MutableCompound` adds,
  removes, moves and replaces compound children, checks every edit before Jolt sees it, and
  publishes the current children as a new `Shape` (equal to `Shape::new_compound` of them) that
  `BodyMut::set_shape` installs on one body at a time. `ShapeError::NoSubShape` reports an index
  past the last child.
- joltc extension: `JPH_PhysicsMaterial_AddRef` and `JPH_PhysicsMaterial_GetRefCount`.
  `JOLTC_EXT_REVISION` is 18: a `JOLTC_LIB_DIR` prefix built before this change is refused until the
  next release's archives.
- Fixed: a character could reference a freed physics material when an update shorter than its
  `min_time_remaining` reused contacts with a body whose shape was replaced or removed. Characters
  now keep the materials of their cached contacts alive.
- Changed: `Shape::new_compound` refuses a hierarchy in which Jolt would form a sub-shape id past
  its 32 bits (a one-child compound under 32 bits of parents), which Jolt accepted and then shifted
  out of range.
- Changed: `Shape::new_compound` and `MutableCompound` refuse a compound of more than
  `limits::MAX_EXPANDED_SUB_SHAPES` (2^20) shapes, counting a child shared by several parents at
  every use, with the new `ShapeError::TooManySubShapes`. Jolt walks every use of a shared child, so
  a deeply shared graph made building, mass and stepping take minutes
  ([limits](docs/limits.md#expanded-compounds)).

- Plane shapes ([guide](docs/guide.md#shapes)): `Shape::new_plane` and `new_plane_with_material`, a
  static ground plane cut to a square of a half extent; static bodies only, like meshes and
  heightfields.
- Point queries ([guide](docs/guide.md#queries)): `PhysicsWorld::collide_point` with `PointHit`
  (filtered like the other queries, sorted by body and sub-shape id) and `Shape::collide_point`,
  with Jolt's containment rule per shape.
- Buoyancy ([guide](docs/bodies.md#buoyancy)): `BodyMut::apply_buoyancy_impulse` with
  `BuoyancySettings` (surface, buoyancy factor, linear and angular drag, fluid velocity) and the
  gravity passed per call. Every product of Jolt's arithmetic is bounded, the buoyant velocity
  change like an impulse's, and both velocities are clamped right after the call
  ([limits](docs/limits.md#buoyancy)).
- Collision estimates ([guide](docs/events.md#impact-estimates)): `EventSettings::collision_estimates`
  gives every added contact Jolt's `CollisionEstimate` of its impulses and the bodies' velocities
  after the impact.
- joltc extension: `JPH_Body_GetSubmergedVolume` and `JPH_Body_ApplyBuoyancyImpulse2` (Jolt's volume
  overload of the buoyancy impulse). `JOLTC_EXT_REVISION` is 17: a `JOLTC_LIB_DIR` prefix built
  before this change is refused until the next release's archives.
- Changed (breaking): `ContactEvent::Added` has an `estimate` field (`None` unless collision estimates are
  on), so patterns that name every field need `..` or the new field.

- Contact validation ([guide](docs/events.md#validating-contacts)): `ContactListener` gains
  `contact_validate`, called with a `ContactCandidate` (both bodies in Jolt's collide order, their
  user data, sub-shapes, world-space points, penetration axis and depth) and answering a
  `ValidateResult`. The default accepts every hit, as Jolt does without a listener; a panic accepts
  the hit and is resumed by `step`.
- `BodyMut::invalidate_contact_cache`: the next step that simulates collides the body's pairs
  afresh, so a changed validation answer reaches resting pairs. It wakes the body, and the pending
  request is saved in `WorldState` (also by `save_state_of`) and replaced on restore.
- Character contact listener ([guide](docs/events.md#character-contacts-charactercontactlistener)):
  `CharacterContactListener` and `PhysicsWorld::set_character_contact_listener` adjust the velocity
  a character sees for a body (`BodyVelocity`, bounded like body velocities), validate contacts
  with bodies and characters, set `CharacterContactSettings` (can push the character, receives
  impulses) and report added, persisted and removed contacts (`CharacterContactKey`). Removals are
  delivered after the update, sorted.
- Collision groups ([guide](docs/bodies.md#collision-groups)): `GroupFilterTableBuilder`,
  `GroupFilterTable` (at most `GroupFilterTable::MAX_SUB_GROUPS` sub groups), `CollisionGroup`
  (group ids up to `CollisionGroup::MAX_GROUP_ID`) and `CollisionGroupError`;
  `BodySettings::collision_group` and `SoftBodySettings::collision_group`, fixed at creation.
- joltc extension: `JPH_ContactListener2_SetProcs`, `_Create`, `_Destroy` and
  `JPH_PhysicsSystem_SetContactListener2`, a contact listener whose validate callback gets the
  collide result without faces, so validation allocates nothing.
- Changed: the world's native contact listener is the extension's `JPH_ContactListener2`; joltc's
  own `JPH_ContactListener_*` is no longer used by `oxijolt`. `oxijolt` now also installs joltc's
  character contact listener proc table, which other code linking both crates must not set.
- Changed: ragdoll parts use the collision group id `2^31 + RagdollId::to_raw()` instead of the
  raw id itself, so caller groups never share an id with a ragdoll; ragdoll ids are unchanged, and
  a ragdoll part refuses a collision group.
- Fixed: a state saved right after `BodyMut::set_shape` replayed with the old shape's cached
  contacts, because Jolt's contact-cache invalidation was not part of the saved state.

- Tracked vehicles ([guide](docs/vehicles.md)): `PhysicsWorld::create_tracked_vehicle` with
  `TrackedVehicleSettings`, two `VehicleTrackSettings` that own their `TrackedWheelSettings`, and
  Jolt's tracked engine and transmission defaults. `TrackedDriverInput` sets throttle, brake and a
  speed ratio per track, at least `1/limits::MAX_RATIO` in magnitude; `VehicleRef::tracks`,
  `track`, `track_wheels` (`TrackState`, `TrackSide`) read them back. A track's inertia is within
  `limits::MIN_TRACK_INERTIA..=MAX_TRACK_INERTIA` and its wheel radii within a factor
  `limits::MAX_RATIO` of its driven wheel's, the two tracks' inertias within a factor
  `limits::MAX_TRACK_INERTIA_RATIO` (100) of each other, because Jolt's ground contact diverged
  between very unequal tracks (`docs/limits.md`, Track inertia ratio), and the track speeds the
  drivetrain can reach must keep the tracked step finite (`docs/limits.md`, Track drive envelope).
  `create_tracked_vehicle` refuses tracks heavy against their chassis: every wheel's track inertia
  over its radius squared, times the chassis' largest inverse effective mass at the wheel over the
  wheel's forward/up plane, is at most `limits::MAX_TRACK_MASS_RATIO` (0.25), because Jolt's ground
  contact diverged and spun the chassis up to its angular velocity clamp from a ratio of 6 at steps
  up to 0.1 s (`docs/limits.md`, Track mass ratio). Jolt's `TankTest` hull accepts tracks up to 11 kg·m².
- Motorcycles: `PhysicsWorld::create_motorcycle` with `MotorcycleSettings`, which wraps a
  two-wheeled `VehicleSettings` and adds the lean controller: max lean angle up to
  `MotorcycleSettings::MAX_LEAN_ANGLE` (80°), a lean spring bounded against the chassis' inertia,
  smoothing, and the lean controller and lean steering limit switches, fixed at creation.
  `VehicleRef::lean` (`MotorcycleLean`) reads the target lean and the lean angles. A rotating
  `rebase` rotates each motorcycle's target lean.
- `VehicleError::LeanSpringIntegrationNotSaved`: Jolt does not save a motorcycle's integrated lean
  angle, so a nonzero lean spring integration coefficient is refused.
- joltc extension: `JPH_MotorcycleController_GetTargetLean` and `_SetTargetLean`, which check the
  controller's kind.
- Changed (breaking): an engine's torque curve (`VehicleEngineSettings::normalized_torque`, for
  every vehicle kind) needs x within `0..=1`, at least `limits::MIN_TORQUE_CURVE_SPACING` apart
  up to rounding x to `f32` (points at `i / 1000` pass), and y within
  `0..=limits::MAX_NORMALIZED_TORQUE`. Any finite points with increasing x were accepted before,
  and wide spans overflowed Jolt's interpolation (`docs/limits.md`, Torque curves).
- Changed (breaking): vehicle ids are typed by kind. `VehicleId` is now `VehicleId<K>` with `K`
  `WheeledVehicle` (the default), `TrackedVehicle` or `Motorcycle`, and `VehicleRef` and
  `VehicleMut` take the same parameter. `vehicle_ids` and `vehicle_of_body` return `AnyVehicleId`
  (with `kind` and `downcast`), `remove_vehicle` takes either id, and `VehicleError::NotFound` and
  `WrongWorld` carry an `AnyVehicleId`. An id's `Debug` output names its kind:
  `VehicleId(Wheeled, 1)`.

## 0.5.0 — 2026-10-05

- Shapes: convex hulls (`Shape::new_convex_hull`, `new_convex_hull_with_material`, `HullError`),
  triangle meshes with a material per triangle for static and kinematic bodies (`Shape::new_mesh`,
  `new_mesh_with_settings`, `MeshSettings`, `MeshBuildQuality`, `MeshError`), scaled shapes
  (`Shape::scaled`) and tapered capsules and cylinders, cones among them
  (`Shape::new_tapered_capsule`, `new_tapered_cylinder`). Point clouds, scales and radii that Jolt
  cannot build or collide with reliably are refused with typed errors; anything else Jolt refuses
  comes back as `ShapeError::Rejected` with Jolt's message (`JoltMessage`, cut after a whole word).
- Mesh triangles too small or too thin to collide with are dropped and returned as
  `DroppedTriangles` (indices and area). The rule counts the rounding of the triangle in the space
  of the convex shape it collides with, up to `MeshSettings::max_convex_extent` (default 1100 m,
  below `limits::MAX_SHAPE_EXTENT`; see `docs/limits.md`), and does not depend on the order of a
  triangle's corners. `Shape::scaled` checks a mesh inside it for the extent the mesh was built
  with and refuses a scale that leaves its triangles too thin with `ShapeError::ThinTriangles`
  (`ThinTrianglesError`: the scale and the extent).
- With the `asserts` feature Jolt's hull builder can abort on point clouds with many nearly
  coplanar faces that pass the hull rules: about one densely sampled noisy box in 1000 and 1 to 2 %
  of dense flat cones and domes; without it Jolt refuses most of them as `ShapeError::Rejected`
  and builds the rest. See `docs/limits.md`. The
  `HullError::Coplanar` message suggests thickening the cloud or centring it on the shape origin.
- joltc extension: `JPH_ShapeSettings_CreateShapeWithError`, `JPH_MeshShapeSettings_Create3`
  (materials), `JPH_MeshShapeSettings_GetTriangleCount` and `JPH_Shape_GetTriangles`.
- Body controls ([guide](docs/bodies.md)): impulses (`BodyMut::add_impulse`, `add_angular_impulse`,
  `add_impulse_at_point`), bounded by the new `limits::MAX_VELOCITY_CHANGE` and
  `limits::MAX_ANGULAR_VELOCITY_CHANGE`; kinematic moves (`BodyMut::move_kinematic`); waking and
  sleeping on demand (`BodyMut::activate`, `deactivate`) and `PhysicsWorld::activate_bodies_in_box`,
  which picks bodies by their exact bounds in body-id order.
- Configuration fixed at creation, with getters on `BodyRef`: sensor bodies
  (`BodySettings::sensor`, contacts reported as sensor contacts), per-body user data
  (`BodySettings::user_data`), locked axes (`BodySettings::allowed_dofs`, `AllowedDofs`) and the
  movement capability of static bodies (`BodySettings::allow_dynamic_or_kinematic`).
- Structural changes: `BodyMut::set_motion_type` and `BodyMut::set_shape`. Every successful
  `set_shape` call, also one with the body's current shape, and every change of motion type makes
  earlier `WorldState`s unrestorable; `set_motion_type` with the current motion type changes
  nothing. Both refuse bodies that a character, vehicle, ragdoll or constraint holds. A shape
  change wakes the bodies inside the box enclosing the body's old and new bounds.
- New `BodyError` variants: `RestrictedDofs`, `NotKinematic`, `CannotMove`. Constraints, vehicles,
  ragdoll parts and rotating rebases refuse bodies with fewer than six degrees of freedom; ragdoll
  parts refuse user data, which Jolt overwrites.
- `BodyMut` now borrows its world (the signature of `body_mut` is unchanged).
- Fixed: `BodyMut::add_force_at_point` checked the exact torque, while Jolt's `f32` cross product,
  with a product fused into the subtraction, keeps a rounding error. A force parallel to a long lever
  on a very thin body had an exact torque of 0 and was accepted, and the next step overflowed the
  angular velocity (a Jolt assertion, or an angular velocity of exactly 0 in release). The torque
  rule now counts that rounding (`docs/limits.md`, Impulses), as `add_impulse_at_point` does.

## 0.4.0 — 2026-10-04

First release on crates.io.


- Renamed the crates: `joltc-sys` is now `oxijolt-sys` (`crates/oxijolt-sys`, Rust path `oxijolt_sys`)
  and `rolt` is now `oxijolt` (`crates/oxijolt`); the old names belong to the upstream project on crates.io. Before
  the first release the crates were briefly called `joltphysics-sys` and `joltphysics`. The repository is
  now https://github.com/pockerhead/oxijolt. The prebuilt manifest is now `oxijolt-sys-manifest.txt`;
  `JOLTC_LIB_DIR` and the features are unchanged. Existing checkouts run
  `git submodule sync && git submodule update --init` once.
- Project rules (`AGENTS.md`) and lineage (`LINEAGE.md`) for the fork.
- The build script's Android NDK setup and `cross-platform-deterministic` switch were rewritten with
  the same CMake options; no code from jolt-rust remains, and `LICENSE-MIT` now carries this
  project's copyright line.
- The raw layer moved from JoltC to [joltc](https://github.com/amerkoleci/joltc) over Jolt Physics 5.6.0:
  `oxijolt-sys` now exposes joltc's `JPH_*` API. joltc and Jolt are pinned submodules under
  `crates/oxijolt-sys/vendor`, and the native build no longer fetches anything from the network.
- Layout assertions for the FFI types in use, on the C++ side and the Rust side.
- Eight joltc ragdoll and skeleton-mapper functions that cast 4-aligned matrices to 16-aligned ones are
  left out of the bindings.
- Removed the `object-layer-u32` feature: joltc always uses 32-bit object layers.
- `JOLTC_LIB_DIR` links a prebuilt native library and skips CMake; the prefix is validated against a
  manifest. Rust-only changes no longer rerun CMake.
- `oxijolt` is a new safe API on the joltc raw layer: a physics world with collision layers, box and
  sphere shapes and rigid bodies, with a headless `hello_world` example.
- CI on GitHub Actions (Windows MSVC: build, test, clippy, docs, formatting) with a cached native build.
- `PhysicsWorld::step` returns `Result<StepReport, StepError>`. `Err` now means the step was rejected and
  the world did not advance (`StepError::InvalidDeltaTime`); a step that ran but dropped contacts because
  a fixed-size Jolt buffer was full returns `Ok` with the matching `StepReport` flag set
  (`StepReport::is_complete` is false). `StepError::CacheFull` is gone. The rule for the safe API: `Err`
  means nothing happened; anything that happened but was degraded is reported in the `Ok` value.
- Bodies: kinematic bodies, mass override, continuous collision (`MotionQuality::LinearCast`),
  gravity factor, enhanced internal edge removal, forces at a point and torques, `reset_forces`,
  and sleep and active readout. Invalid values are rejected with typed errors before they reach Jolt.
- `WorldSettings::worker_threads` accepts 1 to `WorldSettings::MAX_WORKER_THREADS` (64).
  `PhysicsWorld` is `Send` and `Sync`: many threads may read one world, and separate worlds step in
  parallel.
- `PhysicsWorld::remove_body` wakes the non-static bodies whose bounds overlap the removed one, in
  body-id order, so a stack whose bottom is removed falls.
- Shapes: Y-cylinders and Y-capsules, boxes and cylinders with a chosen convex radius (0 for sharp
  edges), heightfields (`Shape::new_height_field`, `HeightFieldSettings`: holes, block size, bits per
  sample, active-edge threshold) and compounds whose children carry their own pose and user data
  (`Shape::new_compound`, `CompoundChild`). One shape serves many bodies in many worlds.
- Scene queries on `&PhysicsWorld`: `cast_ray`, `cast_shape` (with target distance, start
  penetration and deepest point) and `collide_shape`, all filtered by `QueryFilter` (object layers,
  compound child groups, one excluded body). Hits report the outward normal of the obstacle and the
  compound child that was hit. `optimize_broad_phase` makes queries fast after many single inserts.
- `PhysicsWorld::rebase` moves the whole world into a new frame (floating origin) without waking or
  putting to sleep any body.
- A determinism gate: scenes run in two processes with 1 and 4 worker threads must match bit for
  bit for 1000 ticks, and another creation order must not. CI runs it, and the whole suite, in the
  default, `cross-platform-deterministic` and `double-precision` configurations.
- `debug-renderer` feature: `PhysicsWorld::debug_lines` returns the wireframe of the colliders around
  a point as line data, with layer and group filters and a line cap. Jolt's debug renderer is left
  out of the native libraries without the feature.
- A guide (`docs/guide.md`) with a headless example of a terrain, a chunk compound, an item, queries
  and a rebase, run as a doctest.
- `oxijolt-sys` gains a C extension in joltc's naming (`native/joltc_ext`): a state recorder,
  `JPH_CharacterVirtual_SaveState` and `_RestoreState`, and `JPH_CharacterVirtual_ExtendedUpdate2` and
  `_RefreshContacts2`, which take explicit gravity, filters and temp allocator.
- Virtual characters (Jolt `CharacterVirtual`) owned by the world: `PhysicsWorld::create_character`,
  `update_character` (Jolt `ExtendedUpdate`: move by the velocity, stick to floor, walk stairs),
  `refresh_character_contacts` and `remove_character`, configured by `CharacterSettings` and
  `ExtendedUpdateSettings` with Jolt's defaults; invalid values are rejected with `CharacterError`.
- Characters take up and rotation before every update, so they walk on a spherical planet with
  radial up.
- Character readout: ground state, normal, position, velocity and body, the compound child stood on,
  and the active contacts with their body or character, layer, compound child group and normals.
  Updates and refreshes take the same `QueryFilter` as queries.
- A character can carry a kinematic inner body that bodies and queries see; `remove_body` refuses it
  with `BodyError::OwnedByCharacter`, and it goes with its character.
- Characters can collide with each other (`CharacterSettings::collide_with_characters`).
- Each world numbers its characters from 1 in creation order (`CharacterId`), so contacts between
  characters come in the same order in every run.
- `CharacterRef::save_state` and `CharacterMut::restore_state` continue a character bit for bit in a
  world rebuilt the same way, for chained replays.
- `PhysicsWorld::rebase` moves characters with the bodies.
- The determinism gate also runs the game's reference walker for 600 ticks with 1 and 4 worker
  threads, and a leak gate covers characters.
- The guide has a section on characters, with a character walking on a small planet run as a
  doctest, and explains how a game builds its own autostep for steps with sharp edges.
- Benchmarks against the game's budgets (`cargo bench -p oxijolt --bench budgets`, results in
  `docs/benchmarks.md`); the `character_cost` example moved into this bench.
- `Shape::new_offset_center_of_mass` (Jolt `OffsetCenterOfMassShape`) moves a shape's centre of
  mass without moving its surface, for a vehicle chassis with a low centre of mass.
- Wheeled vehicles (Jolt `VehicleConstraint` with the wheeled controller) on a dynamic chassis
  body: `PhysicsWorld::create_vehicle`, `remove_vehicle`, `vehicle`, `vehicle_mut`, configured by
  `VehicleSettings` with wheels, suspension, engine, automatic transmission, differentials and
  anti-roll bars at Jolt's defaults, and validated with `VehicleError`.
- Vehicle wheels find the ground with a ray, sphere or cylinder cast (`VehicleCollisionTester`)
  on their own object layer, and report contact, suspension length and impulses (`WheelState`).
- Vehicles take driver input, a gravity override for gravity the caller applies, and a pitch and
  roll limit; they report engine rpm and gear. An override whose force would overflow is refused.
- `remove_body` refuses a vehicle's chassis (`BodyError::UsedByVehicle`), and dropping a world
  removes its vehicles and ragdolls first.
- `PhysicsWorld::rebase` rotates each vehicle's gravity override and the up of its ray or sphere
  tester.
- `PhysicsWorld::step`, `update_character` and the kinematic ragdoll drive reject time steps below
  `PhysicsWorld::MIN_DELTA_TIME` (1 µs). Jolt divides by the step; the bound keeps the divisor
  away from subnormal values, where those quotients become infinite.
- `BodySettings::linear_damping` and `angular_damping` (Jolt's default 0.05), and `BodyRef::mass`
  for dynamic bodies, for callers that apply gravity as a force.
- `PhysicsWorld::were_bodies_in_contact` reports whether two bodies touched in the last step.
- Constraint settings for ragdoll joints with Jolt's defaults and validation:
  `SwingTwistConstraintSettings`, `HingeConstraintSettings` with limits and
  `SixDofConstraintSettings` with asymmetric pyramid limits, with `MotorSettings` and
  `SpringSettings`.
- Ragdolls (Jolt `Ragdoll`): `Skeleton`, `RagdollSettings` (and `new_stabilized` for Jolt's
  `Stabilize`), `PhysicsWorld::create_ragdoll` and `remove_ragdoll`. Parts of one ragdoll never
  collide with each other; different ragdolls do. `remove_body` refuses a part
  (`BodyError::OwnedByRagdoll`).
- Ragdolls report their pose, root transform and joint readings, take a pose, are driven to a
  pose with motors (six-DOF joints included) or kinematically, switch motion type and take
  velocities; `SettleDetector` tells when a ragdoll has come to rest.
- Determinism gates for a car driving a route, a fleet of 40 vehicles and a pile of 16 ragdolls
  (1 against 4 worker threads), and leak gates for vehicles and ragdolls.
- `oxijolt-sys` extension: `JPH_VehicleConstraint_AsConstraint`, ragdoll part and joint
  setters that keep every setting (`JPH_RagdollSettings_SetPart`,
  `_SetPartToParentSwingTwist`, `_SetPartToParentHinge`, `_SetPartToParentSixDOF`),
  `JPH_RagdollSettings_CalculateConstraintPriorities`, and swing-twist and hinge motor access.
  Layout assertions now cover the vehicle and constraint settings and every `JPH_PhysicsSettings`
  field.
- The guide has sections on vehicles and ragdolls, with a car on terrain and a ragdoll in a
  second world run as a doctest; the README and the ragdoll docs state how far joints pass their
  limits on impact.
- `PhysicsWorld::step` also rejects time steps above `PhysicsWorld::MAX_DELTA_TIME` (1 s), a guard
  against overflow-scale steps; it is not a Jolt limit.
- A magnitude policy for the safe API: the public `oxijolt::limits` module holds the bounds
  (positions within `MAX_POSITION`, shape extents, velocities, accelerations, masses, friction,
  spring coefficients, ratios, ...), and the setters of those inputs check them before they reach
  Jolt. Some inputs, such as damping and ray directions, are only checked to be finite.
  `docs/limits.md` shows how the bounds follow from Jolt's arithmetic; `docs/coverage.md` says
  which values only tests check and which no bound covers. New checks refuse values that were
  accepted before, among them `WorldSettings::max_contact_constraints` above
  `WorldSettings::MAX_CONTACT_CONSTRAINTS`, body friction above `limits::MAX_FRICTION`, a
  character weight impulse above `limits::MAX_WEIGHT_IMPULSE` and six-DOF translation limits
  beyond `limits::MAX_SHAPE_EXTENT`.
- The `asserts` feature of `oxijolt` (forwarded to `oxijolt-sys/asserts`), and Jolt's assertion
  handler: oxijolt installs it before `JPH_Init`; with `asserts` a failed assertion prints its
  expression, message, file and line and aborts the process, except the physics-update-error
  assertion, whose condition `step` reports in its `StepReport`. CI runs the whole test suite with
  `asserts` as a fifth configuration.
- Behaviour change: the rigid-body inertia floor. A dynamic or kinematic body (also a character's
  inner body and a ragdoll part) whose inertia tensor is not exactly diagonal (a rotated compound
  child, an offset centre of mass) is refused with `InvalidValue` unless its smallest principal
  moment is at least about 4.8e-4 of the tensor's norm, the rule soft bodies use. The earlier rule
  accepted tensors on which Jolt's eigen decomposition asserted (`docs/limits.md`, "Rigid body
  inertia"). In a rotated child this refuses a square needle box
  past about 54 times longer than wide and a capsule or cylinder past about 47 times longer than
  its diameter. Shapes with an exactly diagonal tensor, such as unrotated primitives, are not
  affected.
- World constraints of twelve kinds: `PhysicsWorld::create_constraint` with
  `FixedConstraintSettings`, `PointConstraintSettings`, `DistanceConstraintSettings`,
  `HingeConstraintSettings`, `SliderConstraintSettings`, `ConeConstraintSettings`,
  `SwingTwistConstraintSettings`, `SixDofConstraintSettings`, `GearConstraintSettings`,
  `RackAndPinionConstraintSettings`, `PulleyConstraintSettings` and `PathConstraintSettings`
  (along a `HermitePath`). The typed `ConstraintId<K>` selects the motor, target, limit and
  readout methods of `ConstraintRef` and `ConstraintMut`; `remove_constraint`, `constraint_ids`
  and `constraints_of_body` manage them, and `ConstraintError` reports refusals.
- `remove_body` refuses a body a constraint uses (`BodyError::UsedByConstraint`), and a hinge or
  slider a gear or rack references cannot be removed before the coupling. Gears, racks and
  pulleys need two dynamic bodies. Gear ratios lie within `1..=limits::MAX_GEAR_RATIO` (10),
  because of a defect in Jolt 5.6's gear solver. Every point where a constraint holds a dynamic
  body has a lever-arm ratio of at most `limits::MAX_LEVER_ARM_RATIO`. Creating,
  changing or removing a constraint wakes its bodies. `rebase` recreates pulleys in the new frame.
- Caller job systems: the `JobSystem` trait and `Job` run a world's jobs on the caller's thread
  pool, such as Rayon (`WorldSettings::job_system`, `WorldSettings::MAX_CONCURRENCY`). A job run
  inside `queue_job` is left to the stepping thread, a step finishes even when the pool runs a
  job late or never, and a panic in `queue_job` is resumed from `step` after the world advanced.
  The determinism gates also run with a Rayon pool and an inline job system.
- World state save and restore: `PhysicsWorld::save_state`, `save_state_of` and `restore_state`
  with an opaque `WorldState`, for rollback and replays. A state restores only into the world
  that saved it while its structure is unchanged (`StateError`); configuration Jolt does not save
  stays as it is (`docs/state.md`).
- Soft bodies: `SoftBodySharedSettings` (built and checked by `SoftBodySharedSettingsBuilder`,
  with generated or explicit edge, bend and volume constraints), `SoftBodySettings`,
  `PhysicsWorld::create_soft_body`, vertex readout (`soft_body`) and writes (`soft_body_mut`:
  velocities, pinning, kinematic moves). Body-level velocity, torque and point-force setters,
  constraints and vehicles refuse soft bodies (`BodyError::SoftBody`); vehicle wheels look through
  them. Mass, inertia and pressure rules are checked at creation, and vertex masses and forces
  again in the vertex setters; they refuse some thin or far-from-origin bodies Jolt would simulate
  (`docs/soft-bodies.md`). A pressurised body whose volume shrinks later is not covered
  (`docs/coverage.md`, "Not covered").
- Events: `PhysicsWorld::set_event_settings` and `take_events` record contact, body activation
  and soft body contact events (`EventSettings`, `WorldEvents`), sorted per step into an order
  that does not depend on the thread count. A `ContactListener` (`set_contact_listener`) may change
  each contact's `ContactSettings`; settings that do not fit their contact are not applied and are
  reported in `WorldEvents::rejected_contact_settings` and the new
  `StepReport::rejected_contact_settings`. Panics in callbacks are resumed by `step` or
  `take_events`.
- `PhysicsMaterial` carries the caller's user data; `Shape::new_box_with_material`,
  `new_sphere_with_material`, `new_capsule_with_material`, `new_cylinder_with_material` and
  `new_height_field_with_materials` make shapes of materials, and contacts report each side's
  material.
- `oxijolt-sys` extension: `JPH_PhysicsSystem_SaveState` and `_RestoreState` with a body
  selection, path, pulley and rack-and-pinion constraints and motor accessors, soft body
  functions, vehicle collision testers that skip soft bodies, materials with user data
  (`JPH_PhysicsMaterial_Create2`, `JPH_ConvexShapeSettings_SetMaterial`,
  `JPH_HeightFieldShapeSettings_Create2`), a soft body contact listener and sub-shape pair
  getters.
- Builds without LLVM: the raw bindings are committed under `crates/oxijolt-sys/src/bindings/`,
  one file per ABI family (`msvc`, `gnu`) and configuration, and `cargo xtask bindings`
  regenerates them. The new `bindgen` feature (forwarded by `oxijolt`) generates them at build time
  instead. The supported targets are the 64-bit ones in `build/targets.rs`; 32-bit targets,
  including the Android armv7 and x86 ones accepted before, are refused when the build starts.
  Raw API changes: the bindings no longer carry joltc's header comments, and on `gnu` targets
  joltc's enum typedefs are `c_uint`.
- CI runs on Linux (GCC) as well as Windows (MSVC), and a `Committed bindings` job checks the
  bindings against LLVM 18. A release workflow builds prebuilt native libraries for
  `JOLTC_LIB_DIR` (x86_64 Windows MSVC and Linux GNU, eight feature subsets each) and attaches
  them to a tagged release. A prebuilt prefix's headers are compared without regard to line
  endings.
- A readability pass with no public API change: large modules are split by concern, the limits
  derivations and the per-input audit table moved to `docs/limits.md` and `docs/coverage.md`
  (both new), and 18 validation error messages are shorter; callers that compare exact
  `InvalidValue` texts see the new ones.
- Documentation: the README is a front page; the per-feature test table moved to
  `docs/coverage.md`; new guides for constraints, soft bodies, events, state save and restore,
  caller job systems, determinism and building (`docs/`), whose examples run as doctests.
- `CharacterRef::penetration_recovery_speed` and `CharacterMut::set_penetration_recovery_speed`
  read and set a character's penetration recovery speed while it runs (finite, `0..=1`).
- A study of which character laws CharacterVirtual's built-in mechanisms carry
  ([docs/character-study.md](docs/character-study.md)): law scenarios and pinned results in
  `tests/character_study.rs`, and a bench of every configuration's cost per move
  (`cargo bench -p oxijolt --bench character_study`).
- `PhysicsWorld::active_body_poses` and `active_body_poses_into` return a `BodyPose` (id, position,
  rotation) for every awake body, sorted by `BodyId` and read under one multi-body read lock.
- `oxijolt::error::Error` wraps every area error with a `From` impl, and
  `oxijolt::error::Result<T>` is `Result<T, oxijolt::error::Error>`, so `?` works across areas.
  Both live only in the now public `error` module, so `use oxijolt::*` gains no `Error` or
  `Result`. Existing signatures are unchanged.
- `oxijolt::prelude` holds the commonly used types without the math types, `Error` or `Result`, so
  it can be glob-imported next to `bevy::prelude::*`; the math types are in `oxijolt::prelude::math`.
