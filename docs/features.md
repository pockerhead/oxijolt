# Features

What the safe API, `oxijolt`, covers today. The [roadmap](../README.md#roadmap) in the README lists
what is planned next.

- Rigid bodies: static, kinematic and dynamic; box, sphere, cylinder, capsule, tapered capsule and
  cylinder (cones too), convex hull, triangle mesh (with a material per triangle and a report of
  the triangles too thin to collide with, for static and kinematic bodies), heightfield, plane
  (static ground cut to a square), compound and scaled shapes, compound edits at run time
  (`MutableCompound`); physics materials with user data;
  forces, sleeping, continuous collision; impulses, kinematic moves, waking and sleeping on demand,
  sensors, per-body user data, locked axes (allowed degrees of freedom), shape and motion type
  changes, contact-cache invalidation, buoyancy and drag in a fluid, and the list of every body
  of a world (`PhysicsWorld::body_ids`) ([guide](bodies.md)).
- Body properties changed on a live body: friction, restitution, gravity factor, linear and
  angular damping, motion quality, object layer, collision group and the sensor flag, each with
  its getter and the checks of its creation setting
  ([guide](bodies.md#changing-body-properties)).
- Collision groups: tables of sub groups that say which pairs collide, given to rigid and soft
  bodies at creation or later, for ragdolls, chains and vehicles that ignore their driver
  ([guide](bodies.md#collision-groups)).
- Scene queries: ray casts (closest hit or every hit, sorted), shape casts, collide-shape and
  point queries (in the world and on a shape), filtered by layer, compound child group, an
  excluded body and a closure that keeps or rejects each body.
- A character controller (Jolt's `CharacterVirtual`) with stair walking, floor sticking and an up
  direction that can change every update, for walking on a planet, and a character contact
  listener for moving platforms, ignored contacts, push settings and added, persisted and removed
  contacts ([guide](events.md#character-contacts-charactercontactlistener)); a humanoid preset,
  `CharacterSettings::humanoid(height, radius)`.
- Wheeled vehicles with suspension, engine, automatic transmission, differentials and anti-roll
  bars; tracked vehicles (tanks) with two tracks; motorcycles with a lean controller
  ([guide](vehicles.md)); presets for a car and a motorcycle (`WheeledVehicleSettings::car`,
  `MotorcycleSettings::bike`), and each wheel's pose in world space for drawing
  (`VehicleRef::wheel_world_transform`). Engine, transmission, differentials and tracks change
  while a vehicle drives, and a per-vehicle callback sets each tire's grip
  ([guide](vehicles.md#tuning-a-vehicle-while-it-drives)).
- A body's mass and inertia given directly instead of computed from its shape
  (`BodySettings::mass_and_inertia`) ([guide](bodies.md#mass-and-inertia)).
- The world-space inverse inertia a dynamic body has, rotation and locked axes included
  (`BodyRef::inverse_inertia`) ([guide](bodies.md#mass-and-inertia)).
- A shape's mass and inertia tensor about its centre of mass (`Shape::mass_properties`), in the
  form `BodySettings::mass_and_inertia` takes ([guide](bodies.md#mass-and-inertia)).
- Centre of mass readouts: a shape's in its own space (`Shape::center_of_mass`) and a body's in
  world space (`BodyRef::center_of_mass_position`) ([guide](bodies.md#mass-and-inertia)).
- Shape changes of static bodies that constraints use: the constraints keep their world
  attachment points ([guide](constraints.md#bodies)).
- A shape change checked without making it (`BodyMut::check_shape`), with the error
  `set_shape` would return ([guide](bodies.md#shape-and-motion-type-changes)).
- Shape changes that wake only the bodies near caller-given regions, for a large compound that
  changes in a small part (`BodyMut::set_shape_waking_in`)
  ([guide](bodies.md#shape-and-motion-type-changes)).
- Ragdolls from a skeleton, posed, motor-driven or kinematic, with settle detection, and a skeleton
  mapper to and from a detailed animation skeleton ([guide](guide.md#ragdolls)).
- Twelve kinds of constraints with motors, springs and limits: fixed, point, distance, hinge,
  slider, cone, swing-twist, six-DOF, gear, rack and pinion, pulley and path
  ([guide](constraints.md)), with impulse readouts for breakable constraints
  ([guide](constraints.md#breakable-constraints), [scene](playground.md#-breakable-bonds-bonds)).
- Per-constraint solver settings for every kind: velocity and position step overrides for the
  constraint's island and a solving priority, at creation and at run time
  ([guide](constraints.md#solver-overrides)).
- The hinges and slider a gear or rack and pinion references, read from its settings or the
  created constraint, so couplings can be removed before them
  ([guide](constraints.md#references-between-constraints)).
- Soft bodies: cloth, pressurised and volume-preserving bodies, with vertex readout and pinning
  ([guide](soft-bodies.md)).
- Soft body skinning: cloth such as a cape skinned to an animated skeleton or a ragdoll's pose,
  with max distances, back stops and hard skinning ([guide](soft-bodies.md#skinning)).
- Contact, activation and soft body contact events in an order that does not depend on the
  thread count, and a contact listener that changes friction, restitution, mass scales or surface
  velocity per contact, or rejects contacts before they form (one-way platforms), and Jolt's
  estimate of each new contact's impulses for impact sounds and damage ([guide](events.md)).
- Saving and restoring a world's state for rollback and replays, into reused buffers without
  allocating, of every body, the bodies that can move or chosen ones, and restoring chosen
  bodies only ([guide](state.md)).
- Shape cooking: built shapes with their children and materials saved to bytes and restored,
  checked against the build that wrote them ([guide](shape-cooking.md)); meshes and hulls tested
  on real models from open sources ([report](real-meshes.md)).
- A floating origin (`PhysicsWorld::rebase`) and optional `f64` world positions.
- Jolt's jobs on Jolt's thread pool, on Jolt's single-threaded job system
  (`WorldSettings::single_threaded`) or on your own, such as Rayon ([guide](job-system.md));
  `PhysicsWorld::job_system_kind` and `max_concurrency` read back which one a world got,
  `WorldSettings::job_system_kind` which one settings choose.
- A [comparison with Rapier and Avian](comparison.md) on Rapier's stress scenes: speed at 1 to 16
  threads, quality, determinism and features, with the scripts and raw results.
- A world reads back the limits and collision layers it was created with
  (`PhysicsWorld::max_bodies`, `max_body_pairs`, `max_contact_constraints`,
  `temp_allocator_size`, `layers`).
- Solver velocity and position steps chosen per world (`WorldSettings::velocity_steps`,
  `position_steps`), within Jolt's limits ([limits](limits.md#solver-step-counts)).
- Penetration slop and speculative contact distance chosen per world
  (`WorldSettings::penetration_slop`, `speculative_contact_distance`)
  ([limits](limits.md#contact-distances)).
- Several collision steps in one update (`PhysicsWorld::step_with_collision_steps`, up to
  `MAX_COLLISION_STEPS`) to sub-step a fast scene ([limits](limits.md#collision-steps)).
- Debug wireframes as line data (feature `debug-renderer`); nothing is drawn.
- Optional `glam032`, `glam033` (glam 0.33, Bevy 0.20) and `mint` features with exact conversions
  for `Vec3`, `RVec3` and `Quat`.
- The poses of every awake body in one call, in a deterministic order
  (`PhysicsWorld::active_body_poses`).
- `oxijolt::prelude` for the common types, and `oxijolt::error::{Error, Result}` that wraps the
  error of every area.
- Prebuilt native libraries on x86_64 Windows (MSVC), x86_64 Linux (GNU), macOS on Apple
  silicon and `wasm32-unknown-unknown` (with wasi-sdk's runtime inside, so a wasm build needs
  neither wasi-sdk nor CMake): the build script downloads the release archive that matches the build, checks it
  against a sha256 packaged in the crate and builds from source when none fits (feature
  `prebuilt`, on by default;
  [building](building.md#prebuilt-native-libraries)).
- The raw layer, `oxijolt-sys`, builds for `wasm32-unknown-unknown` with Rust 1.91 or newer:
  its `JPH_Init` runs the module's C++ static constructors once, and the module imports no WASI
  functions ([building](building.md#webassembly)). The safe API builds for it too, single-threaded:
  worlds default to Jolt's single-threaded job system there, `WorldSettings::worker_threads` is
  refused, and a panic in a callback traps the module ([job systems](job-system.md)). CI checks
  that a wasm module, in Node and in headless Chrome, steps a test scene to the same bits as a
  native Linux build, both with `cross-platform-deterministic`
  ([determinism](determinism.md#how-it-is-checked)). A browser example, `examples/web`, steps a
  pile of balls in such a module and draws it on a canvas with plain JavaScript
  ([building](building.md#browser-example)).
- One set of API rules for names, constructors, settings, errors and derives
  ([api-guidelines.md](api-guidelines.md)), and Rust 1.88 as the minimum version, checked in CI.

The [playground](playground.md) shows them in thirteen small scenes with a window, and runs the same
scenes headless. In its breakable wall, pieces that land hard break again where they hit
([scene](playground.md#8-breakable-wall-destruction)).

The raw layer, `oxijolt-sys`, has the joltc functions for all of them.
[coverage.md](coverage.md) lists every bound feature with the tests that check it, and
the rest of what is not bound yet.
