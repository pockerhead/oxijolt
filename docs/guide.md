# Guide: a headless game world

This guide builds the scene `oxijolt` was written for: terrain, a building made of several
parts with their own collision groups, a dropped item, the queries a game runs every frame and a
floating origin, then a character walking on a small planet, then a car on terrain and a ragdoll
in a second world. Everything runs headless; nothing is drawn. The three complete programs run as
tests (`cargo test -p oxijolt --doc`).

Other topics have guides of their own: [constraints](constraints.md), [soft bodies](soft-bodies.md),
[tanks and motorcycles](vehicles.md),
[events and contact listeners](events.md), [saving and restoring a world](state.md),
[running jobs on your own thread pool](job-system.md), [determinism](determinism.md) and
[building](building.md).

Units are metres, seconds, kilograms and radians. Jolt is right-handed with Y up.

## Layers and groups

Every body lives in one **object layer**, and each object layer is kept in one **broad-phase
layer** (a tree of the broad phase; static and moving bodies usually get one each). A
`CollisionLayers` table says which pairs of object layers collide. A group whose members are
whole bodies (terrain, chunks, items, actors) gets its own object layer.

Some groups share one body. A chunk is one static body whose shape is a compound of many parts,
and each part has a group: structures you can stand on, features (foliage, canopies) you walk
through. Store the group as the compound child's `user_data`. Queries read it back from hits
(`compound_child`) and filter on it with `QueryFilter::child_groups`, a bit mask over the user
data values 0 to 31.

`QueryFilter` has four parts that combine: `object_layers` selects whole bodies by layer,
`child_groups` selects compound children by group, `exclude_body` skips one body (the querying
actor's own), and `body_filter` asks a closure about each body, for sets a layer cannot express
(the bodies a projectile already went through). An unset part accepts everything. The closure runs
on the querying thread for every candidate body, before its shape is tested, so a rejected body
does not hide the ones behind it; it may be asked about a body more than once and in any order,
so its answer should depend only on the id.

Layers decide which bodies the solver collides by class. For pairs within a class, such as a
vehicle that must not collide with its driver or a chain whose neighbouring links overlap, give the
bodies a `CollisionGroup` at creation; queries and characters ignore those groups
([bodies.md](bodies.md#collision-groups)). For per-contact decisions (a one-way platform, a
projectile that passes its shooter), a `ContactListener` validates contacts
([events.md](events.md#validating-contacts)).

## Shapes

- `Shape::new_box`, `new_sphere`, `new_cylinder` and `new_capsule` (cylinders and capsules along
  local Y). Boxes and cylinders have Jolt's convex radius of 0.05 m, which rounds their edges for
  contacts and shape casts; `new_box_with_convex_radius(.., 0.0)` gives sharp edges. Rays always
  see the sharp shape.
- `Shape::new_height_field` takes `n * n` samples in Jolt's row-major order: sample `(x, z)` is
  `samples[z * n + x]`. A column-major source must be transposed. `f32::MAX` is a hole. With the
  default block size 2, `n = 33` is padded to 34 and covers exactly the 32 x 32 cells given.
  Heightfields are for static bodies only.
- `Shape::new_plane(normal, constant, half_extent)` is static ground: everything behind
  `normal · p + constant = 0` is solid, within a square of `2 * half_extent` around the plane and
  `half_extent` deep. Nothing collides beyond that square. Only static bodies take a plane.
- `Shape::new_compound` places children with their own pose and user data. Child order is part of
  the shape.
- `MutableCompound` edits a compound's children at run time and publishes new shapes, which
  `BodyMut::set_shape` installs ([bodies.md](bodies.md#compound-edits)).
- A dynamic or kinematic body needs an inertia tensor Jolt can decompose. When the tensor is not
  exactly diagonal (a rotated compound child, an offset centre of mass), a slender part is refused
  past about 54 times longer than wide for a square box and about 47 times longer than its
  diameter for a capsule or cylinder ([limits.md](limits.md#rigid-body-inertia)). A shape with an
  exactly diagonal tensor, such as an unrotated primitive, is not affected. An inertia given with
  `BodySettings::mass_and_inertia` has rules of its own ([bodies](bodies.md#mass-and-inertia)).

A `Shape` holds one Jolt reference and every body holds its own, so a shape can be dropped after
creating bodies and shared between bodies and worlds.

## Stepping

`PhysicsWorld::step(dt)` runs one collision step. `Err` means the step was rejected (a `dt` that is
not finite or outside `MIN_DELTA_TIME` (1 µs) to `MAX_DELTA_TIME` (1 s)) and nothing happened. `Ok(report)` means the world advanced; `report.is_complete()` is false when
Jolt dropped contacts because a fixed-size buffer was full, and the flag names the `WorldSettings`
limit to raise. The world stays valid either way.

`PhysicsWorld::step_with_collision_steps(dt, n)` advances by `dt` in `n` collision steps of `dt / n`
inside one Jolt update, with `n` from 1 to `PhysicsWorld::MAX_COLLISION_STEPS` (4); `step(dt)` is
the same call with `n = 1`. A count outside that range is `StepError::InvalidValue`, a collision
step shorter than `MIN_DELTA_TIME` is `StepError::InvalidDeltaTime`. The call gives one report and
one event batch. It is not the same as calling `step(dt / n)` `n` times: separate calls clear
forces, check sleep and batch events at each call, and let the game change inputs in between
([limits](limits.md#collision-steps)).

```rust
use oxijolt::prelude::*;

fn main() -> oxijolt::error::Result<()> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    // 1/30 s in two collision steps of 1/60 s.
    let report = world.step_with_collision_steps(1.0 / 30.0, 2)?;
    assert!(report.is_complete());
    assert!(world.step_with_collision_steps(1.0 / 30.0, 0).is_err());
    Ok(())
}
```

Gravity is per world (`WorldSettings::gravity`, `set_gravity`). A game with radial gravity sets it
to zero and adds a force to each body before the step (`BodyMut::add_force`).

After a step, a game copies body poses into its own entities.
`PhysicsWorld::active_body_poses_into` fills a buffer with the id, position and rotation of every
body that is awake when it is called, in ascending `BodyId` order, under one lock; sleeping and
static bodies are absent. The poses are copies, so the world is free again once the call returns.

The awake set is not the set of bodies that moved. A body that falls asleep at the end of a step
still moved in that step and is absent from the readout after it; with
`EventSettings::body_activation` on, its `ActivationEvent::Deactivated` from `take_events` says to
read it once more with `world.body(id)`. A pose written to a sleeping body
(`Activation::DontActivate`), `rebase` and `restore_state` move bodies that stay asleep, and
`restore_state` changes the awake set without events: after those, sync every body the game
tracks.

```rust
use oxijolt::prelude::math::*;
use oxijolt::prelude::*;

fn main() -> oxijolt::error::Result<()> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let ball = Shape::new_sphere(0.5)?;
    let mut balls = Vec::new();
    for x in 0..3 {
        let at = RVec3::new(2.0 * x as Real, 5.0, 0.0);
        balls.push(world.create_body(&ball, &BodySettings::new_dynamic().position(at))?);
    }
    let mut poses = Vec::new();
    for _ in 0..10 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
        world.active_body_poses_into(&mut poses);
        for pose in &poses {
            // Here a game writes `pose.position` and `pose.rotation` to the entity of `pose.id`.
            assert!(balls.contains(&pose.id));
        }
    }
    assert_eq!(poses.len(), 3);
    Ok(())
}
```

With Bevy, glob-import `oxijolt::prelude::*` next to `bevy::prelude::*`: the prelude leaves `Vec3`,
`Quat` and `Result` to Bevy. The rustdoc of `oxijolt::prelude` has a transform-sync system.

## Queries

Queries take `&PhysicsWorld`, so many threads may run them while nobody steps the world. They see
created, moved and removed bodies at once, without a step. Every normal they report is the
outward surface normal of the obstacle: a floor below gives a normal pointing up.

- `cast_ray`: the closest hit along a ray. Convex shapes are solid (a ray that starts inside hits at
  fraction 0) and triangles are hit from both sides.
- `cast_ray_all`: every hit along a ray under the same rules, one per convex shape and one per
  triangle crossed, sorted by fraction, then body id, then sub-shape id. The first is `cast_ray`'s
  hit.
- `cast_shape`: the first obstacle a moving shape hits, with the distance it can move, the contact
  point and the obstacle's normal. `ShapeCast::target_distance` (spheres and capsules) stops the
  cast short of the obstacle.
- `collide_shape`: every obstacle a shape at a pose overlaps, with penetration depth and normal.
  The hits come in no particular order; sort them when order matters.
- `collide_point`: every body whose shape contains a point, sorted by body and sub-shape id.
  "Contains" is Jolt's rule per shape: solid convex shapes, meshes by the parity of the
  triangles above the point (the inside of a closed mesh, except where the upward ray passes
  through an edge or a vertex: the centre of a cube mesh is reported outside), never a
  heightfield, and strictly behind a plane.
  `Shape::collide_point` asks one shape, in its own frame.

## Floating origin

`PhysicsWorld::rebase(bodies, rotation, translation)` moves the whole world into a new frame with
one rigid change of coordinates: positions, rotations, velocities and gravity. No body wakes up or
falls asleep, and resting bodies keep their contacts, because Jolt caches contacts relative to the
bodies. The list must name every body of the world once, in a stable order of the caller's choice.
Rebase between steps, before adding the tick's forces: forces added since the last step are not
rotated. Queries see the new poses at once; `optimize_broad_phase` afterwards is optional and only
makes queries faster until the next step.

Awake bodies restart Jolt's sleep timer, as for every pose change, so they may fall asleep later
than without the rebase. New positions are only checked to be finite, not to lie within
`limits::MAX_POSITION`: a body the simulation carried out of the frame does not block a rebase.
A rotating rebase refuses a world with a body created with fewer than six degrees of freedom
(`BodyError::RestrictedDofs`), because Jolt locks world axes and a rotation would turn the body out
of them; a translation alone is accepted.

The rest of the world moves along:
- **Characters** move in id order after the bodies: position and rotation as for bodies, up and
  velocity as vectors. Their cached contacts and ground stay in the old frame; after a rotating
  rebase call `refresh_character_contacts` for every character before its next update.
- **Vehicles** move with their chassis. A rotation also rotates each vehicle's gravity override,
  the up of a ray or sphere tester and each motorcycle's target lean. The wheel contacts stay in the
  old frame until the next step.
- **Ragdolls, soft bodies and constraints** are stored relative to their bodies and need no change.
  A soft body's vertices turn with it, which leaves its body rotation non-identity.
- **Pulleys** are the exception: their fixed points are world points, so a rebase recreates each
  pulley in the new frame, in id order after the vehicles. A translation alone recreates pulleys
  too. A pulley keeps its id, enabled state, ratio, lengths, priority and step overrides, and drops
  its warm start. It also drops its cached rope directions: Jolt starts them at -Y and keeps the
  old one for a rope segment of zero length, so only such a segment notices. Jolt removes the old
  pulley by moving its last constraint into the freed place and appends the new pulley at the end,
  so the order of other constraints changes too. The same calls give the same order.
  A taut rope's length then rounds differently in `f32`; a step in which it comes out just under
  the maximum length leaves the rope slack, and a hanging pair drifts by a few millimetres from
  where it would be without the rebase before the drift decays; 0.0023 m was measured after a turn
  of 0.4 rad. In double precision the slack step can come later than the first step after the
  rebase.

A rebase that moves anything makes earlier `WorldState`s unrestorable ([state.md](state.md)).

## Determinism

The requirement is that on one machine the same calls in the same order give bit-identical
results whatever the job system. The tests check it with 1 and 4 worker threads and with caller
job systems, on the scenes [determinism.md](determinism.md) lists. The order in which bodies are
created and removed is part of the state: it decides the `BodyId`s. Keep hash-map iteration order,
time and thread identity out of the calls that drive the world, and sort `collide_shape` hits
before acting on them. Equal results across platforms and compilers need the
`cross-platform-deterministic` feature.

## Debug lines

With the `debug-renderer` feature, `PhysicsWorld::debug_lines` returns a `DebugLines` buffer with the
wireframe of the colliders around a point, filtered like a query; `debug_lines_into` refills one
buffer every frame. It produces line data only and
draws nothing. It has no level of detail, so cap the output with `DebugLineSettings::max_lines`.

## Characters

A character is Jolt's `CharacterVirtual`: a convex shape, usually a capsule, that is not a body.
`PhysicsWorld::create_character` adds one; each tick the game sets its velocity and calls
`update_character`, which moves it by collision queries, slides it along what it hits and then
applies `ExtendedUpdateSettings`: stick to the floor and Jolt's walk stairs. Gravity is an argument
of the update and is not added to the velocity; the caller feeds the vertical speed itself.

- **Shape and pose.** The character position is where the shape's offset starts. With a capsule of
  half height `h` and radius `r`, `shape_offset(0, h + r, 0)` puts the capsule's bottom at the
  position, and Jolt adds `character_padding` (0.02 m) along up, so a character at rest stands that
  far above the ground. `CharacterSettings::humanoid(height, radius)` builds such a capsule and keeps
  it in the settings, so no `Shape` has to outlive them.
- **Up.** Up and rotation can change before every update (`CharacterMut::set_up`, `set_rotation`),
  so on a planet up is radial. The `ExtendedUpdateSettings` vectors are in world space and default
  to +Y; with another up, pass them along it.
- **Ground.** `ground_state` is `OnGround`, `OnSteepGround` (steeper than `max_slope_angle`),
  `NotSupported` or `InAir`. Below about 0.81° Jolt turns the slope limit off, so with
  `max_slope_angle(0.0)` no ground is too steep. A new character reports `InAir` until its first update or
  `refresh_character_contacts`; refresh one that starts on the ground, or stick to floor (which
  needs support before the update) does nothing on the first tick.
- **Contacts.** `active_contacts` lists what the last update touched. Normals point toward the
  character. `contact_compound_child` gives the touched compound child's `user_data` (the game's
  collision group), `contact_object_layer` the touched body's layer.
- **Filters.** `update_character` and `refresh_character_contacts` take the same `QueryFilter` as
  queries. A game that keeps a kinematic capsule body per actor excludes the character's own with
  `exclude_body`; a character's optional inner body (`CharacterSettings::inner_body`) is never hit
  by its own character.
- **Replays.** `save_state` returns the character's persistent state (pose, velocity, up, ground and
  the contacts with collision); `restore_state` into a world rebuilt the same way continues the run
  bit for bit. Values the caller carries between ticks, such as its vertical speed, belong in the
  replay too. Every world numbers its characters from 1 in creation order, so a rebuilt world gives
  the same ids.
- **Rebase.** `PhysicsWorld::rebase` moves the characters with the bodies; after a rotating rebase,
  call `refresh_character_contacts` for each character before its next update.
- **Contact callbacks.** `set_character_contact_listener` installs a `CharacterContactListener` for
  moving platforms and conveyors (`adjust_body_velocity`, read back as `ground_velocity`), contacts a
  character ignores, whether a contact pushes the character or the character pushes it, and added,
  persisted and removed contacts ([events.md](events.md#character-contacts-charactercontactlistener)).

### Sharp steps and the game's own autostep

Jolt's walk stairs and its steep-slope test judge an obstacle by the surface normal at the contact.
On a box with sharp edges (`new_box_with_convex_radius(.., 0.0)`) a capsule pressing on a face
touches it at the top edge. There Jolt picks the top face's normal or the side's by float
rounding of the contact point, so walk stairs on sharp steps is unreliable: the same settings
climb a 0.45 m step and refuse a 0.40 m one, and the outcome changes from scene to scene.
The capsule also creeps onto low sharp edges by itself (up to about 0.37 m in the walker tests).
No `CharacterSettings` value makes this dependable. On Jolt's default rounded boxes walk stairs
works (a 0.25 m step with a 0.4 m step-up is climbed), but the height it climbs is about
step-up + padding + `r (1 - cos max_slope_angle)`, not the step-up itself, and the step's rounding
changes it too, so measure it for your capsule.

A game with sharp structures turns walk stairs off (`walk_stairs_step_up(Vec3::ZERO)`) and steps
in its own code after the update, with shape casts, which report the geometry's own normal: when a
grounded walker was held back by a steep contact, cast the capsule up by the step height (head
room), forward by the free room the step needs, and down onto the step; if the top is walkable and
not a dynamic body, set the character's position there and refresh its contacts. The repository's
reference controller for VOIDRUN does exactly this in `crates/oxijolt/tests/common/walker.rs`
(a 0.45 m step with 0.5 m of free room on a planet of radius 99, with underground recovery, the
vertical speed rules, stick to floor as the floor snap and a chained replay). It is test support,
not part of the API: copy and adapt it.

## The whole example

```rust
use oxijolt::prelude::math::*;
use oxijolt::prelude::*;

/// The game's collision groups, stored as compound child user data.
const STRUCTURE: u32 = 1;
const FEATURE: u32 = 2;

fn main() -> oxijolt::error::Result<()> {
    // Layers: static terrain and chunks, moving items. Items collide with everything.
    let mut layers = CollisionLayers::new(2);
    let fixed = BroadPhaseLayer::new(0);
    let moving = BroadPhaseLayer::new(1);
    let terrain = layers.add_object_layer(fixed);
    let chunk = layers.add_object_layer(fixed);
    let item = layers.add_object_layer(moving);
    layers
        .enable_collision(item, terrain)
        .enable_collision(item, chunk)
        .enable_collision(item, item);
    let mut world = PhysicsWorld::new(
        WorldSettings::default()
            .layers(layers)
            .worker_threads(2),
    )?;

    // Terrain: a flat 33 x 33 heightfield at y = 0, one sample per metre, x and z in [-16, 16].
    let ground = Shape::new_height_field(
        33,
        &[0.0; 33 * 33],
        &HeightFieldSettings::default().offset(Vec3::new(-16.0, 0.0, -16.0)),
    )?;
    let terrain_body =
        world.create_body(&ground, &BodySettings::new_static().object_layer(terrain))?;

    // A chunk at (5, 0, 5): a porch you can stand on (top at y = 1) under a canopy you walk
    // through (top at y = 3.5). One static body, two groups.
    let porch = Shape::new_box(Vec3::new(2.0, 0.5, 2.0))?;
    let canopy = Shape::new_cylinder(0.5, 2.0)?;
    let house = Shape::new_compound(&[
        CompoundChild {
            shape: &porch,
            position: Vec3::new(0.0, 0.5, 0.0),
            rotation: Quat::IDENTITY,
            user_data: STRUCTURE,
        },
        CompoundChild {
            shape: &canopy,
            position: Vec3::new(0.0, 3.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: FEATURE,
        },
    ])?;
    let chunk_body = world.create_body(
        &house,
        &BodySettings::new_static()
            .position(RVec3::new(5.0, 0.0, 5.0))
            .object_layer(chunk),
    )?;

    // An item dropped from 2 m: a 0.5 m crate of 2 kg with continuous collision detection.
    let crate_shape = Shape::new_box(Vec3::new(0.25, 0.25, 0.25))?;
    let item_body = world.create_body(
        &crate_shape,
        &BodySettings::new_dynamic()
            .position(RVec3::new(-5.0, 2.0, 0.0))
            .mass(2.0)
            .friction(0.5)
            .motion_quality(MotionQuality::LinearCast)
            .object_layer(item),
    )?;

    // Two seconds at 60 Hz.
    for _ in 0..120 {
        let report = world.step(1.0 / 60.0)?;
        assert!(report.is_complete(), "raise a WorldSettings limit: {report:?}");
    }
    let resting = world.body(item_body)?.position();
    assert!((resting.y - 0.25).abs() < 0.03, "the crate rests on the terrain");

    // Ground under the house: terrain and chunks, structures only, so the canopy is skipped.
    let ground_layers = [terrain, chunk];
    let ground_filter = QueryFilter::new()
        .object_layers(&ground_layers)
        .child_groups(1 << STRUCTURE);
    let down = RayCast::new(RVec3::new(5.0, 10.0, 5.0), Vec3::new(0.0, -20.0, 0.0));
    let hit = world.cast_ray(&down, &ground_filter)?.expect("the porch");
    assert_eq!(hit.body, chunk_body);
    assert_eq!(hit.compound_child.map(|child| child.user_data), Some(STRUCTURE));
    assert!((down.point_at(hit.fraction).y - 1.0).abs() < 1e-3);
    assert!(hit.normal.y > 0.99, "a floor's normal points up");

    // The same ray without a group filter stops on the canopy.
    let hit = world.cast_ray(&down, &QueryFilter::new())?.expect("the canopy");
    assert_eq!(hit.compound_child.map(|child| child.user_data), Some(FEATURE));

    // An actor capsule (feet 0.8 m below its origin) cast 5 m down onto the terrain.
    let actor = Shape::new_capsule(0.5, 0.3)?;
    let fall = ShapeCast::new(
        &actor,
        RVec3::new(-10.0, 3.0, -10.0),
        Quat::IDENTITY,
        Vec3::new(0.0, -5.0, 0.0),
    );
    let hit = world.cast_shape(&fall, &QueryFilter::new())?.expect("the terrain");
    assert_eq!(hit.body, terrain_body);
    assert!((hit.distance - 2.2).abs() < 1e-3);
    assert!(hit.normal.y > 0.99);

    // A ball sunk 0.3 m into the porch, skipping the item: one overlap, pushed out upwards.
    let ball = Shape::new_sphere(0.5)?;
    let probe = CollideShape::new(&ball, RVec3::new(5.0, 1.2, 5.0), Quat::IDENTITY);
    let mut hits = world.collide_shape(&probe, &QueryFilter::new().exclude_body(item_body))?;
    hits.sort_by_key(|hit| (hit.body.to_raw(), hit.sub_shape_id.to_raw()));
    assert_eq!(hits.len(), 1);
    assert!((hits[0].penetration_depth - 0.3).abs() < 1e-3);
    assert!(hits[0].normal.y > 0.99);

    // Floating origin: move everything so the house sits at the origin. Every body, in a
    // stable order (here: creation order).
    world.rebase(
        &[terrain_body, chunk_body, item_body],
        Quat::IDENTITY,
        RVec3::new(-5.0, 0.0, -5.0),
    )?;
    let moved = world.body(item_body)?.position();
    assert!((moved.x - (resting.x - 5.0)).abs() < 1e-4);
    assert!((moved.z - (resting.z - 5.0)).abs() < 1e-4);
    let down = RayCast::new(RVec3::new(0.0, 10.0, 0.0), Vec3::new(0.0, -20.0, 0.0));
    let hit = world.cast_ray(&down, &ground_filter)?.expect("the porch");
    assert_eq!(hit.body, chunk_body);

    // The world keeps stepping in the new frame.
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    Ok(())
}
```

## A walking character

A character walks on a planet of radius 50 m, a static sphere at the origin, so up is radial and
changes every tick. Gravity is applied by the caller. The helpers that turn radial up into a
rotation and a walking direction work at every point of the sphere, which the example checks by
starting a short walk at both poles and on the equator. A chunk compound stands in its path: a
feature (a bush) the character walks through and a structure (a wall) that stops it, told apart by
the filter's group mask. Halfway through the walk the character's state is saved, and a rebuilt
world continues from it bit for bit.

```rust
use oxijolt::*;

/// The game's collision groups, stored as compound child user data.
const STRUCTURE: u32 = 1;
const FEATURE: u32 = 2;
const PLANET_RADIUS: f32 = 50.0;
const GRAVITY: f32 = 9.8;
const DT: f32 = 1.0 / 60.0;

fn scale(v: Vec3, s: f32) -> Vec3 {
    Vec3::new(v.x * s, v.y * s, v.z * s)
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x + b.x, a.y + b.y, a.z + b.z)
}

fn dot(a: Vec3, b: Vec3) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn normalize(v: Vec3) -> Vec3 {
    scale(v, 1.0 / dot(v, v).sqrt())
}

/// Radial up at `p`, away from the planet's centre.
fn up_at(p: RVec3) -> Vec3 {
    normalize(Vec3::new(p.x as f32, p.y as f32, p.z as f32))
}

/// A rotation that turns +Y into the unit vector `up`. On the upper half it is the shortest one;
/// on the lower half it first turns +Y half a turn about X to -Y and then takes the shortest way,
/// so no division comes near zero.
fn rotation_to(up: Vec3) -> Quat {
    let q = if up.y >= 0.0 {
        Quat::from_xyzw(up.z, 0.0, -up.x, 1.0 + up.y)
    } else {
        Quat::from_xyzw(1.0 - up.y, up.x, 0.0, up.z)
    };
    let length = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
    Quat::from_xyzw(q.x / length, q.y / length, q.z / length, q.w / length)
}

/// The walking direction at unit `up`: east (+X along the ground), or +Z along the ground near
/// the two points where up is ±X and east does not exist.
fn forward_at(up: Vec3) -> Vec3 {
    let reference = if up.x.abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };
    normalize(add(reference, scale(up, -dot(reference, up))))
}

/// Distance of `p` above the planet's surface.
fn height(p: RVec3) -> f32 {
    let p = Vec3::new(p.x as f32, p.y as f32, p.z as f32);
    dot(p, p).sqrt() - PLANET_RADIUS
}

/// The planet, a chunk 3 m east of the north pole and a character standing on the ground where
/// up is `start`.
fn build(start: Vec3) -> Result<(PhysicsWorld, CharacterId), Box<dyn std::error::Error>> {
    // Radial gravity is the caller's, so the world has none.
    let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO))?;
    let planet = Shape::new_sphere(PLANET_RADIUS)?;
    world.create_body(&planet, &BodySettings::new_static())?;

    // The chunk's origin is on the ground with its local Y along up: a bush 1.5 m before a
    // 3 m wall, both sunk 0.5 m into the ground.
    let angle = 3.0 / PLANET_RADIUS;
    let ground = Vec3::new(PLANET_RADIUS * angle.sin(), PLANET_RADIUS * angle.cos(), 0.0);
    let wall = Shape::new_box(Vec3::new(0.25, 1.5, 2.0))?;
    let bush = Shape::new_cylinder(1.0, 0.4)?;
    let chunk = Shape::new_compound(&[
        CompoundChild {
            shape: &wall,
            position: Vec3::new(0.0, 1.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: STRUCTURE,
        },
        CompoundChild {
            shape: &bush,
            position: Vec3::new(-1.5, 0.5, 0.0),
            rotation: Quat::IDENTITY,
            user_data: FEATURE,
        },
    ])?;
    world.create_body(
        &chunk,
        &BodySettings::new_static()
            .position(RVec3::new(ground.x as Real, ground.y as Real, 0.0))
            .rotation(rotation_to(normalize(ground))),
    )?;

    // A capsule 1.6 m tall whose bottom is at the character position.
    let capsule = Shape::new_capsule(0.5, 0.3)?;
    let settings = CharacterSettings::new(&capsule)
        .shape_offset(Vec3::new(0.0, 0.8, 0.0))
        .max_slope_angle(45.0_f32.to_radians());
    let ground = scale(start, PLANET_RADIUS);
    let position = RVec3::new(ground.x as Real, ground.y as Real, ground.z as Real);
    let id = world.create_character(&settings, position, rotation_to(start))?;
    // A new character knows no ground until its contacts are refreshed.
    world.refresh_character_contacts(id, &walk_filter())?;
    Ok((world, id))
}

/// What the character collides with: every body, but of compounds only the structures.
fn walk_filter() -> QueryFilter<'static> {
    QueryFilter::new().child_groups(1 << STRUCTURE)
}

/// One tick: walk forward at 2 m/s along the ground. `vel_up` is the vertical speed the caller
/// carries from tick to tick.
fn tick(
    world: &mut PhysicsWorld,
    id: CharacterId,
    vel_up: &mut f32,
) -> Result<(), Box<dyn std::error::Error>> {
    let character = world.character(id)?;
    let up = up_at(character.position());
    *vel_up = if character.ground_state() == GroundState::OnGround {
        // One tick of gravity, so that stick to floor has something to follow.
        -GRAVITY * DT
    } else {
        *vel_up - GRAVITY * DT
    };
    let forward = forward_at(up);
    let mut character = world.character_mut(id)?;
    character.set_up(up)?;
    character.set_rotation(rotation_to(up))?;
    character.set_linear_velocity(add(scale(forward, 2.0), scale(up, *vel_up)))?;
    // Stick to floor along -up; walk stairs off, as for a game with sharp steps.
    let extended = ExtendedUpdateSettings::default()
        .stick_to_floor_step_down(scale(up, -0.3))
        .walk_stairs_step_up(Vec3::ZERO);
    world.update_character(id, DT, scale(up, -GRAVITY), &extended, &walk_filter())?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let north = Vec3::new(0.0, 1.0, 0.0);
    let (mut world, id) = build(north)?;
    assert_eq!(world.character(id)?.ground_state(), GroundState::OnGround);

    // One second of walking, then save the state and the caller's vertical speed.
    let mut vel_up = 0.0;
    for _ in 0..60 {
        tick(&mut world, id, &mut vel_up)?;
    }
    let saved = (world.character(id)?.save_state(), vel_up);

    // Another second: the walk ends at the wall, after passing through the bush.
    for _ in 0..60 {
        tick(&mut world, id, &mut vel_up)?;
    }
    let character = world.character(id)?;
    assert_eq!(character.ground_state(), GroundState::OnGround);
    let p = character.position();
    let above = height(p);
    assert!(above.abs() < 0.01, "on the ground: {above}");
    let walked = PLANET_RADIUS * (p.x as f32).atan2(p.y as f32);
    assert!((2.3..2.5).contains(&walked), "stopped at the wall: {walked}");
    let wall = character
        .active_contacts()
        .iter()
        .filter(|contact| contact.had_collision)
        .find_map(|contact| character.contact_compound_child(contact))
        .expect("a contact with the wall");
    assert_eq!(wall.user_data, STRUCTURE);
    let finished = character.save_state();

    // A world built the same way, restored from the saved state, ends the same, bit for bit.
    let (mut replay, replay_id) = build(north)?;
    replay.character_mut(replay_id)?.restore_state(&saved.0)?;
    let mut vel_up = saved.1;
    for _ in 0..60 {
        tick(&mut replay, replay_id, &mut vel_up)?;
    }
    assert_eq!(replay.character(replay_id)?.save_state(), finished);

    // Half a second of walking from the south pole and from the four equator points stays on
    // the ground and covers about a metre.
    for start in [
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, -1.0),
    ] {
        let (mut world, id) = build(start)?;
        let mut vel_up = 0.0;
        for _ in 0..30 {
            tick(&mut world, id, &mut vel_up)?;
        }
        let character = world.character(id)?;
        assert_eq!(character.ground_state(), GroundState::OnGround);
        let p = character.position();
        assert!(height(p).abs() < 0.01, "on the ground from {start:?}: {p:?}");
        let ground = scale(start, PLANET_RADIUS);
        let moved = Vec3::new(p.x as f32 - ground.x, p.y as f32 - ground.y, p.z as f32 - ground.z);
        let moved = dot(moved, moved).sqrt();
        assert!((0.9..1.1).contains(&moved), "walked from {start:?}: {moved}");
    }
    Ok(())
}
```

## Vehicles

A vehicle is Jolt's `VehicleConstraint` with the wheeled controller, attached to a dynamic body the
caller created, its chassis. `PhysicsWorld::create_wheeled_vehicle(chassis, &settings)` adds it; from then
on it runs inside every `step`: at the start of the step each wheel casts against the scene and
gravity is applied, then engine, brakes and tire friction act through the constraint. Tracked
vehicles and motorcycles work the same way and have [a guide of their own](vehicles.md); what
follows holds for every kind unless it names the wheeled settings.

- **Chassis.** The chassis shape gives the vehicle its centre of mass, and its mass and inertia
  unless `BodySettings::mass` or `mass_and_inertia` overrides them ([bodies](bodies.md#mass-and-inertia)).
  `Shape::new_offset_center_of_mass` moves only the centre of mass, not the collision surface, so a
  box hull can carry a low centre of mass that keeps the car from rolling over. `remove_body`
  refuses a chassis while its vehicle exists; call `remove_vehicle` first.
- **Settings.** `WheeledVehicleSettings::new(wheels, differentials, collision_tester)`, with builders for
  the rest; the defaults are Jolt's. Positions and directions are in the chassis body's space,
  with up +Y and forward +Z by default. Wheels are numbered in the order given, and differentials
  and anti-roll bars name them by index; Jolt's samples put the left wheels at +X. A vehicle needs
  at least one differential, and the engine torque ratios of all differentials add up to 1.
  Values Jolt asserts on or divides by are checked first: `create_wheeled_vehicle` returns
  `VehicleError::InvalidValue` for them and creates nothing. `WheeledVehicleSettings::car` is a ready
  front-wheel-drive car of four wheels mirrored from the front left one, with Jolt's default engine
  and transmission.
- **Wheels and the ground.** `VehicleCollisionTester::ray`, `cast_sphere` or `cast_cylinder`.
  The wheels see the bodies whose object layer collides with the tester's object layer, never
  their own chassis. A tester cannot skip compound children by group, so give the wheels their
  own object layer that collides with exactly what they drive on.
- **Gravity.** For gravity the game applies itself (radial on a planet), create the chassis with
  `gravity_factor(0.0)` and `allow_sleeping(false)`, and call `VehicleMut::set_gravity` with the
  gravity at the car before every step. Jolt then adds that gravity times the chassis mass as a
  force on every step while the chassis is awake, once per collision step with
  `step_with_collision_steps`, as world gravity acts; a sleeping chassis gets nothing. The opposite of
  that gravity is also the up of the pitch and roll limit (`WheeledVehicleSettings::max_pitch_roll_angle`).
- **Driving.** `set_driver_input(DriverInput { forward, right, brake, hand_brake })`: forward and
  right in `[-1, 1]`, the brakes in `[0, 1]`. Right 1 steers fully right, which Jolt reports as a
  negative steer angle. The input stays until it is set again.
- **Readout.** `VehicleRef::wheels` gives each wheel's `WheelState`: the ground contact (body,
  sub-shape, point, normal), the suspension length, the wheel's rotation speed and the impulses
  of the last step; `engine_rpm` and `current_gear` read the drivetrain. Contacts are found at the
  start of the step, at the chassis pose before the step moved it. Normals are the ground's
  outward normal, as in queries. A wheel whose cast starts inside a solid body reports suspension
  length 0 and `hit_hard_point`, not how deep it is; `collide_shape` gives the depth.
  `wheel_world_transform` gives a wheel's centre and rotation in world space for drawing it, for a
  wheel model whose axle is its local Y.
- **Rebase.** `PhysicsWorld::rebase` moves the chassis like any body and rotates each vehicle's
  gravity override, the up of a ray or sphere tester and a motorcycle's target lean; the reported
  contacts stay in the old frame until the next step.
- **Tester accuracy.** `wheel_contacts_match_the_ground_geometry` checks every tester on flat
  ground within 1e-3 m. A one-off measurement on a chassis rolled 6° or pitched -4° over box and
  heightfield ground and a 6° slope found the ray and sphere testers' suspension length within
  1.1e-7 m of the analytic value and their contact point within 2.5e-6 m. The cylinder tester's
  suspension length was within 5.6e-5 m, but its contact point slid up to 1.7 mm along the rim
  (Jolt resolves it with GJK/EPA, which converges in height, not along the rim). Judge cylinder
  contacts by suspension length, normal and distance to the ground.
- **Soft bodies.** Wheels look through soft bodies and report only the rigid ground below them:
  Jolt's vehicle constraint solves the body under a wheel as a rigid body.

## Ragdolls

A ragdoll is Jolt's `Ragdoll`: one body per joint of a `Skeleton`, each joined to its parent part
by a constraint. `RagdollSettings::new(&skeleton, &parts)` builds the settings once. Each
`RagdollPart` is a shape, `BodySettings` (motion type, layer, mass, friction, damping, gravity
factor, initial velocity) at the part's bind pose in world space, and the joint to its parent.

- **Joints.** `RagdollJoint::SwingTwist` (spine, neck, shoulders), `Hinge` with limits (elbows,
  knees) and `SixDof` (hips). By default their frames are given in world space at the bind pose.
  In a joint's frame X is the twist or hinge axis and Y and Z are the swing axes. A cone swing has
  symmetric limits; `SwingType::Pyramid` allows a different range on each side, which a hip needs.
- **Collisions.** Parts of one ragdoll never collide with each other: every pair of parts is
  disabled in a group filter, and each ragdoll gets its own group, so two ragdolls still collide.
  To keep ragdolls out of the main simulation, put them in a second `PhysicsWorld` whose static
  bodies are made from the same `Shape` handles as the main world's: Jolt shares a shape, it does
  not copy it.
- **Gravity.** For the game's own gravity, give every part `gravity_factor(0.0)` and add
  gravity times mass to each part before every step (`BodyRef::mass`, `BodyMut::add_force`).
- **Pose and drive.** `RagdollRef::pose` and `RagdollMut::set_pose` read and write every part at
  once. `drive_to_pose_using_motors` aims each joint's motors at its part's rotation relative to
  the parent part in the pose. Motors keep the parts awake only while something wakes them, so
  drive every tick. `drive_to_pose_using_kinematics` moves kinematic parts to the pose in one step.
- **Rest.** `SettleDetector::default()` reports a ragdoll as settled once every part has moved
  slower than 0.05 m/s and 0.1 rad/s for 30 updates in a row; the timeout is the caller's. In the
  repository's tests a humanoid came to rest reliably only with joint friction (about 1 N·m on
  every joint, `max_friction_torque` and `max_friction`); without it the near-spherical head kept
  rocking on the ground and several drops did not settle.
- **Joint limits on impact.** In each solver iteration Jolt solves contacts after constraints, so
  when a falling ragdoll hits the ground the contacts win and joints pass their limits for a few
  dozen ticks before the constraints pull them mostly back. These are measurements, not bounds:
  in the repository's drop test (a 12-part humanoid dropped with its pelvis 1.5 m above the
  terrain) the worst overshoot is 0.29 rad and 0.0037 rad remain at rest; the test's bounds,
  0.40 rad during the fall and 0.01 rad at rest, hold for that drop only. A sweep of 126 drops
  of that humanoid, run once outside the test suite and not kept in the repository (7 sideways
  and 3 forward offsets, drops of 1.0, 1.5 and 2.5 m with different yaw, raw and stabilized
  masses, caller-applied gravity), found an overshoot above 0.40 rad in 4 drops (up to 0.48 rad),
  joints more than 0.01 rad outside their limits at rest in 40 (up to 0.15 rad), 3 drops that
  needed more than 600 ticks to settle, and hinges bent about their fixed axes by up to 0.57 rad
  on impact. Check joint limits at rest, with a tolerance, not on every tick. In that sweep the
  centre of a thin limb dropped from 2.5 m ended up to 0.15 m below the terrain surface with the
  default `MotionQuality::Discrete`; `MotionQuality::LinearCast` on the limbs is the setting to
  try for high falls.
- **Lifecycle.** `remove_ragdoll` removes the parts and wakes what rested on them; `remove_body`
  refuses a part.
- **Animation skeletons.** A `SkeletonMapper` (Jolt's `SkeletonMapper`) maps poses between the
  ragdoll's skeleton and a detailed animation skeleton. Joints are matched by name, and the
  animation skeleton keeps the ragdoll's hierarchy; it may add joints between the ragdoll's, above
  its root and below its leaves. `map_reverse` turns an animation pose into a ragdoll pose from the
  matched joints only: a target for `set_pose` and both drives. `map` turns the simulated
  `RagdollRef::pose` and the animation's local pose into an animation pose to render. Unmatched
  joints between a ragdoll joint and one of its children form a chain: the chain's start turns so
  that the chain points at the child's ragdoll joint. Jolt builds one chain per start joint, along
  the longest path (on a tie, toward the lowest ragdoll joint index); the joints between the start
  and its other children keep their local transforms, as do extra roots (relative to the root
  offset) and leaves. `TranslationLocks` keep animation joints at their neutral offset from their
  parent, which hides the joints' stretch under load; Jolt applies them after the unmatched joints,
  so an unmatched child of a locked joint follows the joint's unlocked position. Both neutral poses
  describe the character standing in one place. A chain shorter than
  `limits::MIN_MAPPED_CHAIN_LENGTH` is refused with `RagdollError::DegenerateChain`; far from the
  pose's root offset, rounding makes the allowance larger
  ([limits.md](limits.md#skeleton-mapper-chains)).

```rust
use oxijolt::*;

# fn main() -> Result<(), RagdollError> {
let joint = |name, parent| SkeletonJoint { name, parent };
let ragdoll = Skeleton::new(&[joint("pelvis", None), joint("chest", Some(0))])?;
let animation = Skeleton::new(&[
    joint("pelvis", None),
    joint("spine", Some(0)),
    joint("chest", Some(1)),
])?;
// Joints along +Y at these heights above a root offset 1 m up.
let pose = |heights: &[f32]| SkeletonPose {
    root_offset: RVec3::new(0.0, 1.0, 0.0),
    joints: heights
        .iter()
        .map(|&y| JointTransform {
            translation: Vec3::new(0.0, y, 0.0),
            rotation: Quat::IDENTITY,
        })
        .collect(),
};
let mapper = SkeletonMapper::new(
    MappedSkeleton { skeleton: &ragdoll, neutral_pose: &pose(&[0.0, 0.5]) },
    MappedSkeleton { skeleton: &animation, neutral_pose: &pose(&[0.0, 0.25, 0.5]) },
    TranslationLocks::None,
)?;

// Animation to ragdoll: a target for `set_pose` or the drives.
let target = mapper.map_reverse(&pose(&[0.0, 0.3, 0.6]))?;
assert_eq!(target.joints.len(), 2);

// Ragdoll to animation: the local pose (each joint relative to its parent) lays out the spine.
let local = pose(&[0.0, 0.25, 0.25]).joints;
let shown = mapper.map(&target, &local)?;
assert_eq!(shown.joints.len(), 3);
# Ok(())
# }
```

## A car and a ragdoll

A car drives over rolling terrain in the main world while a ragdoll falls in a second world that
shares the terrain shape. Neither world has gravity of its own: the caller applies it, as on a
planet, where it would point at the centre. The ragdoll faces +Z in its bind pose: a pelvis, chest
and head joined by swing-twist joints, thighs on six-DOF hips with asymmetric limits and shins on
hinged knees.

```rust
use oxijolt::*;

type Error = Box<dyn std::error::Error>;

const DT: f32 = 1.0 / 60.0;
/// The gravity the caller applies. On a planet it would point at the centre and change with the
/// position; here it is constant to keep the example short.
const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// The object layers of both worlds.
struct Layers {
    ground: ObjectLayer,
    moving: ObjectLayer,
    wheels: ObjectLayer,
}

/// A world without gravity of its own, in which wheels see the ground only.
fn world() -> Result<(PhysicsWorld, Layers), Error> {
    let mut table = CollisionLayers::new(2);
    let fixed = BroadPhaseLayer::new(0);
    let moving = BroadPhaseLayer::new(1);
    let layers = Layers {
        ground: table.add_object_layer(fixed),
        moving: table.add_object_layer(moving),
        wheels: table.add_object_layer(moving),
    };
    table
        .enable_collision(layers.moving, layers.ground)
        .enable_collision(layers.moving, layers.moving)
        .enable_collision(layers.wheels, layers.ground);
    let world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO).layers(table))?;
    Ok((world, layers))
}

/// A gentle swell, metres.
fn swell(x: f32, z: f32) -> f32 {
    0.3 * (0.35 * x).sin() * (0.25 * z).cos()
}

/// The terrain: 33 x 33 samples of the swell, one per metre, x and z in [-16, 16].
fn terrain() -> Result<Shape, Error> {
    let mut samples = Vec::with_capacity(33 * 33);
    for z in 0..33 {
        for x in 0..33 {
            samples.push(swell(x as f32 - 16.0, z as f32 - 16.0));
        }
    }
    let settings = HeightFieldSettings::default().offset(Vec3::new(-16.0, 0.0, -16.0));
    Ok(Shape::new_height_field(33, &samples, &settings)?)
}

/// The height of the ground below `(x, z)`.
fn ground_at(world: &PhysicsWorld, layers: &Layers, x: Real, z: Real) -> Result<Real, Error> {
    let ray = RayCast::new(RVec3::new(x, 10.0, z), Vec3::new(0.0, -20.0, 0.0));
    let ground = [layers.ground];
    let hit = world.cast_ray(&ray, &QueryFilter::new().object_layers(&ground))?;
    Ok(ray.point_at(hit.ok_or("no ground")?.fraction).y)
}

/// Adds the force of `GRAVITY` to a dynamic body.
fn apply_gravity(world: &mut PhysicsWorld, body: BodyId) -> Result<(), Error> {
    let mass = world.body(body)?.mass().ok_or("a dynamic body")?;
    let force = Vec3::new(GRAVITY.x * mass, GRAVITY.y * mass, GRAVITY.z * mass);
    world.body_mut(body)?.add_force(force)?;
    Ok(())
}

/// A front-wheel-drive car with a low centre of mass, 8 m south of the centre, facing +Z.
fn add_car(world: &mut PhysicsWorld, layers: &Layers) -> Result<(BodyId, VehicleId), Error> {
    let hull = Shape::new_box(Vec3::new(0.9, 0.3, 2.0))?;
    let chassis_shape = Shape::new_offset_center_of_mass(&hull, Vec3::new(0.0, -0.3, 0.0))?;
    let ground = ground_at(world, layers, 0.0, -8.0)?;
    let chassis = world.create_body(
        &chassis_shape,
        &BodySettings::new_dynamic()
            .position(RVec3::new(0.0, ground + 1.0, -8.0))
            .object_layer(layers.moving)
            .mass(1500.0)
            .allow_sleeping(false)
            .gravity_factor(0.0),
    )?;
    // Left wheels at +X. The front wheels steer, the hand brake holds the rear ones.
    let wheel = |x: f32, z: f32| WheelSettings::new(Vec3::new(x, -0.1, z)).radius(0.35).width(0.2);
    let front = |x| wheel(x, 1.4).max_steer_angle(0.5).max_hand_brake_torque(0.0);
    let rear = |x| wheel(x, -1.4).max_steer_angle(0.0);
    let settings = WheeledVehicleSettings::new(
        vec![front(0.9), front(-0.9), rear(0.9), rear(-0.9)],
        vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
        VehicleCollisionTester::cast_sphere(layers.wheels, 0.2),
    )
    .anti_roll_bars(vec![VehicleAntiRollBar::new(0, 1), VehicleAntiRollBar::new(2, 3)]);
    let car = world.create_wheeled_vehicle(chassis, &settings)?;
    Ok((chassis, car))
}

/// The ragdoll's skeleton: the name and parent of each joint, parents first.
const JOINTS: [(&str, Option<u32>); 7] = [
    ("pelvis", None),
    ("chest", Some(0)),
    ("head", Some(1)),
    ("thigh_l", Some(0)),
    ("shin_l", Some(3)),
    ("thigh_r", Some(0)),
    ("shin_r", Some(5)),
];

/// The bind pose with the pelvis at the origin: each part's centre, capsule half height and
/// radius, and mass in kg. Every capsule stands along Y.
const PARTS: [([f32; 3], f32, f32, f32); 7] = [
    ([0.0, 0.0, 0.0], 0.1, 0.15, 12.0),
    ([0.0, 0.42, 0.0], 0.12, 0.15, 18.0),
    ([0.0, 0.8, 0.0], 0.04, 0.11, 5.0),
    ([0.12, -0.42, 0.0], 0.14, 0.08, 8.0),
    ([0.12, -0.86, 0.0], 0.14, 0.07, 4.0),
    ([-0.12, -0.42, 0.0], 0.14, 0.08, 8.0),
    ([-0.12, -0.86, 0.0], 0.14, 0.07, 4.0),
];

/// Joint friction, N·m: it lets the ragdoll come to rest instead of rocking.
const FRICTION: f32 = 1.0;
/// How far a knee bends, radians.
const KNEE_BEND: f32 = 2.2;

/// The joint of `part` to its parent, in world space at the bind pose.
fn joint(part: usize) -> Option<RagdollJoint> {
    let up = Vec3::new(0.0, 1.0, 0.0);
    let down = Vec3::new(0.0, -1.0, 0.0);
    let side = Vec3::new(1.0, 0.0, 0.0);
    let x = PARTS[part].0[0];
    let at = |y: Real| RVec3::new(x as Real, y, 0.0);
    Some(match part {
        0 => return None,
        // Waist and neck: the twist axis runs up the spine.
        1 | 2 => {
            let (anchor, cone) = if part == 1 { (at(0.2), 0.3) } else { (at(0.62), 0.5) };
            RagdollJoint::SwingTwist(
                SwingTwistConstraintSettings::new(anchor, up, side)
                    .half_cone_angles(cone, cone)
                    .twist_limits(-0.3, 0.3)
                    .max_friction_torque(FRICTION),
            )
        }
        // Hips: the twist axis runs down the thigh, Y is the side axis and Z points forward. A
        // negative turn about Y swings the leg forward, a positive turn about Z swings it toward
        // +X: the leg swings 1.4 rad forward but 0.3 back, 0.5 out but 0.2 in.
        3 | 5 => {
            let (out, inward) = (0.5, 0.2);
            let about_z = if x > 0.0 { (-inward, out) } else { (-out, inward) };
            let mut hip = SixDofConstraintSettings::new(at(-0.24), down, side)
                .swing_type(SwingType::Pyramid);
            for axis in [
                SixDofConstraintAxis::TranslationX,
                SixDofConstraintAxis::TranslationY,
                SixDofConstraintAxis::TranslationZ,
            ] {
                hip = hip.axis(axis, SixDofAxis::Fixed);
            }
            for (axis, (min, max)) in [
                (SixDofConstraintAxis::RotationX, (-0.3, 0.3)),
                (SixDofConstraintAxis::RotationY, (-1.4, 0.3)),
                (SixDofConstraintAxis::RotationZ, about_z),
            ] {
                hip = hip
                    .axis(axis, SixDofAxis::Limited { min, max })
                    .max_friction(axis, FRICTION);
            }
            RagdollJoint::SixDof(hip)
        }
        // Knees: a hinge about the side axis; a positive angle swings the shin back, to -Z.
        _ => RagdollJoint::Hinge(
            HingeConstraintSettings::new(at(-0.64), side, down)
                .limits(0.0, KNEE_BEND)
                .max_friction_torque(FRICTION),
        ),
    })
}

/// The ragdoll's settings, its parts in `layer` under the caller's gravity.
fn ragdoll_settings(layer: ObjectLayer) -> Result<RagdollSettings, Error> {
    let joints: Vec<SkeletonJoint<'_>> = JOINTS
        .iter()
        .map(|&(name, parent)| SkeletonJoint { name, parent })
        .collect();
    let skeleton = Skeleton::new(&joints)?;
    let shapes = PARTS
        .iter()
        .map(|&(_, half_height, radius, _)| Shape::new_capsule(half_height, radius))
        .collect::<Result<Vec<_>, _>>()?;
    let parts: Vec<RagdollPart<'_>> = PARTS
        .iter()
        .zip(&shapes)
        .enumerate()
        .map(|(part, (&([x, y, z], _, _, mass), shape))| RagdollPart {
            shape,
            body: BodySettings::new_dynamic()
                .position(RVec3::new(x as Real, y as Real, z as Real))
                .object_layer(layer)
                .mass(mass)
                .friction(0.6)
                .gravity_factor(0.0),
            joint: joint(part),
        })
        .collect();
    Ok(RagdollSettings::new(&skeleton, &parts)?)
}

/// The bind pose turned a quarter turn about Z, lying on its side, with the pelvis at `pelvis`.
fn lying_pose(pelvis: RVec3) -> SkeletonPose {
    let half_angle = std::f32::consts::FRAC_PI_4;
    let rotation = Quat::from_xyzw(0.0, 0.0, half_angle.sin(), half_angle.cos());
    let joints = PARTS
        .iter()
        // The quarter turn about Z takes (x, y) to (-y, x).
        .map(|&([x, y, z], ..)| JointTransform { translation: Vec3::new(-y, x, z), rotation })
        .collect();
    SkeletonPose { root_offset: pelvis, joints }
}

fn main() -> Result<(), Error> {
    // One terrain shape in two worlds.
    let terrain = terrain()?;
    let (mut main, main_layers) = world()?;
    let ground = BodySettings::new_static().object_layer(main_layers.ground);
    let main_terrain = main.create_body(&terrain, &ground)?;
    let (mut ragdolls, layers) = world()?;
    let ground = BodySettings::new_static().object_layer(layers.ground);
    let ragdoll_terrain = ragdolls.create_body(&terrain, &ground)?;
    drop(terrain); // each body holds its own reference
    assert_eq!(
        ground_at(&main, &main_layers, 3.3, -2.7)?.to_bits(),
        ground_at(&ragdolls, &layers, 3.3, -2.7)?.to_bits(),
    );

    // Three seconds at full throttle; the caller's gravity is set before every step.
    let (chassis, car) = add_car(&mut main, &main_layers)?;
    for _ in 0..180 {
        let mut vehicle = main.vehicle_mut(car)?;
        vehicle.set_gravity(GRAVITY)?;
        vehicle.set_driver_input(DriverInput { forward: 1.0, ..DriverInput::default() })?;
        assert!(main.step(DT)?.is_complete());
    }
    let body = main.body(chassis)?;
    assert!(body.position().z > -4.0, "the car drove: {:?}", body.position());
    // The y component of the chassis' up, the rotated +Y, is 1 - 2 (x² + z²).
    let r = body.rotation();
    assert!(1.0 - 2.0 * (r.x * r.x + r.z * r.z) > 0.9, "the car is upright");
    let vehicle = main.vehicle(car)?;
    assert!(vehicle.current_gear() >= 1);
    for wheel in vehicle.wheels() {
        let contact = wheel.contact.ok_or("every wheel is on the terrain")?;
        assert_eq!(contact.body, main_terrain);
        assert!(contact.normal.y > 0.9, "the terrain's normal points up");
    }

    // The ragdoll falls from 1.2 m, lying on its side, with a push; gravity is the caller's.
    let settings = ragdoll_settings(layers.moving)?;
    let pelvis = RVec3::new(5.0, ground_at(&ragdolls, &layers, 5.0, 5.0)? + 1.2, 5.0);
    let ragdoll =
        ragdolls.create_ragdoll(&settings, Some(&lying_pose(pelvis)), Activation::Activate)?;
    ragdolls
        .ragdoll_mut(ragdoll)?
        .set_linear_and_angular_velocity(Vec3::new(1.0, 0.0, 0.5), Vec3::new(0.0, 1.0, 0.0))?;
    let parts = ragdolls.ragdoll(ragdoll)?.body_ids().to_vec();
    let mut detector = SettleDetector::default();
    let mut landed = false;
    let mut settled = false;
    for _ in 0..600 {
        for &part in &parts {
            apply_gravity(&mut ragdolls, part)?;
        }
        assert!(ragdolls.step(DT)?.is_complete());
        for (i, &a) in parts.iter().enumerate() {
            landed |= ragdolls.were_bodies_in_contact(a, ragdoll_terrain)?;
            for &b in &parts[i + 1..] {
                assert!(!ragdolls.were_bodies_in_contact(a, b)?, "parts of one ragdoll touched");
            }
        }
        if detector.update(&ragdolls.ragdoll(ragdoll)?) {
            settled = true;
            break;
        }
    }
    assert!(landed && settled, "the ragdoll fell onto the terrain and came to rest");

    // At rest the knees are within their limits; on impact they may have passed them.
    let rest = ragdolls.ragdoll(ragdoll)?;
    for knee in [4, 6] {
        let Some(JointReading::Hinge { current_angle }) = rest.joint(knee) else {
            return Err("a knee is a hinge".into());
        };
        assert!((-0.05..=KNEE_BEND + 0.05).contains(&current_angle), "{current_angle}");
    }
    let pelvis = rest.pose().root_offset;
    let ground = ground_at(&ragdolls, &layers, pelvis.x, pelvis.z)?;
    assert!(pelvis.y - ground < 0.4, "the pelvis lies on the terrain");

    // Removing the ragdoll removes its parts; the terrain stays.
    ragdolls.remove_ragdoll(ragdoll)?;
    assert_eq!(ragdolls.body_count(), 1);
    Ok(())
}
```
