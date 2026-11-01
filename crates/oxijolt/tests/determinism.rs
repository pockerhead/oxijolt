//! Same-machine determinism through the safe API: the per-tick state of a scene must be
//! bit-identical whether the world steps with 1 or 4 worker threads, also across a rebase, and
//! the order in which bodies are created must decide their ids.
//!
//! Two scenes are gated. The stacks scene topples cubes on a floor for 120 ticks. The chunk
//! scene is the game's case: a static compound chunk on a heightfield terrain, three items the
//! caller pulls towards a planet centre, created, coming to rest and removed under the game's
//! item rules, and a tilted rebase in the middle, recorded for ticks 0 to 1000 with the static
//! shapes and the body states of every tick. Creating the same bodies in another order must
//! fail the gate. The walker scene runs the game's scripted near steps on a character, with three
//! items dropped along its path, in the game's order of a tick: actors, items, step. The vehicle
//! scene drives the acceptance route over terrain; the fleet scene runs 40 vehicles, enough for
//! Jolt to spread their step listeners over a different number of jobs with 1 and 4 workers,
//! with the couplings that would show a dependence on listener order. The ragdoll pile drops 16
//! humanoid ragdolls into a pit, in contact with each other from the first tick, so that their
//! joints and contacts form an island large enough for Jolt to split it for parallel solving.
//! The constraints scene runs a hinge chain, a motor-driven slider, a gear pair, a pulley and a
//! path, and removes and re-creates a constraint halfway. The soft body scene drapes a cloth
//! pinned at two corners over a sphere and drops a pressurised ball on a box, unpins one corner
//! and moves the other halfway, and records every vertex.
//!
//! Each scene is also gated with caller job systems: a Rayon pool of 4 threads and an inline job
//! system must record what Jolt's thread pool with 1 worker records.
//!
//! Each run happens in its own child process (this test binary, running the ignored
//! `determinism_child` test), so no state leaks between runs; see `common::determinism`.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::determinism::*;
use common::jobs::{self, JobChoice};
use common::math::{self, cross, rvec3, scale, v3, vec3, V3};
use common::ragdoll as humanoid;
use common::walker::{
    add_walker, record_walker, script_scene, script_start, script_tick, up_at, Player,
};
use common::*;
use oxijolt::*;

/// Ticks of the stacks scene.
const TICKS: usize = 120;

/// Bytes recorded per body per tick by [`record_body`]: id, position, rotation, linear and
/// angular velocity, and the sleeping flag.
const BODY_RECORD_SIZE: usize = 4 + 3 * size_of::<Real>() + 4 * 4 + 3 * 4 + 3 * 4 + 1;

/// Bodies in the stacks scene: the floor and the cubes.
fn stacks_body_count() -> usize {
    1 + stacks_scene().len()
}

/// The rotation of the tilted rebases: 30 degrees about a tilted axis.
fn tilted_rotation() -> Quat {
    let axis = Vec3::new(1.0, 2.0, 0.5);
    let axis = Vec3::new(
        axis.x / length(axis),
        axis.y / length(axis),
        axis.z / length(axis),
    );
    quat_about(axis, 30.0_f32.to_radians())
}

/// The translation of the tilted rebases.
fn tilted_translation() -> RVec3 {
    RVec3::new(12.5, -3.0, 7.25)
}

/// Runs the stacks scene and returns one state record per step, of every body in scene order.
/// Bodies are created in scene order (floor first), or in reverse for `"reversed"`.
/// `"rebased"` creates them in scene order and moves the world into a tilted frame halfway
/// through. `"sub_stepped"` creates them in scene order and steps with
/// [`PhysicsWorld::MAX_COLLISION_STEPS`] collision steps.
fn run_stacks(worker_threads: u32, variant: &str) -> Digest {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), worker_threads);
    let ids = match variant {
        "forward" | "rebased" | "sub_stepped" => build_stacks(&mut world),
        "reversed" => {
            let mut ids: Vec<BodyId> = stacks_scene()
                .into_iter()
                .rev()
                .map(|position| add_cube(&mut world, position))
                .collect();
            ids.push(add_floor(&mut world));
            ids.reverse();
            ids
        }
        variant => panic!("unknown stacks variant {variant}"),
    };
    let collision_steps = if variant == "sub_stepped" {
        PhysicsWorld::MAX_COLLISION_STEPS
    } else {
        1
    };
    let mut digest = Digest::new();
    for tick in 0..TICKS {
        if variant == "rebased" && tick == TICKS / 2 {
            world
                .rebase(&ids, tilted_rotation(), tilted_translation())
                .unwrap();
        }
        assert!(world
            .step_with_collision_steps(DT, collision_steps)
            .unwrap()
            .is_complete());
        let record = digest.push();
        for &id in &ids {
            record_body(&world, id, &mut record.state);
        }
    }
    digest
}

/// Columns of [`run_pile`] along x and z.
const PILE_COLUMNS: usize = 10;
/// Cubes in each column of [`run_pile`].
const PILE_LAYERS: usize = 6;

/// Extra unrecorded runs of [`run_pile`] the Rayon child may need before its pool executes a
/// Jolt job; each misses with a probability of at most about a quarter on one core.
const PILE_POOL_RETRIES: usize = 10;

/// A floor and 600 cubes in leaning columns that topple into one pile: enough work per step
/// that the threads of a caller's pool run Jolt jobs even on a loaded machine, where the
/// stepping thread can finish every job of the eight-cube stacks scene before a pool thread is
/// scheduled.
fn run_pile(worker_threads: u32) -> Digest {
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), worker_threads);
    let mut ids = vec![add_floor(&mut world)];
    for column in 0..PILE_COLUMNS * PILE_COLUMNS {
        let (x, z) = (
            (column % PILE_COLUMNS) as Real,
            (column / PILE_COLUMNS) as Real,
        );
        for layer in 0..PILE_LAYERS {
            let lean = 0.3 * layer as Real;
            let position = RVec3::new(1.2 * x + lean, 0.5 + layer as Real, 1.2 * z);
            ids.push(add_cube(&mut world, position));
        }
    }
    let mut digest = Digest::new();
    for _ in 0..TICKS {
        assert!(world.step(DT).unwrap().is_complete());
        let record = digest.push();
        for &id in &ids {
            record_body(&world, id, &mut record.state);
        }
    }
    digest
}

/// The raw body ids of the first tick, in scene order.
fn first_tick_ids(digest: &Digest) -> Vec<u32> {
    digest.ticks[0]
        .state
        .as_chunks::<BODY_RECORD_SIZE>()
        .0
        .iter()
        .map(|record| u32::from_le_bytes(record[..4].try_into().unwrap()))
        .collect()
}

/// The raw id Jolt gives the `n`-th body added to a fresh world: index `n`, sequence 1.
fn nth_body_id(n: usize) -> u32 {
    (1 << 23) | n as u32
}

/// Ticks of the chunk scene after the initial record at tick 0.
const CHUNK_TICKS: u32 = 1000;
/// The world moves into a tilted frame between this tick and the next.
const REBASE_TICK: u32 = 500;
/// Item 3 is created before the step after this tick, so it falls through the rebase.
const LATE_ITEM_TICK: u32 = 470;
/// Calm ticks in a row after which an item counts as at rest and is removed (the game's item
/// rest rule).
const REST_TICKS: u32 = 30;
/// Ticks after which an item is removed even if it never came to rest.
const ITEM_TIMEOUT: u32 = 600;
const ITEM_MASS: f32 = 1.2;
/// Strength of the pull towards the planet centre, m/s².
const GRAVITY: f32 = 9.8;
/// Height of the terrain's mean surface above the scene origin, metres.
const SURFACE: f32 = 2.0;
/// Radius of the planet whose surface the terrain is, centred below the scene.
const PLANET_RADIUS: f64 = 99.0;
const ITEM_HALF_EXTENT: Vec3 = Vec3::new(0.35, 0.06, 0.04);
/// Samples per side of the terrain heightfield.
const TERRAIN_SAMPLES: u32 = 33;
/// Convex radius of the chunk's boxes and cylinder, metres (Jolt's default).
const CHILD_CONVEX_RADIUS: f32 = 0.05;

/// `p` rotated by `rotation` and moved by `translation`, in `f64`.
fn map_point(rotation: Quat, translation: RVec3, p: V3) -> V3 {
    let [x, y, z, w] = [rotation.x, rotation.y, rotation.z, rotation.w].map(f64::from);
    let q = [x, y, z];
    let t = cross(q, p).map(|c| 2.0 * c);
    let u = cross(q, t);
    let translation = v3(translation);
    [0, 1, 2].map(|i| p[i] + w * t[i] + u[i] + translation[i])
}

/// The world for the chunk scene: terrain and chunk layers in broad-phase layer 0, the item
/// layer in 1; items collide with terrain, chunks and each other. No world gravity: the caller
/// pulls each item towards the planet centre.
fn chunk_world(worker_threads: u32) -> (PhysicsWorld, [ObjectLayer; 3]) {
    let mut layers = CollisionLayers::new(2);
    let terrain = layers.add_object_layer(BroadPhaseLayer::new(0));
    let chunk = layers.add_object_layer(BroadPhaseLayer::new(0));
    let item = layers.add_object_layer(BroadPhaseLayer::new(1));
    layers
        .enable_collision(item, terrain)
        .enable_collision(item, chunk)
        .enable_collision(item, item);
    let settings = jobs::with_threads(
        WorldSettings::default().gravity(Vec3::ZERO).layers(layers),
        worker_threads,
    );
    (PhysicsWorld::new(settings).unwrap(), [terrain, chunk, item])
}

/// What one compound child is made of, as the caller authored it.
#[derive(Clone, Copy)]
enum ChildKind {
    Box { half_extent: Vec3 },
    Cylinder { half_height: f32, radius: f32 },
}

/// One authored child of the chunk's static compound.
struct ChildSpec {
    kind: ChildKind,
    convex_radius: f32,
    position: Vec3,
    rotation: Quat,
    group: u32,
}

/// The chunk's children in attach order. The chunk body is turned 90 degrees about +y, so local
/// `(x, y, z)` is world `(z, y, -x)`.
fn chunk_children() -> [ChildSpec; 3] {
    [
        // A block whose top is at y = 3, centred on world (0, 2.5, -3): items 1 and 2 land on it.
        ChildSpec {
            kind: ChildKind::Box {
                half_extent: Vec3::new(1.0, 0.5, 1.0),
            },
            position: Vec3::new(3.0, 2.5, 0.0),
            rotation: Quat::IDENTITY,
            convex_radius: CHILD_CONVEX_RADIUS,
            group: Groups::STRUCTURE,
        },
        ChildSpec {
            kind: ChildKind::Box {
                half_extent: Vec3::new(0.8, 0.3, 0.8),
            },
            position: Vec3::new(-3.0, 2.6, 2.0),
            rotation: quat_about(Vec3::new(0.0, 0.0, 1.0), 20.0_f32.to_radians()),
            convex_radius: CHILD_CONVEX_RADIUS,
            group: Groups::STRUCTURE,
        },
        // A pillar centred on world (-3, 3, 0) whose top is at y = 4: item 3 hits its edge.
        ChildSpec {
            kind: ChildKind::Cylinder {
                half_height: 1.0,
                radius: 0.4,
            },
            position: Vec3::new(0.0, 3.0, -3.0),
            rotation: Quat::IDENTITY,
            convex_radius: CHILD_CONVEX_RADIUS,
            group: Groups::FEATURE,
        },
    ]
}

/// The chunk: a static compound keyed by cube face and grid cell.
struct Chunk {
    key: (u8, i32, i32),
    body: BodyId,
    children: [ChildSpec; 3],
}

fn create_chunk(world: &mut PhysicsWorld, layer: ObjectLayer) -> Chunk {
    let children = chunk_children();
    let shapes: Vec<Shape> = children
        .iter()
        .map(|child| match child.kind {
            ChildKind::Box { half_extent } => {
                Shape::new_box_with_convex_radius(half_extent, child.convex_radius).unwrap()
            }
            ChildKind::Cylinder {
                half_height,
                radius,
            } => Shape::new_cylinder_with_convex_radius(half_height, radius, child.convex_radius)
                .unwrap(),
        })
        .collect();
    let compound_children: Vec<CompoundChild<'_>> = children
        .iter()
        .zip(&shapes)
        .map(|(child, shape)| CompoundChild {
            shape,
            position: child.position,
            rotation: child.rotation,
            user_data: child.group,
        })
        .collect();
    let shape = Shape::new_compound(&compound_children).unwrap();
    let body = world
        .create_body(
            &shape,
            &BodySettings::new_static()
                .rotation(quat_about(Vec3::new(0.0, 1.0, 0.0), 90.0_f32.to_radians()))
                .object_layer(layer),
        )
        .unwrap();
    Chunk {
        key: (0, 0, 0),
        body,
        children,
    }
}

/// The terrain under the chunk: a sloped, ridged heightfield as the caller authored it.
struct Terrain {
    body: BodyId,
    shape: Shape,
    samples: Vec<f32>,
    settings_offset: Vec3,
    settings_scale: Vec3,
}

fn create_terrain(world: &mut PhysicsWorld, layer: ObjectLayer) -> Terrain {
    let n = TERRAIN_SAMPLES as usize;
    let mut samples = Vec::with_capacity(n * n);
    for z in 0..n {
        for x in 0..n {
            let slope = 0.05 * (z as f32 - 16.0);
            let ridges = 0.04 * ((7 * x + 3 * z) % 5) as f32;
            samples.push(SURFACE + slope + ridges);
        }
    }
    let settings_offset = Vec3::new(-16.0, 0.0, -16.0);
    let settings_scale = Vec3::new(1.0, 1.0, 1.0);
    let settings = HeightFieldSettings::default()
        .offset(settings_offset)
        .scale(settings_scale);
    let shape = Shape::new_height_field(TERRAIN_SAMPLES, &samples, &settings).unwrap();
    let body = add_static_in(world, &shape, RVec3::ZERO, layer);
    Terrain {
        body,
        shape,
        samples,
        settings_offset,
        settings_scale,
    }
}

/// Where an item is in its life.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Phase {
    Pending,
    Live(BodyId),
    Removed,
}

/// Why an item was removed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Reason {
    None = 0,
    Rest = 1,
    Timeout = 2,
}

/// A dropped item, keyed by the caller.
struct Item {
    key: u64,
    position: RVec3,
    rotation: Quat,
    /// The tick before whose step the item is created.
    created_at: u32,
    phase: Phase,
    calm_ticks: u32,
    age: u32,
    reason: Reason,
    /// The item's last [`record_body`] bytes, taken just before its removal.
    last_body: Vec<u8>,
}

/// The three items in key order.
fn items() -> Vec<Item> {
    let item = |key, position, rotation, created_at| Item {
        key,
        position,
        rotation,
        created_at,
        phase: Phase::Pending,
        calm_ticks: 0,
        age: 0,
        reason: Reason::None,
        last_body: Vec::new(),
    };
    vec![
        // One centimetre above the chunk's block.
        item(1, RVec3::new(0.0, 3.07, -3.0), Quat::IDENTITY, 1),
        // Across item 1, dropped onto it.
        item(
            2,
            RVec3::new(0.0, 3.6, -3.0),
            quat_about(Vec3::new(0.0, 1.0, 0.0), 90.0_f32.to_radians()),
            1,
        ),
        // High above the edge of the pillar's top.
        item(
            3,
            RVec3::new(-2.8, 12.0, 0.1),
            Quat::IDENTITY,
            LATE_ITEM_TICK + 1,
        ),
    ]
}

fn create_item(world: &mut PhysicsWorld, layer: ObjectLayer, item: &mut Item) {
    let shape = Shape::new_box(ITEM_HALF_EXTENT).unwrap();
    let body = world
        .create_body(
            &shape,
            &BodySettings::new_dynamic()
                .position(item.position)
                .rotation(item.rotation)
                .mass(ITEM_MASS)
                .friction(0.8)
                .restitution(0.1)
                .motion_quality(MotionQuality::LinearCast)
                .object_layer(layer),
        )
        .unwrap();
    item.phase = Phase::Live(body);
}

/// The world-space bounds `(min, max)` of a live item: its box turned by the body's rotation,
/// as Jolt's `AABox::Transformed` computes them.
fn item_bounds(world: &PhysicsWorld, id: BodyId) -> (V3, V3) {
    let body = world.body(id).unwrap();
    let centre = v3(body.position());
    let half = <[f32; 3]>::from(ITEM_HALF_EXTENT).map(f64::from);
    let rotation = body.rotation();
    let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .map(|axis| map_point(rotation, RVec3::ZERO, axis));
    let reach: V3 = [0, 1, 2].map(|i| (0..3).map(|a| axes[a][i].abs() * half[a]).sum());
    (
        [0, 1, 2].map(|i| centre[i] - reach[i]),
        [0, 1, 2].map(|i| centre[i] + reach[i]),
    )
}

/// Whether two bounds overlap, touching included, as Jolt's `AABox::Overlaps`.
fn overlap((a_min, a_max): (V3, V3), (b_min, b_max): (V3, V3)) -> bool {
    (0..3).all(|i| a_min[i] <= b_max[i] && b_min[i] <= a_max[i])
}

fn put_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn put_f32s(out: &mut Vec<u8>, values: impl IntoIterator<Item = f32>) {
    for value in values {
        out.extend_from_slice(&value.to_bits().to_le_bytes());
    }
}

/// The body's raw id, then its position (`Real` bits) and rotation.
fn put_body_pose(world: &PhysicsWorld, id: BodyId, out: &mut Vec<u8>) {
    let body = world.body(id).unwrap();
    put_u32(out, id.to_raw());
    for value in <[Real; 3]>::from(body.position()) {
        out.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    put_f32s(out, <[f32; 4]>::from(body.rotation()));
}

/// The shape section of one tick: the chunk's key, anchor and children, then the terrain's
/// samples as authored and as Jolt stores them.
fn record_shape(world: &PhysicsWorld, chunk: &Chunk, terrain: &Terrain, out: &mut Vec<u8>) {
    let (face, x, z) = chunk.key;
    out.push(face);
    out.extend_from_slice(&x.to_le_bytes());
    out.extend_from_slice(&z.to_le_bytes());
    put_body_pose(world, chunk.body, out);
    put_u32(out, chunk.children.len() as u32);
    for (index, child) in chunk.children.iter().enumerate() {
        // The attach index is the child's identity, as in `CompoundSubShape::index`.
        put_u32(out, index as u32);
        match child.kind {
            ChildKind::Box { half_extent } => {
                out.push(0);
                put_f32s(out, <[f32; 3]>::from(half_extent));
            }
            ChildKind::Cylinder {
                half_height,
                radius,
            } => {
                out.push(1);
                put_f32s(out, [half_height, radius]);
            }
        }
        put_f32s(out, [child.convex_radius]);
        put_f32s(out, <[f32; 3]>::from(child.position));
        put_f32s(out, <[f32; 4]>::from(child.rotation));
        put_u32(out, child.group);
    }

    put_body_pose(world, terrain.body, out);
    put_u32(out, TERRAIN_SAMPLES);
    put_f32s(out, <[f32; 3]>::from(terrain.settings_offset));
    put_f32s(out, <[f32; 3]>::from(terrain.settings_scale));
    put_f32s(out, terrain.samples.iter().copied());
    for y in 0..TERRAIN_SAMPLES {
        for x in 0..TERRAIN_SAMPLES {
            match terrain.shape.height_field_position(x, y) {
                None => out.push(0),
                Some(position) => {
                    out.push(1);
                    put_f32s(out, <[f32; 3]>::from(position));
                }
            }
        }
    }
}

/// The state section of one tick: chunk and terrain bodies, then every item slot in key order.
fn record_state(world: &PhysicsWorld, chunk: &Chunk, terrain: &Terrain, items: &[Item]) -> Vec<u8> {
    let mut out = Vec::new();
    record_body(world, chunk.body, &mut out);
    record_body(world, terrain.body, &mut out);
    for item in items {
        out.extend_from_slice(&item.key.to_le_bytes());
        let tag = match item.phase {
            Phase::Pending => 0,
            Phase::Live(_) => 1,
            Phase::Removed => 2,
        };
        out.push(tag);
        if item.phase == Phase::Pending {
            continue;
        }
        put_u32(&mut out, item.calm_ticks);
        put_u32(&mut out, item.age);
        out.push(item.reason as u8);
        match item.phase {
            Phase::Live(id) => record_body(world, id, &mut out),
            _ => out.extend_from_slice(&item.last_body),
        }
    }
    out
}

/// Runs the chunk scene and returns its records for ticks `0..=CHUNK_TICKS`.
///
/// A static compound chunk, a heightfield terrain and three items that the caller pulls towards
/// the planet centre. Items follow the game's lifecycle: an item that stays calm for
/// [`REST_TICKS`] ticks in a row, or reaches [`ITEM_TIMEOUT`] ticks of age, is removed. Item 3
/// is created late, so it is falling when the world moves into a tilted frame after tick
/// [`REBASE_TICK`]. `"forward"` creates chunk, terrain, item 1 and item 2 in that order;
/// `"permuted"` creates them in reverse. Both record in key order.
///
/// Panics unless the run covers what the gate is for: every step complete, item 3 falling at
/// the rebase in a reused body slot, all three items created, and an item removed at rest while
/// a live item's bounds overlap its own, so the removal's wake has a dynamic neighbour to find.
fn run_chunk(worker_threads: u32, variant: &str) -> Digest {
    let (mut world, [terrain_layer, chunk_layer, item_layer]) = chunk_world(worker_threads);
    let mut items = items();
    let (chunk, terrain) = match variant {
        "forward" => {
            let chunk = create_chunk(&mut world, chunk_layer);
            let terrain = create_terrain(&mut world, terrain_layer);
            for item in items.iter_mut().filter(|item| item.created_at == 1) {
                create_item(&mut world, item_layer, item);
            }
            (chunk, terrain)
        }
        "permuted" => {
            for item in items.iter_mut().rev().filter(|item| item.created_at == 1) {
                create_item(&mut world, item_layer, item);
            }
            let terrain = create_terrain(&mut world, terrain_layer);
            let chunk = create_chunk(&mut world, chunk_layer);
            (chunk, terrain)
        }
        variant => panic!("unknown chunk variant {variant}"),
    };
    let mut centre = [0.0, f64::from(SURFACE) - PLANET_RADIUS, 0.0];

    let mut digest = Digest::new();
    let record = |world: &PhysicsWorld, items: &[Item], digest: &mut Digest| {
        let tick = digest.push();
        record_shape(world, &chunk, &terrain, &mut tick.shape);
        tick.state = record_state(world, &chunk, &terrain, items);
    };
    record(&world, &items, &mut digest);

    let mut rested_touching_a_live_item = false;
    for tick in 1..=CHUNK_TICKS {
        for item in items.iter_mut() {
            if item.created_at == tick && item.phase == Phase::Pending {
                create_item(&mut world, item_layer, item);
            }
        }
        if tick == REBASE_TICK + 1 {
            let Phase::Live(late) = items[2].phase else {
                panic!("item 3 is not live at the rebase");
            };
            // Items 1 and 2 have come to rest and gone, so item 3 reuses a freed body index.
            assert_eq!(late.sequence(), 2, "item 3 got a fresh body index");
            let speed = length(world.body(late).unwrap().linear_velocity());
            assert!(
                speed > 0.5,
                "item 3 moves at only {speed} m/s at the rebase"
            );
            let mut bodies = vec![chunk.body, terrain.body];
            bodies.extend(items.iter().filter_map(|item| match item.phase {
                Phase::Live(id) => Some(id),
                _ => None,
            }));
            let (rotation, translation) = (tilted_rotation(), tilted_translation());
            world.rebase(&bodies, rotation, translation).unwrap();
            centre = map_point(rotation, translation, centre);
        }
        for item in &items {
            let Phase::Live(id) = item.phase else {
                continue;
            };
            let position = v3(world.body(id).unwrap().position());
            let towards: V3 = [0, 1, 2].map(|i| centre[i] - position[i]);
            let distance =
                (towards[0] * towards[0] + towards[1] * towards[1] + towards[2] * towards[2])
                    .sqrt();
            let pull = towards.map(|c| f64::from(ITEM_MASS * GRAVITY) * c / distance);
            let mut body = world.body_mut(id).unwrap();
            body.reset_forces();
            body.add_force(Vec3::new(pull[0] as f32, pull[1] as f32, pull[2] as f32))
                .unwrap();
        }
        assert!(world.step(DT).unwrap().is_complete(), "tick {tick}");

        let mut leaving = Vec::new();
        for (slot, item) in items.iter_mut().enumerate() {
            let Phase::Live(id) = item.phase else {
                continue;
            };
            let calm = is_calm(&world.body(id).unwrap());
            item.calm_ticks = if calm { item.calm_ticks + 1 } else { 0 };
            item.age += 1;
            if item.calm_ticks == REST_TICKS {
                item.reason = Reason::Rest;
                leaving.push(slot);
            } else if item.age == ITEM_TIMEOUT {
                item.reason = Reason::Timeout;
                leaving.push(slot);
            }
        }
        for slot in leaving {
            let Phase::Live(id) = items[slot].phase else {
                unreachable!("only live items leave");
            };
            let bounds = item_bounds(&world, id);
            let touches_a_live_item = items.iter().any(|item| match item.phase {
                Phase::Live(other) => other != id && overlap(bounds, item_bounds(&world, other)),
                _ => false,
            });
            if items[slot].reason == Reason::Rest && touches_a_live_item {
                rested_touching_a_live_item = true;
            }
            let mut last_body = Vec::new();
            record_body(&world, id, &mut last_body);
            world.remove_body(id).unwrap();
            let item = &mut items[slot];
            item.last_body = last_body;
            item.phase = Phase::Removed;
        }
        record(&world, &items, &mut digest);
    }

    assert!(
        items.iter().all(|item| item.phase != Phase::Pending),
        "an item was never created"
    );
    assert!(
        rested_touching_a_live_item,
        "no item came to rest touching a live item"
    );
    digest
}

/// Ticks of the walker scene.
const WALKER_TICKS: usize = 600;

/// The walker scene: the scripted walker with three items dropped along its path. Per tick the
/// walker moves, the items get their radial gravity, the world steps, and the walker's state
/// and contacts and the items' states are recorded.
fn run_walker(worker_threads: u32) -> Digest {
    let (mut world, layers) = script_scene(worker_threads);
    let walker = add_walker(&mut world, &layers, script_start());
    let item_shape = Shape::new_box(ITEM_HALF_EXTENT).unwrap();
    let items: Vec<BodyId> = [[-4.0, 2.0, 1.3], [-1.0, 2.5, 0.7], [0.5, 3.0, -1.2]]
        .into_iter()
        .map(|position| {
            world
                .create_body(
                    &item_shape,
                    &BodySettings::new_dynamic()
                        .position(rvec3(position))
                        .mass(ITEM_MASS)
                        .friction(0.8)
                        .restitution(0.1)
                        .motion_quality(MotionQuality::LinearCast)
                        .object_layer(layers.item),
                )
                .unwrap()
        })
        .collect();
    let mut player = Player::new(&mut world, &walker);
    let mut digest = Digest::new();
    for tick in 0..WALKER_TICKS {
        let out = script_tick(&mut world, &walker, &mut player, tick);
        for &item in &items {
            let position = v3(world.body(item).unwrap().position());
            let gravity = vec3(scale(up_at(position), -f64::from(ITEM_MASS * GRAVITY)));
            let mut body = world.body_mut(item).unwrap();
            body.reset_forces();
            body.add_force(gravity).unwrap();
        }
        assert!(world.step(DT).unwrap().is_complete());
        let record = digest.push();
        record_walker(&world, &walker, &out, &mut record.state);
        for &item in &items {
            record_body(&world, item, &mut record.state);
        }
    }
    digest
}

/// Ticks of the constraints scene.
const CONSTRAINT_TICKS: usize = 240;
/// The tick at which the constraints scene removes its slider and creates it again.
const CONSTRAINT_EVENT_TICK: usize = 120;

/// The constraints scene: a chain of six hinged links swinging from a static anchor, a slider
/// driven by a velocity motor, a gear pair driven through its first hinge, a pulley with two
/// hanging boxes and a box sliding down a path, stepped for 240 ticks. At tick 120 the slider is
/// removed and created again. Every dynamic body is recorded on every tick.
fn run_constraints(worker_threads: u32) -> Digest {
    let x = Vec3::new(1.0, 0.0, 0.0);
    let y = Vec3::new(0.0, 1.0, 0.0);
    let z = Vec3::new(0.0, 0.0, 1.0);
    let mut world = world(Vec3::new(0.0, -9.81, 0.0), worker_threads);
    let small = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    let anchor = |world: &mut PhysicsWorld, at: [f64; 3]| {
        world
            .create_body(&small, &BodySettings::new_static().position(rvec3(at)))
            .unwrap()
    };
    let dynamic = |world: &mut PhysicsWorld, shape: &Shape, at: [f64; 3]| {
        world
            .create_body(shape, &BodySettings::new_dynamic().position(rvec3(at)))
            .unwrap()
    };
    let mut bodies = Vec::new();

    // The chain: links of 0.8 m along x, each hinged about z to the one before.
    let link = Shape::new_box(Vec3::new(0.4, 0.05, 0.05)).unwrap();
    let mut previous = anchor(&mut world, [0.0, 10.0, 0.0]);
    for i in 0..6 {
        let left = 0.9 * i as f64;
        let body = dynamic(&mut world, &link, [left + 0.45, 10.0, 0.0]);
        world
            .create_constraint(
                previous,
                body,
                &HingeConstraintSettings::new(rvec3([left, 10.0, 0.0]), z, x),
            )
            .unwrap();
        bodies.push(body);
        previous = body;
    }

    // The slider, along x, 10 m in front of the chain.
    let rail = anchor(&mut world, [0.0, 5.0, 10.0]);
    let carriage_shape = Shape::new_box(Vec3::new(0.3, 0.1, 0.1)).unwrap();
    let carriage = dynamic(&mut world, &carriage_shape, [2.0, 5.0, 10.0]);
    bodies.push(carriage);
    let slider = SliderConstraintSettings::new(rvec3([2.0, 5.0, 10.0]), x, y);
    let drive_slider = |world: &mut PhysicsWorld, id: ConstraintId<SliderConstraint>| {
        let mut motor = world.constraint_mut(id).unwrap();
        motor.set_target_velocity(1.0).unwrap();
        motor.set_motor_state(MotorState::Velocity);
    };
    let slider_id = world.create_constraint(rail, carriage, &slider).unwrap();
    drive_slider(&mut world, slider_id);

    // The gear pair, 20 m to the side, without gravity's help: discs about z.
    let base = anchor(&mut world, [20.0, 0.0, 0.0]);
    let disc = Shape::new_box(Vec3::new(0.5, 0.5, 0.1)).unwrap();
    let discs = [20.0, 23.0].map(|cx| dynamic(&mut world, &disc, [cx, 3.0, 0.0]));
    let hinges = [0, 1].map(|i| {
        let cx = [20.0, 23.0][i];
        world
            .create_constraint(
                base,
                discs[i],
                &HingeConstraintSettings::new(rvec3([cx, 3.0, 0.0]), z, x),
            )
            .unwrap()
    });
    world
        .create_constraint(
            discs[0],
            discs[1],
            &GearConstraintSettings::new(z, z, 2.0).constraints(hinges[0], hinges[1]),
        )
        .unwrap();
    let mut motor = world.constraint_mut(hinges[0]).unwrap();
    motor.set_target_angular_velocity(3.0).unwrap();
    motor.set_motor_state(MotorState::Velocity);
    bodies.extend(discs);

    // The pulley, 20 m to the other side.
    let crate_shape = Shape::new_box(Vec3::new(0.2, 0.2, 0.2)).unwrap();
    let pair = [(-21.0, 2.0), (-19.0, 1.0)].map(|(px, mass)| {
        world
            .create_body(
                &crate_shape,
                &BodySettings::new_dynamic()
                    .mass(mass)
                    .position(rvec3([px, 2.0, 0.0])),
            )
            .unwrap()
    });
    world
        .create_constraint(
            pair[0],
            pair[1],
            &PulleyConstraintSettings::new(
                rvec3([-21.0, 2.0, 0.0]),
                rvec3([-21.0, 5.0, 0.0]),
                rvec3([-19.0, 2.0, 0.0]),
                rvec3([-19.0, 5.0, 0.0]),
            ),
        )
        .unwrap();
    bodies.extend(pair);

    // The path: a descending S curve 20 m behind the chain.
    let point = |position: [f32; 3], tangent: [f32; 3]| HermitePathPoint {
        position: Vec3::from(position),
        tangent: Vec3::from(tangent),
    };
    let path = HermitePath::new(
        z,
        vec![
            point([0.0, 0.0, 0.0], [1.0, -0.3, 0.0]),
            point([1.0, -0.3, 0.0], [1.0, -0.5, 0.0]),
            point([2.0, -0.9, 0.0], [1.0, -0.5, 0.0]),
            point([3.0, -1.2, 0.0], [1.0, -0.3, 0.0]),
        ],
        false,
    )
    .unwrap();
    let track = anchor(&mut world, [0.0, 5.0, -20.0]);
    let sled = dynamic(&mut world, &small, [0.5, 5.0, -20.0]);
    world
        .create_constraint(
            track,
            sled,
            &PathConstraintSettings::new(path).path_position(Vec3::new(0.5, 0.0, 0.0)),
        )
        .unwrap();
    bodies.push(sled);

    let mut digest = Digest::new();
    let mut slider_id = slider_id;
    for tick in 0..CONSTRAINT_TICKS {
        if tick == CONSTRAINT_EVENT_TICK {
            world.remove_constraint(slider_id).unwrap();
            slider_id = world.create_constraint(rail, carriage, &slider).unwrap();
            drive_slider(&mut world, slider_id);
        }
        assert!(world.step(DT).unwrap().is_complete());
        let record = digest.push();
        for &body in &bodies {
            record_body(&world, body, &mut record.state);
        }
    }
    digest
}

#[test]
#[ignore = "child process of the determinism gates"]
fn determinism_child() {
    let Some((scenario, threads, variant)) = child_request() else {
        return;
    };
    let job_choice = JobChoice::from_env();
    let digest = match scenario.as_str() {
        "stacks" => run_stacks(threads, &variant),
        "pile" => run_pile(threads),
        "chunk" => run_chunk(threads, &variant),
        "walker" => run_walker(threads),
        "vehicle" => run_vehicle(threads),
        "fleet" => run_fleet(threads, &variant),
        "ragdoll_pile" => run_ragdoll_pile(threads),
        "constraints" => run_constraints(threads),
        "soft_bodies" => run_soft_bodies(threads),
        scenario => panic!("unknown scenario {scenario}"),
    };
    // Without these checks a choice that never reached the worlds, or a Rayon pool that never
    // ran a job, would pass as the native run: Jolt's update barrier runs every job itself.
    match job_choice {
        JobChoice::Native | JobChoice::SingleThreaded => {
            assert_eq!(jobs::queued(), 0, "a caller job system was used")
        }
        _ => assert!(
            jobs::queued() > 0,
            "the {job_choice:?} job system was handed no job"
        ),
    }
    // Only the pile is busy enough: in the smaller scenes a loaded machine's stepping thread may
    // run every job before a pool thread is scheduled. Even the pile can miss the pool in one run
    // when the child has a single core, so fresh piles are stepped, not recorded, until the pool
    // executed a job or the retries run out; the digest stays the first run's.
    if job_choice == JobChoice::Rayon && scenario == "pile" {
        for _ in 0..PILE_POOL_RETRIES {
            if jobs::queued_from_pool_jobs() > 0 {
                break;
            }
            run_pile(threads);
        }
        assert!(
            jobs::queued_from_pool_jobs() > 0,
            "the Rayon pool executed no Jolt job"
        );
    }
    finish_child(&digest);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn stacks_digest_is_identical_with_caller_job_systems() {
    assert_caller_job_systems_agree("determinism_child", "stacks", "forward");
    assert_caller_job_systems_agree("determinism_child", "stacks", "rebased");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn pile_digest_is_identical_with_caller_job_systems() {
    // The Rayon child also asserts that its pool executed Jolt jobs.
    assert_caller_job_systems_agree("determinism_child", "pile", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn chunk_digest_is_identical_with_caller_job_systems() {
    assert_caller_job_systems_agree("determinism_child", "chunk", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn walker_digest_is_identical_with_caller_job_systems() {
    assert_caller_job_systems_agree("determinism_child", "walker", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn vehicle_digest_is_identical_with_caller_job_systems() {
    assert_caller_job_systems_agree("determinism_child", "vehicle", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn fleet_digest_is_identical_with_caller_job_systems() {
    // The 40 vehicle listeners run in 2, 5 and 3 jobs (`PhysicsSystem.cpp:243`).
    assert_caller_job_systems_agree("determinism_child", "fleet", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn ragdoll_pile_digest_is_identical_with_caller_job_systems() {
    assert_caller_job_systems_agree("determinism_child", "ragdoll_pile", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn constraint_digest_is_identical_with_caller_job_systems() {
    assert_caller_job_systems_agree("determinism_child", "constraints", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn soft_body_digest_is_identical_with_caller_job_systems() {
    assert_caller_job_systems_agree("determinism_child", "soft_bodies", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn stacks_digest_is_identical_with_the_single_threaded_job_system() {
    for variant in ["forward", "rebased", "sub_stepped"] {
        assert_single_threaded_agrees("determinism_child", "stacks", variant);
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn pile_digest_is_identical_with_the_single_threaded_job_system() {
    assert_single_threaded_agrees("determinism_child", "pile", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn chunk_digest_is_identical_with_the_single_threaded_job_system() {
    assert_single_threaded_agrees("determinism_child", "chunk", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn walker_digest_is_identical_with_the_single_threaded_job_system() {
    assert_single_threaded_agrees("determinism_child", "walker", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn vehicle_digest_is_identical_with_the_single_threaded_job_system() {
    assert_single_threaded_agrees("determinism_child", "vehicle", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn fleet_digest_is_identical_with_the_single_threaded_job_system() {
    for variant in ["forward", "sub_stepped"] {
        assert_single_threaded_agrees("determinism_child", "fleet", variant);
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn ragdoll_pile_digest_is_identical_with_the_single_threaded_job_system() {
    assert_single_threaded_agrees("determinism_child", "ragdoll_pile", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn constraint_digest_is_identical_with_the_single_threaded_job_system() {
    assert_single_threaded_agrees("determinism_child", "constraints", "forward");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn soft_body_digest_is_identical_with_the_single_threaded_job_system() {
    assert_single_threaded_agrees("determinism_child", "soft_bodies", "forward");
}

fn stacks_in_child(threads: u32, variant: &str) -> Digest {
    digest_in_child("determinism_child", "stacks", threads, variant)
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn stacks_digest_is_identical_across_thread_counts_with_collision_steps() {
    let one_thread = stacks_in_child(1, "sub_stepped");
    let four_threads = stacks_in_child(4, "sub_stepped");
    assert_eq!(one_thread.ticks.len(), TICKS);
    assert_same(
        "sub-stepped stacks, 1 vs 4 worker threads",
        &one_thread,
        &four_threads,
    );
    // The collision step count reached the child: sub-stepping changes the run.
    assert!(first_divergence(&one_thread, &stacks_in_child(1, "forward")).is_some());
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn stacks_digest_is_identical_across_thread_counts() {
    let one_thread = stacks_in_child(1, "forward");
    let four_threads = stacks_in_child(4, "forward");
    let reversed = stacks_in_child(1, "reversed");
    let rebased_one_thread = stacks_in_child(1, "rebased");
    let rebased_four_threads = stacks_in_child(4, "rebased");

    assert_eq!(one_thread.ticks.len(), TICKS);
    assert_eq!(
        one_thread.ticks[0].state.len(),
        stacks_body_count() * BODY_RECORD_SIZE
    );
    assert_same("stacks, 1 vs 4 worker threads", &one_thread, &four_threads);
    assert_same(
        "rebased stacks, 1 vs 4 worker threads",
        &rebased_one_thread,
        &rebased_four_threads,
    );
    // The rebase happened: the rebased run leaves the forward run's frame.
    assert!(first_divergence(&rebased_one_thread, &one_thread).is_some());

    // Insertion order is part of the state: reversing creation reverses the ids of the same
    // bodies, so the digest changes.
    let created_forward: Vec<u32> = (0..stacks_body_count()).map(nth_body_id).collect();
    let created_reversed: Vec<u32> = (0..stacks_body_count()).rev().map(nth_body_id).collect();
    assert_eq!(first_tick_ids(&one_thread), created_forward);
    assert_eq!(first_tick_ids(&reversed), created_reversed);
    assert!(first_divergence(&reversed, &one_thread).is_some());
}

fn chunk_in_child(threads: u32, variant: &str) -> Digest {
    digest_in_child("determinism_child", "chunk", threads, variant)
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn chunk_digest_is_identical_across_thread_counts() {
    let one_thread = chunk_in_child(1, "forward");
    let four_threads = chunk_in_child(4, "forward");
    assert_eq!(one_thread.ticks.len(), CHUNK_TICKS as usize + 1);
    for (tick, record) in one_thread.ticks.iter().enumerate() {
        assert!(!record.shape.is_empty(), "tick {tick}: empty shape record");
        assert!(!record.state.is_empty(), "tick {tick}: empty state record");
    }
    assert_same("chunk, 1 vs 4 worker threads", &one_thread, &four_threads);
}

fn walker_in_child(threads: u32) -> Digest {
    digest_in_child("determinism_child", "walker", threads, "forward")
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn walker_digest_is_identical_across_thread_counts() {
    let one_thread = walker_in_child(1);
    let four_threads = walker_in_child(4);
    assert_eq!(one_thread.ticks.len(), WALKER_TICKS);
    assert_same("walker, 1 vs 4 worker threads", &one_thread, &four_threads);
}

/// Offset of the chunk's raw body id in a shape record: after the key (`u8`, `i32`, `i32`).
const CHUNK_ID_IN_SHAPE: usize = 1 + 4 + 4;
/// Bytes of an item slot's header before its body record: key, phase, calm ticks, age and
/// reason.
const ITEM_HEADER_SIZE: usize = 8 + 1 + 4 + 4 + 1;

fn u32_at(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

/// The raw ids of chunk, terrain, item 1 and item 2 at tick 0, read from the state record, and
/// the chunk's id read from the shape record.
fn tick_zero_ids(digest: &Digest) -> ([u32; 4], u32) {
    let tick = &digest.ticks[0];
    let item_slot = ITEM_HEADER_SIZE + BODY_RECORD_SIZE;
    let ids = [
        0,
        BODY_RECORD_SIZE,
        2 * BODY_RECORD_SIZE + ITEM_HEADER_SIZE,
        2 * BODY_RECORD_SIZE + item_slot + ITEM_HEADER_SIZE,
    ]
    .map(|offset| u32_at(&tick.state, offset));
    (ids, u32_at(&tick.shape, CHUNK_ID_IN_SHAPE))
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn permuted_insertion_order_fails_the_gate() {
    let forward = chunk_in_child(1, "forward");
    let permuted = chunk_in_child(4, "permuted");
    let divergence =
        first_divergence(&forward, &permuted).expect("a permuted insertion order must diverge");
    assert!(
        matches!(divergence, Divergence::Tick { tick: 0, .. }),
        "{divergence}"
    );

    let (forward_ids, forward_chunk) = tick_zero_ids(&forward);
    let (permuted_ids, permuted_chunk) = tick_zero_ids(&permuted);
    assert_eq!(forward_ids, [0, 1, 2, 3].map(nth_body_id));
    assert_eq!(permuted_ids, [3, 2, 1, 0].map(nth_body_id));
    assert_eq!(forward_chunk, forward_ids[0]);
    assert_eq!(permuted_chunk, permuted_ids[0]);

    let gate = std::panic::catch_unwind(|| assert_same("permuted", &forward, &permuted));
    assert!(
        gate.is_err(),
        "the gate accepted a permuted insertion order"
    );
    println!("permuted insertion order diverges at {divergence}");
}

/// A digest of `ticks` ticks whose sections hold the given bytes.
fn digest_of(ticks: &[(&[u8], &[u8])]) -> Digest {
    let mut digest = Digest::new();
    for &(shape, state) in ticks {
        let tick = digest.push();
        tick.shape.extend_from_slice(shape);
        tick.state.extend_from_slice(state);
    }
    digest
}

#[test]
fn child_requests_are_parsed_or_rejected() {
    assert_eq!(
        parse_child_request("chunk,4,permuted"),
        ("chunk".to_owned(), 4, "permuted".to_owned())
    );
    for (request, expected) in [
        ("chunk", "has no threads field"),
        ("chunk,4", "has no variant field"),
        ("chunk,four,forward", "bad thread count"),
    ] {
        let panic = std::panic::catch_unwind(|| parse_child_request(request)).unwrap_err();
        let message = panic.downcast_ref::<String>().unwrap();
        assert!(message.contains(expected), "{request:?}: {message}");
    }
}

#[test]
fn digest_encoding_round_trips() {
    for digest in [
        Digest::new(),
        digest_of(&[(&[], &[])]),
        digest_of(&[(&[1, 2, 3], &[]), (&[], &[4]), (&[5, 6], &[7, 8, 9])]),
    ] {
        assert_eq!(Digest::decode(&digest.encode()), Ok(digest));
    }
}

#[test]
fn digest_decoding_rejects_malformed_input() {
    let bytes = digest_of(&[(&[1, 2, 3], &[4, 5])]).encode();
    // Tick count 1, shape length 3: offsets 0 and 4; the shape bytes start at offset 8.
    let truncated_length = &bytes[..6];
    let truncated_section = &bytes[..9];
    let mut trailing = bytes.clone();
    trailing.push(0);
    for (input, expected) in [
        (truncated_length, "truncated length at offset 4"),
        (truncated_section, "truncated section at offset 8"),
        (&trailing[..], "1 trailing bytes at offset 17"),
    ] {
        assert_eq!(Digest::decode(input), Err(expected.to_owned()));
    }
}

#[test]
fn first_divergence_names_tick_section_and_byte() {
    let base = digest_of(&[(&[1, 2], &[3, 4]), (&[5, 6], &[7, 8, 9])]);
    assert_eq!(first_divergence(&base, &base.clone()), None);

    let state = digest_of(&[(&[1, 2], &[3, 4]), (&[5, 6], &[7, 0, 9])]);
    let expected = Divergence::Tick {
        tick: 1,
        section: Section::State,
        byte: 1,
    };
    assert_eq!(first_divergence(&base, &state), Some(expected));
    assert_eq!(expected.to_string(), "tick 1, state byte 1");

    // The shape section of a tick is compared before its state section.
    let shape_and_state = digest_of(&[(&[1, 2], &[3, 4]), (&[5, 0], &[0, 8, 9])]);
    assert_eq!(
        first_divergence(&base, &shape_and_state),
        Some(Divergence::Tick {
            tick: 1,
            section: Section::Shape,
            byte: 1,
        })
    );

    let shorter = digest_of(&[(&[1, 2], &[3, 4]), (&[5, 6], &[7, 8])]);
    assert_eq!(
        first_divergence(&base, &shorter),
        Some(Divergence::Tick {
            tick: 1,
            section: Section::State,
            byte: 2,
        })
    );

    let fewer_ticks = digest_of(&[(&[1, 2], &[3, 4])]);
    let expected = Divergence::TickCount { a: 2, b: 1 };
    assert_eq!(first_divergence(&base, &fewer_ticks), Some(expected));
    assert_eq!(expected.to_string(), "tick counts 2 and 1");
}

#[test]
fn assert_same_panics_with_the_divergence() {
    let a = digest_of(&[(&[1], &[2])]);
    let b = digest_of(&[(&[1], &[3])]);
    assert_same("equal", &a, &a.clone());
    let payload = std::panic::catch_unwind(|| assert_same("runs", &a, &b)).unwrap_err();
    assert_eq!(
        payload.downcast_ref::<String>().map(String::as_str),
        Some("runs: digests diverge at tick 0, state byte 0")
    );
}

/// Runs the acceptance route of `tests/vehicle.rs` and records the vehicle after every tick.
fn run_vehicle(worker_threads: u32) -> Digest {
    let mut digest = Digest::new();
    let report = vehicle::drive_route(worker_threads, |world, car| {
        vehicle::record_vehicle(world, car, &mut digest.push().state);
    });
    assert_eq!(report.reached.len(), vehicle::WAYPOINTS.len(), "{report:?}");
    assert!(report.all_complete, "{report:?}");
    digest
}

/// Ticks of the fleet scene.
const FLEET_TICKS: usize = 300;
/// The tick at which the fleet scene removes a vehicle and the nudged variant changes an input.
const FLEET_EVENT_TICK: usize = 150;
/// Key of the vehicle the fleet scene removes.
const REMOVED_VEHICLE: usize = 3;
/// Cars on the open terrain, the keys before the slab cars.
const OPEN_CARS: usize = 32;
/// Cars on the shared slab.
const SLAB_CARS: usize = 4;
/// Key of the carrier vehicle, after the open and slab cars; the parked car follows it, then
/// the two seam cars.
const CARRIER: usize = OPEN_CARS + SLAB_CARS;
const PARKED_CAR: usize = CARRIER + 1;
const SEAM_CARS: [usize; 2] = [CARRIER + 2, CARRIER + 3];
/// Ticks on which each left seam wheel must stand across the seam on each of the two seam
/// boxes. Which box wins depends on hits a few ulps apart, so the split differs between
/// precisions and compilers (17 of 300 ticks on one box in a Windows double-precision build).
const MIN_SEAM_BOX_TICKS: usize = FLEET_TICKS / 30;
/// x of the seam between the two boxes the seam cars drive on.
const SEAM_X: f32 = -80.0;
/// Convex radius of the cylinder tester of the seam cars: Jolt's default fraction 0.1 of
/// `min(width / 2, radius)`.
const SEAM_CYLINDER_CONVEX_RADIUS: f32 = 0.1 * vehicle::WHEEL_WIDTH / 2.0;

/// The vehicles of the fleet scene with what the gate checks about them.
struct Fleet {
    world: PhysicsWorld,
    /// Chassis bodies in key order.
    chassis: Vec<BodyId>,
    /// Vehicles in key order; the removed one becomes `None`.
    vehicles: Vec<Option<VehicleId>>,
    slab: BodyId,
    /// The two abutting coplanar boxes under the seam cars, lower x first. The seam between
    /// them runs under the cars' left wheels.
    seam_boxes: [BodyId; 2],
    /// A twin of the lower seam box at its exact pose, under the seam cars' right wheels.
    seam_twin: BodyId,
    /// The object layer of the static ground.
    ground: ObjectLayer,
}

/// The carrier: a 10 m flatbed vehicle with a low centre of mass.
fn carrier_settings(layers: &vehicle::CarLayers) -> (Shape, WheeledVehicleSettings) {
    let hull = Shape::new_box(Vec3::new(2.5, 0.3, 5.0)).unwrap();
    let shape = Shape::new_offset_center_of_mass(&hull, Vec3::new(0.0, -0.3, 0.0)).unwrap();
    let wheels = [(2.3, 4.0), (-2.3, 4.0), (2.3, -4.0), (-2.3, -4.0)]
        .into_iter()
        .map(|(x, z)| {
            WheelSettings::new(Vec3::new(x, -0.1, z))
                .radius(vehicle::WHEEL_RADIUS)
                .width(vehicle::WHEEL_WIDTH)
                .suspension_min_length(vehicle::SUSPENSION_MIN)
                .suspension_max_length(vehicle::SUSPENSION_MAX)
                .max_steer_angle(0.0)
        })
        .collect();
    let settings = WheeledVehicleSettings::new(
        wheels,
        vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
        VehicleCollisionTester::ray(layers.probe),
    );
    (shape, settings)
}

/// Builds the fleet scene: a flat terrain, the seam boxes and their twin, and the slab, then the
/// vehicles in key order: 32 cars on the open terrain, 4 cars on the shared dynamic slab, the
/// carrier, the car parked on the carrier's deck and the two cylinder-tester cars on the seam.
fn build_fleet(worker_threads: u32) -> Fleet {
    let (mut world, layers) = vehicle::car_world(Vec3::ZERO, worker_threads);
    let samples = vec![0.0; 257 * 257];
    let terrain_settings = HeightFieldSettings::default().offset(Vec3::new(-128.0, 0.0, -128.0));
    let terrain = Shape::new_height_field(257, &samples, &terrain_settings).unwrap();
    let ground = |position| {
        BodySettings::new_static()
            .position(position)
            .object_layer(layers.ground)
    };
    world.create_body(&terrain, &ground(RVec3::ZERO)).unwrap();
    let seam_box = Shape::new_box(Vec3::new(5.0, 0.25, 25.0)).unwrap();
    let seam_box_at = |x: f32| ground(RVec3::new(x as Real, 0.25, 0.0));
    let seam_boxes = [SEAM_X - 5.0, SEAM_X + 5.0]
        .map(|x| world.create_body(&seam_box, &seam_box_at(x)).unwrap());
    let seam_twin = world
        .create_body(&seam_box, &seam_box_at(SEAM_X - 5.0))
        .unwrap();
    let slab = world
        .create_body(
            &Shape::new_box(Vec3::new(8.0, 0.2, 8.0)).unwrap(),
            &BodySettings::new_dynamic()
                .position(RVec3::new(0.0, 0.2, 40.0))
                .object_layer(layers.moving)
                .mass(20000.0),
        )
        .unwrap();

    let place = |world: &mut PhysicsWorld, x: f32, y: f32, z: f32, tester| {
        let body = vehicle::chassis_settings(
            &layers,
            RVec3::new(x as Real, y as Real, z as Real),
            Quat::IDENTITY,
        );
        vehicle::add_car_with(world, &body, tester)
    };
    let mut placed = Vec::new();
    let ray = VehicleCollisionTester::ray(layers.probe);
    for key in 0..OPEN_CARS {
        let (column, row) = ((key % 8) as f32, (key / 8) as f32);
        placed.push(place(
            &mut world,
            -42.0 + 12.0 * column,
            0.85,
            -70.0 + 14.0 * row,
            ray,
        ));
    }
    for (x, z) in [(-3.5, 36.0), (3.5, 36.0), (-3.5, 44.0), (3.5, 44.0)] {
        placed.push(place(&mut world, x, 0.4 + 0.85, z, ray));
    }
    let (carrier_shape, carrier) = carrier_settings(&layers);
    let carrier_body = world
        .create_body(
            &carrier_shape,
            &vehicle::chassis_settings(&layers, RVec3::new(60.0, 0.85, 20.0), Quat::IDENTITY)
                .mass(6000.0),
        )
        .unwrap();
    placed.push((
        carrier_body,
        world
            .create_wheeled_vehicle(carrier_body, &carrier)
            .unwrap(),
    ));
    placed.push(place(&mut world, 60.0, 0.85 + 0.3 + 0.85, 20.0, ray));
    let cylinder = VehicleCollisionTester::cast_cylinder(layers.probe);
    for z in [-15.0, 0.0] {
        placed.push(place(&mut world, SEAM_X - 0.9, 0.5 + 0.85, z, cylinder));
    }
    let chassis: Vec<BodyId> = placed.iter().map(|&(body, _)| body).collect();
    let vehicles: Vec<Option<VehicleId>> = placed.iter().map(|&(_, car)| Some(car)).collect();
    assert_eq!(vehicles.len(), 40);
    Fleet {
        world,
        chassis,
        vehicles,
        slab,
        seam_boxes,
        seam_twin,
        ground: layers.ground,
    }
}

/// The driver input of the fleet vehicle `key` at `tick`.
fn fleet_input(key: usize, tick: usize, nudged: bool) -> DriverInput {
    let phase = key as f32 + tick as f32 / 40.0;
    let mut input = match key {
        CARRIER => DriverInput {
            forward: 0.2,
            ..DriverInput::default()
        },
        PARKED_CAR => DriverInput {
            hand_brake: 1.0,
            ..DriverInput::default()
        },
        k if SEAM_CARS.contains(&k) => DriverInput {
            forward: 0.3,
            ..DriverInput::default()
        },
        k if k >= OPEN_CARS => DriverInput {
            forward: 0.2 * (tick as f32 / 25.0 + key as f32).sin(),
            ..DriverInput::default()
        },
        _ => DriverInput {
            forward: 0.2 + 0.2 * phase.sin(),
            right: 0.6 * (phase * 0.7).cos(),
            ..DriverInput::default()
        },
    };
    if nudged && key == 0 && tick == FLEET_EVENT_TICK {
        input.right = (input.right + 0.5).clamp(-1.0, 1.0);
    }
    input
}

/// Whether the seam wheel `wheel` of the car on `chassis` lies across the seam: both boxes
/// under its tread, outside the cylinder's rounded edges.
fn straddles_seam(world: &PhysicsWorld, chassis: BodyId, wheel: usize) -> bool {
    let body = world.body(chassis).unwrap();
    let offset = math::rotate(body.rotation(), math::f3(vehicle::WHEEL_POSITIONS[wheel]));
    let x = v3(body.position())[0] + offset[0];
    let limit = 0.5 * vehicle::WHEEL_WIDTH - SEAM_CYLINDER_CONVEX_RADIUS;
    (x - f64::from(SEAM_X)).abs() <= f64::from(limit)
}

/// Hamilton product `a * b`: the rotation `b` followed by `a`.
fn quat_mul(a: Quat, b: Quat) -> Quat {
    Quat::from_xyzw(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )
}

/// What the tie check of the seam twins needs: the world, its ground layer, the lower seam box,
/// its twin and a cylinder of the wheel's size with the tester's convex radius.
struct TwinProbe<'a> {
    world: &'a PhysicsWorld,
    ground: ObjectLayer,
    seam_box: BodyId,
    twin: BodyId,
    cylinder: &'a Shape,
}

impl TwinProbe<'_> {
    /// Whether the cylinder, cast down the suspension of wheel `wheel` of the car on `chassis`
    /// as Jolt's cylinder tester casts it, hits the lower seam box and its twin each alone at
    /// the same fraction: the two candidate hits of the tester tie exactly.
    fn twins_tie(&self, chassis: BodyId, wheel: usize) -> bool {
        let body = self.world.body(chassis).unwrap();
        let rotation = body.rotation();
        let attachment = math::rotate(rotation, math::f3(vehicle::WHEEL_POSITIONS[wheel]));
        let origin = math::rvec3(math::add(v3(body.position()), attachment));
        let travel = [0.0, -f64::from(vehicle::SUSPENSION_MAX), 0.0];
        let direction = math::vec3(math::rotate(rotation, travel));
        // The cylinder's axis turned onto the wheel's axle, body x for the unsteered seam
        // wheels.
        let axle = quat_mul(
            rotation,
            quat_about(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::FRAC_PI_2),
        );
        let cast = ShapeCast::new(self.cylinder, origin, axle, direction);
        let ground = [self.ground];
        let hit_without = |other: BodyId| {
            let filter = QueryFilter::new()
                .object_layers(&ground)
                .exclude_body(other);
            let hit = self.world.cast_shape(&cast, &filter).unwrap()?;
            Some((hit.body, hit.fraction.to_bits()))
        };
        match (hit_without(self.twin), hit_without(self.seam_box)) {
            (Some((first, first_fraction)), Some((second, second_fraction))) => {
                first == self.seam_box && second == self.twin && first_fraction == second_fraction
            }
            _ => false,
        }
    }
}

/// Runs the fleet scene and records every vehicle (or, once removed, its chassis) and the slab
/// after every tick, in key order. `"nudged"` changes car 0's steering on one tick;
/// `"sub_stepped"` steps with [`PhysicsWorld::MAX_COLLISION_STEPS`] collision steps.
///
/// The scene couples vehicles in the ways a step listener order could show: cars sharing a
/// dynamic slab as wheel ground, a car whose wheels stand on another vehicle's chassis, and
/// cylinder wheels on two coincident boxes, whose hits tie exactly and leave the choice to the
/// order of the broad-phase traversal. Cylinder wheels on the seam of two abutting coplanar
/// boxes add hits on two bodies a few ulps apart. The run asserts that each coupling actually
/// happens.
fn run_fleet(worker_threads: u32, variant: &str) -> Digest {
    let (nudged, collision_steps) = match variant {
        "forward" => (false, 1),
        "nudged" => (true, 1),
        "sub_stepped" => (false, PhysicsWorld::MAX_COLLISION_STEPS),
        variant => panic!("unknown fleet variant {variant}"),
    };
    let Fleet {
        mut world,
        chassis,
        mut vehicles,
        slab,
        seam_boxes,
        seam_twin,
        ground,
    } = build_fleet(worker_threads);
    let seam_cylinder = Shape::new_cylinder_with_convex_radius(
        0.5 * vehicle::WHEEL_WIDTH,
        vehicle::WHEEL_RADIUS,
        SEAM_CYLINDER_CONVEX_RADIUS,
    )
    .unwrap();
    let mut slab_ticks = [0_usize; SLAB_CARS];
    let mut carrier_ticks = 0;
    let mut seam_ticks = [[0_usize; 4]; SEAM_CARS.len()];
    // Per seam car and left wheel, the ticks it straddles the seam on the lower and on the upper
    // seam box.
    let mut seam_box_ticks = [[[0_usize; 2]; 4]; SEAM_CARS.len()];
    let mut digest = Digest::new();
    for tick in 0..FLEET_TICKS {
        if tick == FLEET_EVENT_TICK {
            let removed = vehicles[REMOVED_VEHICLE].take().unwrap();
            world.remove_vehicle(removed).unwrap();
        }
        for (key, car) in vehicles.iter().enumerate() {
            let Some(car) = *car else { continue };
            let mut vehicle = world.vehicle_mut(car).unwrap();
            vehicle.set_gravity(vehicle::GRAVITY).unwrap();
            vehicle
                .set_driver_input(fleet_input(key, tick, nudged))
                .unwrap();
        }
        assert!(
            world
                .step_with_collision_steps(DT, collision_steps)
                .unwrap()
                .is_complete(),
            "tick {tick}"
        );

        let state = &mut digest.push().state;
        for (key, car) in vehicles.iter().enumerate() {
            match car {
                Some(car) => vehicle::record_vehicle(&world, *car, state),
                None => record_body(&world, chassis[key], state),
            }
        }
        record_body(&world, slab, state);

        let contacts_on = |key: usize, body: BodyId| {
            let car = vehicles[key].unwrap();
            world
                .vehicle(car)
                .unwrap()
                .wheels()
                .iter()
                .filter(|wheel| wheel.contact.is_some_and(|contact| contact.body == body))
                .count()
        };
        for (i, ticks) in slab_ticks.iter_mut().enumerate() {
            if contacts_on(OPEN_CARS + i, slab) >= 2 {
                *ticks += 1;
            }
        }
        if contacts_on(PARKED_CAR, chassis[CARRIER]) >= 3 {
            carrier_ticks += 1;
        }
        // A left seam wheel counts when it stands on a seam box across the seam, a right one
        // when it stands on the lower box or its twin and their hits tie. Each left wheel must
        // also land on each of the two seam boxes on some ticks, or the seam would not choose.
        let twins = TwinProbe {
            world: &world,
            ground,
            seam_box: seam_boxes[0],
            twin: seam_twin,
            cylinder: &seam_cylinder,
        };
        for ((&key, ticks), box_ticks) in SEAM_CARS
            .iter()
            .zip(&mut seam_ticks)
            .zip(&mut seam_box_ticks)
        {
            let wheels = world.vehicle(vehicles[key].unwrap()).unwrap().wheels();
            for (wheel, ticks) in ticks.iter_mut().enumerate() {
                let contact_body = wheels[wheel].contact.map(|contact| contact.body);
                let on_seam = if vehicle::WHEEL_POSITIONS[wheel].x > 0.0 {
                    let straddles = straddles_seam(&world, chassis[key], wheel);
                    for (seam_box, box_ticks) in seam_boxes.iter().zip(&mut box_ticks[wheel]) {
                        if straddles && contact_body == Some(*seam_box) {
                            *box_ticks += 1;
                        }
                    }
                    contact_body.is_some_and(|body| seam_boxes.contains(&body) || body == seam_twin)
                        && straddles
                } else {
                    contact_body.is_some_and(|body| body == seam_boxes[0] || body == seam_twin)
                        && twins.twins_tie(chassis[key], wheel)
                };
                if on_seam {
                    *ticks += 1;
                }
            }
        }
    }
    let enough = |ticks: usize| ticks as f64 >= 0.9 * FLEET_TICKS as f64;
    assert!(
        slab_ticks.into_iter().all(enough),
        "slab cars: {slab_ticks:?}"
    );
    assert!(
        enough(carrier_ticks),
        "parked car on the carrier: {carrier_ticks}"
    );
    assert!(
        seam_ticks.into_iter().flatten().all(enough),
        "seam wheels: {seam_ticks:?}"
    );
    let left_wheels = (0..4).filter(|&wheel| vehicle::WHEEL_POSITIONS[wheel].x > 0.0);
    assert!(
        seam_box_ticks.iter().all(|car| left_wheels
            .clone()
            .all(|wheel| car[wheel].iter().all(|&ticks| ticks >= MIN_SEAM_BOX_TICKS))),
        "left seam wheels on the lower and upper seam box: {seam_box_ticks:?}"
    );
    digest
}

fn vehicle_in_child(threads: u32) -> Digest {
    digest_in_child("determinism_child", "vehicle", threads, "forward")
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn vehicle_digest_is_identical_across_thread_counts() {
    let one_thread = vehicle_in_child(1);
    let four_threads = vehicle_in_child(4);
    assert!(!one_thread.ticks.is_empty());
    assert_same(
        "vehicle route, 1 vs 4 worker threads",
        &one_thread,
        &four_threads,
    );
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn constraint_digest_is_identical_across_thread_counts() {
    let one_thread = digest_in_child("determinism_child", "constraints", 1, "forward");
    let four_threads = digest_in_child("determinism_child", "constraints", 4, "forward");
    assert_eq!(one_thread.ticks.len(), CONSTRAINT_TICKS);
    assert_same(
        "constraints, 1 vs 4 worker threads",
        &one_thread,
        &four_threads,
    );
}

/// Ticks of the soft body scene.
const SOFT_BODY_TICKS: usize = 300;
/// The tick before which the soft body scene unpins one cloth corner and moves the other.
const SOFT_BODY_EVENT_TICK: usize = 120;

/// Appends soft body `id`: its body record, then every vertex's position and velocity bits.
fn record_soft_body(world: &PhysicsWorld, id: BodyId, out: &mut Vec<u8>) {
    record_body(world, id, out);
    for vertex in world.soft_body(id).unwrap().vertices() {
        let position: [Real; 3] = vertex.position.into();
        for value in position {
            out.extend_from_slice(&value.to_bits().to_le_bytes());
        }
        put_f32s(out, <[f32; 3]>::from(vertex.velocity));
    }
}

fn run_soft_bodies(worker_threads: u32) -> Digest {
    use common::soft_body::{sphere, Cloth};

    let mut world = world(Vec3::new(0.0, -9.81, 0.0), worker_threads);
    let mut rigid = vec![add_floor(&mut world)];
    let ball_shape = Shape::new_sphere(0.5).unwrap();
    rigid.push(
        world
            .create_body(
                &ball_shape,
                &BodySettings::new_static().position(RVec3::new(0.0, 1.0, 0.0)),
            )
            .unwrap(),
    );
    let box_shape = Shape::new_box(Vec3::new(0.4, 0.4, 0.4)).unwrap();
    rigid.push(
        world
            .create_body(
                &box_shape,
                &BodySettings::new_dynamic()
                    .position(RVec3::new(3.0, 0.4, 0.0))
                    .allow_sleeping(false),
            )
            .unwrap(),
    );

    let cloth = Cloth::new(21, 0.1);
    let [left, right] = cloth.first_row_corners();
    let cloth_settings = cloth.pin(&[left, right]).settings();
    let awake = SoftBodySettings::default().allow_sleeping(false);
    let cloth = world
        .create_soft_body(
            &cloth_settings,
            &awake
                .clone()
                .position(RVec3::new(0.0, 1.65, 0.0))
                .linear_damping(2.0),
        )
        .unwrap();
    let (vertices, faces) = sphere(0.3, 6, 10);
    let ball_settings = SoftBodySharedSettings::builder(vertices, faces)
        .create_constraints(
            SoftBodyBendType::None,
            SoftBodyVertexAttributes::default().compliance(1.0e-4),
        )
        .build()
        .unwrap();
    let ball = world
        .create_soft_body(
            &ball_settings,
            &awake.position(RVec3::new(3.0, 2.0, 0.0)).pressure(500.0),
        )
        .unwrap();

    let mut digest = Digest::new();
    for tick in 0..SOFT_BODY_TICKS {
        if tick == SOFT_BODY_EVENT_TICK {
            let mut body = world.soft_body_mut(cloth).unwrap();
            body.set_vertex_inverse_mass(left, 1.0).unwrap();
            let at = body.vertices()[right as usize].position;
            let target = RVec3::new(at.x, at.y + 0.05, at.z);
            body.move_kinematic_vertex(right, target, DT).unwrap();
        }
        if tick == SOFT_BODY_EVENT_TICK + 1 {
            world
                .soft_body_mut(cloth)
                .unwrap()
                .set_vertex_velocity(right, Vec3::ZERO)
                .unwrap();
        }
        assert!(world.step(DT).unwrap().is_complete());
        let record = digest.push();
        for &body in &rigid {
            record_body(&world, body, &mut record.state);
        }
        for body in [cloth, ball] {
            record_soft_body(&world, body, &mut record.state);
        }
    }
    digest
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn soft_body_digest_is_identical_across_thread_counts() {
    let one_thread = digest_in_child("determinism_child", "soft_bodies", 1, "forward");
    let four_threads = digest_in_child("determinism_child", "soft_bodies", 4, "forward");
    assert_eq!(one_thread.ticks.len(), SOFT_BODY_TICKS);
    assert_same(
        "soft bodies, 1 vs 4 worker threads",
        &one_thread,
        &four_threads,
    );
}

fn fleet_in_child(threads: u32, variant: &str) -> Digest {
    digest_in_child("determinism_child", "fleet", threads, variant)
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn fleet_digest_is_identical_across_thread_counts() {
    // Jolt runs the 40 vehicle listeners in min(40 / 8, workers + 1) jobs: 2 with one worker,
    // 5 with four. The second four-worker run samples scheduling at an equal job count.
    let one_thread = fleet_in_child(1, "forward");
    let four_threads = fleet_in_child(4, "forward");
    let four_threads_again = fleet_in_child(4, "forward");
    assert_eq!(one_thread.ticks.len(), FLEET_TICKS);
    assert_same("fleet, 1 vs 4 worker threads", &one_thread, &four_threads);
    assert_same(
        "fleet, 4 worker threads in two processes",
        &four_threads,
        &four_threads_again,
    );
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn fleet_digest_is_identical_across_thread_counts_with_collision_steps() {
    // 40 vehicle listeners give 2 listener jobs with one worker and 5 with four, in every
    // collision step.
    let one_thread = fleet_in_child(1, "sub_stepped");
    let four_threads = fleet_in_child(4, "sub_stepped");
    assert_eq!(one_thread.ticks.len(), FLEET_TICKS);
    assert_same(
        "sub-stepped fleet, 1 vs 4 worker threads",
        &one_thread,
        &four_threads,
    );
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn fleet_digest_detects_a_changed_input() {
    let forward = fleet_in_child(4, "forward");
    let nudged = fleet_in_child(4, "nudged");
    let divergence = first_divergence(&forward, &nudged).expect("a changed input must diverge");
    assert!(
        matches!(divergence, Divergence::Tick { tick, .. } if tick >= FLEET_EVENT_TICK),
        "{divergence}"
    );
}

/// Ticks of the ragdoll pile.
const PILE_TICKS: usize = 300;
/// Humanoids per row and per column of the pile.
const PILE_SIDE: usize = 4;
/// Distance in metres between row neighbours of the pile: their hand tips overlap by 2 cm.
const PILE_ROW_SPACING: f64 = 1.34;
/// Distance in metres between column neighbours of the pile: the head of one penetrates the
/// feet of the next by 2 cm (head and foot spheres are 0.09 m apart sideways).
const PILE_COLUMN_SPACING: f64 = 1.834;
/// Inner half extents of the pit along x and z, metres.
const PIT_HALF: [f32; 2] = [3.0, 4.0];
/// Jolt's `LargeIslandSplitter::cLargeIslandTreshold`: an island with at least this many
/// contacts and constraints is split for parallel solving (`LargeIslandSplitter.cpp`, inclusive).
const LARGE_ISLAND_THRESHOLD: usize = 128;

/// The four walls of the pit as one static compound.
fn pit_walls() -> Shape {
    let [x, z] = PIT_HALF;
    let side = Shape::new_box(Vec3::new(0.1, 1.5, z + 0.2)).unwrap();
    let end = Shape::new_box(Vec3::new(x + 0.2, 1.5, 0.1)).unwrap();
    let wall = |shape, position| CompoundChild {
        shape,
        position,
        rotation: Quat::IDENTITY,
        user_data: 0,
    };
    Shape::new_compound(&[
        wall(&side, Vec3::new(x + 0.1, 0.5, 0.0)),
        wall(&side, Vec3::new(-x - 0.1, 0.5, 0.0)),
        wall(&end, Vec3::new(0.0, 0.5, z + 0.1)),
        wall(&end, Vec3::new(0.0, 0.5, -z - 0.1)),
    ])
    .unwrap()
}

/// A union-find over the ragdoll parts of the pile.
struct Components(Vec<usize>);

impl Components {
    fn new(count: usize) -> Self {
        Self((0..count).collect())
    }

    fn find(&mut self, mut a: usize) -> usize {
        while self.0[a] != a {
            self.0[a] = self.0[self.0[a]];
            a = self.0[a];
        }
        a
    }

    fn union(&mut self, a: usize, b: usize) {
        let (a, b) = (self.find(a), self.find(b));
        self.0[a.max(b)] = a.min(b);
    }
}

/// The most joints between parts that were awake before the last step, in one group of such
/// parts that joints and that step's contacts between neighbouring ragdolls connect. Jolt puts
/// each such group into one island (`IslandBuilder` links awake bodies through every contact
/// and every constraint between them), so the island holds at least this many constraints.
fn largest_awake_joint_group(
    world: &PhysicsWorld,
    ragdolls: &[RagdollId],
    awake: &[bool],
) -> usize {
    let parts: Vec<BodyId> = ragdolls
        .iter()
        .flat_map(|&ragdoll| world.ragdoll(ragdoll).unwrap().body_ids().to_vec())
        .collect();
    let index = |key: usize, part: usize| key * humanoid::PART_COUNT + part;
    let mut components = Components::new(parts.len());
    let joints: Vec<(usize, usize)> = (0..ragdolls.len())
        .flat_map(|key| {
            humanoid::PARTS
                .iter()
                .enumerate()
                .filter_map(move |(part, spec)| {
                    spec.parent
                        .map(|parent| (index(key, parent as usize), index(key, part)))
                })
        })
        .filter(|&(a, b)| awake[a] && awake[b])
        .collect();
    for &(a, b) in &joints {
        components.union(a, b);
    }
    for a_key in 0..ragdolls.len() {
        for b_key in a_key + 1..ragdolls.len() {
            let (ai, aj) = (a_key % PILE_SIDE, a_key / PILE_SIDE);
            let (bi, bj) = (b_key % PILE_SIDE, b_key / PILE_SIDE);
            if ai.abs_diff(bi) > 1 || aj.abs_diff(bj) > 1 {
                continue;
            }
            for a_part in 0..humanoid::PART_COUNT {
                for b_part in 0..humanoid::PART_COUNT {
                    let (a, b) = (index(a_key, a_part), index(b_key, b_part));
                    if awake[a]
                        && awake[b]
                        && world.were_bodies_in_contact(parts[a], parts[b]).unwrap()
                    {
                        components.union(a, b);
                    }
                }
            }
        }
    }
    let mut joints_per_group = vec![0; parts.len()];
    for &(a, _) in &joints {
        joints_per_group[components.find(a)] += 1;
    }
    joints_per_group.into_iter().max().unwrap_or(0)
}

/// Runs the ragdoll pile: 16 humanoids lying face up in a 4 x 4 grid in a walled pit on the
/// relief, row neighbours hand to hand and column neighbours head to feet in contact, dropped
/// 0.5 m with small death velocities under the caller's radial gravity. Records every part of
/// every ragdoll each tick, in ragdoll and part order, and checks that some step solved an
/// island past Jolt's large-island threshold.
fn run_ragdoll_pile(worker_threads: u32) -> Digest {
    let (mut world, layers) = humanoid::ragdoll_world(worker_threads);
    let static_settings = BodySettings::new_static().object_layer(layers.fixed);
    world
        .create_body(&humanoid::relief_terrain(), &static_settings)
        .unwrap();
    world.create_body(&pit_walls(), &static_settings).unwrap();

    let [half_x, half_z] = PIT_HALF;
    let highest_sample = (-(half_x as i32)..=half_x as i32)
        .flat_map(|x| (-(half_z as i32)..=half_z as i32).map(move |z| (x, z)))
        .map(|(x, z)| humanoid::relief(x as f32, z as f32))
        .fold(f32::MIN, f32::max);
    // The thickest part, the chest, has a radius of 0.13 m.
    let height = f64::from(highest_sample) + 0.5 + 0.13;
    let face_up = quat_about(humanoid::X, -std::f32::consts::FRAC_PI_2);
    let settings = humanoid::humanoid_settings(layers.ragdoll);
    let centre = (PILE_SIDE as f64 - 1.0) / 2.0;
    let ragdolls: Vec<RagdollId> = (0..PILE_SIDE * PILE_SIDE)
        .map(|key| {
            let (i, j) = ((key % PILE_SIDE) as f64, (key / PILE_SIDE) as f64);
            let at = [
                (i - centre) * PILE_ROW_SPACING,
                height,
                (j - centre) * PILE_COLUMN_SPACING,
            ];
            let pose = humanoid::transformed_pose(&humanoid::bind_pose(), face_up, at);
            let ragdoll = world
                .create_ragdoll(&settings, Some(&pose), Activation::Activate)
                .unwrap();
            let k = key as f32;
            let velocity = Vec3::new(0.35 * (1.3 * k + 0.2).sin(), 0.0, 0.35 * (0.7 * k).cos());
            world
                .ragdoll_mut(ragdoll)
                .unwrap()
                .set_linear_and_angular_velocity(velocity, Vec3::ZERO)
                .unwrap();
            ragdoll
        })
        .collect();

    let mut digest = Digest::new();
    let mut split_ticks = 0;
    for tick in 0..PILE_TICKS {
        for &ragdoll in &ragdolls {
            humanoid::apply_gravity(&mut world, ragdoll, humanoid::PLANET_CENTRE);
        }
        let awake: Vec<bool> = ragdolls
            .iter()
            .flat_map(|&ragdoll| world.ragdoll(ragdoll).unwrap().body_ids().to_vec())
            .map(|part| world.body(part).unwrap().is_active())
            .collect();
        assert!(world.step(DT).unwrap().is_complete(), "tick {tick}");

        let state = &mut digest.push().state;
        for &ragdoll in &ragdolls {
            for &part in world.ragdoll(ragdoll).unwrap().body_ids() {
                record_body(&world, part, state);
            }
        }
        if largest_awake_joint_group(&world, &ragdolls, &awake) >= LARGE_ISLAND_THRESHOLD {
            split_ticks += 1;
        }
    }
    // Measured: 25 of the 300 ticks.
    eprintln!("ragdoll pile: {split_ticks} ticks with an island past the split threshold");
    assert!(
        split_ticks > 0,
        "no step had an island of {LARGE_ISLAND_THRESHOLD} or more joints and contacts"
    );
    digest
}

fn ragdoll_pile_in_child(threads: u32) -> Digest {
    digest_in_child("determinism_child", "ragdoll_pile", threads, "forward")
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn ragdoll_pile_digest_is_identical_across_thread_counts() {
    let one_thread = ragdoll_pile_in_child(1);
    let four_threads = ragdoll_pile_in_child(4);
    assert_eq!(one_thread.ticks.len(), PILE_TICKS);
    assert_same(
        "ragdoll pile, 1 vs 4 worker threads",
        &one_thread,
        &four_threads,
    );
}
