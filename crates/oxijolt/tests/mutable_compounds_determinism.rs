//! Determinism of compound edits: a static ledge loses supports under sleeping cubes, gains a
//! ramp that catches a falling cube, moves a plate and swaps a tile for a taller block while a
//! dynamic debris body loses a child.
//! The runs match bit for bit with 1 and 4 worker threads, in one process and in two, and after
//! a rollback with a detour from a state saved right after the last edit.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::controls::fall_asleep;
use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const TICKS: usize = 240;
/// Half extent of a ledge tile; tiles 2 m apart touch.
const TILE: Vec3 = Vec3::new(1.0, 0.25, 1.0);
const LEDGE_Y: Real = 2.0;
const TILES: u32 = 12;
/// The ledge tiles that carry a cube.
const CARRYING: u32 = 8;
/// The tiles removed at tick 40, each under a cube.
const REMOVED: [u32; 3] = [5, 3, 1];
/// The tick of the last edits: a tile swapped and a child of a debris body removed.
const LAST_EDIT: usize = 160;

/// The x of tile `index` in the ledge.
fn tile_offset(index: u32) -> f32 {
    2.0 * f32::from(index as u16)
}

/// The world x of tile `index`.
fn tile_x(index: u32) -> Real {
    2.0 * Real::from(index as u16)
}

/// The scene and its editors.
struct Scene {
    world: PhysicsWorld,
    tile: Shape,
    ledge_editor: MutableCompound,
    ledge: BodyId,
    debris_editors: Vec<MutableCompound>,
    debris: Vec<BodyId>,
    cubes: Vec<BodyId>,
    faller: BodyId,
    sleeper: BodyId,
    /// Every body, in creation order.
    bodies: Vec<BodyId>,
}

impl Scene {
    fn new(threads: u32) -> Self {
        let mut world = world(GRAVITY, threads);
        let floor = add_floor(&mut world);
        let tile = Shape::new_box(TILE).unwrap();
        let children: Vec<_> = (0..TILES)
            .map(|i| child(&tile, Vec3::new(tile_offset(i), 0.0, 0.0), 100 + i))
            .collect();
        let ledge_editor = MutableCompound::from_children(&children).unwrap();
        drop(children);
        let ledge = world
            .create_body(
                &ledge_editor.to_shape().unwrap(),
                &BodySettings::new_static().position(RVec3::new(0.0, LEDGE_Y, 0.0)),
            )
            .unwrap();
        let cubes: Vec<_> = (0..CARRYING)
            .map(|i| add_cube(&mut world, RVec3::new(tile_x(i), LEDGE_Y + 0.75, 0.0)))
            .collect();
        for &cube in &cubes {
            fall_asleep(&mut world, cube);
        }
        assert!(cubes
            .iter()
            .all(|&cube| world.body(cube).unwrap().is_sleeping()));

        let block = Shape::new_box(Vec3::new(0.3, 0.3, 0.3)).unwrap();
        let ball = Shape::new_sphere(0.3).unwrap();
        let debris_editors: Vec<_> = (0..2)
            .map(|_| {
                MutableCompound::from_children(&[
                    child(&block, Vec3::ZERO, 0),
                    child(&block, Vec3::new(0.6, 0.0, 0.0), 1),
                    child(&ball, Vec3::new(1.2, 0.1, 0.0), 2),
                ])
                .unwrap()
            })
            .collect();
        let debris: Vec<_> = debris_editors
            .iter()
            .zip([0.0, 5.0])
            .map(|(editor, z)| {
                world
                    .create_body(
                        &editor.to_shape().unwrap(),
                        &BodySettings::new_dynamic()
                            .position(RVec3::new(-10.0, 1.5, z))
                            .rotation(quat_about(Vec3::new(0.0, 0.6, 0.8), 0.4)),
                    )
                    .unwrap()
            })
            .collect();
        // Above the ramp added at tick 80.
        let faller = add_cube(&mut world, RVec3::new(tile_x(TILES + 1), 14.0, 0.0));
        let sleeper = world
            .create_body(
                &cube_shape(),
                &BodySettings::new_dynamic()
                    .position(RVec3::new(-30.0, 0.5, -30.0))
                    .activation(Activation::DontActivate),
            )
            .unwrap();
        let mut bodies = vec![floor, ledge];
        bodies.extend(&cubes);
        bodies.extend(&debris);
        bodies.extend([faller, sleeper]);
        Self {
            world,
            tile,
            ledge_editor,
            ledge,
            debris_editors,
            debris,
            cubes,
            faller,
            sleeper,
            bodies,
        }
    }

    /// Installs the ledge editor's shape.
    fn commit_ledge(&mut self) {
        let shape = self.ledge_editor.to_shape().unwrap();
        self.world
            .body_mut(self.ledge)
            .unwrap()
            .set_shape(&shape, None, Activation::DontActivate)
            .unwrap();
    }

    /// The edits of the schedule, before the step of `tick`.
    fn edit(&mut self, tick: usize) {
        match tick {
            40 => {
                for index in REMOVED {
                    self.ledge_editor.remove_shape(index).unwrap();
                }
                self.commit_ledge();
                for index in REMOVED {
                    let cube = self.cubes[index as usize];
                    assert!(!self.world.body(cube).unwrap().is_sleeping());
                }
            }
            80 => {
                let ramp = CompoundChild {
                    rotation: quat_about(Vec3::new(0.0, 0.0, 1.0), 0.1),
                    ..child(&self.tile, Vec3::new(tile_offset(TILES + 1), 0.0, 0.0), 200)
                };
                self.ledge_editor.add_shape(&ramp).unwrap();
                self.commit_ledge();
            }
            120 => {
                // The last carrying tile rises under its cube.
                let index = CARRYING - 1 - REMOVED.len() as u32;
                let position = Vec3::new(tile_offset(CARRYING - 1), 0.2, 0.0);
                self.ledge_editor
                    .modify_shape(index, position, Quat::IDENTITY, None)
                    .unwrap();
                self.commit_ledge();
            }
            LAST_EDIT => {
                // The first tile becomes a taller block of the same volume under its cube,
                // which keeps the ledge's centre of mass and so the cube's pair as Jolt caches
                // it; a debris body loses a child.
                let tall = Shape::new_box(Vec3::new(0.5, 0.5, 1.0)).unwrap();
                self.ledge_editor
                    .modify_shape(0, Vec3::ZERO, Quat::IDENTITY, Some(&tall))
                    .unwrap();
                self.commit_ledge();
                self.debris_editors[0].remove_shape(2).unwrap();
                let shape = self.debris_editors[0].to_shape().unwrap();
                self.world
                    .body_mut(self.debris[0])
                    .unwrap()
                    .set_shape(&shape, None, Activation::Activate)
                    .unwrap();
            }
            _ => {}
        }
    }

    fn record(&self, digest: &mut Digest) {
        let tick = digest.push();
        for &id in &self.bodies {
            record_body(&self.world, id, &mut tick.state);
        }
        let count = self.ledge_editor.sub_shape_count();
        tick.shape.extend_from_slice(&count.to_le_bytes());
        // A probe through tile 9, whose index shifts with the removals.
        let ray = RayCast::new(
            RVec3::new(tile_x(9), LEDGE_Y + 0.1, 0.0),
            Vec3::new(0.0, -0.2, 0.0),
        );
        let hit = self.world.cast_ray(&ray, &QueryFilter::new()).unwrap();
        let probe = hit.and_then(|hit| hit.compound_child);
        tick.shape.extend(format!("{probe:?}").bytes());
    }

    /// Different inputs for a while, with no change of structure: kicks, a teleport, the
    /// sleeper woken and another gravity.
    fn detour_tick(&mut self, tick: usize) {
        self.world.set_gravity(Vec3::new(1.0, -4.0, 0.0)).unwrap();
        if tick.is_multiple_of(5) {
            let kicked = self.cubes[tick % self.cubes.len()];
            self.world
                .body_mut(kicked)
                .unwrap()
                .add_impulse(Vec3::new(0.0, 5.0, 2.0))
                .unwrap();
        }
        if tick == 10 {
            self.world
                .body_mut(self.debris[1])
                .unwrap()
                .set_position(RVec3::new(-10.0, 3.0, 8.0), Activation::Activate)
                .unwrap();
            self.world.body_mut(self.sleeper).unwrap().activate();
        }
        step(&mut self.world, 1);
    }
}

/// What a run checked about the scene besides its digest.
fn assert_scene_played(scene: &Scene) {
    let y = |id: BodyId| scene.world.body(id).unwrap().position().y;
    for index in REMOVED {
        let fallen = y(scene.cubes[index as usize]);
        assert!(fallen < 1.0, "cube {index} fell to the floor: {fallen}");
    }
    let caught = y(scene.faller);
    assert!(caught > LEDGE_Y, "the ramp caught the faller: {caught}");
    let lifted = y(scene.cubes[CARRYING as usize - 1]);
    assert!(
        lifted > LEDGE_Y + 0.85,
        "the plate lifted its cube: {lifted}"
    );
    assert_eq!(scene.debris_editors[0].sub_shape_count(), 2);
}

/// The scene run for [`TICKS`] ticks with `threads` workers.
fn run(threads: u32) -> (Digest, Scene) {
    let mut scene = Scene::new(threads);
    let mut digest = Digest::new();
    for tick in 0..TICKS {
        scene.edit(tick);
        step(&mut scene.world, 1);
        scene.record(&mut digest);
    }
    (digest, scene)
}

fn compounds(threads: u32) -> Digest {
    run(threads).0
}

/// The scene run with a rollback: saved right after the last edit and before its step, sent
/// on a detour, restored, and replayed; the digests of every tick.
fn replayed(threads: u32) -> Digest {
    let mut scene = Scene::new(threads);
    let mut digest = Digest::new();
    for tick in 0..LAST_EDIT {
        scene.edit(tick);
        step(&mut scene.world, 1);
        scene.record(&mut digest);
    }
    scene.edit(LAST_EDIT);
    let saved = scene.world.save_state();
    for tick in 0..30 {
        scene.detour_tick(tick);
    }
    assert!(!scene.world.body(scene.sleeper).unwrap().is_sleeping());
    scene.world.restore_state(&saved).unwrap();
    // Jolt's saved state holds the world's gravity, which the detour changed.
    assert_eq!(scene.world.gravity(), GRAVITY);
    step(&mut scene.world, 1);
    scene.record(&mut digest);
    for tick in LAST_EDIT + 1..TICKS {
        scene.edit(tick);
        step(&mut scene.world, 1);
        scene.record(&mut digest);
    }
    assert_scene_played(&scene);
    digest
}

#[test]
#[ignore = "child process of the compound edits determinism gate"]
fn mutable_compounds_determinism_child() {
    let Some((scenario, threads, _)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "compounds");
    finish_child(&compounds(threads));
}

#[test]
fn mutable_compounds_match_with_1_and_4_workers() {
    let (one, scene) = run(1);
    assert_eq!(one.ticks.len(), TICKS);
    assert_scene_played(&scene);
    assert_same("1 vs 4 workers in one process", &one, &compounds(4));
}

#[test]
fn a_rollback_after_the_last_edit_replays_exactly() {
    for threads in [1, 4] {
        assert_same(
            "straight run vs rollback and replay",
            &compounds(threads),
            &replayed(threads),
        );
    }
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn mutable_compounds_match_across_processes() {
    let one = digest_in_child("mutable_compounds_determinism_child", "compounds", 1, "");
    let four = digest_in_child("mutable_compounds_determinism_child", "compounds", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_same("one process vs a child", &compounds(1), &one);
}
