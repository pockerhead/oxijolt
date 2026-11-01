//! Determinism of the constraint impulse readouts: a hanging graph of welded cubes large enough
//! for Jolt to split its island for parallel solving, next to a loaded constraint of every kind,
//! records the same readouts bit for bit with 1 and 4 worker threads, in two processes. A second
//! run gives the grid welds differing solver priorities and step overrides, changed again while
//! the island is split.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::constraint_rigs::{
    all_kinds_rig, anchor, assert_every_getter_loaded, cube, is_enabled, readout_bits, shifted,
    HALF,
};
use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::{record_body, world, DT};
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const TICKS: usize = 300;
/// Cubes per row and per column of the grid.
const GRID: usize = 10;
/// Distance between neighbouring cube centres, metres: a 1 cm gap between their faces.
const PITCH: f32 = 2.0 * HALF + 0.01;
/// Centre of the grid's top-left cube; the grid extends along +X and -Y.
const GRID_ORIGIN: RVec3 = RVec3::new(0.0, 20.0, 0.0);
/// Mass of a grid cube, kg.
const MASS: f32 = 5.0;
/// Jolt's `LargeIslandSplitter::cLargeIslandTreshold`: an island with at least this many
/// contacts and constraints is split for parallel solving (`LargeIslandSplitter.cpp`, inclusive).
const LARGE_ISLAND_THRESHOLD: usize = 128;
/// The tick before whose step the bottom-right cube is kicked out of the grid's plane.
const KICK_TICK: usize = 60;
/// The tick before whose step four welds of the right column are disabled.
const CUT_TICK: usize = 150;
/// The tick after whose step every readout getter must read a load.
const LOADED_TICK: usize = 149;
/// The tick before whose step the solver overrides of some grid welds change, in the
/// [`SOLVER_OVERRIDES`] run.
const RETUNE_TICK: usize = 100;
/// The scenario variant whose grid welds carry solver priorities and step overrides.
const SOLVER_OVERRIDES: &str = "solver_overrides";

/// The weld graph, its anchor and [`all_kinds_rig`].
struct Scene {
    world: PhysicsWorld,
    /// Every body, in raw id order.
    bodies: Vec<BodyId>,
    /// The grid cubes, row by row from the top, each row from the left.
    grid: Vec<BodyId>,
    /// The welds between grid neighbours with the grid indices of the cubes they join.
    welds: Vec<(ConstraintId<FixedConstraint>, [usize; 2])>,
    /// Every constraint, in creation order.
    constraints: Vec<AnyConstraintId>,
    /// Whether the grid welds carry solver priorities and step overrides.
    overrides: bool,
}

impl Scene {
    fn new(threads: u32, overrides: bool) -> Self {
        let mut world = world(GRAVITY, threads);
        let grid: Vec<BodyId> = (0..GRID * GRID)
            .map(|i| cube(&mut world, cell_centre(i), MASS))
            .collect();
        let mut welds = Vec::new();
        for i in 0..GRID * GRID {
            let (row, column) = (i / GRID, i % GRID);
            let right = (column + 1 < GRID).then_some((i + 1, [PITCH / 2.0, 0.0, 0.0]));
            let below = (row + 1 < GRID).then_some((i + GRID, [0.0, -PITCH / 2.0, 0.0]));
            for (j, offset) in right.into_iter().chain(below) {
                let point = shifted(cell_centre(i), offset);
                let mut weld = FixedConstraintSettings::new(point, X, Y);
                if overrides {
                    let k = welds.len();
                    weld = weld
                        .constraint_priority((k % 3) as u32)
                        .velocity_steps_override(if k % 7 == 0 { 16 } else { 0 })
                        .position_steps_override(if k % 11 == 0 { 4 } else { 0 });
                }
                let id = world.create_constraint(grid[i], grid[j], &weld).unwrap();
                welds.push((id, [i, j]));
            }
        }
        assert_eq!(welds.len(), 2 * GRID * (GRID - 1));
        let top = cell_centre(0);
        let hook = anchor(&mut world, shifted(top, [0.0, 1.0, 0.0]));
        let hang = FixedConstraintSettings::new(shifted(top, [0.0, HALF, 0.0]), X, Y);
        let hang = world.create_constraint(hook, grid[0], &hang).unwrap();
        let mut constraints: Vec<AnyConstraintId> =
            welds.iter().map(|&(id, _)| id.into()).collect();
        constraints.push(hang.into());
        constraints.extend(all_kinds_rig(&mut world));
        let mut bodies: Vec<BodyId> = world.body_ids().collect();
        bodies.sort_by_key(|id| id.to_raw());
        Self {
            world,
            bodies,
            grid,
            welds,
            constraints,
            overrides,
        }
    }

    /// The changes of `tick`, the coverage check, then one step.
    fn tick(&mut self, tick: usize) {
        if tick == KICK_TICK {
            let corner = self.grid[GRID * GRID - 1];
            self.world
                .body_mut(corner)
                .unwrap()
                .add_impulse(Vec3::new(0.0, 0.0, 10.0))
                .unwrap();
        }
        if self.overrides && tick == RETUNE_TICK {
            for (k, &(id, _)) in self.welds.iter().enumerate().filter(|(k, _)| k % 5 == 0) {
                let mut weld = self.world.constraint_mut(id).unwrap();
                weld.set_constraint_priority(5 - (k % 2) as u32);
                weld.set_velocity_steps_override(20).unwrap();
                weld.set_position_steps_override(3).unwrap();
            }
        }
        if tick == CUT_TICK {
            // The vertical welds of the right column's top four cubes.
            for row in 0..4 {
                let i = row * GRID + GRID - 1;
                let (id, _) = *self
                    .welds
                    .iter()
                    .find(|(_, ends)| *ends == [i, i + GRID])
                    .unwrap();
                self.world.constraint_mut(id).unwrap().set_enabled(false);
            }
        }
        let largest = self.largest_weld_group();
        assert!(
            largest >= LARGE_ISLAND_THRESHOLD,
            "tick {tick}: the largest group of enabled welds has {largest}"
        );
        assert!(self.world.step(DT).unwrap().is_complete());
        if tick == LOADED_TICK {
            let when = format!("tick {tick}");
            assert_every_getter_loaded(&self.world, &self.constraints, &when);
        }
    }

    /// The number of enabled grid welds in the largest group of grid cubes they connect.
    fn largest_weld_group(&self) -> usize {
        let mut parent: Vec<usize> = (0..self.grid.len()).collect();
        fn root(parent: &mut [usize], mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        let enabled: Vec<[usize; 2]> = self
            .welds
            .iter()
            .filter(|&&(id, _)| is_enabled(&self.world, id.into()))
            .map(|&(_, ends)| ends)
            .collect();
        for &[a, b] in &enabled {
            let (a, b) = (root(&mut parent, a), root(&mut parent, b));
            parent[a] = b;
        }
        let mut welds_per_root = vec![0; self.grid.len()];
        for &[a, _] in &enabled {
            welds_per_root[root(&mut parent, a)] += 1;
        }
        welds_per_root.into_iter().max().unwrap()
    }

    /// Every body's state, then per constraint its raw id, enabled flag and readout bits.
    fn record(&self, digest: &mut Digest) {
        let tick = digest.push();
        for &id in &self.bodies {
            let position = self.world.body(id).unwrap().position();
            assert!(
                position.x.is_finite() && position.y.is_finite() && position.z.is_finite(),
                "{id:?} at {position:?}"
            );
            record_body(&self.world, id, &mut tick.state);
        }
        for &id in &self.constraints {
            tick.state.extend_from_slice(&id.to_raw().to_le_bytes());
            tick.state.push(u8::from(is_enabled(&self.world, id)));
            for bits in readout_bits(&self.world, id) {
                tick.state.extend_from_slice(&bits.to_le_bytes());
            }
        }
    }
}

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);

/// The centre of grid cube `i`.
fn cell_centre(i: usize) -> RVec3 {
    let (row, column) = ((i / GRID) as f32, (i % GRID) as f32);
    shifted(GRID_ORIGIN, [column * PITCH, -row * PITCH, 0.0])
}

/// The scene run for [`TICKS`] ticks with `threads` workers, with solver overrides on the grid
/// welds when `overrides`.
fn weld_graph(threads: u32, overrides: bool) -> Digest {
    let mut scene = Scene::new(threads, overrides);
    let mut digest = Digest::new();
    for tick in 1..=TICKS {
        scene.tick(tick);
        scene.record(&mut digest);
    }
    digest
}

#[test]
#[ignore = "child process of the constraint impulse determinism gate"]
fn constraint_forces_child() {
    let Some((scenario, threads, variant)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "weld_graph");
    finish_child(&weld_graph(threads, variant == SOLVER_OVERRIDES));
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn constraint_impulses_are_identical_with_1_and_4_workers() {
    let one = digest_in_child("constraint_forces_child", "weld_graph", 1, "");
    let four = digest_in_child("constraint_forces_child", "weld_graph", 4, "");
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    assert_ne!(one.ticks[KICK_TICK].state, one.ticks[TICKS - 1].state);
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn solver_overrides_are_identical_with_1_and_4_workers() {
    let one = digest_in_child("constraint_forces_child", "weld_graph", 1, SOLVER_OVERRIDES);
    let four = digest_in_child("constraint_forces_child", "weld_graph", 4, SOLVER_OVERRIDES);
    assert_eq!(one.ticks.len(), TICKS);
    assert_same("1 vs 4 workers in two processes", &one, &four);
    let plain = digest_in_child("constraint_forces_child", "weld_graph", 1, "");
    for tick in [RETUNE_TICK - 2, TICKS - 1] {
        assert_ne!(
            one.ticks[tick].state, plain.ticks[tick].state,
            "tick {tick}"
        );
    }
}
