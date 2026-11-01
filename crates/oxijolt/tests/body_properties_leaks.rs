//! A leak gate for the filter setters: a new collision group table given to a body and taken
//! away again, and an object layer change back and forth, round after round in one world.
//!
//! It measures the private bytes of the process (Windows `K32GetProcessMemoryInfo`), because
//! group tables are allocated by C++, which a Rust global allocator does not see. As in
//! `body_controls_leaks.rs`, it measures after 3000 warm-up rounds: nine consecutive blocks of
//! 500 rounds. It fails when the median block grows by 100 bytes per round or more, or when all
//! blocks together grow by 4 MiB or more. A control run then keeps every round's table alive
//! (about 1 KiB each) and must fail the median gate, which shows the gate sees a leak of that
//! size. This is a regression gate for the table references `set_collision_group` hands to Jolt,
//! not a proof of ownership. The file holds exactly one test, so its binary runs alone and no
//! parallel test disturbs the counter.
#![cfg(windows)]

mod common;

use common::memory::private_bytes;
use common::*;
use oxijolt::*;

const WARM_UP_ROUNDS: usize = 3000;
const BLOCK_ROUNDS: usize = 500;
const BLOCKS: usize = 9;
const MAX_ROUND_GROWTH: usize = 100;
const MAX_TOTAL_GROWTH: usize = 4 * 1024 * 1024;
/// Sub groups of each round's table: 128² bits, about 1 KiB.
const SUB_GROUPS: u32 = 128;

/// The body every round drives.
struct Scene {
    world: PhysicsWorld,
    cube: BodyId,
}

impl Scene {
    fn new() -> Self {
        let mut world = world(Vec3::ZERO, 1);
        let cube = add_cube(&mut world, RVec3::ZERO);
        Self { world, cube }
    }

    /// One round: the cube gets a group of a new table, the caller's handles are dropped, the
    /// group is removed (Jolt releases the table), the layer changes twice; one step. Returns
    /// the round's group for the control to keep.
    fn round(&mut self) -> CollisionGroup {
        let table = GroupFilterTableBuilder::new(SUB_GROUPS).unwrap().build();
        let group = CollisionGroup::new(&table, 1, SUB_GROUPS - 1).unwrap();
        let mut cube = self.world.body_mut(self.cube).unwrap();
        cube.set_collision_group(Some(&group), Activation::Activate)
            .unwrap();
        cube.set_object_layer(ObjectLayer::NON_MOVING, Activation::Activate)
            .unwrap();
        cube.set_object_layer(ObjectLayer::MOVING, Activation::Activate)
            .unwrap();
        step(&mut self.world, 1);
        self.world
            .body_mut(self.cube)
            .unwrap()
            .set_collision_group(None, Activation::Activate)
            .unwrap();
        group
    }
}

/// The private-bytes growth of each of [`BLOCKS`] consecutive blocks of `round`s.
fn block_growth(mut round: impl FnMut()) -> Vec<usize> {
    (0..BLOCKS)
        .map(|_| {
            let before = private_bytes();
            for _ in 0..BLOCK_ROUNDS {
                round();
            }
            private_bytes().saturating_sub(before)
        })
        .collect()
}

fn median(blocks: &[usize]) -> usize {
    let mut sorted = blocks.to_vec();
    sorted.sort_unstable();
    sorted[sorted.len() / 2]
}

#[test]
fn filter_setters_do_not_leak() {
    let mut scene = Scene::new();
    for _ in 0..WARM_UP_ROUNDS {
        scene.round();
    }
    let blocks = block_growth(|| drop(scene.round()));
    eprintln!("filter setters: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    let (median_growth, total): (usize, usize) = (median(&blocks), blocks.iter().sum());
    assert!(
        median_growth < MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "median block growth {median_growth} bytes: a filter setter leaks ({blocks:?})"
    );
    assert!(
        total < MAX_TOTAL_GROWTH,
        "total growth {total} bytes: a filter setter leaks ({blocks:?})"
    );

    // The control: the same rounds, each keeping its table alive, must show.
    let blocks = block_growth(|| std::mem::forget(scene.round()));
    eprintln!("control: growth per block of {BLOCK_ROUNDS} rounds: {blocks:?}");
    assert!(
        median(&blocks) >= MAX_ROUND_GROWTH * BLOCK_ROUNDS,
        "the control kept a table per round but the median block grew only by {} ({blocks:?})",
        median(&blocks)
    );
}
