//! Rollback with reused state buffers and selected bodies: `save_state_into` gives the same
//! states as `save_state`, a buffer belongs to the world that saved into it last, a ring of
//! buffers rolls back over a mispredicted detour and replays bit for bit with 1 and 4 workers,
//! movable states leave static bodies out, and `restore_state_of` restores the selected bodies
//! only, the same way with 1 and 4 workers in two processes.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

mod common;

use common::determinism::{assert_same, child_request, digest_in_child, finish_child, Digest};
use common::rollback::{Inputs, RollbackScene};
use oxijolt::*;

/// Ticks before the tests start saving.
const BEFORE: usize = 30;
/// The rollback window: buffers in the ring.
const RING: usize = 8;

/// Runs `ticks` ticks with `inputs` and records each.
fn recorded_run(scene: &mut RollbackScene, inputs: Inputs, ticks: usize) -> Digest {
    let mut digest = Digest::new();
    for _ in 0..ticks {
        scene.tick(inputs);
        scene.record(&mut digest);
    }
    digest
}

#[test]
fn a_state_saved_into_a_used_buffer_replays_like_save_state() {
    let mut scene = RollbackScene::new(1);
    let mut buffer = WorldState::new();
    scene
        .world
        .save_state_into(BodySelection::Only(&[scene.sleeper]), &mut buffer)
        .unwrap();
    for _ in 0..BEFORE {
        scene.tick(Inputs::PLAYED);
    }
    scene
        .world
        .save_state_into(BodySelection::All, &mut buffer)
        .unwrap();
    let fresh = scene.world.save_state();
    assert_eq!(buffer.data_size(), fresh.data_size());

    let first = recorded_run(&mut scene, Inputs::PLAYED, 60);
    scene.world.restore_state(&buffer).unwrap();
    let from_buffer = recorded_run(&mut scene, Inputs::PLAYED, 60);
    assert_same("replay from the reused buffer", &first, &from_buffer);
    scene.world.restore_state(&fresh).unwrap();
    let from_fresh = recorded_run(&mut scene, Inputs::PLAYED, 60);
    assert_same("replay from save_state", &first, &from_fresh);
}

#[test]
fn a_buffer_belongs_to_the_world_that_saved_into_it_last() {
    let mut a = RollbackScene::new(1);
    let mut b = RollbackScene::new(1);
    for _ in 0..5 {
        a.tick(Inputs::PLAYED);
        b.tick(Inputs::PREDICTED);
    }
    let mut buffer = WorldState::new();
    a.world
        .save_state_into(BodySelection::All, &mut buffer)
        .unwrap();
    b.world
        .save_state_into(BodySelection::All, &mut buffer)
        .unwrap();
    let a_now = a.body_bits();
    assert_eq!(a.world.restore_state(&buffer), Err(StateError::WrongWorld));
    assert!(a.body_bits() == a_now, "the refusal changed the world");

    let b_at_save = b.body_bits();
    b.tick(Inputs::PLAYED);
    b.world.restore_state(&buffer).unwrap();
    assert!(b.body_bits() == b_at_save, "the buffer restores b");
}

#[test]
fn a_new_state_restores_into_no_world() {
    let mut scene = RollbackScene::new(1);
    scene.tick(Inputs::PLAYED);
    let before = scene.body_bits();
    for state in [WorldState::new(), WorldState::default()] {
        assert_eq!(state.data_size(), 0);
        assert_eq!(
            scene.world.restore_state(&state),
            Err(StateError::WrongWorld)
        );
    }
    assert!(scene.body_bits() == before, "the refusal changed the world");
}

#[test]
fn a_refused_selection_leaves_the_buffer_as_it_was() {
    let mut scene = RollbackScene::new(1);
    let removed = add_extra_cube(&mut scene);
    scene.world.remove_body(removed).unwrap();
    scene.tick(Inputs::PLAYED);
    let mut buffer = scene.world.save_state();
    let at_save = scene.body_bits();
    assert_eq!(
        scene
            .world
            .save_state_into(BodySelection::Only(&[scene.sleeper, removed]), &mut buffer),
        Err(StateError::Body(BodyError::NotFound(removed)))
    );
    scene.tick(Inputs::PLAYED);
    scene.world.restore_state(&buffer).unwrap();
    assert!(
        scene.body_bits() == at_save,
        "the buffer still holds the full save"
    );
}

fn add_extra_cube(scene: &mut RollbackScene) -> BodyId {
    common::add_cube(&mut scene.world, RVec3::new(9.0, 0.5, 9.0))
}

/// The run that counts: [`BEFORE`] ticks, then `3 * RING` recorded ticks, all with the played
/// inputs.
fn played_run(threads: u32) -> Digest {
    let mut scene = RollbackScene::new(threads);
    for _ in 0..BEFORE {
        scene.tick(Inputs::PLAYED);
    }
    recorded_run(&mut scene, Inputs::PLAYED, 3 * RING)
}

/// What [`rolled_back_run`] recorded.
struct RolledBack {
    /// Ticks `0..RING` before the misprediction and `RING..3 * RING` of the replay.
    run: Digest,
    /// The last tick of the abandoned prediction.
    predicted: Digest,
    /// The ticks after the second rollback.
    tail: Digest,
}

/// The same run as [`played_run`] by way of rollbacks: a save into a ring of [`RING`] buffers
/// before every tick; ticks `RING..2 * RING` run on predicted inputs with a [detour] that wakes
/// the sleeper, then the world rolls back to tick `RING` and replays with the played inputs,
/// saving into the same buffers, up to `3 * RING`. A second rollback to tick `2 * RING + 4`, from
/// a buffer written during the replay, replays the last ticks once more.
///
/// [detour]: RollbackScene::detour
fn rolled_back_run(threads: u32) -> RolledBack {
    let mut scene = RollbackScene::new(threads);
    for _ in 0..BEFORE {
        scene.tick(Inputs::PLAYED);
    }
    let mut ring = vec![WorldState::new(); RING];
    let mut run = Digest::new();
    let mut predicted = Digest::new();
    for tick in 0..2 * RING {
        scene
            .world
            .save_state_into(BodySelection::All, &mut ring[tick % RING])
            .unwrap();
        if tick < RING {
            scene.tick(Inputs::PLAYED);
            scene.record(&mut run);
        } else if tick == RING + 3 {
            assert!(scene.world.body(scene.sleeper).unwrap().is_sleeping());
            scene.detour();
            assert!(!scene.world.body(scene.sleeper).unwrap().is_sleeping());
        } else {
            scene.tick(Inputs::PREDICTED);
        }
    }
    scene.record(&mut predicted);

    scene.world.restore_state(&ring[RING % RING]).unwrap();
    assert!(scene.world.body(scene.sleeper).unwrap().is_sleeping());
    for tick in RING..3 * RING {
        scene
            .world
            .save_state_into(BodySelection::All, &mut ring[tick % RING])
            .unwrap();
        scene.tick(Inputs::PLAYED);
        scene.record(&mut run);
    }

    let second_rollback = 2 * RING + 4;
    scene
        .world
        .restore_state(&ring[second_rollback % RING])
        .unwrap();
    let tail = recorded_run(&mut scene, Inputs::PLAYED, 3 * RING - second_rollback);
    RolledBack {
        run,
        predicted,
        tail,
    }
}

#[test]
fn a_ring_of_buffers_rolls_back_over_a_detour_bit_for_bit() {
    for threads in [1, 4] {
        let expected = played_run(threads);
        let rolled_back = rolled_back_run(threads);
        assert_same(
            &format!("rollback over a detour, {threads} workers"),
            &expected,
            &rolled_back.run,
        );
        assert_ne!(
            rolled_back.predicted.ticks[0],
            expected.ticks[2 * RING - 1],
            "the prediction differs from the played run"
        );
        let mut expected_tail = Digest::new();
        expected_tail.ticks = expected.ticks[2 * RING + 4..].to_vec();
        assert_same(
            &format!("second rollback, {threads} workers"),
            &expected_tail,
            &rolled_back.tail,
        );
    }
}

/// Bytes Jolt's stream holds for a static body: id, active flag, position and rotation
/// (`BodyManager::SaveState`, `Body::SaveState`).
fn static_body_bytes() -> usize {
    4 + 1 + 3 * size_of::<Real>() + 16
}

/// The bodies of `scene` that are not static, in creation order.
fn movable_bodies(scene: &RollbackScene) -> Vec<BodyId> {
    let statics = scene.static_bodies();
    scene
        .all_bodies()
        .into_iter()
        .filter(|id| !statics.contains(id))
        .collect()
}

#[test]
fn a_movable_state_leaves_static_bodies_out() {
    let mut scene = RollbackScene::new(1);
    for _ in 0..BEFORE {
        scene.tick(Inputs::PLAYED);
    }
    let all = scene.world.save_state();
    let movable = scene.world.save_state_of(BodySelection::Movable).unwrap();
    assert_eq!(
        all.data_size() - movable.data_size(),
        scene.static_bodies().len() * static_body_bytes()
    );
    let listed = scene
        .world
        .save_state_of(BodySelection::Only(&movable_bodies(&scene)))
        .unwrap();
    assert_eq!(listed.data_size(), movable.data_size());

    let wall_at_save = scene.world.body(scene.wall).unwrap().position();
    let moved = RVec3::new(-9.0, 1.0, 5.0);
    scene
        .world
        .body_mut(scene.wall)
        .unwrap()
        .set_position(moved, Activation::DontActivate)
        .unwrap();
    scene.tick(Inputs::PREDICTED);
    scene.world.restore_state(&movable).unwrap();
    assert_eq!(scene.world.body(scene.wall).unwrap().position(), moved);
    scene.world.restore_state(&all).unwrap();
    assert_eq!(
        scene.world.body(scene.wall).unwrap().position(),
        wall_at_save
    );
}

/// Bytes Jolt's stream holds for a body with motion properties: those of a static body plus
/// linear and angular velocity, force, torque, the sleep test offset in double precision, the
/// three sleep test spheres, the sleep timer and the allow-sleep flag
/// (`MotionProperties::SaveState`).
fn moving_body_bytes() -> usize {
    let sleep_test_offset = if size_of::<Real>() == 8 { 24 } else { 0 };
    static_body_bytes() + 4 * 12 + sleep_test_offset + 3 * 16 + 4 + 1
}

#[test]
fn a_movable_state_holds_kinematic_and_inner_bodies() {
    let mut scene = RollbackScene::new(1);
    for _ in 0..BEFORE {
        scene.tick(Inputs::PLAYED);
    }
    let inner = scene.inner_body();
    assert_eq!(
        scene.world.body(scene.platform).unwrap().motion_type(),
        MotionType::Kinematic
    );
    assert_eq!(
        scene.world.body(inner).unwrap().motion_type(),
        MotionType::Kinematic
    );
    let movable = scene.world.save_state_of(BodySelection::Movable).unwrap();
    let without: Vec<BodyId> = movable_bodies(&scene)
        .into_iter()
        .filter(|&id| id != scene.platform && id != inner)
        .collect();
    let without = scene
        .world
        .save_state_of(BodySelection::Only(&without))
        .unwrap();
    assert_eq!(
        movable.data_size() - without.data_size(),
        2 * moving_body_bytes()
    );

    let at_save = scene.body_bits();
    for _ in 0..20 {
        scene.tick(Inputs::PREDICTED);
    }
    assert!(scene.body_bits() != at_save);
    scene.world.restore_state(&movable).unwrap();
    assert!(scene.body_bits() == at_save);
}

#[test]
fn a_movable_state_replays_like_a_full_one() {
    for threads in [1, 4] {
        let mut scene = RollbackScene::new(threads);
        for _ in 0..BEFORE {
            scene.tick(Inputs::PLAYED);
        }
        let all = scene.world.save_state();
        let movable = scene.world.save_state_of(BodySelection::Movable).unwrap();
        let first = recorded_run(&mut scene, Inputs::PLAYED, 60);

        scene.world.restore_state(&all).unwrap();
        scene.detour();
        scene.world.restore_state(&movable).unwrap();
        let from_movable = recorded_run(&mut scene, Inputs::PLAYED, 60);
        assert_same(
            &format!("replay from a movable state, {threads} workers"),
            &first,
            &from_movable,
        );
    }
}

/// Every body of `scene` with its bits now, as [`RollbackScene::body_bits`] writes them.
fn bits_by_body(scene: &RollbackScene) -> Vec<(BodyId, Vec<u8>)> {
    scene
        .all_bodies()
        .into_iter()
        .zip(scene.body_bits())
        .collect()
}

/// Asserts that every body of `scene` has the bits `expected` gives for it.
fn assert_bits(scene: &RollbackScene, expected: impl Fn(BodyId) -> Vec<u8>) {
    for (id, bits) in bits_by_body(scene) {
        assert!(bits == expected(id), "{id:?}");
    }
}

/// The bits of `id` in `bits`.
fn bits_of(bits: &[(BodyId, Vec<u8>)], id: BodyId) -> Vec<u8> {
    bits.iter().find(|(body, _)| *body == id).unwrap().1.clone()
}

/// A scene run to the save point and saved whole, then taken through a detour and a few
/// mispredicted ticks: the scene, the state and the bits at the save.
fn saved_and_moved_on() -> (RollbackScene, WorldState, Vec<(BodyId, Vec<u8>)>) {
    let mut scene = RollbackScene::new(1);
    for _ in 0..BEFORE {
        scene.tick(Inputs::PLAYED);
    }
    let state = scene.world.save_state();
    let at_save = bits_by_body(&scene);
    scene.detour();
    for _ in 0..10 {
        scene.tick(Inputs::PREDICTED);
    }
    (scene, state, at_save)
}

#[test]
fn restoring_some_bodies_leaves_the_others_bit_for_bit() {
    let (mut scene, state, at_save) = saved_and_moved_on();
    let restored = [scene.bodies[2], scene.sleeper];
    let inner = scene.inner_body();
    let now = bits_by_body(&scene);
    assert_ne!(
        bits_of(&now, scene.bodies[2]),
        bits_of(&at_save, scene.bodies[2])
    );
    assert!(!scene.world.body(scene.sleeper).unwrap().is_sleeping());
    scene
        .world
        .restore_state_of(&state, BodySelection::Only(&restored))
        .unwrap();
    assert_bits(&scene, |id| {
        if restored.contains(&id) || id == inner {
            bits_of(&at_save, id)
        } else {
            bits_of(&now, id)
        }
    });
    assert!(scene.world.body(scene.sleeper).unwrap().is_sleeping());
}

#[test]
fn restoring_movable_bodies_keeps_static_bodies_where_they_are() {
    let (mut scene, state, at_save) = saved_and_moved_on();
    let moved = RVec3::new(-9.0, 1.0, 5.0);
    scene
        .world
        .body_mut(scene.wall)
        .unwrap()
        .set_position(moved, Activation::DontActivate)
        .unwrap();
    let now = bits_by_body(&scene);
    scene
        .world
        .restore_state_of(&state, BodySelection::Movable)
        .unwrap();
    let statics = scene.static_bodies();
    assert_bits(&scene, |id| {
        if statics.contains(&id) {
            bits_of(&now, id)
        } else {
            bits_of(&at_save, id)
        }
    });
    assert_eq!(scene.world.body(scene.wall).unwrap().position(), moved);
}

#[test]
fn a_selected_body_the_state_does_not_hold_keeps_its_state() {
    let mut scene = RollbackScene::new(1);
    for _ in 0..BEFORE {
        scene.tick(Inputs::PLAYED);
    }
    let (held, not_held) = (scene.bodies[2], scene.bodies[3]);
    let state = scene
        .world
        .save_state_of(BodySelection::Only(&[held]))
        .unwrap();
    let at_save = bits_by_body(&scene);
    scene.detour();
    let now = bits_by_body(&scene);
    scene
        .world
        .restore_state_of(&state, BodySelection::Only(&[not_held]))
        .unwrap();
    let inner = scene.inner_body();
    assert_bits(&scene, |id| {
        if id == inner {
            bits_of(&at_save, id)
        } else {
            bits_of(&now, id)
        }
    });
}

#[test]
fn inner_bodies_follow_their_characters() {
    let (mut scene, state, at_save) = saved_and_moved_on();
    let inner = scene.inner_body();
    assert_ne!(
        bits_of(&bits_by_body(&scene), inner),
        bits_of(&at_save, inner)
    );
    scene
        .world
        .restore_state_of(&state, BodySelection::Only(&[scene.bodies[2]]))
        .unwrap();
    assert_eq!(
        bits_of(&bits_by_body(&scene), inner),
        bits_of(&at_save, inner)
    );
}

#[test]
fn a_refused_filtered_restore_changes_nothing() {
    let mut scene = RollbackScene::new(1);
    let removed = add_extra_cube(&mut scene);
    scene.world.remove_body(removed).unwrap();
    for _ in 0..5 {
        scene.tick(Inputs::PLAYED);
    }
    let state = scene.world.save_state();
    let other = RollbackScene::new(1);
    let foreign_state = other.world.save_state();
    scene.tick(Inputs::PREDICTED);
    let now = scene.body_bits();
    let foreign = other.bodies[1];
    let cube = scene.bodies[1];
    let refusals = [
        (
            scene
                .world
                .restore_state_of(&foreign_state, BodySelection::Movable),
            StateError::WrongWorld,
        ),
        (
            scene
                .world
                .restore_state_of(&WorldState::new(), BodySelection::Only(&[cube])),
            StateError::WrongWorld,
        ),
        (
            scene
                .world
                .restore_state_of(&state, BodySelection::Only(&[cube, removed])),
            StateError::Body(BodyError::NotFound(removed)),
        ),
        (
            scene
                .world
                .restore_state_of(&state, BodySelection::Only(&[foreign])),
            StateError::Body(BodyError::WrongWorld(foreign)),
        ),
    ];
    for (result, error) in refusals {
        assert_eq!(result, Err(error));
    }
    assert!(scene.body_bits() == now, "a refusal changed the world");
}

#[test]
fn a_filtered_restore_after_a_structural_change_is_refused() {
    let mut scene = RollbackScene::new(1);
    scene.tick(Inputs::PLAYED);
    let state = scene.world.save_state();
    add_extra_cube(&mut scene);
    let now = scene.body_bits();
    for selection in [
        BodySelection::Movable,
        BodySelection::Only(&[scene.sleeper]),
    ] {
        assert_eq!(
            scene.world.restore_state_of(&state, selection),
            Err(StateError::WorldChanged)
        );
    }
    assert!(scene.body_bits() == now, "the refusal changed the world");
}

#[test]
fn restoring_all_bodies_is_restore_state() {
    let (mut scene, state, at_save) = saved_and_moved_on();
    scene
        .world
        .restore_state_of(&state, BodySelection::All)
        .unwrap();
    assert!(bits_by_body(&scene) == at_save);
}

/// The filtered rollback the cross-process gate runs: a full save, a detour, a restore of the
/// `variant` selection (`movable` or `only`), then 120 recorded ticks.
fn filtered_rollback(threads: u32, variant: &str) -> Digest {
    let mut scene = RollbackScene::new(threads);
    for _ in 0..BEFORE {
        scene.tick(Inputs::PLAYED);
    }
    let state = scene.world.save_state();
    scene.detour();
    for _ in 0..10 {
        scene.tick(Inputs::PREDICTED);
    }
    let only = [
        scene.bodies[1],
        scene.bodies[5],
        scene.sleeper,
        scene.platform,
    ];
    let selection = match variant {
        "movable" => BodySelection::Movable,
        "only" => BodySelection::Only(&only),
        _ => panic!("unknown variant {variant}"),
    };
    scene.world.restore_state_of(&state, selection).unwrap();
    recorded_run(&mut scene, Inputs::PLAYED, 120)
}

#[test]
#[ignore = "child process of the filtered rollback gate"]
fn state_buffers_child() {
    let Some((scenario, threads, variant)) = child_request() else {
        return;
    };
    assert_eq!(scenario, "filtered", "unknown scenario");
    finish_child(&filtered_rollback(threads, &variant));
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn a_filtered_restore_replays_identically_across_processes() {
    for variant in ["movable", "only"] {
        let one = digest_in_child("state_buffers_child", "filtered", 1, variant);
        let four = digest_in_child("state_buffers_child", "filtered", 4, variant);
        assert_eq!(one.ticks.len(), 120);
        assert_same(
            &format!("filtered restore ({variant}), 1 vs 4 workers in two processes"),
            &one,
            &four,
        );
        assert_ne!(one.ticks[0], one.ticks[119], "the scene moves");
    }
}
