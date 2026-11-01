//! The character study's frame and determinism checks: the same cells in a tilted frame, a
//! repeated run, runs in separate processes with 1 and 4 worker threads, and a restore after a
//! detour. `docs/character-study.md` describes the study.

mod common;
mod study;

use common::math::{norm, sub, V3};
use common::walker::{up_at, Carry};
use oxijolt::*;
use study::config::Config;
use study::controller::{tick, TickReport};
use study::frame::Frame;
use study::laws::{judge, play, Case, Check, Column, Scenes};
use study::matrix;
use study::scenes::{Scene, SceneKey};

/// `case` in the tilted frame: the same scene turned and moved, the same start in its frame.
fn tilted(case: &Case) -> Option<Case> {
    let turn = |key: SceneKey| match key {
        SceneKey::Plane { deg10, bits, .. } => Some(SceneKey::Plane {
            deg10,
            bits,
            tilted: true,
        }),
        SceneKey::Ramp { deg10, .. } => Some(SceneKey::Ramp {
            deg10,
            tilted: true,
        }),
        SceneKey::Ridge { deg10, .. } => Some(SceneKey::Ridge {
            deg10,
            tilted: true,
        }),
        SceneKey::BoxRamp { deg10, .. } => Some(SceneKey::BoxRamp {
            deg10,
            tilted: true,
        }),
        _ => None,
    };
    let frame = Frame::tilted();
    Some(Case {
        scene: turn(case.scene)?,
        start: frame.to_world_point(case.start),
        ..case.clone()
    })
}

/// Most verdicts a row may flip between the frames.
const MAX_FLIPPED_VERDICTS: usize = 2;

/// How a row's tilted runs compare with its untilted ones.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Tilt {
    compared: usize,
    /// Cases whose paths, once mapped back, part by more than the tolerance at some tick.
    diverged: usize,
    /// Cases that pass in one frame and fail in the other.
    flipped: usize,
}

impl Tilt {
    /// Counts one case: whether both frames gave the same verdict, and how far apart its paths
    /// came against the path tolerance, metres. A flipped case counts against both limits.
    fn record(&mut self, same_verdict: bool, apart: f64, tolerance: f64) {
        self.compared += 1;
        self.diverged += usize::from(apart > tolerance);
        self.flipped += usize::from(!same_verdict);
    }

    /// At most one path in twenty outside the tolerance and at most
    /// [`MAX_FLIPPED_VERDICTS`] flipped verdicts.
    fn check(&self) -> Result<(), String> {
        if self.diverged * 20 > self.compared {
            return Err(format!(
                "{} of {} paths diverged",
                self.diverged, self.compared
            ));
        }
        if self.flipped > MAX_FLIPPED_VERDICTS {
            return Err(format!("{} verdicts flipped", self.flipped));
        }
        Ok(())
    }
}

/// The tilt accounting counts a flipped verdict and its divergent path, so neither escapes the
/// limits.
#[test]
fn tilt_accounting_counts_flipped_and_divergent_cases() {
    let mut tilt = Tilt::default();
    for _ in 0..19 {
        tilt.record(true, 0.0, 1e-3);
    }
    tilt.record(false, 0.3, 1e-3);
    assert_eq!(
        tilt,
        Tilt {
            compared: 20,
            diverged: 1,
            flipped: 1,
        }
    );
    assert_eq!(tilt.check(), Ok(()));
    tilt.record(false, 0.3, 1e-3);
    assert!(tilt.check().is_err(), "2 of 21 paths diverged");
    let flips = Tilt {
        compared: 1000,
        diverged: 0,
        flipped: MAX_FLIPPED_VERDICTS + 1,
    };
    assert!(flips.check().is_err(), "too many flipped verdicts");
}

/// spec-d2 and the recommended row give the same cells on laws 1, 2 and 4 in a tilted frame
/// (over the cases whose scene can be tilted). Jolt is equivariant under a change of frame only
/// up to `f32` rounding, so paths are compared, once mapped back, within 1 mm on walkable ground
/// and 1 cm sliding down steep faces, whatever the verdicts; at most one case in twenty may leave
/// that band and at most [`MAX_FLIPPED_VERDICTS`] verdicts may flip; the cells are compared over
/// the cases whose verdicts did not flip. Divergent cases and flipped verdicts are printed.
#[test]
fn the_study_rows_give_the_same_cells_in_a_tilted_frame() {
    for config in [Config::spec_d2(), Config::recommended()] {
        let tilt = tilted_comparison(&config);
        println!("{}: {tilt:?}", config.name);
        assert!(tilt.compared > 100, "{}: {tilt:?}", config.name);
        if let Err(error) = tilt.check() {
            panic!("{}: {error}", config.name);
        }
    }
}

/// Plays `config` on laws 1, 2 and 4 in both frames, asserts equal cells over the cases with the
/// same verdict in both, and counts the cases.
fn tilted_comparison(config: &Config) -> Tilt {
    let mut scenes = Scenes::default();
    let frame = Frame::tilted();
    let mut tilt = Tilt::default();
    for column in [Column::Slope, Column::Floor, Column::Hops] {
        let (mut flat_results, mut turned_results) = (Vec::new(), Vec::new());
        for case in matrix::cases(&mut scenes, column) {
            let Some(turned) = tilted(&case) else {
                continue;
            };
            if case.report_only {
                continue;
            }
            let flat = play(scenes.get(case.scene), config, &case, case.start, false);
            let turned_run = play(
                scenes.get(turned.scene),
                config,
                &turned,
                turned.start,
                false,
            );
            let verdict = judge(&mut scenes, &case, &flat, None);
            let turned_verdict = judge(&mut scenes, &turned, &turned_run, None);
            let steep = matches!(
                case.check,
                Check::Slope(study::laws::slope::Check::Steep { .. })
                    | Check::Slope(study::laws::slope::Check::SteepRidge)
            );
            let tolerance = if steep { 1e-2 } else { 1e-3 };
            let apart = flat
                .reports
                .iter()
                .zip(&turned_run.reports)
                .map(|(a, b)| norm(sub(frame.to_local_point(b.end), a.end)))
                .fold(0.0, f64::max);
            let same_verdict = verdict.is_pass() == turned_verdict.is_pass();
            if apart > tolerance {
                println!("{} {}: paths {apart} m apart", config.name, case.id);
            }
            if !same_verdict {
                println!(
                    "{} {}: {verdict:?} here, {turned_verdict:?} tilted",
                    config.name, case.id
                );
            }
            tilt.record(same_verdict, apart, tolerance);
            // A flipped verdict is counted against its own limit; the cells compare the cases
            // both frames agree on, so one borderline case cannot move a cell's witness.
            if !same_verdict {
                continue;
            }
            flat_results.push(matrix::CaseOutcomes {
                id: case.id.clone(),
                outcomes: vec![verdict],
            });
            turned_results.push(matrix::CaseOutcomes {
                id: case.id.clone(),
                outcomes: vec![turned_verdict],
            });
        }
        assert_eq!(
            matrix::cell(&flat_results),
            matrix::cell(&turned_results),
            "{} column {}",
            config.name,
            column.label()
        );
    }
    tilt
}

/// Appends the bits of a tick report to `out`, for exact comparisons.
fn report_bits(r: &TickReport, out: &mut Vec<u8>) {
    for v in [
        r.start,
        r.up,
        r.maintenance,
        r.moving,
        r.post,
        r.end,
        r.velocity,
    ] {
        for c in v {
            out.extend_from_slice(&c.to_bits().to_le_bytes());
        }
    }
    for v in [r.supplied, r.jolt_velocity] {
        for c in common::math::bits(v) {
            out.extend_from_slice(&c.to_le_bytes());
        }
    }
    out.extend_from_slice(&r.vel_up.to_bits().to_le_bytes());
    out.extend_from_slice(&[
        u8::from(r.grounded),
        u8::from(r.sliding),
        u8::from(r.ceiling),
        r.ground_before as u8,
        r.ground_after_move as u8,
        r.ground as u8,
        u8::from(r.recovered),
        u8::from(r.pushed),
        u8::from(r.deep),
        u8::from(r.still_ran),
        u8::from(r.stepped),
        u8::from(r.snapped),
        u8::from(r.max_hits_exceeded),
    ]);
    out.extend_from_slice(&r.touched_groups.to_le_bytes());
    out.extend_from_slice(&r.ground_body.map_or(u32::MAX, BodyId::to_raw).to_le_bytes());
}

/// The determinism cases: down the radial crest at 7 m/s, and walking along a wall the capsule
/// overlaps by 5 cm.
fn determinism_cases(scenes: &mut Scenes) -> Vec<Case> {
    let mut cases = matrix::cases(scenes, Column::Radial);
    cases.extend(matrix::cases(scenes, Column::PushVelocity));
    cases
        .into_iter()
        .filter(|case| {
            case.id == "radial/2+4/crest44.5/descent/v7" || case.id == "5v/wall0.05/moving"
        })
        .collect()
}

/// The digest of `case_id` under spec-d2 in a scene built with `threads` worker threads: per
/// tick the report and the active contacts (body, sub-shape, normal bits) in Jolt's order.
fn determinism_digest(case_id: &str, threads: u32) -> common::determinism::Digest {
    let mut scenes = Scenes::default();
    let case = determinism_cases(&mut scenes)
        .into_iter()
        .find(|case| case.id == case_id)
        .unwrap();
    let mut scene = Scene::build_with_threads(case.scene, threads);
    let run = play(&mut scene, &Config::spec_d2(), &case, case.start, true);
    let mut digest = common::determinism::Digest::new();
    for (report, traced) in run.reports.iter().zip(&run.traces) {
        let tick = digest.push();
        report_bits(report, &mut tick.state);
        for contact in &traced.contacts {
            tick.state
                .extend_from_slice(&contact.body.unwrap_or(u32::MAX).to_le_bytes());
            tick.state
                .extend_from_slice(&contact.sub_shape.to_le_bytes());
            for c in contact.contact_normal {
                tick.state.extend_from_slice(&c.to_bits().to_le_bytes());
            }
        }
    }
    digest
}

/// The same study run twice in one process gives the same reports, bit for bit.
#[test]
fn a_study_run_is_bit_exact_when_repeated() {
    let mut scenes = Scenes::default();
    let cases = determinism_cases(&mut scenes);
    assert_eq!(cases.len(), 2);
    for case in cases {
        let a = determinism_digest(&case.id, 1);
        let b = determinism_digest(&case.id, 1);
        assert!(!a.ticks.is_empty());
        assert_eq!(a, b, "{}", case.id);
    }
}

/// Child process of `study_runs_match_across_processes_and_worker_counts`.
#[test]
#[ignore = "run as a child process by the determinism gate"]
fn study_child() {
    let Some((scenario, threads, _)) = common::determinism::child_request() else {
        return;
    };
    common::determinism::finish_child(&determinism_digest(&scenario, threads));
}

/// The study's caller gives the same ticks in separate processes with 1 and 4 worker threads.
/// No world step runs: this guards the caller and the character queries, not solver
/// scheduling.
#[test]
#[cfg(not(target_arch = "wasm32"))]
fn study_runs_match_across_processes_and_worker_counts() {
    let mut scenes = Scenes::default();
    for case in determinism_cases(&mut scenes) {
        let one = common::determinism::digest_in_child("study_child", &case.id, 1, "-");
        let four = common::determinism::digest_in_child("study_child", &case.id, 4, "-");
        assert!(!one.ticks.is_empty());
        common::determinism::assert_same(&case.id, &one, &four);
    }
}

/// spec-d2 and the recommended row down the radial crest: the character state and the caller's
/// carry saved at tick 40, a detour of 20 ticks (other input, a teleport, recovery speed 0.3), a
/// restore, and ticks 40 to 80 replay the original run bit for bit.
#[test]
fn a_restored_study_run_continues_bit_for_bit_after_a_detour() {
    for config in [Config::spec_d2(), Config::recommended()] {
        restored_run_continues(&config);
    }
}

/// The detour test for `config`, in a scene of its own.
fn restored_run_continues(config: &Config) {
    let mut scenes = Scenes::default();
    let case = matrix::cases(&mut scenes, Column::Radial)
        .into_iter()
        .find(|case| case.id == "radial/2+4/crest44.5/descent/v1.6")
        .unwrap();
    let scene = scenes.get(case.scene);
    let walker = config.create_character(scene, case.start);
    let mut carry = case.carry;
    let mut original = Vec::new();
    let mut saved = None;
    let step = |scene: &mut Scene, carry: &mut Carry, desired: V3| {
        let report = tick(scene, &walker, config, carry, desired, None);
        scene.place_actor(report.end, common::walker::from_y_to(report.up));
        let mut bits = Vec::new();
        report_bits(&report, &mut bits);
        bits
    };
    for t in 0..80 {
        if t == 40 {
            let state = scene.world.character(walker.id).unwrap().save_state();
            saved = Some((state, carry));
        }
        let origin = study::controller::origin_of(&scene.world, &walker, 0.02);
        let desired = case.input.desired(scene, t, origin);
        original.push(step(scene, &mut carry, desired));
    }
    let (state, saved_carry) = saved.unwrap();
    let restore = |scene: &mut Scene, speed: f32| {
        let mut character = scene.world.character_mut(walker.id).unwrap();
        character.restore_state(&state).unwrap();
        character.set_penetration_recovery_speed(speed).unwrap();
    };

    restore(scene, 0.3);
    let mut detour = Carry {
        vel_up: 2.0,
        grounded: false,
    };
    for t in 0..20 {
        if t == 10 {
            let to = study::laws::resting(scene, 5.0, 0.0);
            scene
                .world
                .character_mut(walker.id)
                .unwrap()
                .set_position(study::config::position_for(to, up_at(to), 0.02))
                .unwrap();
        }
        let origin = study::controller::origin_of(&scene.world, &walker, 0.02);
        let desired = scene.up.tangent(origin, [0.0, 0.0, 1.0], 0.05);
        step(scene, &mut detour, desired);
    }

    restore(scene, config.settings.recovery_speed);
    let mut carry = saved_carry;
    for (t, expected) in original.iter().enumerate().skip(40) {
        let origin = study::controller::origin_of(&scene.world, &walker, 0.02);
        let desired = case.input.desired(scene, t, origin);
        assert!(
            step(scene, &mut carry, desired) == *expected,
            "{}: tick {t} differs after the restore",
            config.name
        );
    }
}
