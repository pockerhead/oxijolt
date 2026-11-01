//! Bounds of the body property setters: each accepts exactly the range of the matching
//! `BodySettings` value, refuses the next float outside it and every non-finite value without
//! changing anything, and the accepted extremes keep a contact scene finite.

mod common;

use common::{step, world};
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// One value setter with its getter and the word its refusal names.
struct Setter {
    name: &'static str,
    set: fn(&mut BodyMut<'_>, f32) -> Result<(), BodyError>,
    get: fn(&BodyRef<'_>) -> Option<f32>,
    accepted: &'static [f32],
    refused: &'static [f32],
}

const NON_FINITE: [f32; 3] = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY];

fn setters() -> [Setter; 5] {
    const MAX_FRICTION: f32 = limits::MAX_FRICTION;
    const MAX_GRAVITY: f32 = limits::MAX_GRAVITY_FACTOR;
    // The next floats outside each range, as literals because `next_up` is not `const`.
    const BELOW_ZERO: f32 = -f32::from_bits(1);
    const ABOVE_ONE: f32 = 1.0000001;
    const ABOVE_1000: f32 = 1000.00006;
    assert_eq!(BELOW_ZERO, 0.0f32.next_down());
    assert_eq!(ABOVE_ONE, 1.0f32.next_up());
    assert_eq!(ABOVE_1000, MAX_FRICTION.next_up());
    assert_eq!(ABOVE_1000, MAX_GRAVITY.next_up());
    [
        Setter {
            name: "friction",
            set: |body, value| body.set_friction(value),
            get: |body| Some(body.friction()),
            accepted: &[0.0, f32::MIN_POSITIVE, 0.5, MAX_FRICTION],
            refused: &[BELOW_ZERO, ABOVE_1000, -1.0],
        },
        Setter {
            name: "restitution",
            set: |body, value| body.set_restitution(value),
            get: |body| Some(body.restitution()),
            accepted: &[0.0, 0.5, 1.0],
            refused: &[BELOW_ZERO, ABOVE_ONE],
        },
        Setter {
            name: "gravity factor",
            set: |body, value| body.set_gravity_factor(value),
            get: |body| body.gravity_factor(),
            accepted: &[-MAX_GRAVITY, -1.0, 0.0, MAX_GRAVITY],
            refused: &[-ABOVE_1000, ABOVE_1000],
        },
        Setter {
            name: "linear damping",
            set: |body, value| body.set_linear_damping(value),
            get: |body| body.linear_damping(),
            accepted: &[0.0, 0.5, f32::MAX],
            refused: &[BELOW_ZERO, -1.0],
        },
        Setter {
            name: "angular damping",
            set: |body, value| body.set_angular_damping(value),
            get: |body| body.angular_damping(),
            accepted: &[0.0, 0.5, f32::MAX],
            refused: &[BELOW_ZERO, -1.0],
        },
    ]
}

#[test]
fn setters_accept_their_range_and_refuse_beyond() {
    let mut world = world(GRAVITY, 1);
    let id = common::add_cube(&mut world, RVec3::new(0.0, 5.0, 0.0));
    for setter in setters() {
        let mut body = world.body_mut(id).unwrap();
        for &value in setter.accepted {
            (setter.set)(&mut body, value).unwrap();
            assert_eq!(
                (setter.get)(&body).map(f32::to_bits),
                Some(value.to_bits()),
                "{} {value}",
                setter.name
            );
        }
        let kept = (setter.get)(&body).map(f32::to_bits);
        for &value in setter.refused.iter().chain(&NON_FINITE) {
            match (setter.set)(&mut body, value) {
                Err(BodyError::InvalidValue(rule)) => {
                    assert!(rule.contains(setter.name), "{}: {rule}", setter.name)
                }
                other => panic!("{} {value}: {other:?}", setter.name),
            }
            assert_eq!(
                (setter.get)(&body).map(f32::to_bits),
                kept,
                "{} {value}",
                setter.name
            );
        }
    }
}

fn finite(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

fn finite_position(p: RVec3) -> bool {
    p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
}

#[test]
fn accepted_bounds_keep_a_contact_scene_finite() {
    // A spinning, sliding cube 1 cm above a floor, which gives a speculative contact with zero
    // normal impulse, with every property at an accepted extreme: the case in which an
    // infinite combined friction once turned velocities into NaN.
    for gravity_factor in [limits::MAX_GRAVITY_FACTOR, -limits::MAX_GRAVITY_FACTOR] {
        for damping in [0.0, f32::MAX] {
            for quality in [MotionQuality::Discrete, MotionQuality::LinearCast] {
                let mut world = world(GRAVITY, 1);
                let floor = common::add_floor(&mut world);
                let cube = world
                    .create_body(
                        &Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap(),
                        &BodySettings::new_dynamic()
                            .position(RVec3::new(0.0, 0.51, 0.0))
                            .linear_velocity(Vec3::new(1.0, 0.0, 0.0))
                            .angular_velocity(Vec3::new(0.0, 5.0, 0.0))
                            .allow_sleeping(false),
                    )
                    .unwrap();
                for id in [floor, cube] {
                    let mut body = world.body_mut(id).unwrap();
                    body.set_friction(limits::MAX_FRICTION).unwrap();
                    body.set_restitution(1.0).unwrap();
                }
                let mut body = world.body_mut(cube).unwrap();
                body.set_gravity_factor(gravity_factor).unwrap();
                body.set_linear_damping(damping).unwrap();
                body.set_angular_damping(damping).unwrap();
                body.set_motion_quality(quality).unwrap();
                for tick in 0..120 {
                    step(&mut world, 1);
                    let body = world.body(cube).unwrap();
                    assert!(
                        finite_position(body.position())
                            && finite(body.linear_velocity())
                            && finite(body.angular_velocity()),
                        "factor {gravity_factor}, damping {damping}, {quality:?}, tick {tick}"
                    );
                }
            }
        }
    }
}
