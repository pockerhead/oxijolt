//! The cross-platform determinism scene: a pile of boxes, spheres, capsules and cylinders dropped
//! onto a floor, stepped a fixed number of ticks, with one text record per body and tick.
//!
//! The wasm module (`scene_*` exports, driven by `scene.mjs`) and the native example
//! `scene_records` both include this file, so the two runs share the scene and the record format.
//! Setup uses literals and integer arithmetic only: no libm, whose results differ between Rust's
//! wasm math and a native C runtime.

use oxijolt::{BodyId, BodySettings, PhysicsWorld, Quat, RVec3, Real, Shape, Vec3, WorldSettings};

/// Step length in seconds.
const DELTA_TIME: f32 = 1.0 / 60.0;

/// Ticks the scene is stepped; records cover tick 0 (the created scene) to `TICKS`.
pub const TICKS: u32 = 300;

/// Dynamic bodies in the pile: three layers of eight.
const PILE: u32 = 24;

/// Unit quaternions the pile's bodies start in, by index: identity and 45 degrees about each
/// axis and 120 degrees about the diagonal.
const ROTATIONS: [Quat; 5] = [
    Quat::IDENTITY,
    Quat::from_xyzw(0.382_683_43, 0.0, 0.0, 0.923_879_5),
    Quat::from_xyzw(0.0, 0.382_683_43, 0.0, 0.923_879_5),
    Quat::from_xyzw(0.0, 0.0, 0.382_683_43, 0.923_879_5),
    Quat::from_xyzw(0.5, 0.5, 0.5, 0.5),
];

/// A world with a static floor and the pile, and the ids of its bodies in creation order.
pub struct Scene {
    world: PhysicsWorld,
    bodies: Vec<BodyId>,
}

impl Scene {
    /// Creates the scene; `None` when a creation fails.
    pub fn new() -> Option<Self> {
        let mut world = PhysicsWorld::new(WorldSettings::default().single_threaded()).ok()?;
        let floor = Shape::new_box(Vec3::new(20.0, 1.0, 20.0)).ok()?;
        let mut bodies = vec![world
            .create_body(
                &floor,
                &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)),
            )
            .ok()?];
        for i in 0..PILE {
            let settings = pile_body(i);
            bodies.push(world.create_body(&pile_shape(i)?, &settings).ok()?);
        }
        Some(Self { world, bodies })
    }

    /// Steps the world once; `false` when the step fails or is incomplete.
    pub fn step(&mut self) -> bool {
        matches!(self.world.step(DELTA_TIME), Ok(report) if report.is_complete())
    }

    /// Appends one line per body, in creation order: the tick, the body id as `index:sequence`,
    /// then position, rotation, linear and angular velocity as the hexadecimal bits of each
    /// component, and 1 for a sleeping body, else 0. `false` when a body is missing.
    pub fn write_records(&self, tick: u32, out: &mut String) -> bool {
        use std::fmt::Write;
        for &id in &self.bodies {
            let Ok(body) = self.world.body(id) else {
                return false;
            };
            let (p, q) = (body.position(), body.rotation());
            let (v, w) = (body.linear_velocity(), body.angular_velocity());
            let _ = write!(out, "{tick} {}:{}", id.index(), id.sequence());
            for real in [p.x, p.y, p.z] {
                let _ = write!(out, " {:x}", real_bits(real));
            }
            for float in [q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z] {
                let _ = write!(out, " {:x}", float.to_bits());
            }
            let _ = writeln!(out, " {}", u8::from(body.is_sleeping()));
        }
        true
    }

    /// Whether the scene did what the records are meant to cover: every pile body ended below
    /// its start height (it fell and landed) and at least one fell asleep.
    pub fn settled(&self) -> bool {
        let mut sleeping = 0;
        for (i, &id) in (0..PILE).zip(&self.bodies[1..]) {
            let Ok(body) = self.world.body(id) else {
                return false;
            };
            if body.position().y >= start_height(i) - 0.5 {
                return false;
            }
            sleeping += u32::from(body.is_sleeping());
        }
        sleeping > 0
    }
}

/// The bits of a position component: 32 in single precision, 64 in double.
// `Real` is `f64` with the `double-precision` feature, where the conversion is the identity.
#[allow(clippy::useless_conversion)]
fn real_bits(value: Real) -> u64 {
    value.to_bits().into()
}

/// Shape of pile body `i`: box, sphere, capsule and cylinder in turn.
fn pile_shape(i: u32) -> Option<Shape> {
    match i % 4 {
        0 => Shape::new_box(Vec3::new(0.4, 0.3, 0.5)),
        1 => Shape::new_sphere(0.45),
        2 => Shape::new_capsule(0.3, 0.3),
        _ => Shape::new_cylinder(0.35, 0.4),
    }
    .ok()
}

/// Height pile body `i` starts at: layers 1.5 m apart, from 1 m.
fn start_height(i: u32) -> Real {
    1.0 + 1.5 * (i / 8) as Real
}

/// Settings of pile body `i`: a 4 x 2 grid per layer with overlapping footprints (spacing 1 m),
/// each layer shifted by 0.3 m, a rotation from `ROTATIONS` and a small spin.
fn pile_body(i: u32) -> BodySettings {
    let layer = (i / 8) as Real;
    let x = (i % 4) as Real - 1.5 + 0.3 * layer;
    let z = ((i / 4) % 2) as Real - 0.5 + 0.3 * layer;
    let spin = (i % 3) as f32 - 1.0;
    BodySettings::new_dynamic()
        .position(RVec3::new(x, start_height(i), z))
        .rotation(ROTATIONS[(i % 5) as usize])
        .angular_velocity(Vec3::new(spin, 0.5 * spin, -spin))
}
