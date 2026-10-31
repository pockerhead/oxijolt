# Soft bodies

A soft body is Jolt's particle body: vertices joined by constraints, with triangle faces that other
bodies collide with. It suits cloth, flags, ropes, pressurised balls and squishy solids. In oxijolt
a soft body is an ordinary body of its world: `PhysicsWorld::create_soft_body` returns a `BodyId`,
the body counts against the world's body limit, is removed with `remove_body`, saved by
`save_state`, moved by `rebase` and found by queries.

## A cloth on a pole

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;

    // A 5 x 5 cloth of 2 m, pinned (kinematic) at its two far corners.
    let n: u32 = 5;
    let mut vertices = Vec::new();
    for z in 0..n {
        for x in 0..n {
            let position = Vec3::new(0.5 * x as f32 - 1.0, 0.0, 0.5 * z as f32 - 1.0);
            let pinned = z == 0 && (x == 0 || x == n - 1);
            vertices.push(if pinned {
                SoftBodyVertex::kinematic(position)
            } else {
                SoftBodyVertex { inverse_mass: 10.0, ..SoftBodyVertex::new(position) }
            });
        }
    }
    let mut faces = Vec::new();
    for z in 0..n - 1 {
        for x in 0..n - 1 {
            let i = z * n + x;
            faces.push([i, i + n, i + n + 1]);
            faces.push([i, i + n + 1, i + 1]);
        }
    }
    let cloth = SoftBodySharedSettings::builder(vertices, faces)
        .create_constraints(SoftBodyBendType::Distance, SoftBodyVertexAttributes::default())
        .build()?;
    let id = world.create_soft_body(
        &cloth,
        &SoftBodySettings::default().position(RVec3::new(0.0, 3.0, 0.0)),
    )?;

    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    // The free edge hangs below the pinned one.
    let hanging = world.soft_body(id)?.vertices();
    assert!(hanging[(n * n - 1) as usize].position.y < 2.5);

    // Drop the cloth: unpin both corners with a vertex mass of 0.1 kg.
    let mut soft = world.soft_body_mut(id)?;
    soft.set_vertex_inverse_mass(0, 10.0)?;
    soft.set_vertex_inverse_mass(n - 1, 10.0)?;
    for _ in 0..30 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    assert!(world.soft_body(id)?.vertices()[0].position.y < 3.0);
    Ok(())
}
```

## Shared settings

`SoftBodySharedSettings::builder(vertices, faces)` collects the particles and faces, and `build`
checks them all before anything reaches Jolt. One built value may serve many bodies in many worlds
on many threads; each body keeps its own reference, so the settings may be dropped afterwards.

- **Vertices** (`SoftBodyVertex`) have a position relative to the body origin, a velocity and an
  inverse mass: 0 pins the vertex (kinematic, it moves only by its velocity), otherwise the inverse
  of a mass of 1 g to `limits::MAX_MASS`.
- **Faces** are triangles of vertex indices, wound counter-clockwise seen from outside. They are
  what other bodies collide with and what queries hit.
- **Generated constraints.** `create_constraints(bend_type, attributes)` makes Jolt's edge, shear,
  bend and long range attachment constraints from the faces, with the same
  `SoftBodyVertexAttributes` (compliances, LRA type and multiplier) for every vertex;
  `create_constraints_per_vertex` takes one set per vertex. Without either call, or explicit
  constraints, the particles are not tied together.
- **Explicit constraints.** `edge` (`SoftBodyEdge`), `dihedral_bend` (`SoftBodyDihedralBend`) and
  `volume` (`SoftBodyVolume`, one per tetrahedron of a solid body) add constraints to the generated
  ones.

`SoftBodySettings` places one body (position, rotation, object layer) and holds the per-body values:
iterations, damping, friction, restitution, gravity factor, pressure, vertex radius and the speed
limit. The defaults are Jolt's.

## Reading and writing vertices

`PhysicsWorld::soft_body(id)` gives every vertex in world space (`vertices`, or `vertices_into` to
reuse a buffer). `PhysicsWorld::soft_body_mut(id)` writes them; every setter checks its input first,
changes nothing when it fails, and wakes the body when it succeeds:
- `set_vertex_velocity` sets a world-space velocity;
- `set_vertex_inverse_mass` pins (0) or unpins a vertex; Jolt then recomputes the body's mass and
  inertia;
- `move_kinematic_vertex(index, target, dt)` moves a pinned vertex to a target over the next step.
  The velocity it sets stays, as for a kinematic body, until the vertex is moved again or stopped.

`BodyMut::add_force` works on a soft body: Jolt spreads the force over the vertices. Jolt clears a
soft body's force after each collision step, so with `PhysicsWorld::step_with_collision_steps` the
force acts in the first collision step of the call only, while gravity acts in every one. To apply
a force in every sub-step, call `step` once per sub-step and add the force before each.

## What a soft body refuses

- `BodyMut::set_linear_velocity`, `set_angular_velocity` and `add_torque`: Jolt keeps velocities per
  vertex and ignores these for a soft body (`BodyError::NotRigidBody`).
- `BodyMut::add_force_at_point`: Jolt applies the force at the centre but also adds its torque,
  which a soft body never clears.
- Constraints and vehicles: Jolt's constraints cannot operate on soft bodies
  (`ConstraintError::Body(BodyError::NotRigidBody)`); pin a vertex and move it instead.
- Wheels look through soft bodies: Jolt's vehicle constraint solves the body under a wheel as a
  rigid body, so the wheel collision testers report only the rigid ground below.
- Soft bodies collide with rigid bodies, not with each other; Jolt does not implement that.

## Mass, inertia and pressure rules

These rules keep Jolt's mass, inertia and pressure arithmetic finite for the body as created;
[limits.md](limits.md) derives them. They are checked when the body is created, and vertex masses
and forces again when they change. Each rule refuses some bodies Jolt would simulate; the
workarounds are listed with them. A body that shrinks after creation, and solver settings that
diverge, are not covered ([coverage.md](coverage.md#not-covered)).

- **1 g per vertex.** A movable vertex weighs at least `limits::MIN_MASS` (1 g), and the movable
  vertices together at most `limits::MAX_MASS`. A light, finely divided body is refused at its real
  masses: a 1 m² cotton cloth of 0.15 kg at 21 × 21 vertices (0.34 g each), a 2 m flag of 0.4 kg at
  30 × 30, or a 57 g ball of 162 vertices. Use fewer vertices or give each at least 1 g. Gravity and
  damping move a vertex the same at any mass; pressure and added forces move a heavier vertex less,
  and a heavier body pushes the rigid bodies it touches harder.
- **Vertices around the origin.** Jolt computes a body's inertia in `f32` about the body origin and
  decomposes it unless a vertex is kinematic. Vertices far from the origin compared with their
  spread, or on one line, are refused ([limits.md](limits.md#soft-body-inertia)). Author the
  vertices around the origin and place the body with `SoftBodySettings::position`. The check is
  conservative for thin bodies: a free ribbon narrower than about 1/70 of its length, or a tube of
  radius below about 1/130 of its length, is refused even centred on the origin, although Jolt
  decomposes it. Widen it or make a vertex kinematic, which skips the check.
- **Pressure** needs counter-clockwise faces that enclose, about the body origin, a volume large
  enough for it at the start geometry ([limits.md](limits.md#soft-body-pressure)). The error bound
  of that volume grows with the cube of the distance from the origin, so a pressurised body must
  also keep its vertices around the origin.
- **Forces.** The force added to a soft body in one step is bounded by the acceleration it gives its
  lightest vertex, and unpinning a vertex rechecks it.

## Saving, rebasing, events

- A `WorldState` holds the vertex positions and velocities and the bounds; vertex inverse masses are
  configuration it does not save ([state.md](state.md)).
- `rebase` moves the vertices with the body: they are stored relative to it.
- Soft body contacts and validations are events of their own (`EventSettings::soft_body_contacts`,
  `soft_body_validations`), and a `ContactListener` can change or reject a soft body contact
  ([events.md](events.md)).

## Skinning

Skinned constraints tie vertices to the joints of an animated skeleton: a cape on a character's
shoulders, a skirt on its hips. Each skinning call moves every skinned vertex's target, its skinned
position, with the joints; the step then keeps the simulated vertex within a max distance of that
target and outside a back stop sphere behind it.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;

    // The character's skeleton in the world, as `RagdollRef::pose` returns it: one joint, the
    // chest, 1.4 m up and `x` to the side.
    let chest_at = |x: f32| SkeletonPose {
        root_offset: RVec3::new(0.0, 1.4, 0.0),
        joints: vec![JointTransform { translation: Vec3::new(x, 0.0, 0.0), rotation: Quat::IDENTITY }],
    };

    // A 5 x 5 cape of 1 m hanging behind the chest. Its top row is attached to the chest and
    // kinematic, the next row may stray 5 cm from its skinned position, the rest is free.
    let n: u32 = 5;
    let mut vertices = Vec::new();
    for row in 0..n {
        for x in 0..n {
            let position = Vec3::new(0.25 * x as f32 - 0.5, -0.25 * row as f32, 0.0);
            vertices.push(if row == 0 {
                SoftBodyVertex::kinematic(position)
            } else {
                SoftBodyVertex { inverse_mass: 10.0, ..SoftBodyVertex::new(position) }
            });
        }
    }
    let mut faces = Vec::new();
    for row in 0..n - 1 {
        for x in 0..n - 1 {
            let i = row * n + x;
            faces.push([i, i + n, i + n + 1]);
            faces.push([i, i + n + 1, i + 1]);
        }
    }
    // The cape's vertex frame sits 20 cm behind the chest while the chest is in its start pose.
    let cape_position = RVec3::new(0.0, 1.4, -0.2);
    let binds = SoftBodyInvBind::from_pose(&chest_at(0.0), cape_position, Quat::IDENTITY)?;
    let mut chest_only = [SoftBodySkinWeight::default(); 4];
    chest_only[0] = SoftBodySkinWeight::new(0, 1.0);
    let mut builder = SoftBodySharedSettings::builder(vertices, faces)
        .create_constraints(SoftBodyBendType::Distance, SoftBodyVertexAttributes::default())
        .inv_binds(binds);
    for x in 0..n {
        builder = builder
            .skinned(SoftBodySkinned { max_distance: Some(0.0), ..SoftBodySkinned::new(x, chest_only) })
            .skinned(SoftBodySkinned { max_distance: Some(0.05), ..SoftBodySkinned::new(n + x, chest_only) });
    }
    let cape = builder.build()?;
    let id = world.create_soft_body(&cape, &SoftBodySettings::default().position(cape_position))?;

    // Skin before every step: the chest moves 0.5 m to the side within a second.
    for tick in 1..=60 {
        world.soft_body_mut(id)?.skin_vertices(&chest_at(tick as f32 / 120.0))?;
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    // The attached corner, bound 0.5 m left of the chest, went with it.
    let corner = world.soft_body(id)?.vertices()[0].position;
    assert!(corner.x.abs() < 1.0e-3);
    Ok(())
}
```

- **Frames.** An inverse bind (`SoftBodyInvBind`) is a joint's pose in the settings frame, the
  frame of the vertex positions, at the moment the vertices are in their bind positions.
  `SoftBodyInvBind::from_pose(pose, position, rotation)` makes one per joint from a world pose and
  the place the settings frame had in the world then. `skin_vertices` takes a world pose, such as
  `RagdollRef::pose` returns, and converts it to the body's centre of mass frame itself, in double
  precision with the `double-precision` feature. Every joint up to the largest bound one, bound
  or not, must stay within `limits::MAX_SHAPE_EXTENT` (per axis) of the body's centre of mass
  ([limits.md](limits.md#soft-body-skinning)).
- **Start.** A new skinned body starts with its skin at the bind pose and holds it until the first
  skinning call.
- **Weights.** A vertex takes up to four `SoftBodySkinWeight`s, each naming an inverse bind by the
  order the binds were added. Zero weights are ignored; the others are normalised to add up to 1.
- **Max distance.** `None` leaves the vertex free, `Some(0.0)` attaches it to its skinned position,
  `Some(d)` keeps it within `d` metres.
- **Back stop.** `SoftBodyBackStop { distance, radius }` is a sphere whose surface is `distance`
  behind the skinned position, against the average normal of the faces around the vertex whose
  three vertices are all skinned; a large radius approximates a plane. `build` refuses a back stop
  whose distance is not below the max distance, since Jolt would ignore it, and one on a vertex whose
  faces give it no normal at the bind pose: none of its faces has three skinned vertices, or their
  normals cancel, as for a face listed with both windings
  ([limits.md](limits.md#soft-body-skinning)). Jolt computes the normal again from the skinned
  positions at each skinning call. A pose that folds two such faces exactly onto each other, or
  collapses them, leaves the vertex without a normal until the next call; the sphere then sits on
  the skinned position and pushes the vertex out to its radius. Jolt skins in `f32`, so a sliver
  face, thinner than about `1e-7` of its size, can lose its normal the same way without any fold.
- **Calls per step.** Call `skin_vertices` once before each step; during the step the skinned
  positions move from those of the previous call to these. A step without a call holds the last
  skinned positions. With several collision steps the first one moves the skinned positions and
  the later ones hold them. `hard_skin_vertices` also puts every skinned vertex on its skinned
  position at once with zero velocity, for a teleport. Each call wakes the body.
- **Multiplier.** `set_skinned_max_distance_multiplier` scales every max distance of a body: 0
  attaches every skinned vertex, values above 1 loosen the cloth. Jolt applies a back stop only
  while its distance is below the scaled max distance, so a multiplier below 1 switches off the
  back stops that this no longer holds for.
- **Attached rows.** Jolt solves the skinned constraints before the edges in each step, so the edges
  pull an attached vertex that has mass a little off its target; in the tests' cloth the attached
  row ended a few millimetres away. Make the attached row kinematic (inverse mass 0), as in the
  example, and the edges cannot move it.
- **Pressure.** Animation can squeeze a pressurised body below the volume its pressure was checked
  against at creation, a case that is not covered ([coverage.md](coverage.md#not-covered)).

A `WorldState` saves each skinned vertex's skinned positions and normal. It does not save the
multiplier, nor whether a skinning call is waiting for the next step. A restore followed by a replay
is exact when the caller skins before every step and saves between a step and the next skinning
call; this is what the tests check ([state.md](state.md#what-is-not-saved)).
