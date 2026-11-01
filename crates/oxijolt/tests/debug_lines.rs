//! Debug wireframe as line data: colliders within a radius, filters, the line cap and the
//! buffer's release.
#![cfg(feature = "debug-renderer")]

mod common;

use common::*;
use oxijolt::*;

/// The lines around the origin within `radius`, through `filter`.
fn lines_near(
    world: &PhysicsWorld,
    radius: f32,
    max_lines: usize,
    filter: &QueryFilter<'_>,
) -> DebugLines {
    let mut lines = DebugLines::new();
    let settings = DebugLineSettings::new(RVec3::ZERO, radius).max_lines(max_lines);
    world
        .debug_lines_into(&settings, filter, &mut lines)
        .unwrap();
    lines
}

/// The lines of one collider: a body and its compound child user data.
fn lines_of(lines: &DebugLines, body: BodyId, child: Option<u32>) -> Vec<DebugLine> {
    lines
        .lines()
        .iter()
        .filter(|line| line.body == body && line.child_user_data == child)
        .copied()
        .collect()
}

/// Asserts that every endpoint lies in the box `center ± half` (plus 1e-4).
fn assert_inside(lines: &[DebugLine], center: [Real; 3], half: [Real; 3], what: &str) {
    assert!(!lines.is_empty(), "{what} has no lines");
    for line in lines {
        for point in [line.from, line.to] {
            let p = [point.x, point.y, point.z];
            for axis in 0..3 {
                assert!(
                    (p[axis] - center[axis]).abs() <= half[axis] + 1e-4,
                    "{what}: {point:?} outside {center:?} ± {half:?}"
                );
            }
        }
    }
}

#[test]
fn near_colliders_present_far_absent_terrain_present() {
    let scene = wireframe_scene();
    let lines = lines_near(&scene.world, 20.0, usize::MAX, &QueryFilter::new());
    assert!(!lines.is_truncated());

    assert_inside(
        &lines_of(&lines, scene.terrain, None),
        [0.0, 0.0, 0.0],
        [16.0, 0.0, 16.0],
        "terrain",
    );
    let structure = lines_of(&lines, scene.chunk, Some(Groups::STRUCTURE));
    assert_eq!(structure.len() % 36, 0, "a box is 12 triangles");
    let (sin, cos) = Real::from(WIREFRAME_BOX_TURN).sin_cos();
    assert_inside(
        &structure,
        [5.0, 1.0, 5.0],
        [cos + sin, 1.0, cos + sin],
        "structure box",
    );
    assert_inside(
        &lines_of(&lines, scene.chunk, Some(Groups::FEATURE)),
        [9.0, 1.0, 5.0],
        [0.5, 1.0, 0.5],
        "feature cylinder",
    );
    assert_inside(
        &lines_of(&lines, scene.capsule, None),
        [-5.0, 1.5, 0.0],
        [0.4, 1.1, 0.4],
        "capsule",
    );
    assert!(lines.lines().iter().all(|line| line.body != scene.far_box));
    assert_eq!(
        lines.lines().len(),
        [
            lines_of(&lines, scene.terrain, None).len(),
            structure.len(),
            lines_of(&lines, scene.chunk, Some(Groups::FEATURE)).len(),
            lines_of(&lines, scene.capsule, None).len(),
        ]
        .iter()
        .sum::<usize>(),
        "every line belongs to one of the near colliders"
    );
}

#[test]
fn line_cap_gives_exactly_the_cap_and_truncated() {
    let scene = wireframe_scene();
    let filter = QueryFilter::new();
    let all = lines_near(&scene.world, 20.0, usize::MAX, &filter);
    let n = all.lines().len();
    assert!(!all.is_truncated());

    // Caps inside the terrain, inside the chunk and inside the capsule.
    let terrain = lines_of(&all, scene.terrain, None).len();
    for k in [1, terrain / 2, terrain + 10, n - 1] {
        let capped = lines_near(&scene.world, 20.0, k, &filter);
        assert_eq!(capped.lines(), &all.lines()[..k], "cap {k}");
        assert!(capped.is_truncated(), "cap {k}");
    }
    let exact = lines_near(&scene.world, 20.0, n, &filter);
    assert_eq!(exact.lines(), all.lines());
    assert!(!exact.is_truncated());

    let none = lines_near(&scene.world, 20.0, 0, &filter);
    assert!(none.lines().is_empty());
    assert!(none.is_truncated());
}

#[test]
fn hidden_groups_vanish() {
    let scene = wireframe_scene();
    let [terrain, chunk, feature, item, actor] = scene.layers;

    let structures_only = QueryFilter::new().child_groups(1 << Groups::STRUCTURE);
    let lines = lines_near(&scene.world, 20.0, usize::MAX, &structures_only);
    assert!(!lines_of(&lines, scene.chunk, Some(Groups::STRUCTURE)).is_empty());
    assert!(lines_of(&lines, scene.chunk, Some(Groups::FEATURE)).is_empty());
    assert!(!lines_of(&lines, scene.terrain, None).is_empty());

    let without_terrain = [chunk, feature, item, actor];
    let filter = QueryFilter::new().object_layers(&without_terrain);
    let lines = lines_near(&scene.world, 20.0, usize::MAX, &filter);
    assert!(lines.lines().iter().all(|line| line.body != scene.terrain));
    assert!(!lines_of(&lines, scene.capsule, None).is_empty());

    let only_terrain = [terrain];
    let filter = QueryFilter::new().object_layers(&only_terrain);
    let lines = lines_near(&scene.world, 20.0, usize::MAX, &filter);
    assert!(lines.lines().iter().all(|line| line.body == scene.terrain));

    let filter = QueryFilter::new().exclude_body(scene.capsule);
    let lines = lines_near(&scene.world, 20.0, usize::MAX, &filter);
    assert!(lines.lines().iter().all(|line| line.body != scene.capsule));
    assert!(!lines_of(&lines, scene.terrain, None).is_empty());
}

#[test]
fn off_releases_buffers() {
    let scene = wireframe_scene();
    let filter = QueryFilter::new();
    let settings = DebugLineSettings::new(RVec3::ZERO, 20.0).max_lines(10);
    let mut lines = DebugLines::new();
    scene
        .world
        .debug_lines_into(&settings, &filter, &mut lines)
        .unwrap();
    assert_eq!(lines.lines().len(), 10);
    assert!(lines.is_truncated());

    lines.release();
    assert!(lines.lines().is_empty());
    assert!(!lines.is_truncated());

    let uncapped = DebugLineSettings::new(RVec3::ZERO, 20.0);
    scene
        .world
        .debug_lines_into(&uncapped, &filter, &mut lines)
        .unwrap();
    assert_eq!(
        lines.lines(),
        lines_near(&scene.world, 20.0, usize::MAX, &filter).lines()
    );
    assert!(!lines.is_truncated());
}

/// The raw body id, the child user data and the bits of every coordinate of each line.
fn fingerprint(lines: &DebugLines) -> Vec<(u32, Option<u32>, Vec<u8>)> {
    lines
        .lines()
        .iter()
        .map(|line| {
            let (from, to) = (line.from, line.to);
            let coords = [from.x, from.y, from.z, to.x, to.y, to.z];
            (
                line.body.to_raw(),
                line.child_user_data,
                coords.iter().flat_map(|c| c.to_le_bytes()).collect(),
            )
        })
        .collect()
}

#[test]
fn same_calls_give_identical_lines() {
    let scene = wireframe_scene();
    let filter = QueryFilter::new();
    let first = fingerprint(&lines_near(&scene.world, 20.0, usize::MAX, &filter));
    let second = fingerprint(&lines_near(&scene.world, 20.0, usize::MAX, &filter));
    let other = wireframe_scene();
    let rebuilt = fingerprint(&lines_near(&other.world, 20.0, usize::MAX, &filter));
    assert_eq!(first, second);
    assert_eq!(first, rebuilt);
    let settings = DebugLineSettings::new(RVec3::ZERO, 20.0);
    let allocated = scene.world.debug_lines(&settings, &filter).unwrap();
    assert_eq!(fingerprint(&allocated), first);
}

#[test]
fn invalid_input_is_rejected() {
    let scene = wireframe_scene();
    let other = wireframe_scene();
    let mut lines = DebugLines::new();
    let settings = DebugLineSettings::new(RVec3::ZERO, 20.0);
    let foreign = QueryFilter::new().exclude_body(other.capsule);
    assert_eq!(
        scene
            .world
            .debug_lines_into(&settings, &foreign, &mut lines),
        Err(QueryError::WrongWorld(other.capsule))
    );
    for settings in [
        DebugLineSettings::new(RVec3::ZERO, -1.0),
        DebugLineSettings::new(RVec3::ZERO, f32::NAN),
        DebugLineSettings::new(RVec3::new(Real::INFINITY, 0.0, 0.0), 1.0),
    ] {
        assert!(matches!(
            scene
                .world
                .debug_lines_into(&settings, &QueryFilter::new(), &mut lines),
            Err(QueryError::InvalidValue(_))
        ));
    }
}

#[test]
fn center_and_radius_are_bounded_by_the_frame() {
    let scene = wireframe_scene();
    let mut lines = DebugLines::new();
    let bound = limits::MAX_POSITION;
    // `Real` is `f32` without the `double-precision` feature, so the cast is a no-op there.
    #[allow(clippy::unnecessary_cast)]
    let largest_radius = (2.0 * bound) as f32;
    let accepted = [
        DebugLineSettings::new(RVec3::new(bound, -bound, bound), 1.0),
        DebugLineSettings::new(RVec3::ZERO, largest_radius),
    ];
    for settings in accepted {
        scene
            .world
            .debug_lines_into(&settings, &QueryFilter::new(), &mut lines)
            .unwrap();
    }
    let rejected = [
        DebugLineSettings::new(RVec3::new(bound.next_up(), 0.0, 0.0), 1.0),
        DebugLineSettings::new(RVec3::ZERO, largest_radius.next_up()),
    ];
    for settings in rejected {
        assert!(matches!(
            scene
                .world
                .debug_lines_into(&settings, &QueryFilter::new(), &mut lines),
            Err(QueryError::InvalidValue(_))
        ));
    }
}

#[test]
fn radius_edge_far_from_origin() {
    let (mut world, [_, chunk, ..]) = five_layer_world();
    let unit = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let radius = 5.0;
    let center = RVec3::new(790.0, 0.0, 0.0);
    // Nearest faces at radius - 0.001 (on +x) and radius + 0.01 (on -x) from the center.
    let inside = add_static_in(
        &mut world,
        &unit,
        RVec3::new(790.0 + 4.999 + 0.5, 0.0, 0.0),
        chunk,
    );
    let outside = add_static_in(
        &mut world,
        &unit,
        RVec3::new(790.0 - 5.01 - 0.5, 0.0, 0.0),
        chunk,
    );
    let mut lines = DebugLines::new();
    world
        .debug_lines_into(
            &DebugLineSettings::new(center, radius),
            &QueryFilter::new(),
            &mut lines,
        )
        .unwrap();
    assert_eq!(lines.lines().len(), 36);
    assert!(lines.lines().iter().all(|line| line.body == inside));
    assert!(lines.lines().iter().all(|line| line.body != outside));
}

/// `v` turned about y by `angle` radians.
fn turn_about_y(v: [Real; 3], angle: f32) -> [Real; 3] {
    let (sin, cos) = Real::from(angle).sin_cos();
    [cos * v[0] + sin * v[2], v[1], -sin * v[0] + cos * v[2]]
}

#[test]
fn compound_children_follow_body_and_child_rotation() {
    let (mut world, [_, chunk, ..]) = five_layer_world();
    let (body_turn, child_turn) = (0.5, 0.3);
    let half = [1.0, 0.5, 0.25];
    let block = Shape::new_box(Vec3::new(1.0, 0.5, 0.25)).unwrap();
    let marker = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    let up = Vec3::new(0.0, 1.0, 0.0);
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &block,
            position: Vec3::new(2.0, 1.0, 0.0),
            rotation: quat_about(up, child_turn),
            user_data: Groups::STRUCTURE,
        },
        CompoundChild {
            shape: &marker,
            position: Vec3::new(-1.0, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: Groups::FEATURE,
        },
    ])
    .unwrap();
    let origin = [5.0, 0.0, 5.0];
    let body = world
        .create_body(
            &compound,
            &BodySettings::new_static()
                .position(RVec3::new(origin[0], origin[1], origin[2]))
                .rotation(quat_about(up, body_turn))
                .object_layer(chunk),
        )
        .unwrap();

    let lines = lines_near(&world, 20.0, usize::MAX, &QueryFilter::new());
    let block_lines = lines_of(&lines, body, Some(Groups::STRUCTURE));
    assert_eq!(block_lines.len(), 36);
    let mut corners = Vec::new();
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            for sz in [-1.0, 1.0] {
                let local = turn_about_y([sx * half[0], sy * half[1], sz * half[2]], child_turn);
                let in_body = [local[0] + 2.0, local[1] + 1.0, local[2]];
                let world = turn_about_y(in_body, body_turn);
                corners.push([
                    world[0] + origin[0],
                    world[1] + origin[1],
                    world[2] + origin[2],
                ]);
            }
        }
    }
    let corner_of = |p: RVec3| {
        corners.iter().position(|c| {
            (p.x - c[0]).abs() < 1e-4 && (p.y - c[1]).abs() < 1e-4 && (p.z - c[2]).abs() < 1e-4
        })
    };
    let mut seen = [false; 8];
    for line in &block_lines {
        for point in [line.from, line.to] {
            let index = corner_of(point).unwrap_or_else(|| panic!("{point:?} is not a corner"));
            seen[index] = true;
        }
    }
    assert_eq!(seen, [true; 8]);
    assert_eq!(lines_of(&lines, body, Some(Groups::FEATURE)).len(), 36);
}

/// `v` rotated by the unit quaternion `q`, computed in `Real`: `v + 2w (u x v) + 2 u x (u x v)`
/// with `u` the vector part.
fn rotate(q: Quat, v: [Real; 3]) -> [Real; 3] {
    let cross = |a: [Real; 3], b: [Real; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let u = [q.x, q.y, q.z].map(Real::from);
    let w = Real::from(q.w);
    let t = cross(u, v).map(|c| 2.0 * c);
    let c = cross(u, t);
    [
        v[0] + w * t[0] + c[0],
        v[1] + w * t[1] + c[1],
        v[2] + w * t[2] + c[2],
    ]
}

#[test]
fn compound_child_pose_applies_child_rotation_before_body_rotation() {
    // The two rotations do not commute, so a swapped order moves the corners.
    let (mut world, [_, chunk, ..]) = five_layer_world();
    let half = [1.0, 0.5, 0.25];
    let block = Shape::new_box(Vec3::new(1.0, 0.5, 0.25)).unwrap();
    let marker = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    let child_rotation = quat_about(Vec3::new(0.0, 1.0, 0.0), 0.6);
    let axis_length = 1.04_f32.sqrt();
    let body_rotation = quat_about(Vec3::new(1.0 / axis_length, 0.0, 0.2 / axis_length), 0.9);
    let child_position = [2.0, 1.0, -0.5];
    let compound = Shape::new_compound(&[
        CompoundChild {
            shape: &block,
            position: Vec3::new(2.0, 1.0, -0.5),
            rotation: child_rotation,
            user_data: Groups::STRUCTURE,
        },
        CompoundChild {
            shape: &marker,
            position: Vec3::new(-1.0, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data: Groups::FEATURE,
        },
    ])
    .unwrap();
    let origin = [5.0, 0.0, 5.0];
    let body = world
        .create_body(
            &compound,
            &BodySettings::new_static()
                .position(RVec3::new(origin[0], origin[1], origin[2]))
                .rotation(body_rotation)
                .object_layer(chunk),
        )
        .unwrap();

    let mut corners = Vec::new();
    for sx in [-1.0, 1.0] {
        for sy in [-1.0, 1.0] {
            for sz in [-1.0, 1.0] {
                let local = rotate(child_rotation, [sx * half[0], sy * half[1], sz * half[2]]);
                let in_body = [
                    local[0] + child_position[0],
                    local[1] + child_position[1],
                    local[2] + child_position[2],
                ];
                let world = rotate(body_rotation, in_body);
                corners.push([
                    world[0] + origin[0],
                    world[1] + origin[1],
                    world[2] + origin[2],
                ]);
            }
        }
    }
    let lines = lines_near(&world, 30.0, usize::MAX, &QueryFilter::new());
    let block_lines = lines_of(&lines, body, Some(Groups::STRUCTURE));
    assert_eq!(block_lines.len(), 36);
    for line in &block_lines {
        for point in [line.from, line.to] {
            assert!(
                corners.iter().any(|c| {
                    (point.x - c[0]).abs() < 1e-4
                        && (point.y - c[1]).abs() < 1e-4
                        && (point.z - c[2]).abs() < 1e-4
                }),
                "{point:?} is not a corner"
            );
        }
    }
}

#[test]
fn compound_child_beyond_radius_is_skipped_and_nested_compounds_keep_top_group() {
    let (mut world, [_, chunk, ..]) = five_layer_world();
    let unit = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let unit_child = |position: Vec3, user_data: u32| CompoundChild {
        shape: &unit,
        position,
        rotation: Quat::IDENTITY,
        user_data,
    };
    let inner = Shape::new_compound(&[
        unit_child(Vec3::ZERO, 9),
        unit_child(Vec3::new(0.0, 2.0, 0.0), 10),
    ])
    .unwrap();
    let compound = Shape::new_compound(&[
        unit_child(Vec3::ZERO, 1),
        unit_child(Vec3::new(50.0, 0.0, 0.0), 2),
        CompoundChild {
            shape: &inner,
            position: Vec3::new(0.0, 0.0, 3.0),
            rotation: Quat::IDENTITY,
            user_data: 3,
        },
    ])
    .unwrap();
    let body = add_static_in(&mut world, &compound, RVec3::ZERO, chunk);

    let lines = lines_near(&world, 10.0, usize::MAX, &QueryFilter::new());
    assert!(lines.lines().iter().all(|line| line.body == body));
    assert_eq!(lines_of(&lines, body, Some(1)).len(), 36);
    assert!(
        lines_of(&lines, body, Some(2)).is_empty(),
        "far child drawn"
    );
    assert_eq!(lines_of(&lines, body, Some(3)).len(), 72, "nested compound");
    assert_eq!(lines.lines().len(), 108);

    // The far child is drawn once the sphere reaches it.
    let mut far = DebugLines::new();
    let settings = DebugLineSettings::new(RVec3::new(50.0, 0.0, 0.0), 1.0);
    world
        .debug_lines_into(&settings, &QueryFilter::new(), &mut far)
        .unwrap();
    assert_eq!(far.lines().len(), 36);
    assert!(far
        .lines()
        .iter()
        .all(|line| line.child_user_data == Some(2)));
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn worlds_sharing_a_height_field_draw_it_concurrently() {
    // The shape's debug geometry is built lazily by the first draw, so both threads race for it.
    let terrain = flat_height_field();
    let worlds: Vec<PhysicsWorld> = (0..2)
        .map(|_| {
            let (mut world, [terrain_layer, ..]) = five_layer_world();
            add_static_in(&mut world, &terrain, RVec3::ZERO, terrain_layer);
            world
        })
        .collect();
    let start = std::sync::Barrier::new(worlds.len());
    let drawn: Vec<_> = std::thread::scope(|scope| {
        let threads: Vec<_> = worlds
            .iter()
            .map(|world| {
                let start = &start;
                scope.spawn(move || {
                    start.wait();
                    (0..20)
                        .map(|_| {
                            fingerprint(&lines_near(world, 20.0, usize::MAX, &QueryFilter::new()))
                        })
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        threads
            .into_iter()
            .map(|thread| thread.join().unwrap())
            .collect()
    });
    let reference = fingerprint(&lines_near(
        &worlds[0],
        20.0,
        usize::MAX,
        &QueryFilter::new(),
    ));
    assert!(!reference.is_empty());
    for calls in drawn {
        assert!(calls.iter().all(|lines| *lines == reference));
    }
}

#[test]
fn a_cloth_is_drawn_as_the_edges_of_its_faces() {
    let mut world = world(Vec3::ZERO, 1);
    let cloth = common::soft_body::Cloth::new(4, 0.5);
    let shared = cloth.builder().build().unwrap();
    let id = world
        .create_soft_body(&shared, &SoftBodySettings::default())
        .unwrap();
    let lines = lines_near(&world, 10.0, usize::MAX, &QueryFilter::new());
    let drawn = lines_of(&lines, id, None);
    // Three lines per face.
    assert_eq!(drawn.len(), 3 * cloth.faces.len());
    assert_inside(&drawn, [0.0, 0.0, 0.0], [0.75, 0.0, 0.75], "cloth");
}

/// A two-triangle mesh: the unit square at y = 0 with corners (0, 0) and (1, 1) in x and z.
fn square_mesh() -> Shape {
    let vertices = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    ];
    Shape::new_mesh(&vertices, &[[0, 1, 2], [0, 2, 3]])
        .unwrap()
        .0
}

/// Line count Jolt's debug renderer gives a two-triangle mesh: measured once and pinned.
const SQUARE_MESH_LINES: usize = 6;

#[test]
fn new_shapes_are_drawn_within_their_bounds() {
    let mut world = world(Vec3::ZERO, 1);
    let add = |world: &mut PhysicsWorld, shape: &Shape, settings: BodySettings| {
        world.create_body(shape, &settings).unwrap()
    };
    let mesh = add(
        &mut world,
        &square_mesh(),
        BodySettings::new_static().position(RVec3::new(10.0, 0.0, 0.0)),
    );
    let points = common::meshes::irregular_points();
    let hull = add(
        &mut world,
        &Shape::new_convex_hull_with_convex_radius(&points, 0.05).unwrap(),
        BodySettings::new_dynamic().position(RVec3::new(0.0, 3.0, 0.0)),
    );
    let scaled_box = Shape::new_scaled(&cube_shape(), Vec3::new(1.0, 2.0, 3.0)).unwrap();
    let scaled = add(
        &mut world,
        &scaled_box,
        BodySettings::new_static().position(RVec3::new(-5.0, 1.0, 0.0)),
    );
    let tapered = add(
        &mut world,
        &Shape::new_tapered_capsule(0.4, 0.15, 0.3).unwrap(),
        BodySettings::new_static().position(RVec3::new(0.0, 1.0, 6.0)),
    );
    let lines = lines_near(&world, 30.0, usize::MAX, &QueryFilter::new());
    let mesh_lines = lines_of(&lines, mesh, None);
    assert_eq!(mesh_lines.len(), SQUARE_MESH_LINES);
    assert_inside(&mesh_lines, [10.5, 0.0, 0.5], [0.5, 0.0, 0.5], "mesh");
    assert_inside(
        &lines_of(&lines, hull, None),
        [0.0, 3.0, 0.0],
        [0.6, 0.65, 0.6],
        "hull",
    );
    assert_inside(
        &lines_of(&lines, scaled, None),
        [-5.0, 1.0, 0.0],
        [0.5, 1.0, 1.5],
        "scaled box",
    );
    assert_inside(
        &lines_of(&lines, tapered, None),
        [0.0, 1.0, 6.0],
        [0.3, 0.75, 0.3],
        "tapered capsule",
    );
    let again = lines_near(&world, 30.0, usize::MAX, &QueryFilter::new());
    assert_eq!(fingerprint(&lines), fingerprint(&again));
}

#[test]
fn a_mesh_can_be_drawn_again_after_its_shape_is_freed() {
    let mut world = world(Vec3::ZERO, 1);
    let shape = square_mesh();
    let id = world
        .create_body(&shape, &BodySettings::new_static())
        .unwrap();
    // The first draw builds the shape's debug geometry, which lives as long as the shape.
    let first = lines_near(&world, 10.0, usize::MAX, &QueryFilter::new());
    assert_eq!(first.lines().len(), SQUARE_MESH_LINES);
    world.remove_body(id).unwrap();
    drop(shape);
    assert!(lines_near(&world, 10.0, usize::MAX, &QueryFilter::new())
        .lines()
        .is_empty());
    let id = world
        .create_body(&square_mesh(), &BodySettings::new_static())
        .unwrap();
    let again = lines_near(&world, 10.0, usize::MAX, &QueryFilter::new());
    assert_eq!(lines_of(&again, id, None).len(), SQUARE_MESH_LINES);
}

#[test]
fn a_plane_is_drawn_as_two_triangles() {
    let mut world = world(Vec3::ZERO, 1);
    let plane = Shape::new_plane(Vec3::new(0.0, 1.0, 0.0), 0.0, 5.0).unwrap();
    let floor = world
        .create_body(&plane, &BodySettings::new_static())
        .unwrap();
    let lines = lines_near(&world, 20.0, usize::MAX, &QueryFilter::new());
    let plane_lines = lines_of(&lines, floor, None);
    assert_eq!(plane_lines.len(), 6, "two wire triangles");
    assert_inside(&plane_lines, [0.0, 0.0, 0.0], [5.0, 0.0, 5.0], "plane");
}
