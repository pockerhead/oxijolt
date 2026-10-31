use super::*;
use crate::limits::MAX_SHAPE_EXTENT;
use crate::soft_body::{SoftBodySharedSettings, SoftBodySharedSettingsBuilder, SoftBodyVertex};
use crate::{JointTransform, Real};

fn weight(inv_bind: u32, weight: f32) -> SoftBodySkinWeight {
    SoftBodySkinWeight::new(inv_bind, weight)
}

/// Weights with `first` as the only non-default entry.
fn single(first: SoftBodySkinWeight) -> [SoftBodySkinWeight; 4] {
    [
        first,
        SoftBodySkinWeight::default(),
        SoftBodySkinWeight::default(),
        SoftBodySkinWeight::default(),
    ]
}

fn bind(joint: u32) -> SoftBodyInvBind {
    SoftBodyInvBind::new(joint, Vec3::ZERO, Quat::IDENTITY)
}

fn triangle() -> SoftBodySharedSettingsBuilder {
    let vertices = vec![
        SoftBodyVertex::new(Vec3::new(0.0, 0.0, 0.0)),
        SoftBodyVertex::new(Vec3::new(1.0, 0.0, 0.0)),
        SoftBodyVertex::new(Vec3::new(0.0, 0.0, 1.0)),
    ];
    SoftBodySharedSettings::builder(vertices, vec![[0, 2, 1]])
}

/// A triangle whose vertices 0 and 1 are skinned to joint 0 and vertex 2 by `third`.
fn skinned_triangle(third: SoftBodySkinned) -> SoftBodySharedSettingsBuilder {
    triangle()
        .inv_bind(bind(0))
        .skinned(SoftBodySkinned::new(0, single(weight(0, 1.0))))
        .skinned(SoftBodySkinned::new(1, single(weight(0, 1.0))))
        .skinned(third)
}

fn third(weights: [SoftBodySkinWeight; 4]) -> SoftBodySkinned {
    SoftBodySkinned::new(2, weights)
}

fn rejected(builder: SoftBodySharedSettingsBuilder) -> &'static str {
    match builder.build() {
        Err(SoftBodyError::InvalidValue(what)) => what,
        Err(other) => panic!("unexpected error {other:?}"),
        Ok(_) => panic!("accepted"),
    }
}

#[test]
fn a_skinned_triangle_builds_with_its_counts() {
    let settings = skinned_triangle(third(single(weight(0, 1.0))))
        .inv_bind(bind(5))
        .build()
        .unwrap();
    assert_eq!(settings.inv_bind_count(), 2);
    assert_eq!(settings.skinned_constraint_count(), 3);
    assert_eq!(settings.skin_joint_count(), 6);
    let unskinned = triangle().build().unwrap();
    assert_eq!(unskinned.inv_bind_count(), 0);
    assert_eq!(unskinned.skinned_constraint_count(), 0);
    assert_eq!(unskinned.skin_joint_count(), 0);
}

#[test]
fn inverse_bind_joints_are_bounded_and_unique() {
    let with_joint = |joint| skinned_triangle(third(single(weight(0, 1.0)))).inv_bind(bind(joint));
    let settings = with_joint(MAX_SKIN_JOINTS - 1).build().unwrap();
    assert_eq!(settings.skin_joint_count(), MAX_SKIN_JOINTS);
    assert_eq!(rejected(with_joint(MAX_SKIN_JOINTS)), JOINT_RULE);
    assert_eq!(rejected(with_joint(0)), UNIQUE_JOINT_RULE);
}

#[test]
fn inverse_bind_poses_are_checked() {
    let with_bind = |bind| skinned_triangle(third(single(weight(1, 1.0)))).inv_bind(bind);
    let at = |position| SoftBodyInvBind::new(1, position, Quat::IDENTITY);
    with_bind(at(Vec3::new(MAX_SHAPE_EXTENT, -MAX_SHAPE_EXTENT, 0.0)))
        .build()
        .unwrap();
    let beyond = MAX_SHAPE_EXTENT.next_up();
    assert_eq!(
        rejected(with_bind(at(Vec3::new(0.0, beyond, 0.0)))),
        BIND_POSITION_RULE
    );
    assert_eq!(
        rejected(with_bind(at(Vec3::new(f32::NAN, 0.0, 0.0)))),
        BIND_POSITION_RULE
    );
    let turned = SoftBodyInvBind::new(1, Vec3::ZERO, Quat::from_xyzw(0.0, 0.0, 0.0, 2.0));
    assert_eq!(rejected(with_bind(turned)), ROTATION_RULE);
}

#[test]
fn inverse_binds_and_skinned_constraints_go_together() {
    assert_eq!(rejected(triangle().inv_bind(bind(0))), PAIRING_RULE);
    let lonely = triangle().skinned(SoftBodySkinned::new(0, single(weight(0, 1.0))));
    assert_eq!(rejected(lonely), PAIRING_RULE);
}

#[test]
fn skinned_vertices_exist_and_are_unique() {
    let missing = SoftBodySkinned::new(3, single(weight(0, 1.0)));
    assert_eq!(rejected(skinned_triangle(missing)), VERTEX_RULE);
    let twice = SoftBodySkinned::new(1, single(weight(0, 1.0)));
    assert_eq!(rejected(skinned_triangle(twice)), UNIQUE_VERTEX_RULE);
}

#[test]
fn weights_are_finite_and_not_negative() {
    let with_second =
        |w: f32| third([weight(0, 1.0), weight(0, w), weight(0, 0.0), weight(0, 0.0)]);
    assert_eq!(
        rejected(skinned_triangle(with_second(f32::NAN))),
        WEIGHT_RULE
    );
    assert_eq!(
        rejected(skinned_triangle(with_second(f32::INFINITY))),
        WEIGHT_RULE
    );
    assert_eq!(
        rejected(skinned_triangle(with_second(-1.0e-45))),
        WEIGHT_RULE
    );
    skinned_triangle(with_second(-0.0)).build().unwrap();
    let only_negative_zero = third(single(weight(0, -0.0)));
    assert_eq!(
        rejected(skinned_triangle(only_negative_zero)),
        POSITIVE_WEIGHT_RULE
    );
}

#[test]
fn only_positive_weights_need_an_inverse_bind() {
    let all_zero = third(single(weight(0, 0.0)));
    assert_eq!(rejected(skinned_triangle(all_zero)), POSITIVE_WEIGHT_RULE);
    let zero_out_of_range = third([
        weight(7, 0.0),
        weight(0, 1.0),
        weight(9, 0.0),
        weight(0, 0.0),
    ]);
    skinned_triangle(zero_out_of_range).build().unwrap();
    let positive_out_of_range = third([
        weight(0, 1.0),
        weight(1, 1.0),
        weight(0, 0.0),
        weight(0, 0.0),
    ]);
    assert_eq!(
        rejected(skinned_triangle(positive_out_of_range)),
        WEIGHT_INDEX_RULE
    );
}

#[test]
fn weights_are_normalised_in_order_after_dropping_zeros() {
    let weights = [
        weight(2, 0.0),
        weight(1, 3.0),
        weight(0, 0.0),
        weight(3, 1.0),
    ];
    let normalized = normalized_weights(&weights, 4).unwrap();
    let pairs = normalized.map(|w| (w.invBindIndex, w.weight));
    assert_eq!(pairs, [(1, 0.75), (3, 0.25), (0, 0.0), (0, 0.0)]);
}

#[test]
fn the_largest_weights_normalise_without_overflow() {
    // Jolt's own `NormalizeWeights` sums in `f32` and would reach infinity here.
    let normalized = normalized_weights(&[weight(0, f32::MAX); 4], 1).unwrap();
    assert!(normalized.iter().all(|w| w.weight == 0.25));
}

#[test]
fn a_weight_that_vanishes_when_normalised_is_refused() {
    let weights = [
        weight(0, f32::MAX),
        weight(0, 1.0e-45),
        weight(0, 0.0),
        weight(0, 0.0),
    ];
    assert_eq!(
        rejected(skinned_triangle(third(weights))),
        NORMALIZED_WEIGHT_RULE
    );
}

#[test]
fn max_distances_are_bounded() {
    let with_max = |max_distance| {
        skinned_triangle(SoftBodySkinned {
            max_distance: Some(max_distance),
            ..third(single(weight(0, 1.0)))
        })
    };
    with_max(0.0).build().unwrap();
    with_max(MAX_SHAPE_EXTENT).build().unwrap();
    assert_eq!(
        rejected(with_max(MAX_SHAPE_EXTENT.next_up())),
        MAX_DISTANCE_RULE
    );
    assert_eq!(rejected(with_max(f32::NAN)), MAX_DISTANCE_RULE);
    assert_eq!(rejected(with_max(-0.5)), MAX_DISTANCE_RULE);
}

fn with_back_stop(
    max_distance: Option<f32>,
    distance: f32,
    radius: f32,
) -> SoftBodySharedSettingsBuilder {
    skinned_triangle(SoftBodySkinned {
        max_distance,
        back_stop: Some(SoftBodyBackStop { distance, radius }),
        ..third(single(weight(0, 1.0)))
    })
}

#[test]
fn back_stops_are_bounded() {
    with_back_stop(None, MAX_SHAPE_EXTENT, MAX_SHAPE_EXTENT)
        .build()
        .unwrap();
    with_back_stop(Some(0.2), 0.1, 40.0).build().unwrap();
    assert_eq!(
        rejected(with_back_stop(None, f32::NAN, 1.0)),
        BACK_STOP_DISTANCE_RULE
    );
    assert_eq!(
        rejected(with_back_stop(None, -0.1, 1.0)),
        BACK_STOP_DISTANCE_RULE
    );
    assert_eq!(
        rejected(with_back_stop(None, 0.1, 0.0)),
        BACK_STOP_RADIUS_RULE
    );
    assert_eq!(
        rejected(with_back_stop(None, 0.1, f32::NAN)),
        BACK_STOP_RADIUS_RULE
    );
    let too_large = MAX_SHAPE_EXTENT.next_up();
    assert_eq!(
        rejected(with_back_stop(None, 0.1, too_large)),
        BACK_STOP_RADIUS_RULE
    );
}

#[test]
fn a_back_stop_starts_below_the_max_distance() {
    // Jolt ignores a back stop at or beyond the max distance.
    assert_eq!(
        rejected(with_back_stop(Some(0.2), 0.2, 1.0)),
        BACK_STOP_ORDER_RULE
    );
    assert_eq!(
        rejected(with_back_stop(Some(0.0), 0.0, 1.0)),
        BACK_STOP_ORDER_RULE
    );
}

#[test]
fn a_back_stop_needs_an_all_skinned_face() {
    // Vertex 1 is not skinned, so the triangle gives vertex 2 no normal.
    let builder = triangle()
        .inv_bind(bind(0))
        .skinned(SoftBodySkinned::new(0, single(weight(0, 1.0))))
        .skinned(SoftBodySkinned {
            back_stop: Some(SoftBodyBackStop {
                distance: 0.1,
                radius: 1.0,
            }),
            ..SoftBodySkinned::new(2, single(weight(0, 1.0)))
        });
    assert_eq!(rejected(builder), BACK_STOP_FACE_RULE);
}

/// A fan of `faces` triangles around vertex 0, every vertex skinned to joint 0.
fn skinned_fan(faces: u32) -> SoftBodySharedSettingsBuilder {
    let step = 1.5 * std::f32::consts::PI / faces as f32;
    let mut vertices = vec![SoftBodyVertex::new(Vec3::ZERO)];
    vertices.extend((0..=faces).map(|i| {
        let (sin, cos) = (i as f32 * step).sin_cos();
        SoftBodyVertex::new(Vec3::new(cos, 0.0, sin))
    }));
    let triangles = (1..=faces).map(|i| [0, i + 1, i]).collect();
    let count = vertices.len() as u32;
    let mut builder = SoftBodySharedSettings::builder(vertices, triangles).inv_bind(bind(0));
    for vertex in 0..count {
        builder = builder.skinned(SoftBodySkinned::new(vertex, single(weight(0, 1.0))));
    }
    builder
}

#[test]
fn a_skinned_vertex_touches_fewer_than_256_all_skinned_faces() {
    skinned_fan(255).build().unwrap();
    assert_eq!(rejected(skinned_fan(256)), FACES_PER_VERTEX_RULE);
}

/// Checks `groups` skinned triangles of their own, each listed 255 times, plus `extra` more
/// such triangles listed once.
fn validate_skinned_triangles(groups: u32, extra: u32) -> Result<(), SoftBodyError> {
    let triangles = groups + extra;
    let faces: Vec<[u32; 3]> = (0..groups)
        .flat_map(|group| std::iter::repeat_n(group, 255))
        .chain(groups..triangles)
        .map(|group| [3 * group, 3 * group + 1, 3 * group + 2])
        .collect();
    let skinned: Vec<SoftBodySkinned> = (0..3 * triangles)
        .map(|vertex| SoftBodySkinned::new(vertex, single(weight(0, 1.0))))
        .collect();
    validate_skin(3 * triangles as usize, &faces, &[bind(0)], &skinned).map(|_| ())
}

#[test]
fn the_skinned_vertices_touch_fewer_than_2_pow_24_all_skinned_faces_in_total() {
    // 21931 triangles x 255 listings x 3 vertices = 2^24 - 1 incidences, the most below 2^24
    // (every face adds 3); one more face makes 2^24 + 2.
    assert_eq!(21931 * 255 * 3, (1 << 24) - 1);
    validate_skinned_triangles(21931, 0).unwrap();
    match validate_skinned_triangles(21931, 1) {
        Err(SoftBodyError::InvalidValue(what)) => assert_eq!(what, FACE_TOTAL_RULE),
        other => panic!("expected the face total rule, got {other:?}"),
    }
}

/// `value` as `f64`; a no-op with the `double-precision` feature.
#[allow(clippy::useless_conversion)]
fn wide(value: Real) -> f64 {
    f64::from(value)
}

fn quat_about(axis: Vec3, angle: f32) -> Quat {
    let (sin, cos) = (0.5 * angle).sin_cos();
    Quat::from_xyzw(axis.x * sin, axis.y * sin, axis.z * sin, cos)
}

/// Checks that the binds of `pose` taken at the frame `(position, rotation)` put every joint
/// back where the pose has it.
fn assert_round_trip(pose: &SkeletonPose, position: RVec3, rotation: Quat) {
    let binds = SoftBodyInvBind::from_pose(pose, position, rotation).unwrap();
    assert_eq!(binds.len(), pose.joints.len());
    for (index, bind) in binds.iter().enumerate() {
        assert_eq!(bind.joint, index as u32);
        let offset = rotation.rotate_real(RVec3::new(
            Real::from(bind.position.x),
            Real::from(bind.position.y),
            Real::from(bind.position.z),
        ));
        let expected = pose.position(index);
        let back = [
            position.x + offset.x,
            position.y + offset.y,
            position.z + offset.z,
        ];
        let local = f64::from(bind.position.length());
        for (got, want) in back.into_iter().zip([expected.x, expected.y, expected.z]) {
            // Narrowing the local position to `f32` and turning it back cost a few ulps of it,
            // and the world positions a few ulps of their own size.
            let error = wide(got - want).abs();
            let bound = 4.0 * f64::from(f32::EPSILON) * (local + 1.0)
                + 4.0 * wide(Real::EPSILON) * wide(want).abs();
            assert!(error <= bound, "joint {index}: {error} > {bound}");
        }
        let turned = rotation.product(bind.rotation);
        let target = pose.joints[index].rotation;
        let dot =
            turned.x * target.x + turned.y * target.y + turned.z * target.z + turned.w * target.w;
        assert!((dot.abs() - 1.0).abs() <= 1.0e-6, "joint {index}: {dot}");
    }
}

fn pose_at(root_offset: RVec3) -> SkeletonPose {
    let joint = |translation, rotation| JointTransform {
        translation,
        rotation,
    };
    SkeletonPose {
        root_offset,
        joints: vec![
            joint(Vec3::ZERO, Quat::IDENTITY),
            joint(
                Vec3::new(0.25, 1.5, -0.1),
                quat_about(Vec3::new(0.0, 1.0, 0.0), 0.7),
            ),
            joint(
                Vec3::new(-0.2, 1.45, 0.05),
                quat_about(Vec3::new(0.6, 0.0, 0.8), -1.2),
            ),
        ],
    }
}

#[test]
fn binds_from_a_pose_round_trip_at_a_turned_frame() {
    let frame = quat_about(Vec3::new(0.0, 0.6, 0.8), 2.3);
    let root = RVec3::new(100.3, 2.0, -50.7);
    assert_round_trip(&pose_at(root), RVec3::new(100.1, 3.4, -50.2), frame);
}

#[test]
fn binds_from_a_pose_far_from_the_origin_keep_their_precision() {
    if std::mem::size_of::<Real>() != 8 {
        return;
    }
    // Representable only in double precision; in `f32` the fraction would be lost.
    let root = RVec3::new(1.0e6 + 0.3, 0.0, -2.0e6 - 0.7);
    let frame = quat_about(Vec3::new(1.0, 0.0, 0.0), 0.4);
    let position = RVec3::new(root.x + 0.05, root.y + 1.4, root.z);
    let binds = SoftBodyInvBind::from_pose(&pose_at(root), position, frame).unwrap();
    // Joint 0 is at the root, 5 cm behind the frame along x: exact to f32 rounding of 0.05.
    let back = frame.rotate(binds[0].position);
    assert!(
        (back.x + 0.05).abs() < 1.0e-6 && (back.y + 1.4).abs() < 1.0e-6,
        "{back:?}"
    );
    assert_round_trip(&pose_at(root), position, frame);
}

#[test]
fn binds_from_a_pose_are_checked() {
    let pose = pose_at(RVec3::new(1.0, 0.0, 0.0));
    let far = RVec3::new(1.0 + Real::from(MAX_SHAPE_EXTENT) + 2.0, 0.0, 0.0);
    let refused = SoftBodyInvBind::from_pose(&pose, far, Quat::IDENTITY);
    assert_eq!(
        refused,
        Err(SoftBodyError::InvalidValue(BIND_POSITION_RULE))
    );
    let unnormalised = Quat::from_xyzw(0.0, 0.0, 0.0, 2.0);
    let refused = SoftBodyInvBind::from_pose(&pose, RVec3::ZERO, unnormalised);
    assert_eq!(refused, Err(SoftBodyError::InvalidValue(ROTATION_RULE)));
    let mut broken = pose.clone();
    broken.joints[2].translation.y = f32::NAN;
    let refused = SoftBodyInvBind::from_pose(&broken, RVec3::ZERO, Quat::IDENTITY);
    assert_eq!(refused, Err(SoftBodyError::InvalidValue(POSE_RULE)));
}
