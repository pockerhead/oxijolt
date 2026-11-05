use super::*;
use crate::world::ensure_initialized;

#[test]
fn default_settings_match_jolt() {
    assert!(ensure_initialized());
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let ours = CharacterSettings::new(&capsule).to_jph(1);
    // SAFETY: an all-zero struct is valid input for `_Init`, which overwrites it.
    let mut jolt: JPH_CharacterVirtualSettings = unsafe { std::mem::zeroed() };
    // SAFETY: `jolt` is a live local. `_Init` creates an empty shape holding one reference,
    // released below, and empty shape settings whose one reference joltc never releases:
    // one small object per test process.
    unsafe { JPH_CharacterVirtualSettings_Init(&mut jolt) };
    let bits = |v: JPH_Vec3| [v.x, v.y, v.z].map(f32::to_bits);
    assert_eq!(bits(jolt.base.up), bits(ours.base.up));
    assert_eq!(
        bits(jolt.base.supportingVolume.normal),
        bits(ours.base.supportingVolume.normal)
    );
    assert_eq!(
        jolt.base.supportingVolume.distance,
        ours.base.supportingVolume.distance
    );
    assert_eq!(
        jolt.base.maxSlopeAngle.to_bits(),
        ours.base.maxSlopeAngle.to_bits()
    );
    assert_eq!(
        jolt.base.enhancedInternalEdgeRemoval,
        ours.base.enhancedInternalEdgeRemoval
    );
    assert_eq!(jolt.mass, ours.mass);
    assert_eq!(jolt.maxStrength, ours.maxStrength);
    assert_eq!(bits(jolt.shapeOffset), bits(ours.shapeOffset));
    assert_eq!(jolt.backFaceMode, ours.backFaceMode);
    assert_eq!(
        jolt.predictiveContactDistance,
        ours.predictiveContactDistance
    );
    assert_eq!(jolt.maxCollisionIterations, ours.maxCollisionIterations);
    assert_eq!(jolt.maxConstraintIterations, ours.maxConstraintIterations);
    assert_eq!(jolt.minTimeRemaining, ours.minTimeRemaining);
    assert_eq!(jolt.collisionTolerance, ours.collisionTolerance);
    assert_eq!(jolt.characterPadding, ours.characterPadding);
    assert_eq!(jolt.maxNumHits, ours.maxNumHits);
    assert_eq!(jolt.hitReductionCosMaxAngle, ours.hitReductionCosMaxAngle);
    assert_eq!(jolt.penetrationRecoverySpeed, ours.penetrationRecoverySpeed);
    assert!(jolt.innerBodyShape.is_null());
    assert_eq!(jolt.innerBodyLayer, ours.innerBodyLayer);
    // SAFETY: `_Init` returned the empty shape holding one reference, released once here.
    unsafe { JPH_Shape_Destroy(jolt.base.shape.cast_mut()) };
}

#[test]
fn ground_states_convert_and_report_support() {
    let states = [
        (JPH_GroundState_OnGround, GroundState::OnGround, true),
        (
            JPH_GroundState_OnSteepGround,
            GroundState::OnSteepGround,
            true,
        ),
        (
            JPH_GroundState_NotSupported,
            GroundState::NotSupported,
            false,
        ),
        (JPH_GroundState_InAir, GroundState::InAir, false),
    ];
    for (raw, state, supported) in states {
        assert_eq!(GroundState::from_jph(raw), state);
        assert_eq!(state.is_supported(), supported, "{state:?}");
    }
}

#[test]
fn humanoid_puts_the_capsule_bottom_at_the_position() {
    let settings = CharacterSettings::humanoid(1.8, 0.3).unwrap();
    let jolt = settings.to_jph(1);
    assert_eq!(Vec3::from_jph(jolt.shapeOffset), Vec3::new(0.0, 0.9, 0.0));
    let capsule = jolt.base.shape.cast::<JPH_CapsuleShape>();
    // SAFETY: the settings own the capsule, alive for these reads; the getters only read it.
    let (radius, half_height) = unsafe {
        (
            JPH_CapsuleShape_GetRadius(capsule),
            JPH_CapsuleShape_GetHalfHeightOfCylinder(capsule),
        )
    };
    assert_eq!(radius, 0.3);
    assert_eq!(half_height, 0.9 - 0.3);
    // The capsule's lowest point, offset minus half height minus radius, is the position.
    assert!((0.9 - half_height - radius).abs() <= 1.0e-6);
}

#[test]
fn humanoid_refuses_bad_dimensions() {
    use crate::ShapeError;
    const RADIUS_RULE: &str = "humanoid radius must be finite and positive";
    const HEIGHT_RULE: &str = "humanoid height must be finite and more than twice its radius";
    let cases = [
        (1.8, f32::NAN, RADIUS_RULE),
        (1.8, 0.0, RADIUS_RULE),
        (1.8, -0.3, RADIUS_RULE),
        (1.8, f32::INFINITY, RADIUS_RULE),
        (f32::NAN, 0.3, HEIGHT_RULE),
        (f32::INFINITY, 0.3, HEIGHT_RULE),
        (0.6, 0.3, HEIGHT_RULE),
        (0.5, 0.3, HEIGHT_RULE),
        (
            2.0 * crate::limits::MAX_SHAPE_EXTENT + 1.0,
            0.3,
            "shape extends beyond limits::MAX_SHAPE_EXTENT",
        ),
    ];
    for (height, radius, rule) in cases {
        match CharacterSettings::humanoid(height, radius) {
            Err(ShapeError::InvalidValue(what)) => {
                assert_eq!(what, rule, "height {height}, radius {radius}");
            }
            other => panic!("height {height}, radius {radius}: {:?}", other.err()),
        }
    }
}

mod listener {
    use std::any::Any;
    use std::panic::{catch_unwind, AssertUnwindSafe};
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::Arc;

    use super::super::listener::tests::inject_panic;
    use super::super::listener::{sort_removals, Callback};
    use super::*;
    use crate::filter::tests::{inject_panic as inject_filter_panic, Callback as FilterCallback};
    use crate::{
        BodySettings, BodyVelocity, CharacterContactKey, CharacterContactListener,
        CharacterContactSettings, CharacterId, ExtendedUpdateSettings, PhysicsWorld, Quat,
        QueryFilter, WorldSettings,
    };

    const DT: f32 = 1.0 / 60.0;

    #[test]
    fn removals_sort_by_body_character_and_sub_shape() {
        let world = PhysicsWorld::new(WorldSettings::default()).unwrap();
        let tag = world.tag;
        let character = CharacterId::new(1, tag);
        let body = |raw, sub| CharacterContactKey {
            body: Some(BodyId::new(raw, tag)),
            character: None,
            sub_shape_id: SubShapeId::new(sub),
        };
        let other = |raw, sub| CharacterContactKey {
            body: None,
            character: Some(CharacterId::new(raw, tag)),
            sub_shape_id: SubShapeId::new(sub),
        };
        let sorted = [
            body(3, 0),
            body(3, 5),
            body(7, u32::MAX),
            other(2, 1),
            other(4, 0),
        ];
        let mut removed: Vec<_> = sorted.iter().rev().map(|&key| (character, key)).collect();
        removed.swap(1, 3);
        sort_removals(&mut removed);
        let keys: Vec<_> = removed.iter().map(|&(_, key)| key).collect();
        assert_eq!(keys, sorted);
    }

    /// Counts every call and changes nothing unless told to.
    #[derive(Default)]
    struct Counter {
        calls: AtomicU32,
        /// Reports a 1 m/s conveyor under the character.
        conveyor: bool,
        /// Panics in `adjust_body_velocity` after changing the velocity.
        change_then_panic: bool,
        /// Panics in `contact_validate` with a payload whose drop panics.
        panic_on_drop: bool,
    }

    impl Counter {
        fn count(&self) {
            self.calls.fetch_add(1, Ordering::Relaxed);
        }

        fn calls(&self) -> u32 {
            self.calls.load(Ordering::Relaxed)
        }
    }

    struct PanicsOnDrop;

    impl Drop for PanicsOnDrop {
        fn drop(&mut self) {
            panic!("panic payload dropped");
        }
    }

    impl CharacterContactListener for Counter {
        fn adjust_body_velocity(&self, _: CharacterId, _: BodyId, _: u64, v: &mut BodyVelocity) {
            self.count();
            if self.conveyor {
                v.set_linear_velocity(Vec3::new(1.0, 0.0, 0.0)).unwrap();
                if self.change_then_panic {
                    panic!("changed, then panicked");
                }
            }
        }

        fn contact_validate(&self, _: CharacterId, _: &CharacterContact) -> bool {
            self.count();
            if self.panic_on_drop {
                std::panic::panic_any(PanicsOnDrop);
            }
            true
        }

        fn contact_added(
            &self,
            _: CharacterId,
            _: &CharacterContact,
            settings: &mut CharacterContactSettings,
        ) {
            self.count();
            settings.can_receive_impulses = false;
        }

        fn contact_persisted(
            &self,
            _: CharacterId,
            _: &CharacterContact,
            _: &mut CharacterContactSettings,
        ) {
            self.count();
        }

        fn contact_removed(&self, _: CharacterId, _: CharacterContactKey) {
            self.count();
        }
    }

    /// Which contacts a scene exercises.
    #[derive(Clone, Copy, Debug)]
    enum Scene {
        /// A character dropping 0.3 m onto a floor box, which it leaves by jumping.
        Body,
        /// Two characters without gravity, one walking into the other and back.
        Character,
    }

    struct Setup {
        world: PhysicsWorld,
        walker: CharacterId,
        scene: Scene,
    }

    fn setup(scene: Scene, listener: Arc<Counter>) -> Setup {
        let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
        world.set_character_contact_listener(Some(listener));
        let floor = Shape::new_box(Vec3::new(10.0, 0.5, 10.0)).unwrap();
        if let Scene::Body = scene {
            world
                .create_body(
                    &floor,
                    &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
                )
                .unwrap();
        }
        let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
        let settings = CharacterSettings::new(&capsule)
            .shape_offset(Vec3::new(0.0, 0.8, 0.0))
            .collide_with_characters(true);
        let walker = world
            .create_character(&settings, RVec3::new(0.0, 0.3, 0.0), Quat::IDENTITY)
            .unwrap();
        if let Scene::Character = scene {
            world
                .create_character(&settings, RVec3::new(1.0, 0.3, 0.0), Quat::IDENTITY)
                .unwrap();
        }
        world
            .refresh_character_contacts(walker, &QueryFilter::new())
            .unwrap();
        Setup {
            world,
            walker,
            scene,
        }
    }

    /// One update of the walker at tick `tick` of the scene's script.
    fn update(setup: &mut Setup, tick: u32) {
        let down = Vec3::new(0.0, -9.81, 0.0);
        let (velocity, gravity) = match setup.scene {
            Scene::Body if tick < 20 => (Vec3::new(0.0, -1.0, 0.0), down),
            Scene::Body => (Vec3::new(0.0, 5.0, 0.0), down),
            Scene::Character if tick < 30 => (Vec3::new(2.0, 0.0, 0.0), Vec3::ZERO),
            Scene::Character => (Vec3::new(-2.0, 0.0, 0.0), Vec3::ZERO),
        };
        let walker = setup.walker;
        setup
            .world
            .character_mut(walker)
            .unwrap()
            .set_linear_velocity(velocity)
            .unwrap();
        setup
            .world
            .update_character(
                walker,
                DT,
                gravity,
                &ExtendedUpdateSettings::default(),
                &QueryFilter::new(),
            )
            .unwrap();
    }

    /// Runs the first ticks of the body scene's script, after which the walker stands.
    fn land(setup: &mut Setup) {
        for tick in 0..20 {
            update(setup, tick);
        }
        let ground = setup.world.character(setup.walker).unwrap().ground_state();
        assert_eq!(ground, GroundState::OnGround);
    }

    fn message(payload: &(dyn Any + Send)) -> String {
        payload
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| payload.downcast_ref::<&str>().map(|s| (*s).to_owned()))
            .unwrap_or_default()
    }

    /// Runs the scene's script until an update panics; the payload.
    fn update_until_panic(setup: &mut Setup) -> Option<Box<dyn Any + Send>> {
        (0..60).find_map(|tick| catch_unwind(AssertUnwindSafe(|| update(setup, tick))).err())
    }

    /// After a resumed panic the character updates and refreshes as before, and with the
    /// listener removed nothing calls it.
    fn assert_still_works(setup: &mut Setup, listener: &Counter) {
        for tick in 0..60 {
            update(setup, tick);
        }
        setup
            .world
            .refresh_character_contacts(setup.walker, &QueryFilter::new())
            .unwrap();
        setup.world.set_character_contact_listener(None);
        let before = listener.calls();
        for tick in 0..60 {
            update(setup, tick);
        }
        assert_eq!(
            listener.calls(),
            before,
            "no native listener is left attached"
        );
    }

    #[test]
    fn a_panic_in_any_listener_callback_is_resumed_by_the_update() {
        let cases = [
            (Scene::Body, Callback::AdjustBodyVelocity),
            (Scene::Body, Callback::Validate),
            (Scene::Body, Callback::Added),
            (Scene::Body, Callback::Persisted),
            (Scene::Body, Callback::Removed),
            (Scene::Character, Callback::Validate),
            (Scene::Character, Callback::Added),
            (Scene::Character, Callback::Persisted),
            (Scene::Character, Callback::Removed),
        ];
        for (scene, callback) in cases {
            let listener = Arc::new(Counter::default());
            let mut setup = setup(scene, listener.clone());
            inject_panic(Some(callback));
            let payload = update_until_panic(&mut setup);
            inject_panic(None);
            let payload = payload.unwrap_or_else(|| panic!("{scene:?} {callback:?} never ran"));
            assert_eq!(
                message(&*payload),
                format!("injected {callback:?} panic"),
                "{scene:?}"
            );
            assert_still_works(&mut setup, &listener);
        }
    }

    #[test]
    fn a_change_followed_by_a_panic_keeps_the_jolt_values() {
        let listener = Arc::new(Counter {
            conveyor: true,
            ..Counter::default()
        });
        let mut setup = setup(Scene::Body, listener.clone());
        land(&mut setup);
        let walker = setup.walker;
        assert_eq!(
            setup.world.character(walker).unwrap().ground_velocity(),
            Vec3::new(1.0, 0.0, 0.0)
        );
        let panicking = Arc::new(Counter {
            conveyor: true,
            change_then_panic: true,
            ..Counter::default()
        });
        setup
            .world
            .set_character_contact_listener(Some(panicking.clone()));
        let payload = catch_unwind(AssertUnwindSafe(|| update(&mut setup, 0))).unwrap_err();
        assert_eq!(message(&*payload), "changed, then panicked");
        assert_eq!(
            setup.world.character(walker).unwrap().ground_velocity(),
            Vec3::ZERO
        );
        setup
            .world
            .set_character_contact_listener(Some(listener.clone()));
        assert_still_works(&mut setup, &listener);
    }

    #[test]
    fn a_listener_is_not_called_after_a_filter_panicked() {
        let listener = Arc::new(Counter::default());
        let mut setup = setup(Scene::Body, listener.clone());
        land(&mut setup);
        let before = listener.calls();
        let layers = [crate::ObjectLayer::NON_MOVING];
        let filter = QueryFilter::new().object_layers(&layers);
        inject_filter_panic(Some(FilterCallback::ObjectLayer));
        let walker = setup.walker;
        let payload = catch_unwind(AssertUnwindSafe(|| {
            setup.world.update_character(
                walker,
                DT,
                Vec3::new(0.0, -9.81, 0.0),
                &ExtendedUpdateSettings::default(),
                &filter,
            )
        }))
        .unwrap_err();
        inject_filter_panic(None);
        assert_eq!(message(&*payload), "injected ObjectLayer panic");
        assert_eq!(listener.calls(), before, "the listener was skipped");
        assert_still_works(&mut setup, &listener);
    }

    /// A listener that panics with a payload whose drop panics, next to a filter that panics
    /// too: each update resumes one payload, and nothing aborts.
    #[test]
    fn simultaneous_filter_and_listener_failures_keep_one_payload() {
        let mut setup = setup(Scene::Body, Arc::new(Counter::default()));
        land(&mut setup);
        let listener = Arc::new(Counter {
            panic_on_drop: true,
            ..Counter::default()
        });
        setup.world.set_character_contact_listener(Some(listener));
        let layers = [crate::ObjectLayer::NON_MOVING];
        let filter = QueryFilter::new().object_layers(&layers);
        let walker = setup.walker;
        let mut kinds = Vec::new();
        for tick in 0..20 {
            if tick == 10 {
                inject_filter_panic(Some(FilterCallback::ObjectLayer));
            }
            let payload = catch_unwind(AssertUnwindSafe(|| {
                setup.world.update_character(
                    walker,
                    DT,
                    Vec3::new(0.0, -9.81, 0.0),
                    &ExtendedUpdateSettings::default(),
                    &filter,
                )
            }))
            .unwrap_err();
            kinds.push(payload.is::<PanicsOnDrop>());
            // Dropping it would panic in this test.
            std::mem::forget(payload);
        }
        inject_filter_panic(None);
        assert!(kinds[..10].iter().all(|&listener| listener));
        assert!(
            kinds[10..].iter().all(|&listener| !listener),
            "the filter ran first"
        );
        let quiet = Arc::new(Counter::default());
        setup
            .world
            .set_character_contact_listener(Some(quiet.clone()));
        assert_still_works(&mut setup, &quiet);
    }
}

mod materials {
    use super::*;
    use crate::{
        Activation, BodySettings, ExtendedUpdateSettings, PhysicsMaterial, PhysicsWorld, Quat,
        QueryFilter, Real, WorldSettings,
    };

    const RADIUS: f32 = 0.3;
    const HALF_HEIGHT: f32 = 0.5;
    /// How far the character sinks into the floor and the wall, so both contacts collide.
    const OVERLAP: Real = 0.005;
    const WALL_HALF_EXTENT: Vec3 = Vec3::new(0.5, 2.0, 5.0);

    /// A floor of material `floor`, a wall of material `wall` and a character standing in the
    /// corner, refreshed; the wall's shape is returned too.
    struct Corner {
        world: PhysicsWorld,
        wall: BodyId,
        wall_shape: Shape,
        character: CharacterId,
    }

    fn corner(floor: &PhysicsMaterial, wall: &PhysicsMaterial) -> Corner {
        let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
        let floor_shape =
            Shape::new_box_with_material(Vec3::new(5.0, 0.5, 5.0), 0.05, floor).unwrap();
        world
            .create_body(
                &floor_shape,
                &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
            )
            .unwrap();
        let wall_shape = Shape::new_box_with_material(WALL_HALF_EXTENT, 0.05, wall).unwrap();
        // The wall's face at x = RADIUS - OVERLAP.
        let wall_center = RVec3::new(0.3 - OVERLAP + 0.5, 2.0, 0.0);
        let wall = world
            .create_body(
                &wall_shape,
                &BodySettings::new_static().position(wall_center),
            )
            .unwrap();
        let capsule = Shape::new_capsule(HALF_HEIGHT, RADIUS).unwrap();
        let settings = CharacterSettings::new(&capsule).shape_offset(Vec3::new(
            0.0,
            HALF_HEIGHT + RADIUS,
            0.0,
        ));
        let character = world
            .create_character(&settings, RVec3::new(0.0, -OVERLAP, 0.0), Quat::IDENTITY)
            .unwrap();
        world
            .refresh_character_contacts(character, &QueryFilter::new())
            .unwrap();
        Corner {
            world,
            wall,
            wall_shape,
            character,
        }
    }

    fn update(world: &mut PhysicsWorld, id: CharacterId, delta_time: f32) {
        let settings = ExtendedUpdateSettings::default()
            .stick_to_floor_step_down(Vec3::ZERO)
            .walk_stairs_step_up(Vec3::ZERO);
        world
            .update_character(
                id,
                delta_time,
                Vec3::new(0.0, -9.81, 0.0),
                &settings,
                &QueryFilter::new(),
            )
            .unwrap();
    }

    #[test]
    fn cached_contact_materials_are_owned_until_the_contacts_change() {
        let floor_material = PhysicsMaterial::new(1).unwrap();
        let wall_material = PhysicsMaterial::new(2).unwrap();
        let Corner {
            mut world,
            wall,
            wall_shape,
            character: id,
        } = corner(&floor_material, &wall_material);
        // The test's reference, the shape's and the character's list.
        assert_eq!(wall_material.reference_count(), 3);
        // The same, and the ground's.
        assert_eq!(floor_material.reference_count(), 4);

        let plain = Shape::new_box(WALL_HALF_EXTENT).unwrap();
        world
            .body_mut(wall)
            .unwrap()
            .set_shape(&plain, None, Activation::DontActivate)
            .unwrap();
        drop(wall_shape);
        assert_eq!(
            wall_material.reference_count(),
            2,
            "the test's and the list's"
        );

        // Up along the wall's normal makes the cached wall contact the best support, and an
        // update shorter than `min_time_remaining` keeps the cached contacts.
        world
            .character_mut(id)
            .unwrap()
            .set_up(Vec3::new(-1.0, 0.0, 0.0))
            .unwrap();
        update(&mut world, id, 1e-6);
        let character = world.character(id).unwrap();
        assert_eq!(character.ground_body(), Some(wall));
        assert!(character
            .active_contacts()
            .iter()
            .any(|contact| contact.body == Some(wall)));
        assert_eq!(
            wall_material.reference_count(),
            3,
            "the test's, the list's and the ground's"
        );

        world
            .character_mut(id)
            .unwrap()
            .set_up(Vec3::new(0.0, 1.0, 0.0))
            .unwrap();
        update(&mut world, id, 1.0 / 60.0);
        assert_eq!(wall_material.reference_count(), 1, "only the test's");

        world.remove_character(id).unwrap();
        assert_eq!(
            floor_material.reference_count(),
            2,
            "the test's and the shape's"
        );
    }

    #[test]
    fn a_shape_swap_retains_the_materials_of_the_new_contacts_only() {
        let floor_material = PhysicsMaterial::new(1).unwrap();
        let wall_material = PhysicsMaterial::new(2).unwrap();
        let Corner {
            mut world,
            wall_shape: _wall_shape,
            character: id,
            ..
        } = corner(&floor_material, &wall_material);
        assert_eq!(
            wall_material.reference_count(),
            3,
            "the test's, the shape's, the list's"
        );

        // A small sphere at the capsule's centre touches neither the wall nor the floor.
        let sphere = Shape::new_sphere(0.1).unwrap();
        let fits = world
            .character_mut(id)
            .unwrap()
            .set_shape(&sphere, 0.0, &QueryFilter::new())
            .unwrap();
        assert!(fits);
        assert!(world.character(id).unwrap().active_contacts().is_empty());
        assert_eq!(
            wall_material.reference_count(),
            2,
            "the test's and the shape's"
        );

        let capsule = Shape::new_capsule(HALF_HEIGHT, RADIUS).unwrap();
        assert!(world
            .character_mut(id)
            .unwrap()
            .set_shape(&capsule, 0.1, &QueryFilter::new())
            .unwrap());
        assert_eq!(wall_material.reference_count(), 3, "the list's again");
    }

    #[test]
    fn a_world_restore_releases_the_contact_materials() {
        let floor_material = PhysicsMaterial::new(1).unwrap();
        let wall_material = PhysicsMaterial::new(2).unwrap();
        let Corner {
            mut world,
            character: id,
            ..
        } = corner(&floor_material, &wall_material);
        let state = world.save_state();
        assert_eq!(wall_material.reference_count(), 3);
        assert_eq!(floor_material.reference_count(), 4);

        // Restored contacts and ground carry Jolt's default material.
        world.restore_state(&state).unwrap();
        assert_eq!(
            wall_material.reference_count(),
            2,
            "the test's and the shape's"
        );
        assert_eq!(
            floor_material.reference_count(),
            2,
            "the test's and the shape's"
        );

        update(&mut world, id, 1.0 / 60.0);
        assert_eq!(wall_material.reference_count(), 3);
        assert_eq!(floor_material.reference_count(), 4);
    }

    #[test]
    fn a_character_restore_keeps_the_old_materials_until_the_next_update() {
        let floor_material = PhysicsMaterial::new(1).unwrap();
        let wall_material = PhysicsMaterial::new(2).unwrap();
        let Corner {
            mut world,
            character: id,
            ..
        } = corner(&floor_material, &wall_material);
        let state = world.character(id).unwrap().save_state();

        // Restored contacts and ground carry Jolt's default material; the list keeps the
        // materials of the contacts before.
        world
            .character_mut(id)
            .unwrap()
            .restore_state(&state)
            .unwrap();
        assert_eq!(
            wall_material.reference_count(),
            3,
            "the test's, the shape's and the list's"
        );
        assert_eq!(floor_material.reference_count(), 3, "the same, no ground's");

        // An update that moves nothing keeps the restored contacts, of the default material.
        update(&mut world, id, 1e-6);
        assert_eq!(wall_material.reference_count(), 2);
        assert_eq!(floor_material.reference_count(), 2);

        update(&mut world, id, 1.0 / 60.0);
        assert_eq!(wall_material.reference_count(), 3);
        assert_eq!(floor_material.reference_count(), 4);
    }

    #[test]
    fn a_filter_panic_still_releases_the_materials_of_dropped_contacts() {
        use crate::filter::tests::{inject_panic, Callback};
        use std::panic::{catch_unwind, AssertUnwindSafe};

        let floor_material = PhysicsMaterial::new(1).unwrap();
        let wall_material = PhysicsMaterial::new(2).unwrap();
        let Corner {
            mut world,
            character: id,
            ..
        } = corner(&floor_material, &wall_material);
        let far = world
            .create_body(
                &Shape::new_sphere(0.25).unwrap(),
                &BodySettings::new_static().position(RVec3::new(50.0, 0.0, 0.0)),
            )
            .unwrap();
        // Excluding a body makes the refresh run the body filter, which panics and rejects
        // every body from then on.
        let filter = QueryFilter::new().exclude_body(far);
        inject_panic(Some(Callback::Body));
        let result = catch_unwind(AssertUnwindSafe(|| {
            world.refresh_character_contacts(id, &filter)
        }));
        inject_panic(None);
        assert!(result.is_err(), "the refresh resumes the filter's panic");

        assert!(world.character(id).unwrap().active_contacts().is_empty());
        assert_eq!(
            wall_material.reference_count(),
            2,
            "the test's and the shape's"
        );
        assert_eq!(floor_material.reference_count(), 2, "the same, no ground's");
    }

    #[test]
    fn contacts_sharing_a_material_hold_one_reference() {
        let material = PhysicsMaterial::new(1).unwrap();
        let Corner {
            mut world,
            character: id,
            ..
        } = corner(&material, &material);
        // The test's, the floor's and the wall's shapes', the list's and the ground's.
        assert_eq!(material.reference_count(), 5);
        update(&mut world, id, 1.0 / 60.0);
        assert_eq!(material.reference_count(), 5);
        world.remove_character(id).unwrap();
        assert_eq!(material.reference_count(), 3);
    }

    #[test]
    fn a_character_keeps_updating_after_a_character_it_touched_is_removed() {
        let floor_material = PhysicsMaterial::new(1).unwrap();
        let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
        let floor_shape =
            Shape::new_box_with_material(Vec3::new(5.0, 0.5, 5.0), 0.05, &floor_material).unwrap();
        world
            .create_body(
                &floor_shape,
                &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
            )
            .unwrap();
        drop(floor_shape);
        let capsule = Shape::new_capsule(HALF_HEIGHT, RADIUS).unwrap();
        let settings = CharacterSettings::new(&capsule)
            .shape_offset(Vec3::new(0.0, HALF_HEIGHT + RADIUS, 0.0))
            .collide_with_characters(true);
        let at = |x: Real| RVec3::new(x, -OVERLAP, 0.0);
        let id = world
            .create_character(&settings, at(0.0), Quat::IDENTITY)
            .unwrap();
        let neighbour = world
            .create_character(&settings, at(2.0 * 0.3 - OVERLAP), Quat::IDENTITY)
            .unwrap();
        world
            .refresh_character_contacts(id, &QueryFilter::new())
            .unwrap();
        let touches_neighbour = |world: &PhysicsWorld| {
            world
                .character(id)
                .unwrap()
                .active_contacts()
                .iter()
                .any(|contact| contact.character == Some(neighbour))
        };
        assert!(touches_neighbour(&world));
        // The test's, the shape's, the list's and the ground's; the neighbour's contact
        // carries Jolt's default material.
        assert_eq!(floor_material.reference_count(), 4);

        world.remove_character(neighbour).unwrap();
        for _ in 0..3 {
            update(&mut world, id, 1e-6);
            assert!(touches_neighbour(&world), "the cached contact stays");
            assert_eq!(floor_material.reference_count(), 4);
        }
        update(&mut world, id, 1.0 / 60.0);
        assert!(!touches_neighbour(&world));
        assert_eq!(floor_material.reference_count(), 4);
    }
}
