use rapier3d::control::{CharacterLength, KinematicCharacterController};
use rapier3d::prelude::*;

const DT: f32 = 1.0 / 60.0;

fn walk_on_floor(kinematic: bool) -> (f32, usize) {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = DT;
    let floor = if kinematic {
        RigidBodyBuilder::kinematic_position_based()
    } else {
        RigidBodyBuilder::fixed()
    };
    world.insert(
        floor.translation(Vector::new(0.0, -0.25, 0.0)),
        ColliderBuilder::cuboid(20.0, 0.25, 20.0),
    );

    let (character, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(-8.0, 0.91, 0.0)),
        ColliderBuilder::capsule_y(0.45, 0.45),
    );
    world.step();
    let controller = KinematicCharacterController {
        offset: CharacterLength::Absolute(0.01),
        snap_to_ground: Some(CharacterLength::Absolute(0.1)),
        ..Default::default()
    };
    let shape = Capsule::new_y(0.45, 0.45);
    let mut saturated = 0;

    for _ in 0..60 {
        let position = world.bodies[character].translation();
        let mut collisions = 0;
        let movement = {
            let query = world.broad_phase.as_query_pipeline(
                world.narrow_phase.query_dispatcher(),
                &world.bodies,
                &world.colliders,
                QueryFilter::new().exclude_rigid_body(character),
            );
            controller.move_shape(
                DT,
                &query,
                &shape,
                &Pose::from_translation(position),
                Vector::new(0.0, -0.01, -5.0 * DT),
                |_| collisions += 1,
            )
        };
        if collisions >= 20 {
            saturated += 1;
        }
        world
            .bodies
            .get_mut(character)
            .unwrap()
            .set_next_kinematic_translation(position + movement.translation);
        world.step();
    }

    let moved = -world.bodies[character].translation().z;
    (moved, saturated)
}

#[test]
fn walking_on_stationary_kinematic_floor_matches_fixed_floor() {
    let fixed = walk_on_floor(false);
    let kinematic = walk_on_floor(true);
    eprintln!("fixed={fixed:?} kinematic={kinematic:?}");
    assert!(fixed.0 >= 4.8, "fixed floor lost walking speed");
    assert!(
        kinematic.0 >= 4.8 && (kinematic.0 - fixed.0).abs() < 0.1,
        "walking on stopped kinematic platform should behave like a fixed floor"
    );
    assert_eq!(
        kinematic.1, 0,
        "kinematic platform exhausted collision iterations"
    );
}

fn ride_platform(slide: f32, spin: f32, lift: f32) -> (f32, f32, f32, usize) {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = DT;
    let (platform, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, -0.25, 0.0)),
        ColliderBuilder::cuboid(3.0, 0.25, 3.0),
    );
    let (character, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(1.5, 0.91, 0.0)),
        ColliderBuilder::capsule_y(0.45, 0.45),
    );
    world.step();

    let controller = KinematicCharacterController {
        offset: CharacterLength::Absolute(0.01),
        snap_to_ground: Some(CharacterLength::Absolute(0.1)),
        ..Default::default()
    };
    let shape = Capsule::new_y(0.45, 0.45);
    let mut fall_speed: f32 = 0.0;
    let mut angle = 0.0;
    let mut saturated = 0;

    for _ in 0..60 {
        let platform_position = world.bodies[platform].translation();
        world.bodies[platform].set_next_kinematic_translation(
            platform_position + Vector::new(slide * DT, lift * DT, 0.0),
        );
        angle += spin * DT;
        world.bodies[platform]
            .set_next_kinematic_rotation(Rotation::from_axis_angle(Vector::Y, angle));

        fall_speed += 9.81 * DT;
        let character_position = world.bodies[character].translation();
        let mut collisions = 0;
        let movement = {
            let query = world.broad_phase.as_query_pipeline(
                world.narrow_phase.query_dispatcher(),
                &world.bodies,
                &world.colliders,
                QueryFilter::new().exclude_rigid_body(character),
            );
            controller.move_shape(
                DT,
                &query,
                &shape,
                &Pose::from_translation(character_position),
                Vector::new(0.0, -fall_speed * DT, 0.0),
                |_| collisions += 1,
            )
        };
        if movement.grounded {
            fall_speed = 0.0;
        }
        if collisions >= 20 && movement.translation.x.abs() + movement.translation.z.abs() < 1.0e-6
        {
            saturated += 1;
        }
        world.bodies[character]
            .set_next_kinematic_translation(character_position + movement.translation);
        world.step();
    }

    let position = world.bodies[character].translation();
    let rider_angle = (-position.z).atan2(position.x);
    (position.x - 1.5, rider_angle, position.y - 0.91, saturated)
}

#[test]
fn moving_kinematic_platform_carries_character_without_dropped_steps() {
    let (slid, _, _, slide_stalls) = ride_platform(2.0, 0.0, 0.0);
    let (_, turned, _, turn_stalls) = ride_platform(0.0, 1.0, 0.0);
    eprintln!(
        "sliding platform: rider moved {slid:.3}m, dropped={slide_stalls}; turning platform: rider rotated {turned:.3} rad, dropped={turn_stalls}"
    );
    assert!(slid >= 1.9, "kinematic platform did not carry its rider");
    assert!(turned >= 0.9, "turning platform did not carry its rider");
    assert_eq!(slide_stalls, 0, "sliding platform dropped movement steps");
    assert_eq!(turn_stalls, 0, "turning platform dropped movement steps");
}

#[test]
fn vertical_kinematic_platform_lifts_and_lowers_its_rider() {
    let (_, _, risen, up_stalls) = ride_platform(0.0, 0.0, 1.0);
    let (_, _, descended, down_stalls) = ride_platform(0.0, 0.0, -1.0);
    eprintln!(
        "rising platform: rider +{risen:.3}m, stalls={up_stalls}; lowering: rider {descended:.3}m, stalls={down_stalls}"
    );
    assert!(
        (risen - 1.0).abs() < 0.1,
        "rider did not follow platform upward"
    );
    assert!(
        (descended + 1.0).abs() < 0.1,
        "rider did not follow platform downward"
    );
    assert_eq!(
        up_stalls + down_stalls,
        0,
        "platform movement exhausted collision iterations"
    );
}
