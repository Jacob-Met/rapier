//! Walking along a stationary kinematic floor should be equivalent to a fixed floor.
//! A floor at rest must not erase the small separation nudge the controller needs
//! to advance past a zero-TOI ground hit.
use rapier3d::control::{CharacterLength, KinematicCharacterController};
use rapier3d::prelude::*;

fn walked_distance(floor_is_kinematic: bool) -> (Real, usize) {
    let mut world = PhysicsWorld::new();
    let floor_builder = if floor_is_kinematic {
        RigidBodyBuilder::kinematic_position_based()
    } else {
        RigidBodyBuilder::fixed()
    };
    let _ = world.insert(
        floor_builder.translation(Vector::new(0.0, -0.25, 0.0)),
        ColliderBuilder::cuboid(20.0, 0.25, 20.0),
    );
    let (body, collider) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(-8.0, 0.91, 0.0)),
        ColliderBuilder::capsule_y(0.45, 0.45),
    );
    world.step();
    let dt = world.integration_parameters.dt;
    let controller = KinematicCharacterController {
        offset: CharacterLength::Absolute(0.01),
        snap_to_ground: Some(CharacterLength::Absolute(0.1)),
        ..Default::default()
    };
    let start = world.bodies[body].translation().z;
    let mut last_hits = 0;
    for _ in 0..60 {
        let queries = world.broad_phase.as_query_pipeline(
            world.narrow_phase.query_dispatcher(),
            &world.bodies,
            &world.colliders,
            QueryFilter::new().exclude_rigid_body(body),
        );
        let mut hits = 0;
        let movement = controller.move_shape(
            dt,
            &queries,
            world.colliders[collider].shape(),
            world.colliders[collider].position(),
            Vector::new(0.0, -0.01, -5.0 / 60.0),
            |_| hits += 1,
        );
        last_hits = hits;
        let next = world.bodies[body].translation() + movement.translation;
        world.bodies[body].set_next_kinematic_translation(next);
        world.step();
    }
    (start - world.bodies[body].translation().z, last_hits)
}

#[test]
fn stationary_kinematic_floor_allows_walking() {
    let (fixed, fixed_hits) = walked_distance(false);
    let (kinematic, kinematic_hits) = walked_distance(true);
    eprintln!(
        "gh-1072 walked fixed={fixed:.3} (hits {fixed_hits}), kinematic={kinematic:.3} (hits {kinematic_hits}), expected 5"
    );
    assert!(
        (fixed - 5.0).abs() < 0.3,
        "fixed floor baseline walked {fixed}"
    );
    assert!(
        (kinematic - 5.0).abs() < 0.3,
        "stationary kinematic floor must not cancel character travel: walked {kinematic} instead of 5"
    );
}

fn platform_carry(slide: Real, spin: Real) -> (Vector, usize) {
    let mut world = PhysicsWorld::new();
    let (platform, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, -0.25, 0.0)),
        ColliderBuilder::cuboid(3.0, 0.25, 3.0),
    );
    let (body, collider) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(1.5, 0.91, 0.0)),
        ColliderBuilder::capsule_y(0.45, 0.45),
    );
    world.step();
    let dt = world.integration_parameters.dt;
    let controller = KinematicCharacterController {
        offset: CharacterLength::Absolute(0.01),
        snap_to_ground: Some(CharacterLength::Absolute(0.1)),
        ..Default::default()
    };
    let mut fall_speed = 0.0;
    let mut angle = 0.0;
    let mut dropped = 0;
    for _ in 0..60 {
        let next = world.bodies[platform].translation() + Vector::new(slide * dt, 0.0, 0.0);
        world.bodies[platform].set_next_kinematic_translation(next);
        angle += spin * dt;
        world.bodies[platform]
            .set_next_kinematic_rotation(Rotation::from_scaled_axis(Vector::new(0.0, angle, 0.0)));
        fall_speed += 9.81 * dt;
        let queries = world.broad_phase.as_query_pipeline(
            world.narrow_phase.query_dispatcher(),
            &world.bodies,
            &world.colliders,
            QueryFilter::new().exclude_rigid_body(body),
        );
        let mut collisions = 0;
        let movement = controller.move_shape(
            dt,
            &queries,
            world.colliders[collider].shape(),
            world.colliders[collider].position(),
            Vector::new(0.0, -fall_speed * dt, 0.0),
            |_| collisions += 1,
        );
        if movement.grounded {
            fall_speed = 0.0;
        }
        if collisions >= 20 && movement.translation.x.hypot(movement.translation.z) < 1.0e-6 {
            dropped += 1;
        }
        let next = world.bodies[body].translation() + movement.translation;
        world.bodies[body].set_next_kinematic_translation(next);
        world.step();
    }
    (world.bodies[body].translation(), dropped)
}

#[test]
fn sliding_kinematic_platform_carries_rider_without_stalling() {
    let (pos, dropped) = platform_carry(2.0, 0.0);
    eprintln!(
        "gh-1072 sliding platform carried {:.3}m / 2.0m; dropped={dropped}",
        pos.x - 1.5
    );
    assert!(
        pos.x - 1.5 > 1.95,
        "rider lost movement on sliding platform: {pos:?}"
    );
    assert_eq!(dropped, 0, "repeated zero-time-of-impact floor hits");
}

#[test]
fn rotating_kinematic_platform_carries_rider_without_stalling() {
    let (pos, dropped) = platform_carry(0.0, 1.0);
    let angle = (-pos.z).atan2(pos.x);
    eprintln!(
        "gh-1072 turning platform carried {:.3}deg / 57.3deg; dropped={dropped}",
        angle.to_degrees()
    );
    assert!(
        angle > 0.88,
        "rider fell behind turning platform: angle={angle} rad"
    );
    assert_eq!(dropped, 0, "repeated zero-time-of-impact floor hits");
}

fn vertical_platform_carry(vertical_speed: Real) -> (Real, Real) {
    let mut world = PhysicsWorld::new();
    let (platform, _) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, -0.25, 0.0)),
        ColliderBuilder::cuboid(3.0, 0.25, 3.0),
    );
    let (body, collider) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(0.0, 0.91, 0.0)),
        ColliderBuilder::capsule_y(0.45, 0.45),
    );
    world.step();
    let dt = world.integration_parameters.dt;
    let controller = KinematicCharacterController {
        offset: CharacterLength::Absolute(0.01),
        snap_to_ground: Some(CharacterLength::Absolute(0.1)),
        ..Default::default()
    };
    let mut fall_speed = 0.0;
    for _ in 0..60 {
        let next =
            world.bodies[platform].translation() + Vector::new(0.0, vertical_speed * dt, 0.0);
        world.bodies[platform].set_next_kinematic_translation(next);
        fall_speed += 9.81 * dt;
        let query = world.broad_phase.as_query_pipeline(
            world.narrow_phase.query_dispatcher(),
            &world.bodies,
            &world.colliders,
            QueryFilter::new().exclude_rigid_body(body),
        );
        let movement = controller.move_shape(
            dt,
            &query,
            world.colliders[collider].shape(),
            world.colliders[collider].position(),
            Vector::new(0.0, -fall_speed * dt, 0.0),
            |_| {},
        );
        if movement.grounded {
            fall_speed = 0.0;
        }
        let next = world.bodies[body].translation() + movement.translation;
        world.bodies[body].set_next_kinematic_translation(next);
        world.step();
    }
    (
        world.bodies[body].translation().y,
        world.bodies[platform].translation().y,
    )
}

#[test]
fn vertical_kinematic_platform_motion_is_not_lost() {
    for speed in [-0.5, 0.5] {
        let (rider_y, platform_y) = vertical_platform_carry(speed);
        eprintln!("gh-1072 vertical speed={speed}: rider={rider_y:.4}, platform={platform_y:.4}");
        assert!(
            (rider_y - platform_y - 1.16).abs() < 0.07,
            "rider lost vertical platform contact: speed={speed} rider={rider_y} platform={platform_y}"
        );
    }
}
