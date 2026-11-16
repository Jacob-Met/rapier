//! Regression for #1072: walking on a stationary kinematic platform must work like a fixed floor.
use rapier3d::control::{CharacterLength, KinematicCharacterController};
use rapier3d::prelude::*;

fn walk_on_floor(kinematic: bool) -> (Real, usize) {
    let mut world = PhysicsWorld::new();
    let builder = if kinematic {
        RigidBodyBuilder::kinematic_position_based()
    } else {
        RigidBodyBuilder::fixed()
    };
    world.insert(
        builder.translation(Vector::new(0.0, -0.25, 0.0)),
        ColliderBuilder::cuboid(20.0, 0.25, 20.0),
    );
    let (character, collider) = world.insert(
        RigidBodyBuilder::kinematic_position_based().translation(Vector::new(-8.0, 0.91, 0.0)),
        ColliderBuilder::capsule_y(0.45, 0.45),
    );
    world.step();
    let controller = KinematicCharacterController {
        offset: CharacterLength::Absolute(0.01),
        snap_to_ground: Some(CharacterLength::Absolute(0.1)),
        ..Default::default()
    };

    let initial_z = world.bodies[character].translation().z;
    let mut last_collisions = 0;
    for _ in 0..60 {
        let query = world.broad_phase.as_query_pipeline(
            world.narrow_phase.query_dispatcher(),
            &world.bodies,
            &world.colliders,
            QueryFilter::new().exclude_rigid_body(character),
        );
        let mut hits = 0;
        let movement = controller.move_shape(
            1.0 / 60.0,
            &query,
            world.colliders[collider].shape(),
            world.bodies[character].position(),
            Vector::new(0.0, -0.01, -5.0 / 60.0),
            |_| hits += 1,
        );
        last_collisions = hits;
        let next = world.bodies[character].translation() + movement.translation;
        world.bodies[character].set_next_kinematic_translation(next);
        world.step();
    }
    (
        initial_z - world.bodies[character].translation().z,
        last_collisions,
    )
}

#[test]
fn character_walks_on_stationary_kinematic_floor() {
    let (fixed_distance, fixed_hits) = walk_on_floor(false);
    let (kinematic_distance, kinematic_hits) = walk_on_floor(true);
    eprintln!(
        "fixed={fixed_distance:.3} m ({fixed_hits} hits); kinematic={kinematic_distance:.3} m ({kinematic_hits} hits)"
    );
    assert!(
        fixed_distance >= 4.5,
        "control: fixed floor should allow 5 m walking but got {fixed_distance:.3}"
    );
    assert!(
        kinematic_distance >= 4.5,
        "stationary kinematic floor blocked walking: {kinematic_distance:.3} m vs fixed {fixed_distance:.3} m"
    );
}
