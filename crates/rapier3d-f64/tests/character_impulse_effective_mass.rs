//! Reproduction for #1020: an ordinary character push must not launch a light box.
use rapier3d_f64::control::{CharacterLength, KinematicCharacterController};
use rapier3d_f64::prelude::*;

#[test]
fn light_box_does_not_launch_from_character_push() {
    let dt = 1.0 / 64.0;
    let mut world = PhysicsWorld::new();
    let (_, floor) = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(10.0, 0.5, 10.0),
    );
    let (body, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.2, 0.0)),
        ColliderBuilder::cuboid(0.06, 0.2, 0.12),
    );
    world.gravity = Vector::ZERO;
    world.step();

    let controller = KinematicCharacterController {
        offset: CharacterLength::Absolute(0.01),
        snap_to_ground: None,
        ..Default::default()
    };
    let character = SharedShape::cuboid(0.25, 0.25, 0.25);
    let pos = Pose::from_translation(Vector::new(-0.319, 0.259, 0.0));
    let mut collisions = Vec::new();
    let query = world.broad_phase.as_query_pipeline(
        world.narrow_phase.query_dispatcher(),
        &world.bodies,
        &world.colliders,
        QueryFilter::new(),
    );
    controller.move_shape(
        dt,
        &query,
        &*character,
        &pos,
        Vector::new(dt, -8.0 * dt * dt, 0.0),
        |collision| collisions.push(collision),
    );
    assert!(
        collisions.len() >= 2,
        "expected floor/box character collisions"
    );
    assert!(
        collisions.iter().any(|collision| collision.handle == floor),
        "expected a floor hit"
    );

    let PhysicsWorld {
        broad_phase,
        narrow_phase,
        bodies,
        colliders,
        ..
    } = &mut world;
    let mut query = broad_phase.as_query_pipeline_mut(
        narrow_phase.query_dispatcher(),
        bodies,
        colliders,
        QueryFilter::new(),
    );
    controller.solve_character_collision_impulses(dt, &mut query, &*character, 1.0, &collisions);
    let body = &world.bodies[body];
    let speed = body.linvel().length();
    eprintln!(
        "collisions={}, mass={}, speed={speed:.3}, angular={:.3}",
        collisions.len(),
        body.mass(),
        body.angvel().length()
    );
    assert!(
        speed <= 2.0,
        "box launched at {speed:.3} times a character's 1 m/s walking speed"
    );
}
