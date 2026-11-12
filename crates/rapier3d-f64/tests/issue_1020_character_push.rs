use rapier3d_f64::control::{CharacterLength, KinematicCharacterController};
use rapier3d_f64::prelude::*;

#[test]
fn character_push_does_not_launch_light_body() {
    let dt = 1.0 / 64.0;
    let mut world = PhysicsWorld::new();
    let (_, floor) = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(10.0, 0.5, 10.0),
    );
    let (box_handle, _) = world.insert(
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
    let shape = SharedShape::cuboid(0.25, 0.25, 0.25);
    let pos = Pose::from_translation(Vector::new(-0.06 - 0.25 - 0.009, 0.25 + 0.009, 0.0));
    let mut collisions = vec![];
    let query = world.broad_phase.as_query_pipeline(
        world.narrow_phase.query_dispatcher(),
        &world.bodies,
        &world.colliders,
        QueryFilter::new(),
    );
    controller.move_shape(
        dt,
        &query,
        &*shape,
        &pos,
        Vector::new(dt, -8.0 * dt * dt, 0.0),
        |collision| collisions.push(collision),
    );
    assert_eq!(collisions.len(), 3, "expected floor, box, floor contacts");
    assert_eq!(collisions[0].handle, floor);
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
    controller.solve_character_collision_impulses(dt, &mut query, &*shape, 1.0, &collisions);

    let body = &world.bodies[box_handle];
    assert!(
        body.linvel().length() < 2.0,
        "character walking at 1 unit/s launched box at {} units/s",
        body.linvel().length()
    );
    assert!(
        body.angvel().length() < 50.0,
        "character push spun box at {} radians/s",
        body.angvel().length()
    );
}
