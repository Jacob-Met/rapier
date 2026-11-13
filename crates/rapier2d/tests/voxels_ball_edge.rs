use rapier2d::prelude::*;

// Changing the set of contacted voxel cells must not leave stale contact IDs.
#[test]
fn ball_can_roll_off_voxel_floor_edge() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81);

    let cells: Vec<_> = (0..32).map(|x| IVector::new(x, 0)).collect();
    world
        .colliders
        .insert(ColliderBuilder::voxels(Vector::new(1.0, 1.0), &cells));

    let body = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(26.0, 1.4))
            .linvel(Vector::new(4.0, 0.0))
            .can_sleep(false),
    );
    world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(0.4), body, &mut world.bodies);

    for _ in 0..300 {
        world.step();
    }
    let p = world.bodies[body].translation();
    assert!(p.x > 32.0, "ball stuck before edge: x={}", p.x);
    assert!(p.y < 0.0, "ball did not fall: y={}", p.y);
}
