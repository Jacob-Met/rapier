use rapier3d::prelude::*;

// Real game failure: rolling off a voxel ledge previously panicked while
// refreshing contact manifolds as the ball crossed the edge.
#[test]
fn ball_can_roll_off_voxel_floor_edge() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);

    let mut cells = Vec::new();
    for x in 0..32 {
        for z in 0..8 {
            cells.push(IVector::new(x, 0, z));
        }
    }
    world
        .colliders
        .insert(ColliderBuilder::voxels(Vector::new(1.0, 1.0, 1.0), &cells));

    let body = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(26.0, 1.4, 4.3))
            .linvel(Vector::new(4.0, 0.0, 0.0))
            .can_sleep(false),
    );
    world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(0.4), body, &mut world.bodies);

    for _ in 0..300 {
        world.step();
    }
    let p = world.bodies[body].translation();
    assert!(p.x > 32.0, "the ball should cross the ledge, at x={}", p.x);
    assert!(
        p.y < 0.0,
        "the ball should fall off the ledge, at y={}",
        p.y
    );
}
