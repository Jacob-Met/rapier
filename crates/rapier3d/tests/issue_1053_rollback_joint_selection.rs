use rapier3d::prelude::*;

#[test]
fn restoring_cloned_joint_set_rebuilds_solver_constraints() {
    let mut world = PhysicsWorld::new();
    let fixed = world.insert_body(RigidBodyBuilder::fixed());
    let body = world.insert_body(RigidBodyBuilder::dynamic().additional_mass(1.0));
    let joint = world.insert_impulse_joint(fixed, body, FixedJointBuilder::new());
    world.step();

    let saved = world.impulse_joints.clone();
    world.remove_impulse_joint(joint);
    world.step();
    assert_eq!(world.physics_pipeline.counters.solver.nconstraints, 0);

    world.impulse_joints = saved;
    world.step();
    assert_eq!(
        world.physics_pipeline.counters.solver.nconstraints, 1,
        "rolling back a cloned joint set must restore its constraints"
    );
}
