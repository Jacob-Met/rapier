use rapier3d::prelude::*;

// The free body's momentum is an independent measurement of the motor's
// actual impulse. A last-substep-only readout underreports it about 8x.
#[test]
fn motor_impulse_is_whole_step_not_last_substep() {
    for iterations in [1, 2, 8, 16] {
        for warmstart in [false, true] {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.num_solver_iterations = iterations;
            world.integration_parameters.warmstart_joints = warmstart;

            let fixed = world.insert_body(RigidBodyBuilder::fixed());
            let moved = world.insert_body(RigidBodyBuilder::dynamic().additional_mass(1.0));
            let joint = world.insert_impulse_joint(
                fixed,
                moved,
                PrismaticJointBuilder::new(Vector::X)
                    .motor_velocity(20.0, 1.0)
                    .motor_max_force(60.0),
            );
            let mut last_vel = 0.0;
            for step in 0..2 {
                world.step();
                let motor =
                    &world.impulse_joints.get(joint).unwrap().data.motors[JointAxis::LinX as usize];
                let delta_momentum =
                    world.bodies[moved].mass() * (world.bodies[moved].linvel().x - last_vel);
                assert!(
                    (motor.total_impulse + delta_momentum).abs() < 1e-4,
                    "iterations={iterations}, warmstart={warmstart}, step={step}: \
                     total={} delta_momentum={delta_momentum}",
                    motor.total_impulse,
                );
                if iterations == 1 {
                    assert_eq!(motor.total_impulse, motor.impulse);
                }
                last_vel = world.bodies[moved].linvel().x;
            }
            world
                .impulse_joints
                .get_mut(joint, true)
                .unwrap()
                .data
                .set_enabled(false);
            world.step();
            let motor =
                &world.impulse_joints.get(joint).unwrap().data.motors[JointAxis::LinX as usize];
            assert_eq!(
                motor.total_impulse, 0.0,
                "disabled joint must reset readout"
            );
        }
    }
}

// Multiple disjoint joints exercise the SIMD joint chunk and scalar joint paths,
// including their lane-specific accumulated values and force limits.
#[test]
fn independent_joint_motors_report_each_impulse() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.num_solver_iterations = 8;
    let fixed = world.insert_body(RigidBodyBuilder::fixed());
    let mut handles = Vec::new();
    for i in 0..12 {
        let moved = world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(0.0, i as Real * 5.0, 0.0))
                .additional_mass(1.0 + i as Real * 0.5),
        );
        let joint = world.insert_impulse_joint(
            fixed,
            moved,
            PrismaticJointBuilder::new(Vector::X)
                .motor_velocity(10.0 + i as Real, 1.0)
                .motor_max_force(10.0 + i as Real * 5.0)
                .local_anchor1(Vector::new(0.0, i as Real * 5.0, 0.0)),
        );
        handles.push((joint, moved));
    }
    world.step();
    for (joint, moved) in handles {
        let motor = &world.impulse_joints.get(joint).unwrap().data.motors[JointAxis::LinX as usize];
        let momentum = world.bodies[moved].mass() * world.bodies[moved].linvel().x;
        assert!(
            (motor.total_impulse + momentum).abs() < 1e-4,
            "joint {joint:?}: total={} delta_momentum={momentum}",
            motor.total_impulse,
        );
    }
}
