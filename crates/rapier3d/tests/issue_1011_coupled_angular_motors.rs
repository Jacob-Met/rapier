//! Regression for https://github.com/dimforge/rapier/issues/1011.
//! Motor commands on two coupled angular DoFs must act on the pair's swing angle.
use rapier3d::prelude::*;

#[test]
fn coupled_angular_spring_motor_returns_tilted_body_to_rest() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.dt = 1.0 / 120.0;

    let fixed = world.bodies.insert(RigidBodyBuilder::fixed());
    let rotated = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .rotation(Vector::new(0.65, 0.0, 0.0))
            .can_sleep(false),
    );
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.2, 0.2, 0.2),
        rotated,
        &mut world.bodies,
    );

    // Rotation about X or Y changes the angle between the Z axes.
    // Only the first coupled angular motor configuration is significant.
    let joint = GenericJointBuilder::new(JointAxesMask::LIN_AXES)
        .coupled_axes(JointAxesMask::ANG_X | JointAxesMask::ANG_Y)
        .motor_position(JointAxis::AngX, 0.0, 70.0, 18.0);
    world.impulse_joints.insert(fixed, rotated, joint, true);
    for _ in 0..240 {
        world.step();
    }

    let axis = world.bodies[rotated].rotation().mul_vec3(Vector::Z);
    let angle = axis.dot(Vector::Z).clamp(-1.0, 1.0).acos();
    assert!(
        angle < 0.10,
        "coupled angular spring ignored its motor: remaining tilt = {angle} rad",
    );
}

#[test]
fn coupled_angular_motor_handles_each_swing_plane_and_nonzero_target() {
    // 3D angular coupling has three possible pairs of constrained swing axes.
    // The other axis is free; each pair must produce an actual motor row.
    for (pair, drive_axis, initial_rotation, free_axis) in [
        (
            JointAxesMask::ANG_X | JointAxesMask::ANG_Y,
            JointAxis::AngX,
            Vector::new(0.65, 0.0, 0.0),
            Vector::Z,
        ),
        (
            JointAxesMask::ANG_X | JointAxesMask::ANG_Z,
            JointAxis::AngX,
            Vector::new(0.65, 0.0, 0.0),
            Vector::Y,
        ),
        (
            JointAxesMask::ANG_Y | JointAxesMask::ANG_Z,
            JointAxis::AngY,
            Vector::new(0.0, 0.65, 0.0),
            Vector::X,
        ),
    ] {
        let mut world = PhysicsWorld::new();
        world.gravity = Vector::ZERO;
        let fixed = world.bodies.insert(RigidBodyBuilder::fixed());
        let moved = world.bodies.insert(
            RigidBodyBuilder::dynamic()
                .rotation(initial_rotation)
                .can_sleep(false),
        );
        world.colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.2, 0.2, 0.2),
            moved,
            &mut world.bodies,
        );
        world.impulse_joints.insert(
            fixed,
            moved,
            GenericJointBuilder::new(JointAxesMask::LIN_AXES)
                .coupled_axes(pair)
                .motor_position(drive_axis, 0.25, 65.0, 16.0),
            true,
        );

        for _ in 0..240 {
            world.step();
        }
        let tilted = world.bodies[moved].rotation().mul_vec3(free_axis);
        let angle = tilted.dot(free_axis).clamp(-1.0, 1.0).acos();
        assert!(
            (angle - 0.25).abs() < 0.10,
            "coupled plane {pair:?} ignored target 0.25: got {angle} rad"
        );
    }
}

#[test]
fn coupled_angular_velocity_motor_changes_swing_angle() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.dt = 1.0 / 120.0;
    let fixed = world.bodies.insert(RigidBodyBuilder::fixed());
    let moved = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .rotation(Vector::new(0.65, 0.0, 0.0))
            .can_sleep(false),
    );
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.2, 0.2, 0.2),
        moved,
        &mut world.bodies,
    );
    world.impulse_joints.insert(
        fixed,
        moved,
        GenericJointBuilder::new(JointAxesMask::LIN_AXES)
            .coupled_axes(JointAxesMask::ANG_X | JointAxesMask::ANG_Y)
            .motor_velocity(JointAxis::AngX, -1.2, 20.0),
        true,
    );

    for _ in 0..36 {
        world.step();
    }
    let tilted = world.bodies[moved].rotation().mul_vec3(Vector::Z);
    let angle = tilted.dot(Vector::Z).clamp(-1.0, 1.0).acos();
    assert!(
        angle < 0.45,
        "coupled angular velocity motor was ineffective: angle = {angle}"
    );
}

// A one-axis group is not physically coupled; it must still be an operable
// one-axis motor rather than crashing when the row is assembled.
#[test]
fn one_angular_axis_marked_coupled_keeps_working() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let fixed = world.bodies.insert(RigidBodyBuilder::fixed());
    let moved = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .rotation(Vector::new(0.65, 0.0, 0.0))
            .can_sleep(false),
    );
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.2, 0.2, 0.2),
        moved,
        &mut world.bodies,
    );
    world.impulse_joints.insert(
        fixed,
        moved,
        GenericJointBuilder::new(JointAxesMask::LIN_AXES)
            .coupled_axes(JointAxesMask::ANG_X)
            .motor_position(JointAxis::AngX, 0.0, 70.0, 18.0),
        true,
    );
    for _ in 0..240 {
        world.step();
    }
    let tilt = world.bodies[moved]
        .rotation()
        .mul_vec3(Vector::Z)
        .dot(Vector::Z)
        .clamp(-1.0, 1.0)
        .acos();
    assert!(tilt < 0.1, "single angular axis motor: tilt = {tilt}");
}

#[test]
fn three_coupled_angular_axes_recover_full_orientation() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let fixed = world.bodies.insert(RigidBodyBuilder::fixed());
    let moved = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .rotation(Vector::new(0.5, -0.4, 0.3))
            .can_sleep(false),
    );
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.2, 0.2, 0.2),
        moved,
        &mut world.bodies,
    );
    world.impulse_joints.insert(
        fixed,
        moved,
        GenericJointBuilder::new(JointAxesMask::LIN_AXES)
            .coupled_axes(JointAxesMask::ANG_AXES)
            .motor_position(JointAxis::AngX, 0.0, 70.0, 18.0),
        true,
    );
    for _ in 0..240 {
        world.step();
    }
    let q = world.bodies[moved].rotation();
    let angle = 2.0 * q.w.abs().clamp(-1.0, 1.0).acos();
    assert!(angle < 0.10, "full 3-axis angular motor: angle={angle}");
}

// A singleton limit must use the regular angular limit row as well.
#[test]
fn one_angular_axis_marked_coupled_respects_limits() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let fixed = world.bodies.insert(RigidBodyBuilder::fixed());
    let moved = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .rotation(Vector::new(0.65, 0.0, 0.0))
            .can_sleep(false),
    );
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.2, 0.2, 0.2),
        moved,
        &mut world.bodies,
    );
    world.impulse_joints.insert(
        fixed,
        moved,
        GenericJointBuilder::new(JointAxesMask::LIN_AXES)
            .coupled_axes(JointAxesMask::ANG_X)
            .limits(JointAxis::AngX, [-0.1, 0.1]),
        true,
    );
    for _ in 0..360 {
        world.step();
    }
    let tilt = world.bodies[moved]
        .rotation()
        .mul_vec3(Vector::Z)
        .dot(Vector::Z)
        .clamp(-1.0, 1.0)
        .acos();
    assert!(
        tilt < 0.20,
        "singleton angular limit ignored: tilt = {tilt}"
    );
}
