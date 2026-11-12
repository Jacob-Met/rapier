// Physics rollback scenarios adapted from caspark's report at
// https://github.com/dimforge/rapier/issues/1053.
use rapier3d::prelude::*;

/// Helper - clones the simulation data and creates fresh physics, collision, and CCD workspaces.
fn copy_state(w: &PhysicsWorld) -> PhysicsWorld {
    PhysicsWorld {
        gravity: w.gravity,
        integration_parameters: w.integration_parameters,
        islands: w.islands.clone(),
        broad_phase: w.broad_phase.clone(),
        narrow_phase: w.narrow_phase.clone(),
        bodies: w.bodies.clone(),
        colliders: w.colliders.clone(),
        impulse_joints: w.impulse_joints.clone(),
        multibody_joints: w.multibody_joints.clone(),
        soft_bodies: w.soft_bodies.clone(),
        ..PhysicsWorld::new()
    }
}

// 1. Joint builders from another branch
//
// The anchor test puts the body at x=1 instead of x=2.
// The collider-offset test puts it at x=-1 instead of x=0.
// Both branches make one change, so their epoch counters match despite different data.

fn builder_repro(edit: fn(&mut PhysicsWorld, RigidBodyHandle, ImpulseJointHandle, Real)) {
    let mut a = PhysicsWorld::new();
    let fixed = a.insert_body(RigidBodyBuilder::fixed());
    let body = a.insert_body(RigidBodyBuilder::dynamic().additional_mass(1.0));
    let joint = a.insert_impulse_joint(fixed, body, FixedJointBuilder::new());
    a.step();
    let mut b = copy_state(&a);
    edit(&mut a, body, joint, 1.0);
    edit(&mut b, body, joint, 2.0);
    a.step();
    let mut fresh = copy_state(&b);
    b.physics_pipeline = a.physics_pipeline;
    b.step();
    fresh.step();
    assert_eq!(b.bodies[body].position(), fresh.bodies[body].position());
}

#[test]
fn joint_anchor_uses_the_restored_branch() {
    builder_repro(|w, _, joint, x| {
        w.impulse_joints
            .get_mut(joint, false)
            .unwrap()
            .data
            .set_local_anchor1(Vector::X * x);
    });
}

#[test]
fn collider_offset_uses_the_restored_branch() {
    builder_repro(|w, body, _, x| {
        w.insert_collider(
            ColliderBuilder::ball(0.5).translation(Vector::X * x),
            Some(body),
        );
    });
}

// 2. Multibody membership change, without rollback
//
// The body gains linear velocity despite its fixed impulse joint. A fresh pipeline keeps its velocity at zero.

#[test]
fn adding_multibody_membership_keeps_the_impulse_joint_effective() {
    let mut w = PhysicsWorld::new();
    let fixed = w.insert_body(RigidBodyBuilder::fixed());
    let body = w.insert_body(RigidBodyBuilder::dynamic().additional_mass(1.0));
    w.insert_impulse_joint(fixed, body, FixedJointBuilder::new());
    w.insert_collider(
        ColliderBuilder::ball(0.5).translation(Vector::X),
        Some(body),
    );
    w.step();
    w.insert_multibody_joint(fixed, body, RevoluteJointBuilder::new(Vector::Z))
        .unwrap();
    w.step();
    assert_eq!(w.bodies[body].linvel(), Vector::ZERO);
}

// 3. FEM topology from another branch
//
// Both worlds allocate the same soft-body handle, with three particles, one edge, and topology version zero.
// Their edges differ. The first step produces different particle positions with the retained and fresh pipelines.

#[test]
#[cfg(feature = "fem")]
fn fem_uses_the_current_branch_topology() {
    let soft = |edge| {
        SoftBodyBuilder::new(vec![Vector::ZERO, Vector::X, Vector::X * 2.0])
            .edges(vec![edge])
            .solver(SoftBodySolver::Fem)
            .no_surface_collider()
    };
    let mut a = PhysicsWorld::new();
    let mut b = PhysicsWorld::new();
    a.insert_soft_body(soft([0, 1]));
    let handle = b.insert_soft_body(soft([1, 2]));
    a.step();
    b.soft_bodies[handle].set_particle_position(2, Vector::new(2.0, 0.5, 0.0));
    let mut fresh = copy_state(&b);
    b.physics_pipeline = a.physics_pipeline;
    b.step();
    fresh.step();
    assert_eq!(
        b.soft_bodies[handle].particle_position(1),
        fresh.soft_bodies[handle].particle_position(1)
    );
}

// 4. CCD after a fixed wall changes
//
// The fast body passes through the restored wall in both tests.
// The restored world already has a fresh physics pipeline. It retains only the later CCD solver.

fn ccd_repro(edit: fn(&mut PhysicsWorld, RigidBodyHandle)) {
    let mut w = PhysicsWorld::new();
    w.gravity = Vector::ZERO;
    let (wall, _) = w.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(0.05, 5.0, 5.0),
    );
    let (fast, _) = w.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::X * -6.0)
            .linvel(Vector::X * 200.0),
        ColliderBuilder::cuboid(0.1, 0.1, 0.1),
    );
    w.step();
    let mut saved = copy_state(&w);
    edit(&mut w, wall);
    w.step();
    saved.ccd_solver = w.ccd_solver;
    saved.step();
    assert!(saved.bodies[fast].translation().x < 0.0);
}

#[test]
fn ccd_restore_after_wall_removal() {
    ccd_repro(|w, wall| {
        w.remove_body(wall);
    });
}

#[test]
fn ccd_restore_after_wall_move() {
    ccd_repro(|w, wall| {
        w.bodies[wall].set_translation(Vector::X * 100.0, true);
    });
}
