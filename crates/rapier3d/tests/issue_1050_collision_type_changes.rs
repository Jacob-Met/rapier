use rapier3d::prelude::*;

fn contact_after_runtime_change(
    start_all: bool,
    replacement: Option<ActiveCollisionTypes>,
) -> bool {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let fixed = bodies.insert(RigidBodyBuilder::fixed());
    let box_collider =
        colliders.insert_with_parent(ColliderBuilder::cuboid(2.0, 0.5, 2.0), fixed, &mut bodies);
    let kinematic = bodies.insert(RigidBodyBuilder::kinematic_position_based());
    let start_types = if start_all {
        ActiveCollisionTypes::all()
    } else {
        ActiveCollisionTypes::default()
    };
    let ball_collider = colliders.insert_with_parent(
        ColliderBuilder::ball(0.5).active_collision_types(start_types),
        kinematic,
        &mut bodies,
    );
    let mut pipeline = PhysicsPipeline::new();
    let params = IntegrationParameters::default();
    let mut islands = IslandManager::new();
    let mut broad = DefaultBroadPhase::new();
    let mut narrow = NarrowPhase::new();
    let mut joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut soft = SoftBodySet::new();
    let mut ccd = CCDSolver::new();

    for step in 0..10 {
        if step == 2 {
            if let Some(types) = replacement {
                colliders[ball_collider].set_active_collision_types(types);
            }
        }
        pipeline.step(
            Vector::new(0.0, -9.81, 0.0),
            &params,
            &mut islands,
            &mut broad,
            &mut narrow,
            &mut bodies,
            &mut colliders,
            &mut joints,
            &mut multibody_joints,
            &mut soft,
            &mut ccd,
            &(),
            &(),
        );
    }
    narrow
        .contact_pair(box_collider, ball_collider)
        .is_some_and(|p| p.has_any_active_contact())
}

#[test]
fn widening_collision_types_creates_filtered_pair() {
    assert!(contact_after_runtime_change(true, None));
    assert!(!contact_after_runtime_change(false, None));
    assert!(contact_after_runtime_change(
        false,
        Some(ActiveCollisionTypes::all())
    ));
}

#[test]
fn narrowing_collision_types_removes_ineligible_pair() {
    assert!(!contact_after_runtime_change(
        true,
        Some(ActiveCollisionTypes::default()),
    ));
}
