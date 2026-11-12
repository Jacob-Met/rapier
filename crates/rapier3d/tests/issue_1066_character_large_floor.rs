use rapier3d::control::{CharacterLength, KinematicCharacterController};
use rapier3d::prelude::*;

fn stand(
    ground: f32,
    x: f32,
    z: f32,
    nudge: f32,
    offset: f32,
    horizontal_step: Vector,
) -> (f32, f32, f32) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let floor = bodies.insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)));
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(ground / 2.0, 0.5, ground / 2.0),
        floor,
        &mut bodies,
    );
    let mut broad = BroadPhaseBvh::new();
    let mut narrow = NarrowPhase::new();
    let mut islands = IslandManager::new();
    let mut ij = ImpulseJointSet::new();
    let mut mj = MultibodyJointSet::new();
    let mut ccd = CCDSolver::new();
    let mut soft = SoftBodySet::new();
    let mut pipeline = PhysicsPipeline::new();
    pipeline.step(
        Vector::new(0.0, -32.0, 0.0),
        &IntegrationParameters::default(),
        &mut islands,
        &mut broad,
        &mut narrow,
        &mut bodies,
        &mut colliders,
        &mut ij,
        &mut mj,
        &mut soft,
        &mut ccd,
        &(),
        &(),
    );
    let shape = Capsule::new_y(0.9 - 0.35, 0.35);
    let mut pos = Vector::new(x, 0.9 + 0.05, z);
    let mut vy = 0.0f32;
    let mut low = f32::MAX;
    let dt = 1.0 / 60.0;
    for _ in 0..120 {
        vy -= 32.0 * dt;
        let controller = KinematicCharacterController {
            up: Vector::Y,
            offset: CharacterLength::Absolute(offset),
            slide: true,
            autostep: None,
            max_slope_climb_angle: 50f32.to_radians(),
            min_slope_slide_angle: 50f32.to_radians(),
            snap_to_ground: if vy <= 0.0 {
                Some(CharacterLength::Absolute(0.3))
            } else {
                None
            },
            normal_nudge_factor: nudge,
        };
        let queries = broad.as_query_pipeline(
            narrow.query_dispatcher(),
            &bodies,
            &colliders,
            QueryFilter::default(),
        );
        let moved = controller.move_shape(
            dt,
            &queries,
            &shape,
            &Pose::from_translation(pos),
            horizontal_step + Vector::new(0.0, vy * dt, 0.0),
            |_| {},
        );
        pos += moved.translation;
        if moved.grounded {
            vy = 0.0;
        }
        low = low.min(pos.y - 0.9);
    }
    (low, pos.y - 0.9, pos.x)
}

#[test]
fn character_stays_above_large_cuboid() {
    let spots = [
        (-97.0, 6.0),
        (-97.0, 1.0),
        (-41.0, -83.0),
        (-41.0, -3.0),
        (6.0, 6.0),
        (0.0, 0.0),
        (300.0, -200.0),
        (0.0, -3.0),
        (23.0, 14.0),
    ];
    let mut cases = 0;
    for size in [50.0, 300.0, 2048.0] {
        for (nudge, offset) in [(1.0e-4, 0.02), (1.0e-4, 0.05), (1.0e-3, 0.02)] {
            for (x, z) in spots {
                if x.abs() >= size / 2.0 || z.abs() >= size / 2.0 {
                    continue;
                }
                let (low, end, _) = stand(size, x, z, nudge, offset, Vector::ZERO);
                assert!(
                    low >= -0.005 && end >= -0.005 && end <= offset + 0.03,
                    "character crossed or left floor at size={size}, ({x},{z}), nudge={nudge}, offset={offset}: low={low}, end={end}"
                );
                cases += 1;
            }
        }
    }
    eprintln!("checked {cases} character/floor combinations");
}

#[test]
fn walking_character_stays_above_large_floor() {
    let (low, end, x) = stand(2048.0, 6.0, 6.0, 1.0e-4, 0.02, Vector::new(0.06, 0.0, 0.03));
    eprintln!("walking on huge floor: low={low}, end={end}, x={x}");
    assert!(
        low >= -0.005 && end >= -0.005,
        "walking character sank into ground"
    );
    assert!(x > 12.0, "walking character failed to move horizontally");
}

#[test]
fn character_can_fall_when_there_is_no_ground() {
    let (low, end, _) = stand(50.0, 300.0, 300.0, 1.0e-4, 0.02, Vector::ZERO);
    assert!(
        low < -20.0 && end < -20.0,
        "character did not fall in empty space"
    );
}
