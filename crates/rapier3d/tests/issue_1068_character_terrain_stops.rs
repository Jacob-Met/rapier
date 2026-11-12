use rapier3d::control::{CharacterAutostep, CharacterLength, KinematicCharacterController};
use rapier3d::prelude::*;

const GRID: usize = 129;
const SPACING: f32 = 2.0;
const DT: f32 = 1.0 / 60.0;
const SPEED: f32 = 7.0;

fn height(x: f32, z: f32) -> f32 {
    0.03 * x + 0.4 * (x / 9.0).sin() + 0.3 * (z / 7.0).cos()
}

fn mesh_ground() -> ColliderBuilder {
    let mut vertices = Vec::with_capacity(GRID * GRID);
    for row in 0..GRID {
        for col in 0..GRID {
            let x = col as f32 * SPACING;
            let z = row as f32 * SPACING;
            vertices.push(Vector::new(x, height(x, z), z));
        }
    }

    let mut indices = Vec::with_capacity((GRID - 1) * (GRID - 1) * 2);
    for row in 0..GRID - 1 {
        for col in 0..GRID - 1 {
            let a = (row * GRID + col) as u32;
            let b = a + 1;
            let d = a + GRID as u32;
            let e = d + 1;
            indices.push([a, d, e]);
            indices.push([a, e, b]);
        }
    }
    ColliderBuilder::trimesh(vertices, indices).unwrap()
}

fn heightfield_ground() -> ColliderBuilder {
    // Rapier's heightfield is centered on its translation, unlike the mesh
    // built above with [0, 256] horizontal coordinates.
    let heights = Array2::from_fn(GRID, GRID, |row, col| {
        height(col as f32 * SPACING, row as f32 * SPACING)
    });
    ColliderBuilder::heightfield(
        heights,
        Vector::new(
            (GRID - 1) as f32 * SPACING,
            1.0,
            (GRID - 1) as f32 * SPACING,
        ),
    )
    .translation(Vector::new(128.0, 0.0, 128.0))
}

fn run_character(
    query: &QueryPipeline<'_>,
    yaw_index: usize,
    start_x: f32,
    start_z: f32,
) -> Vec<(usize, f32)> {
    let shape = Capsule::new_y(0.6, 0.3);
    let controller = KinematicCharacterController {
        offset: CharacterLength::Absolute(0.02),
        up: Vector::Y,
        slide: true,
        autostep: Some(CharacterAutostep {
            max_height: CharacterLength::Absolute(0.35),
            min_width: CharacterLength::Absolute(0.15),
            include_dynamic_bodies: false,
        }),
        max_slope_climb_angle: 45.0_f32.to_radians(),
        min_slope_slide_angle: 50.0_f32.to_radians(),
        snap_to_ground: Some(CharacterLength::Absolute(0.3)),
        normal_nudge_factor: 1.0e-4,
    };
    let yaw = yaw_index as f32 * 2.0 * std::f32::consts::PI / 72.0 + 0.013;
    let mut pos = Vector::new(start_x, height(start_x, start_z) + 0.5 + 0.9, start_z);
    let mut vy: f32 = 0.0;
    let mut stopped = vec![];

    for tick in 0..540 {
        let moving = tick >= 60;
        vy = (vy - 9.81 * DT).max(-50.0);
        let want = Vector::new(
            if moving { yaw.sin() * SPEED * DT } else { 0.0 },
            vy * DT,
            if moving { -yaw.cos() * SPEED * DT } else { 0.0 },
        );
        let movement = controller.move_shape(
            DT,
            query,
            &shape,
            &Pose::from_translation(pos),
            want,
            |_| {},
        );
        if movement.grounded && vy < 0.0 {
            vy = 0.0;
        }
        let speed = Vector::new(movement.translation.x, 0.0, movement.translation.z).length() / DT;
        if moving && tick > 90 && speed < SPEED / 2.0 {
            stopped.push((tick, speed));
        }
        pos += movement.translation;
    }
    stopped
}

fn assert_terrain_movement(terrain: ColliderBuilder, kind: &str) {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.integration_parameters.dt = DT;
    world.colliders.insert(terrain);
    world.step();
    let query = world.broad_phase.as_query_pipeline(
        world.narrow_phase.query_dispatcher(),
        &world.bodies,
        &world.colliders,
        QueryFilter::default(),
    );
    let starts = [
        (60.0, 60.0),
        (120.0, 90.0),
        (90.0, 150.0),
        (160.0, 130.0),
        (70.0, 190.0),
        (180.0, 60.0),
    ];
    let mut first_stops = Vec::new();
    let mut stopped_count = 0usize;
    for (start_x, start_z) in starts {
        for yaw_index in 0..72 {
            let stops = run_character(&query, yaw_index, start_x, start_z);
            stopped_count += stops.len();
            for (tick, speed) in stops {
                if first_stops.len() < 12 {
                    first_stops.push((start_x, start_z, yaw_index, tick, speed));
                }
            }
        }
    }
    assert_eq!(
        stopped_count, 0,
        "lost motion on gentle {kind} ({stopped_count} stops): {first_stops:?}"
    );
}

#[test]
fn walking_on_smooth_trimesh_keeps_horizontal_motion() {
    assert_terrain_movement(mesh_ground(), "trimesh");
}

#[test]
fn walking_on_smooth_heightfield_keeps_horizontal_motion() {
    assert_terrain_movement(heightfield_ground(), "heightfield");
}
