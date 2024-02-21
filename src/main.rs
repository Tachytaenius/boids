use std::f32::consts::TAU;

use ggez::conf::WindowMode;
use ggez::glam::{Vec2, vec2};
use rand::Rng;
use ggez::{Context, ContextBuilder, GameResult};
use ggez::graphics::{self, Color, DrawMode, DrawParam, InstanceArray, Mesh, MeshBuilder, StrokeOptions};
use ggez::event::{self, EventHandler};

const SIMULATION_WIDTH: i32 = 800;
const SIMULATION_HEIGHT: i32 = 600;
const MARGIN: f32 = 128.0;

const BOID_COUNT: i32 = 350;

const BOID_BODY_RADIUS_SCALE: f32 = 1.0;
const BOID_FACE_LENGTH_SCALE: f32 = 1.0;

const BOID_START_ACCEL: f32 = 10.0;

fn main() {
    let (mut context, event_loop) = ContextBuilder::new("boids", "Tachytaenius")
        .window_mode(WindowMode::default().dimensions(SIMULATION_WIDTH as f32, SIMULATION_HEIGHT as f32))
        .build()
        .expect("Could not create ggez context");
    let boids = Boids::new(&mut context);
    event::run(context, event_loop, boids);
}

struct Boid {
    position: Vec2,
    velocity: Vec2,

    scale: f32,
    min_speed: f32,
    max_speed: f32,
    protected_range: f32,
    visual_range: f32,
    avoidance_factor: f32,
    matching_factor: f32,
    cohesion_factor: f32,
    back_to_bounds_factor: f32
}

struct Boids {
    boids: Vec<Boid>,
    boid_mesh: Mesh,
    boid_instance_array: InstanceArray
}

impl Boids {
    pub fn new(context: &mut Context) -> Boids {
        let mut boid_mesh_builder = MeshBuilder::new();
        boid_mesh_builder.circle(DrawMode::Stroke(StrokeOptions::default()), Vec2::ZERO, BOID_BODY_RADIUS_SCALE, 0.1, Color::WHITE).unwrap();
        boid_mesh_builder.line(&[
            vec2(BOID_BODY_RADIUS_SCALE, 0.0),
            vec2(BOID_BODY_RADIUS_SCALE + BOID_FACE_LENGTH_SCALE, 0.0)
        ], 1.0, Color::WHITE).unwrap();
        let mut state = Boids {
            boids: Vec::<Boid>::new(),
            boid_mesh: Mesh::from_data(context, boid_mesh_builder.build()),
            boid_instance_array: InstanceArray::new(context, None)
        };
        let mut rng = rand::thread_rng();
        for _ in 0..BOID_COUNT {
            let angle = rng.gen::<f32>() * TAU;
            state.boids.push(Boid {
                position: vec2(rng.gen::<f32>() * SIMULATION_WIDTH as f32, rng.gen::<f32>() * SIMULATION_HEIGHT as f32),
                velocity: Vec2::from_angle(angle) * rng.gen::<f32>().sqrt() * 50.0,

                scale: 2.0,
                min_speed: 10.0,
                max_speed: 100.0,
                protected_range: 20.0,
                visual_range: 100.0,
                avoidance_factor: 0.75,
                matching_factor: 1.0,
                cohesion_factor: 0.5,
                back_to_bounds_factor: 150.0
            });
        }

        state
    }
}

impl EventHandler for Boids {
    fn update(&mut self, context: &mut Context) -> GameResult {
        let dt = context.time.delta().as_secs_f32();
        let mut rng = rand::thread_rng();

        for i in 0..self.boids.len() {
            // Split up the vector into boids below the current boid, the current boid, and boids above the current boid
            let (boids_low, boids_high) = self.boids.split_at_mut(i);
            let (boid, boids_high) = boids_high.split_first_mut().unwrap();

            // React to other boids
            let mut close = Vec2::ZERO;
            let mut neighbour_position_sum = Vec2::ZERO;
            let mut neighbour_velocity_sum = Vec2::ZERO;
            let mut neighbours = 0;
            let mut process_other_boid = |other_boid: &Boid| {
                let other_to_boid = boid.position - other_boid.position;
                if other_to_boid.length() <= boid.visual_range {
                    neighbour_position_sum += other_boid.position;
                    neighbour_velocity_sum += other_boid.velocity;
                    neighbours += 1;
                }
                if other_to_boid.length() <= boid.protected_range {
                    close += other_to_boid;
                }
            };
            for other_boid in boids_low.iter() {
                process_other_boid(other_boid);
            }
            for other_boid in boids_high.iter() {
                process_other_boid(other_boid);
            }
            boid.velocity += close * boid.avoidance_factor * dt;
            if neighbours > 0 {
                boid.velocity += (neighbour_position_sum / neighbours as f32 - boid.position) * boid.cohesion_factor * dt;
                boid.velocity += neighbour_velocity_sum / neighbours as f32 * boid.matching_factor * dt;
            }
            
            // Enforce min and max speed
            if dt > 0.0 { // If dt is zero, this will break
                // Give a random direction to velocity if speed is zero
                if boid.velocity == Vec2::ZERO {
                    boid.velocity = Vec2::from_angle(rng.gen::<f32>() * TAU) * BOID_START_ACCEL * dt;
                }
                // Boid velocity should not be the zero vector
                if boid.velocity.length() < boid.min_speed { // Use or zero just in case precision
                    boid.velocity = boid.velocity.normalize_or_zero() * boid.min_speed;
                }
            }
            if boid.velocity.length() > boid.max_speed {
                boid.velocity = boid.velocity.normalize() * boid.max_speed;
            }

            // Return to bounds if out of bounds
            let mut back_to_bounds = Vec2::ZERO;
            if boid.position.x < MARGIN {
                back_to_bounds.x += 1.0;
            }
            if boid.position.x > SIMULATION_WIDTH as f32 - MARGIN {
                back_to_bounds.x -= 1.0;
            }
            if boid.position.y < MARGIN {
                back_to_bounds.y += 1.0;
            }
            if boid.position.y > SIMULATION_HEIGHT as f32 - MARGIN {
                back_to_bounds.y -= 1.0;
            }
            back_to_bounds = back_to_bounds.normalize_or_zero() * boid.back_to_bounds_factor;
            boid.velocity += back_to_bounds * dt;

            boid.position += boid.velocity * dt;
        }

        Ok(())
    }

    fn draw(&mut self, context: &mut Context) -> GameResult {
        let mut canvas = graphics::Canvas::from_frame(context, Color::BLACK);
        self.boid_instance_array.clear();
        for boid in self.boids.iter() {
            self.boid_instance_array.push(DrawParam::default()
                .dest(boid.position)
                .rotation(if boid.velocity != Vec2::ZERO { boid.velocity.y.atan2(boid.velocity.x) } else { 0.0 }) // No to_angle?
                .scale(Vec2::splat(boid.scale))
            );
        }
        canvas.draw_instanced_mesh(self.boid_mesh.clone(), &self.boid_instance_array, DrawParam::default());
        canvas.finish(context)
    }
}
