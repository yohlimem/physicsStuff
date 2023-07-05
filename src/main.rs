use nannou::{prelude::*, state::Mouse, lyon::geom::euclid::num::Floor};
use nannou_egui::{self, egui, Egui};

mod particles;
use crate::particles::Particle;

const PARTICLE_NUM: u32 = 1;
const DELTA_TIME: f32 = 1.0;

struct Model {
    // window: Window,
    egui: Egui,
    particles: Vec<Particle>,
    substeps: u32,
    last_pressed: i32,
    ropes: Vec<(usize, usize)>,
    springs: Vec<(usize, usize)>,
}

fn main() {
    nannou::app(model).update(update).run();
}

fn model(app: &App) -> Model {
    let window_id = app
        .new_window()
        .view(view)
        .raw_event(raw_window_event)
        .build()
        .unwrap();
    let window = app.window(window_id).unwrap();
    let egui = Egui::from_window(&window);
    let mut particles = vec![
        Particle::from(Vec2::ONE, Vec2::ZERO, 10.0),
        Particle::from(vec2(0.0, 50.0), Vec2::ZERO, 10.0),
        Particle::from(vec2(0.0, 100.0), Vec2::ZERO, 10.0),
        Particle::from(vec2(0.0, 150.0), Vec2::ZERO, 10.0),
        // Particle::from(vec2(3.0, 3.0), Vec2::ONE, Vec2::ZERO, 30.0),
    ];
    let substeps = 10;
    let ropes = vec![(0, 1), (1, 2), (2, 3)];
    let springs = vec![];
    particles[0].freezed = true;
    Model {
        egui,
        particles,
        substeps,
        last_pressed: -1,
        ropes,
        springs,
    }
}

fn update(app: &App, model: &mut Model, update: Update) {
    {
        let egui = &mut model.egui;
        egui.set_elapsed_time(update.since_start);

        let ctx = egui.begin_frame();

        egui::Window::new("Rum window").show(&ctx, |ui| {
            ui.label("res");
            ui.add(egui::Slider::new(&mut model.substeps, 1..=1000));
        });
    }
    select_ball(&app.mouse, model);
    solver(model);
    pick_up_ball(&app.mouse, model);
    freeze_balls(&app.mouse, model);
}

fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    model.egui.handle_raw_event(event);
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(GRAY);
    draw.ellipse().radius(400.0).color(BLACK);
    //------------------------------------------------------------------------------------------------\\
    draw_particles(model, &draw);
    for rope in model.ropes.iter() {
        draw.line().start(model.particles[rope.0].position).end(model.particles[rope.1].position).stroke_weight(2.0).color(BLUE);
        draw.text(&rope.0.to_string()).xy(model.particles[rope.0].position).color(BLACK);
        draw.text(&rope.1.to_string()).xy(model.particles[rope.1].position).color(BLACK);
    }
    

    draw.to_frame(app, &frame).unwrap();
    model.egui.draw_to_frame(&frame).unwrap();
}

fn solver(model: &mut Model) {
    let sub_dt = DELTA_TIME / model.substeps as f32;
    for _ in 0..model.substeps.floor() {
        // solve movement
        for p in 0..model.particles.len() {
            // apply gravity
            apply_gravity(&mut model.particles[p]);
            // apply the constraint for the circle outside
            apply_world_constraint(&mut model.particles[p]);
            // enforce collision
            all_collisions(model, p);
            all_ropes(model);
            all_springs(model);
            // apply the physics simulation itself
            model.particles[p].verlet(sub_dt as f32);
        }
    }
}


fn apply_gravity(particle: &mut Particle) {
    particle.acceleration += vec2(0.0, -1.0); // apply gravity
}
// TODO: make rectangle instead of circle
fn apply_world_constraint(particle: &mut Particle) {
    let radius = 400.0;

    let dist_to_particle = particle.position.length(); // if the constraint is in the middle the distance from the middle to the particle is its pos magnitude

    if dist_to_particle > radius - particle.mass {
        let normal = particle.position / dist_to_particle; // just gets the normal
        particle.position = normal * (radius - particle.mass);
    }
}

fn apply_collision_constraint(particle1: &Particle, particle2: &Particle) -> Vec2 {
    let to_particle = particle2.position - particle1.position;
    let distance = to_particle.length();
    if distance < particle1.mass + particle2.mass {
        let normal = to_particle.normalize();
        let delta = (particle1.mass + particle2.mass) - distance;
        return 0.5 * delta * normal;
    }
    Vec2::ZERO
}

fn all_collisions(model: &mut Model, p: usize){
    for p2 in 0..model.particles.len() {
        if p == p2 {
            continue;
        }
        let move_by = apply_collision_constraint(&model.particles[p], &model.particles[p2]);
        model.particles[p2].position += move_by;
        model.particles[p].position -= move_by;
    }
}

fn all_springs(model: &mut Model){
    if model.springs.len() < 1 {return;}
    for spring in model.springs.iter() {
        let constraint = Particle::apply_spring_constraint(&model.particles[spring.0], &model.particles[spring.1], 100.0, 1.0);
        let mass1 = model.particles[spring.0].mass;
        let mass2 = model.particles[spring.1].mass;
        if !model.particles[spring.0].freezed {
            model.particles[spring.0].acceleration -= constraint / mass1;

        }
        if !model.particles[spring.1].freezed {
            model.particles[spring.1].acceleration += constraint / mass2;

        }

    }
}

fn all_ropes(model: &mut Model){
    if model.ropes.len() < 1 {return;}
    for rope in model.ropes.iter_mut() {
        let constraint = Particle::apply_line_constraint(&model.particles[rope.0], &model.particles[rope.1], 100.0);
        if !model.particles[rope.0].freezed {
            model.particles[rope.0].position -= constraint;
        
        }
        if !model.particles[rope.1].freezed {
            model.particles[rope.1].position += constraint;
        
        }


    }
    
}

fn pick_up_ball(mouse: &Mouse, model: &mut Model) {
    if model.last_pressed != -1 {
        // for _ in 0..5 {
        //     let current_pos = model.particles[model.last_pressed as usize].position;
        //     let last_pos = model.particles[model.last_pressed as usize].prev_position;
        //     let vel = current_pos - last_pos;
        //     model.particles[model.last_pressed as usize].position += (0.1 * (mouse.position() - current_pos).normalize()) / vel.length().ceil();

        // }
        model.particles[model.last_pressed as usize].position = mouse.position();
        model.particles[model.last_pressed as usize].prev_position = mouse.position();
    }
}

fn freeze_balls(mouse: &Mouse, model: &mut Model)
{
    if model.last_pressed != -1 && mouse.buttons.right().is_down() {
        model.particles[model.last_pressed as usize].freezed = true;
    }
}

fn draw_particles(model: &Model, draw: &Draw) {
    for p in 0..model.particles.len() {
        model.particles[p].draw(draw);
    }
}

fn select_ball(mouse: &Mouse, model: &mut Model) {
    let index = match model
        .particles
        .iter()
        .position(|t| mouse.position().distance(t.position) <= t.mass)
    {
        Some(x) => x,
        _ => 0,
    };
    if mouse.buttons.left().is_down() && model.last_pressed == -1 {
        match model
            .particles
            .iter_mut()
            .find(|t| mouse.position().distance(t.position) <= t.mass)
        {
            Some(x) => model.last_pressed = index as i32,
            _ => println!("northing"),
        };
    }
    if mouse.buttons.left().is_up() {
        model.last_pressed = -1;
    }
}
