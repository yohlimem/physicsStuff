use nannou::{prelude::*, state::Mouse, lyon::geom::euclid::num::Floor, draw::mesh::vertex::Color};
use nannou_egui::{self, egui::{self, Color32}, Egui};

mod particles;
use crate::particles::Particle;

const MAX_DRAW_BUFFER_LENGTH: usize = 50;
const DELTA_TIME: f32 = 1.0;
const CIRCLE_SIZE: f32 = 500.0;

struct Model {
    // window: Window,

    // nannou egui
    // all objects
    particles: Vec<Particle>,
    // how many substeps per step
    substeps: u32,
    // the last circle pressed
    last_left_pressed: i32,
    // the last circle the mouse was on
    last_right_pressed: i32,
    chosen_balls: [i32; 2],
    ropes: Vec<(usize, usize)>,
    springs: Vec<(usize, usize)>,
    // used to check something once
    once: bool,
    once2: bool,
    // if you connect two balls using right click, this wil ldecide
    stop_sim: bool,
    ball_points_buffer: Vec<Vec2>,
    // gui
        egui: Egui,
        // false = spring, true = rope
        connection_type: bool,
        trail_color: Color32

}

fn main() {
    nannou::app(model).update(update).run();
}

fn model(app: &App) -> Model {
    let window_id = app
        .new_window()
        .size(1600, 1600)
        .view(view)
        .raw_event(raw_window_event)
        .build()
        .unwrap();
    let window = app.window(window_id).unwrap();
    let egui = Egui::from_window(&window);
    let mut particles = vec![
        Particle::from(vec2(0.0, 100.0), Vec2::ZERO, 10.0),
        Particle::from(vec2(0.0, -50.0), Vec2::ZERO, 10.0),
        Particle::from(vec2(0.0, -100.0), Vec2::ZERO, 10.0),
        Particle::from(vec2(0.0, -150.0), Vec2::ZERO, 10.0),
        Particle::from(vec2(3.0, 3.0), Vec2::ZERO, 30.0),
    ];
    let substeps = 10;
    // let ropes = vec![];
    let ropes = vec![(0, 1), (1, 2), (2, 3), (3, 4)];
    let springs = vec![];
    particles[0].freezed = true;
    Model {
        egui,
        particles,
        substeps,
        chosen_balls: [-1, -1],
        last_left_pressed: -1,
        last_right_pressed: -1,
        ropes,
        springs,
        connection_type: false,
        once: false,
        once2: false,
        stop_sim: false,
        ball_points_buffer: vec![],
        trail_color: Color32::from_rgb(255, 255, 40),
    }
}

fn update(app: &App, model: &mut Model, update: Update) {
    // to get egui out fo scope
    {
        let egui = &mut model.egui;
        egui.set_elapsed_time(update.since_start);

        let ctx = egui.begin_frame();

        // model.last_substep = model.substeps;
        egui::Window::new("Rum window").show(&ctx, |ui| {
            ui.label("res");
            ui.add(egui::Checkbox::new(&mut model.connection_type, "spring or cable?"));
            let color_picker_widget = egui::widgets::color_picker::color_picker_color32(
                ui,
                &mut model.trail_color,
                egui::widgets::color_picker::Alpha::OnlyBlend,
            );
            // ui.add(color_picker_widget.);
            let stop_button = ui.add(egui::Button::new("stop simulation"));

            if stop_button.clicked() {
                model.stop_sim = !model.stop_sim;
            }

        });
    }
    select_ball(&app.mouse, model);
    if !model.stop_sim {
        solver(model);
        
        // println!("{}", calculate_total_kinetic_energy(model));
    }
    pick_up_ball(&app.mouse, model);
    connect_balls(&app.mouse, model);
    while model.ball_points_buffer.len() > MAX_DRAW_BUFFER_LENGTH{
        model.ball_points_buffer.remove(0);
    }
}

fn raw_window_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    model.egui.handle_raw_event(event);
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(GRAY);
    draw.ellipse().radius(CIRCLE_SIZE).color(BLACK);
    //------------------------------------------------------------------------------------------------\\
    draw_particles(model, &draw);
    for rope in model.ropes.iter() {
        draw.line().start(model.particles[rope.0].position).end(model.particles[rope.1].position).stroke_weight(10.0).color(BLUE);
    }
    for spring in model.springs.iter() {
        draw.line().start(model.particles[spring.0].position).end(model.particles[spring.1].position).stroke_weight(10.0).color(RED);
    }
    for pos in 0..model.ball_points_buffer.len() as i32{
        let trail_color = rgba(model.trail_color.r(), model.trail_color.g(), model.trail_color.b(), ((pos as f32 / model.ball_points_buffer.len() as f32) * u8::MAX as f32) as u8);
        // println!("ratio: {}, pos: {}, model.ball_points_buffer.len(): {}", (pos as f32 / model.ball_points_buffer.len() as f32) * u8::MAX as f32, pos, model.ball_points_buffer.len());
        draw.line().start(model.ball_points_buffer[pos as usize]).end(model.ball_points_buffer[(pos + 1).min(model.ball_points_buffer.len() as i32 - 1) as usize]).stroke_weight(pos as f32 / 10.0).color(trail_color);
        // println!("{:?}", model.ball_points_buffer[pos as usize]);
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
    model.ball_points_buffer.push(model.particles[4].position);
}


fn apply_gravity(particle: &mut Particle) {
    particle.acceleration += vec2(0.0, -1.0); // apply gravity
}
// TODO: make rectangle instead of circle
fn apply_world_constraint(particle: &mut Particle) {
    let radius = CIRCLE_SIZE;

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
        if !model.particles[p2].freezed {
            model.particles[p2].position += move_by;
        }
        if !model.particles[p].freezed {
            model.particles[p].position -= move_by;
        }
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
    for _ in 0..model.substeps.floor() {
        for rope in model.ropes.iter_mut() {
            let constraint = Particle::apply_line_constraint(&model.particles[rope.0], &model.particles[rope.1], 100.0);
            let mass1 = model.particles[rope.0].mass;
            let mass2 = model.particles[rope.1].mass;
            if !model.particles[rope.0].freezed {
                model.particles[rope.0].position -= constraint / mass1;
            
            }
            if !model.particles[rope.1].freezed {
                model.particles[rope.1].position += constraint / mass2;
            
            }
        }
    }
    
}

fn pick_up_ball(mouse: &Mouse, model: &mut Model) {
    if model.last_left_pressed != -1 {
        // for _ in 0..5 {
        //     let current_pos = model.particles[model.last_pressed as usize].position;
        //     let last_pos = model.particles[model.last_pressed as usize].prev_position;
        //     let vel = current_pos - last_pos;
        //     model.particles[model.last_pressed as usize].position += (0.1 * (mouse.position() - current_pos).normalize()) / vel.length().ceil();

        // }
        model.particles[model.last_left_pressed as usize].position = mouse.position();
        model.particles[model.last_left_pressed as usize].prev_position = mouse.position();
    }
}

// TODO: make a way to connect two balls together.
fn connect_balls(mouse: &Mouse, model: &mut Model)
{
    if model.last_right_pressed == -1 {
        return;
    }

    let mouse_check = mouse.buttons.right().is_down();
    
    if mouse_check && model.chosen_balls[0] == -1 && !model.once {
        model.chosen_balls[0] = model.last_right_pressed;
        model.once = true;
        println!("model.chosen_balls[0]: {}, model.chosen_balls[1]: {}", model.chosen_balls[0], model.chosen_balls[1]);
        
        
    }
    if mouse_check && model.chosen_balls[1] == -1 && model.chosen_balls[0] != model.last_right_pressed && !model.once2 {
        model.chosen_balls[1] = model.last_right_pressed;
        model.once2 = true;
        
        println!("model.chosen_balls[0]: {}, model.chosen_balls[1]: {}", model.chosen_balls[0], model.chosen_balls[1]);
    }

    if model.chosen_balls[0] >= 0 && model.chosen_balls[1] >= 0 && mouse.buttons.right().is_up() {
        if model.connection_type {
            model.ropes.push((model.chosen_balls[0] as usize, model.chosen_balls[1] as usize));
        } else {
            model.springs.push((model.chosen_balls[0] as usize, model.chosen_balls[1] as usize));
        }
        
        if model.once {
            model.once = false;
    
        }
        if model.once2 {
            model.once2 = false;
    
        }
        model.chosen_balls[0] = -1;
        model.chosen_balls[1] = -1;
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
    if mouse.buttons.left().is_down() && model.last_left_pressed == -1 {
        match model
        .particles
        .iter_mut()
        .find(|t| mouse.position().distance(t.position) <= t.mass)
        {
            Some(x) => model.last_left_pressed = index as i32,
            _ => (),
        };
    }
    if mouse.buttons.right().is_down() && model.last_right_pressed == -1 {
        match model
        .particles
        .iter_mut()
        .find(|t| mouse.position().distance(t.position) <= t.mass)
        {
            Some(x) => model.last_right_pressed = index as i32,
            _ => (),
        };
    }
    if mouse.buttons.left().is_up() {
        model.last_left_pressed = -1;
    }
    if mouse.buttons.left().is_up() {
        model.last_right_pressed = -1;
    }
}

fn calculate_kinetic_energy(particle: &Particle) -> f32{
    let vel = (particle.position - particle.prev_position).length();
    (particle.mass / 2.0) * vel * vel
}

fn calculate_total_kinetic_energy(model: &Model) -> f32{
    let mut energy = 0.0;
    for p in 0..model.particles.len() {
        energy += calculate_kinetic_energy(&model.particles[p]);
    }
    energy
}