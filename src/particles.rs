use nannou::prelude::*;

pub struct Particle {
    pub position: Vec2,
    pub prev_position: Vec2,
    pub acceleration: Vec2,
    pub mass: f32,
    pub freezed: bool,
}

impl Particle {
    pub fn verlet(&mut self, dt: f32) {
        if self.freezed {
            return;
        }
        // get the velocity using the direction that was moved
        let velocity = self.position - self.prev_position;

        self.prev_position = self.position;
        // set position
        self.position = self.position + velocity + self.acceleration * dt * dt;

        self.acceleration = Vec2::ZERO;
    }

    pub fn apply_line_constraint(particle1: &Particle, particle2: &Particle, length: f32) -> Vec2 {
        let to_point = particle1.position - particle2.position;
        let distance = to_point.length();
        let desired_distance = distance - length;

        let normal = to_point.normalize();

        normal * (desired_distance / 2.0)
    }

    pub fn apply_spring_constraint(particle1: &Particle, particle2: &Particle, length: f32, stiffness: f32) -> Vec2 {
        let to_point = particle1.position - particle2.position;
        let desired_distance = to_point.length() - length;
        let normal = to_point.normalize();

        (normal * desired_distance * stiffness) * 0.5
    }

    pub fn draw(&self, draw: &Draw) {
        draw.ellipse()
            .radius(self.mass)
            .color(WHITE)
            .xy(self.position);
    }
}

impl Default for Particle {
    fn default() -> Self {
        Particle {
            position: Vec2::ZERO,
            prev_position: Vec2::ZERO,
            acceleration: Vec2::ZERO,
            mass: 10.0,
            freezed: false,
        }
    }
}
impl Particle {
    pub fn from(position: Vec2, acceleration: Vec2, radius: f32) -> Self {
        Particle {
            position,
            prev_position: position,
            acceleration,
            mass: radius,
            freezed: false,
        }
    }
}
