pub mod movement {
    use oort_api::prelude::*;
    use std::{collections::HashMap, time::Duration};

    pub trait Kinematic {
        fn position(&self) -> Vec2;
        fn velocity(&self) -> Vec2;
        fn orientation(&self) -> f64;
        fn rotation(&self) -> f64;
        fn at_time(&self, t: Duration) -> Vec2 {
            self.position() + self.velocity() * t.as_secs_f64()
        }
        fn lead(&self, cannon: Vec2, projectile_speed: f64) -> Option<Vec2> {
            let a =
                self.velocity().x.powi(2) + self.velocity().y.powi(2) - projectile_speed.powi(2);
            let b = 2.0
                * (self.velocity().x * (self.position().x - cannon.x)
                    + self.velocity().y * (self.position().y - cannon.y));
            let c = (self.position().x - cannon.x).powi(2) + (self.position().y - cannon.y).powi(2);
            let disc = b.powi(2) - 4.0 * a * c;
            if disc < 0.0 {
                return None;
            }
            let t1 = (-b + disc.sqrt()) / (2.0 * a);
            let t2 = (-b - disc.sqrt()) / (2.0 * a);
            let t = [t1, t2].into_iter().filter(|t| t > &0.0).reduce(f64::min);
            match t {
                Some(t) => Some(self.at_time(Duration::from_secs_f64(t))),
                None => None,
            }
        }
    }

    pub trait Motor: Kinematic {
        fn max_linear_acceleration(&self) -> f64;
        fn max_angular_acceleration(&self) -> f64;
        fn slow_radius(&self) -> f64 {
            100.0
        }
        fn stop_radius(&self) -> f64 {
            25.0
        }
        fn slow_angle(&self) -> f64 {
            TAU / 2.0
        }
        fn stop_angle(&self) -> f64 {
            self.max_angular_acceleration() / 720.0
        }
        fn time_to_target(&self) -> Duration {
            Duration::from_secs_f64(0.1)
        }
    }

    pub trait MovementStrategy {
        fn execute(&self, motor: &dyn Motor) -> Option<(Vec2, f64)>;
    }

    pub struct Seek {
        pub target: Vec2,
    }

    impl MovementStrategy for Seek {
        fn execute(&self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            let direction = self.target - motor.position();
            match direction.length() {
                len if len == 0.0 => None,
                _ => Some((direction.normalize() * motor.max_linear_acceleration(), 0.0)),
            }
        }
    }

    pub struct Flee {
        pub target: Vec2,
    }

    impl MovementStrategy for Flee {
        fn execute(&self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            let direction = motor.position() - self.target;
            match direction.length() {
                len if len == 0.0 => None,
                _ => Some((direction.normalize() * motor.max_linear_acceleration(), 0.0)),
            }
        }
    }

    pub struct Arrive {
        pub target: Vec2,
    }

    impl MovementStrategy for Arrive {
        fn execute(&self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            let direction = self.target - motor.position();
            let magnitude = match direction.length() {
                radius if radius <= motor.stop_radius() => return None,
                radius if radius <= motor.slow_radius() => {
                    motor.max_linear_acceleration() * radius / motor.slow_radius()
                }
                _ => motor.max_linear_acceleration(),
            };
            let target_velocity = magnitude * direction.normalize();
            let delta_velocity = target_velocity - motor.velocity();
            Some((delta_velocity / motor.time_to_target().as_secs_f64(), 0.0))
        }
    }

    pub struct Align {
        pub target: f64,
    }

    impl MovementStrategy for Align {
        fn execute(&self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            let direction = angle_diff(motor.orientation(), self.target);
            let magnitude = match direction.abs() {
                angle if angle <= motor.stop_angle() => return None,
                angle if angle < motor.slow_angle() => {
                    motor.max_angular_acceleration() * angle / motor.slow_angle()
                }
                _ => motor.max_angular_acceleration(),
            };
            let target_rotation = magnitude * direction / direction.abs();
            let delta_rotation = target_rotation - motor.rotation();
            Some((
                Vec2::default(),
                delta_rotation / motor.time_to_target().as_secs_f64(),
            ))
        }
    }

    pub struct Face {
        pub target: Vec2,
    }

    impl MovementStrategy for Face {
        fn execute(&self, actor: &dyn Motor) -> Option<(Vec2, f64)> {
            let direction = self.target - actor.position();
            let delegate = Align {
                target: direction.angle(),
            };
            delegate.execute(actor)
        }
    }

    pub struct MovementBlend {
        pub moves: Vec<(Box<dyn MovementStrategy>, f64)>,
    }

    impl MovementStrategy for MovementBlend {
        fn execute(&self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            self.moves
                .iter()
                .filter_map(|(movement, weight)| {
                    movement.execute(motor).and_then(|(linear, angular)| {
                        Some((linear * weight.clone(), angular * weight))
                    })
                })
                .reduce(|(sum_linear, sum_angular), (linear, angular)| {
                    (sum_linear + linear, sum_angular + angular)
                })
        }
    }
}

use std::time::Duration;

use movement::*;
use oort_api::prelude::*;

struct TutorialTarget {}

impl Kinematic for TutorialTarget {
    fn position(&self) -> Vec2 {
        target()
    }

    fn velocity(&self) -> Vec2 {
        target_velocity()
    }

    fn orientation(&self) -> f64 {
        0.0
    }

    fn rotation(&self) -> f64 {
        0.0
    }
}

pub struct Ship {}

impl Ship {
    pub fn new() -> Ship {
        Ship {}
    }
    pub fn tick(&mut self) {
        let dummy = TutorialTarget {};
        let target = dummy
            .lead(self.position(), 1000.0)
            .unwrap_or(dummy.position());
        let movement = MovementBlend {
            moves: vec![
                (Box::new(Seek { target }), 1.0),
                (Box::new(Face { target }), 1.0),
            ],
        };
        if let Some((linear, angular)) = movement.execute(self) {
            accelerate(linear);
            torque(angular);
        }
        draw_diamond(target, 15.0, 0xff0000);
        let direction = target - self.position();
        let angle = angle_diff(direction.angle(), self.orientation());
        if angle.abs() <= TAU / 360.0 {
            fire(0);
        }
    }
}

impl Kinematic for Ship {
    fn position(&self) -> Vec2 {
        oort_api::prelude::position()
    }
    fn velocity(&self) -> Vec2 {
        oort_api::prelude::velocity()
    }
    fn orientation(&self) -> f64 {
        oort_api::prelude::heading()
    }
    fn rotation(&self) -> f64 {
        oort_api::prelude::angular_velocity()
    }
}

impl Motor for Ship {
    fn max_linear_acceleration(&self) -> f64 {
        [
            max_forward_acceleration(),
            max_backward_acceleration(),
            max_lateral_acceleration(),
        ]
        .into_iter()
        .reduce(f64::max)
        .unwrap()
    }
    fn max_angular_acceleration(&self) -> f64 {
        oort_api::prelude::max_angular_acceleration()
    }
}
