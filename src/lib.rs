pub mod movement {
    use maths_rs::prelude::{deg_to_rad, Base};
    use oort_api::prelude::*;
    use std::time::Duration;

    pub trait Vector {
        fn sq_length(&self) -> f64;
    }

    impl Vector for Vec2 {
        fn sq_length(&self) -> f64 {
            self.x.powi(2) + self.y.powi(2)
        }
    }

    pub trait Kinematic {
        fn position(&self) -> Vec2;
        fn velocity(&self) -> Vec2;
        fn orientation(&self) -> f64;
        fn rotation(&self) -> f64;
        fn forward(&self) -> Vec2 {
            Vec2::new(1.0, 0.0).rotate(self.orientation())
        }
        fn speed(&self) -> f64 {
            self.velocity().length()
        }
        fn at_time(&self, time: Duration) -> Vec2 {
            self.position() + self.velocity() * time.as_secs_f64()
        }
        fn lead_time(&self, cannon: Vec2, projectile_speed: f64) -> Option<Duration> {
            let a = self.velocity().sq_length() - projectile_speed.powi(2);
            let dp = self.position() - cannon;
            let b = 2.0 * self.velocity().dot(dp);
            let c = dp.sq_length();
            let disc = b.powi(2) - 4.0 * a * c;
            match disc {
                _ if disc < 0.0 => None,
                disc => [1.0, -1.0]
                    .iter()
                    .map(|sign| (-b + sign * disc.sqrt()) / (2.0 * a))
                    .filter(|t| *t > 0.0)
                    .reduce(f64::min)
                    .map(|t| Duration::from_secs_f64(t)),
            }
        }
        fn lead_position(&self, cannon: Vec2, projectile_speed: f64) -> Option<Vec2> {
            self.lead_time(cannon, projectile_speed)
                .map(|t| self.at_time(t))
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
            deg_to_rad(90.0)
        }
        fn stop_angle(&self) -> f64 {
            deg_to_rad(0.5)
        }
        fn time_to_target(&self) -> Duration {
            Duration::from_secs_f64(TICK_LENGTH)
        }
    }

    pub trait MovementStrategy {
        fn execute(&mut self, motor: &dyn Motor) -> Option<(Vec2, f64)>;
    }

    pub struct Seek {
        pub target: Vec2,
    }

    impl MovementStrategy for Seek {
        fn execute(&mut self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            draw_diamond(self.target, 15.0, 0xffff00);
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
        fn execute(&mut self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
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
        fn execute(&mut self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
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
        fn execute(&mut self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            /* let braking_time =
                Duration::from_secs_f64(motor.rotation().abs() / motor.max_angular_acceleration());
            let stop_angle = motor.orientation()
                + motor.rotation() * braking_time.as_secs_f64()
                + 0.5 * motor.max_angular_acceleration() * braking_time.as_secs_f64().powi(2);
            debug!("time to stop: {time_to_stop:?}, stop angle: {stop_angle:.3}, target angle: {:.3}", self.target);
            if stop_angle > self.target {
                return Some((Vec2::zero(), motor.max_angular_acceleration()))
            } else {
                return Some((Vec2::zero(), -motor.max_angular_acceleration()))
            } */

            let direction = angle_diff(motor.orientation(), self.target);
            let magnitude = match direction.abs() {
                angle if angle <= motor.stop_angle() => 0.0,
                angle if angle < motor.slow_angle() => {
                    motor.max_angular_acceleration() * angle / motor.slow_angle()
                }
                _ => motor.max_angular_acceleration(),
            };
            let target_rotation = magnitude * direction / direction.abs();
            let delta_rotation = target_rotation - motor.rotation();
            match delta_rotation {
                _ if delta_rotation == 0.0 => None,
                _ => Some((
                    Vec2::default(),
                    delta_rotation / motor.time_to_target().as_secs_f64(),
                )),
            }
        }
    }

    pub struct Face {
        pub target: Vec2,
    }

    impl MovementStrategy for Face {
        fn execute(&mut self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            let direction = self.target - motor.position();
            draw_line(
                motor.position(),
                motor.position() + motor.forward() * 25.0,
                0xffffff,
            );
            draw_line(
                motor.position(),
                motor.position() + direction.normalize() * 25.0,
                0xffffff,
            );
            match direction {
                _ if direction == Vec2::zero() => None,
                _ => Align {
                    target: direction.angle(),
                }
                .execute(motor),
            }
        }
    }

    pub struct MatchVelocity {
        pub target: Vec2,
    }

    impl MovementStrategy for MatchVelocity {
        fn execute(&mut self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            Some((
                (self.target - motor.velocity()) / motor.time_to_target().as_secs_f64(),
                0.0,
            ))
        }
    }

    pub struct Pursue {
        pub target: Box<dyn Kinematic>,
        pub max_prediction: Duration,
    }

    impl MovementStrategy for Pursue {
        fn execute(&mut self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            let direction = self.target.position() - motor.position();
            let prediction =
                if motor.speed() <= direction.length() / self.max_prediction.as_secs_f64() {
                    self.max_prediction
                } else {
                    Duration::from_secs_f64(direction.length() / motor.speed())
                };
            Seek {
                target: self.target.at_time(prediction),
            }
            .execute(motor)
        }
    }

    pub struct Evade {
        pub target: Box<dyn Kinematic>,
        pub max_prediction: Duration,
    }

    impl MovementStrategy for Evade {
        fn execute(&mut self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            let direction = self.target.position() - motor.position();
            let prediction =
                if motor.speed() <= direction.length() / self.max_prediction.as_secs_f64() {
                    self.max_prediction
                } else {
                    Duration::from_secs_f64(direction.length() / motor.speed())
                };
            Flee {
                target: self.target.at_time(prediction),
            }
            .execute(motor)
        }
    }

    pub struct FaceForward {}

    impl MovementStrategy for FaceForward {
        fn execute(&mut self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            match motor.speed() {
                speed if speed == 0.0 => None,
                _ => Align {
                    target: velocity().angle(),
                }
                .execute(motor),
            }
        }
    }

    pub struct Wander {
        pub radius: f64,
        pub offset: f64,
        pub rate: f64,
        pub orientation: f64,
    }

    impl MovementStrategy for Wander {
        fn execute(&mut self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            self.orientation += rand(-1.0, 1.0) * self.rate * TICK_LENGTH;
            let circle_center = motor.position() + motor.forward() * self.offset;
            let target_position = circle_center
                + Vec2::new(1.0, 0.0).rotate(motor.orientation() + self.orientation) * self.radius;
            draw_polygon(circle_center, self.radius, 20, 0.0, 0xffffff);
            draw_diamond(target_position, 1.0, 0xffffff);
            Some((
                Face {
                    target: target_position,
                }
                .execute(motor)
                .unwrap_or((Vec2::zero(), 0.0))
                .0,
                Seek {
                    target: target_position,
                }
                .execute(motor)
                .unwrap_or((Vec2::zero(), 0.0))
                .1,
            ))
        }
    }

    pub struct MovementBlend {
        pub moves: Vec<(Box<dyn MovementStrategy>, f64)>,
    }

    impl MovementStrategy for MovementBlend {
        fn execute(&mut self, motor: &dyn Motor) -> Option<(Vec2, f64)> {
            self.moves
                .iter_mut()
                .filter_map(|(movement, weight)| {
                    movement
                        .execute(motor)
                        .map(|(linear, angular)| (linear * *weight, angular * *weight))
                })
                .reduce(|(sum_linear, sum_angular), (linear, angular)| {
                    (sum_linear + linear, sum_angular + angular)
                })
        }
    }
}

pub mod perception {
    use std::time::Duration;

    use maths_rs::prelude::Cast;

    struct Kalman {
        count: u32,
        estimate: f64,
    }

    impl Kalman {
        fn new(value: f64) -> Kalman {
            Kalman {
                count: 0,
                estimate: value,
            }
        }
        fn estimate(&self) -> f64 {
            self.estimate
        }
        fn predict(&self) -> f64 {
            self.estimate
        }
        fn update(&mut self, value: f64) -> f64 {
            self.count += 1;
            self.estimate += self.gain() * (value - self.estimate);
            self.estimate
        }
        fn gain(&self) -> f64 {
            1.0 / self.count as f64
        }
    }

    #[test]
    fn test_kalman_static() {
        let mut gold = Kalman::new(1000.0);
        assert_eq!(gold.estimate(), 1000.0);
        let values = [
            (996.0, 996.0),
            (994.0, 995.0),
            (1021.0, 1003.67),
            (1000.0, 1002.75),
            (1002.0, 1002.6),
            (1010.0, 1003.83),
            (983.0, 1000.86),
            (971.0, 997.13),
            (993.0, 996.67),
            (1023.0, 999.3),
        ];
        for (measurement, expected) in values {
            let estimate = gold.update(measurement);
            assert_eq!(round(estimate, 2), expected);
        }
    }

    struct DynKalman {
        alpha: f64,
        beta: f64,
        interval: Duration,
        est_value: f64,
        est_change: f64,
    }

    impl DynKalman {
        fn new(estimate: f64, delta: f64, interval: Duration) -> DynKalman {
            DynKalman {
                alpha: 0.2,
                beta: 0.1,
                interval,
                est_value: estimate,
                est_change: delta,
            }
        }
        fn update(&mut self, measurement: f64) {
            let (pred_pos, pred_vel) = self.predict();
            self.est_value = pred_pos + self.alpha * (measurement - pred_pos);
            self.est_change =
                pred_vel + self.beta * (measurement - pred_pos) / self.interval.as_secs_f64();
        }
        fn estimate(&self) -> (f64, f64) {
            (self.est_value, self.est_change)
        }
        fn predict(&self) -> (f64, f64) {
            (
                self.est_value + self.est_change * self.interval.as_secs_f64(),
                self.est_change,
            )
        }
    }

    #[test]
    fn test_kalman_dynamic() {
        let mut aircraft = DynKalman::new(30_000.0, 40.0, Duration::from_secs(5));
        let (p, v) = aircraft.predict();
        assert_eq!(p, 30_200.0);
        assert_eq!(v, 40.0);
        let values = [
            (30_171.0, 30_194.2, 39.42, 30_391.3, 39.42),
            (30_353.0, 30_383.64, 38.65, 30_576.9, 38.65),
            (30_756.0, 30_612.73, 42.2, 30_823.9, 42.2),
            (30_799.0, 30_818.93, 41.7, 31_027.6, 41.7),
            (31_018.0, 31_025.7, 41.55, 31_233.4, 41.55),
            (31_278.0, 31_242.3, 42.44, 31_454.5, 42.44),
            (31_276.0, 31_418.8, 38.9, 31_613.15, 38.9),
            (31_379.0, 31_566.3, 34.2, 31_737.24, 34.2),
            (31_748.0, 31_739.4, 34.4, 31_911.4, 34.4),
        ];
        let precision = 0;
        for (
            measurement,
            expected_estimated_position,
            expected_estimated_velocity,
            expected_predicted_position,
            expected_predictied_velocity,
        ) in values
        {
            aircraft.update(measurement);
            let (actual_estimated_position, actual_estimated_velocity) = aircraft.estimate();
            assert_eq!(
                round(actual_estimated_position, precision),
                round(expected_estimated_position, precision)
            );
            assert_eq!(
                round(actual_estimated_velocity, precision),
                round(expected_estimated_velocity, precision)
            );
            let (actual_predicted_position, actual_predicted_velocity) = aircraft.predict();
            assert_eq!(
                round(actual_predicted_position, precision),
                round(expected_predicted_position, precision)
            );
            assert_eq!(
                round(actual_predicted_velocity, precision),
                round(expected_predictied_velocity, precision)
            );
        }
    }

    fn round(value: f64, precision: i32) -> f64 {
        let power = 10_f64.powi(precision);
        (value * power).round() / power
    }
    #[test]
    fn test_round() {
        assert_eq!(round(2.0 / 3.0, 1), 0.7);
        assert_eq!(round(2.0 / 3.0, 2), 0.67);
        assert_eq!(round(2.0 / 3.0, 3), 0.667);
        assert_eq!(round(2.0 / 3.0, 4), 0.6667);
    }
}

use movement::*;
use oort_api::prelude::{maths_rs::deg_to_rad, *};

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

impl Ship {
    pub fn new() -> Ship {
        Ship {}
    }
    pub fn tick(&mut self) {
        let dummy = Box::new(TutorialTarget {});
        let target = dummy
            .lead_position(self.position(), 925.0)
            .unwrap_or(dummy.position());
        let mut movement = Face { target };
        if let Some((linear, angular)) = movement.execute(self) {
            accelerate(linear);
            torque(angular);
        }
        draw_diamond(target, 15.0, 0xff0000);
        let direction = target - self.position();
        let angle = angle_diff(direction.angle(), self.orientation());
        if angle.abs() <= deg_to_rad(2.0) {
            // activate_ability(Ability::Boost);
            fire(0);
        }
    }
}
