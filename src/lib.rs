pub mod movement {
    use oort_api::prelude::*;
    pub trait Kinematic {
        fn position(&self) -> Vec2;
        fn velocity(&self) -> Vec2;
        fn heading(&self) -> f64;
        fn angular_velocity(&self) -> f64;
    }
    pub trait Motor {
        fn max_linear_acceleration(&self) -> f64;
        fn max_angular_acceleration(&self) -> f64;
    }
    pub struct Output {
        pub linear: Vec2,
        pub angular: f64,
    }

    pub trait Move {
        fn execute(&self, actor: &(impl Kinematic + Motor)) -> Output;
    }

    pub struct Seek {
        pub target: Vec2,
    }

    impl Move for Seek {
        fn execute(&self, actor: &(impl Kinematic + Motor)) -> Output {
            let direction = self.target - actor.position();
            Output {
                linear: direction.normalize() * actor.max_linear_acceleration(),
                angular: 0.0,
            }
        }
    }
}

use movement::*;
use oort_api::prelude::*;

pub struct Ship {}

impl Ship {
    pub fn new() -> Ship {
        Ship {}
    }
    pub fn tick(&mut self) {
        let action = Seek { target: target() };
        let result = action.execute(self);
        accelerate(result.linear);
    }
}

impl Kinematic for Ship {
    fn position(&self) -> Vec2 {
        oort_api::prelude::position()
    }
    fn velocity(&self) -> Vec2 {
        oort_api::prelude::velocity()
    }
    fn heading(&self) -> f64 {
        oort_api::prelude::heading()
    }
    fn angular_velocity(&self) -> f64 {
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
        .unwrap_or(0.0)
    }
    fn max_angular_acceleration(&self) -> f64 {
        oort_api::prelude::max_angular_acceleration()
    }
}