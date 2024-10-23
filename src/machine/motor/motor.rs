use std::error::Error;
use crate::machine::pid_controller::PIDController;
use super::motor_samples::MotorSamples;
use std::time::Instant;

pub struct ElevatorMotor {
    motor_samples: Vec<MotorSamples>,
    current_properties: MotorSamples,
    gear_ratio: f32,
    current_speed: f32,
    speed_pid: PIDController,
    pub total_energy_used: f32,
    pub max_force: f32, // max force of the gearbox output shaft
}

impl ElevatorMotor {
    pub fn new(
        properties_path: &str,
        gear_ratio: f32,
    ) -> Result<Self, Box<dyn Error>> {
        let motor_samples = MotorSamples::get(properties_path)?;
        let max_rpm = MotorSamples::get_max_rpm(&motor_samples);
        let max_force = MotorSamples::get_max_tnm(&motor_samples) * gear_ratio;
        let max_current = MotorSamples::get_max_current(&motor_samples);

        let speed_pid = PIDController::new(
            1., 
            0., 
            0., 
            30.,
            0.,
            true,
            max_rpm,
            -max_rpm,
            max_current,
            -max_current
        );

        let current_properties = MotorSamples::simulate_properties_from_current(&motor_samples, 0.)
            .unwrap();

        Ok( 
            Self {
                motor_samples,
                gear_ratio,
                speed_pid,
                current_properties,
                current_speed: 0.0,
                total_energy_used: 0.0,
                max_force,
            }
        )
    }

    pub fn get_current_speed(&self) -> f32 {
        // this function gives the speed of the output shaft of the gear box
        self.current_speed / self.gear_ratio
    }

    pub fn set_target_speed(&mut self, target: f32) -> bool {
        // this function sets the speed of the output shaft of the gear box
        let motor_target = target*self.gear_ratio;

        // rpm limit is applied in pid controller
        self.speed_pid.set_target(motor_target)
    }

    pub fn get_total_energy_used(&self) -> f32 {
        self.total_energy_used
    }

    pub fn has_reached_target(&self) -> bool {
        self.speed_pid.has_reached_target(self.current_speed)
    }

    fn give_current(&mut self, current: f32) {
        self.current_properties = MotorSamples::simulate_properties_from_current(&self.motor_samples, current)
            .expect("Motor current limit exceeded");

        self.current_speed = self.current_properties.rpm;
    }

    pub fn update(&mut self, delta_time: f32) {
        self.total_energy_used += self.current_properties.kwp_in * delta_time;

        let new_current = self.speed_pid.update(self.current_speed, delta_time);
        self.give_current(new_current);
    }
}


#[cfg(test)]
mod tests {
    // Import the outer module's functions
    use super::*;

    #[test]
    fn give_current() {
        let mut motor = ElevatorMotor::new("data/motor_samples.csv", 1.).unwrap();
        let max_current = MotorSamples::get_max_current(&motor.motor_samples);
        motor.give_current(max_current-5.);
        assert!(motor.current_speed > 0.);
    }

    #[test]
    fn give_negative_current() {
        let mut motor = ElevatorMotor::new("data/motor_samples.csv", 1.).unwrap();
        let max_current = MotorSamples::get_max_current(&motor.motor_samples);
        motor.give_current(-max_current+5.);
        assert!(motor.current_speed < 0.);
    }

    #[test]
    #[should_panic]
    fn overcurrent() {
        let mut motor = ElevatorMotor::new("data/motor_samples.csv", 1.).unwrap();
        let max_current = MotorSamples::get_max_current(&motor.motor_samples);
        motor.give_current(max_current+5.);
    }

    #[test]
    #[should_panic]
    fn negative_overcurrent() {
        let mut motor = ElevatorMotor::new("data/motor_samples.csv", 1.).unwrap();
        let max_current = MotorSamples::get_max_current(&motor.motor_samples);
        motor.give_current(-max_current-5.);
    }

    #[test]
    fn set_speed() {
        let mut motor = ElevatorMotor::new("data/motor_samples.csv", 1.).unwrap();
        let max_rpm = MotorSamples::get_max_rpm(&motor.motor_samples);
        let target_speed = max_rpm/2.;

        motor.set_target_speed(target_speed);
        
        let start = Instant::now();
        loop {
            motor.update(0.01);
            if motor.has_reached_target() {
                assert!(motor.current_speed == target_speed);
            }

            // time limit
            let elapsed = start.elapsed().as_secs_f32();
            if elapsed >= 3. {
                panic!("Timeout");
            }
        }
        
    }

    #[test]
    fn set_negative_speed() {
        let mut motor = ElevatorMotor::new("data/motor_samples.csv", 1.).unwrap();
        let max_rpm = MotorSamples::get_max_rpm(&motor.motor_samples);
        let target_speed = -max_rpm/2.;

        motor.set_target_speed(target_speed);
        
        let start = Instant::now();
        loop {
            motor.update(0.01);
            if motor.has_reached_target() {
                assert!(motor.current_speed == target_speed);
            }

            // time limit
            let elapsed = start.elapsed().as_secs_f32();
            if elapsed >= 3. {
                panic!("Timeout");
            }
        }
        
    }

}