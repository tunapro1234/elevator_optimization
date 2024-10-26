// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2024 Tuna Gül

use std::error::Error;

use crate::machine::pid_controller::PIDController;
use crate::util::LinePlotter;
use super::motor_samples::MotorSamples;
use super::motor_parameters::MotorParameters;

pub struct ElevatorMotor {
    motor_samples: Vec<MotorSamples>,
    current_properties: MotorSamples,
    gearbox_ratio: f32,
    current_current: f32,
    speed_pid: PIDController,
    pub total_energy_used: f32,
    pub max_force: f32, // max force of the gearbox output shaft
    line_plotter: Option<LinePlotter>,
}

impl ElevatorMotor {
    pub fn from_file(
        file_path: &str,        
    ) -> Result<Self, Box<dyn Error>> {
        let parameters = MotorParameters::from_file(file_path)?;
        Self::new(parameters)
    }

    pub fn new(
        parameters: MotorParameters,
    ) -> Result<Self, Box<dyn Error>> {

        let mut speed_pid: PIDController = PIDController::new(parameters.pid_parameters);        
        let motor_samples = MotorSamples::from_file(parameters.sample_path.as_str())?;

        // hız limitleri
        let max_rpm = MotorSamples::get_max_rpm(&motor_samples);
        let max_soft_rpm = parameters.soft_rpm_limit;
        // hangisi daha küçükse onu limit olarak kullanacağız
        let rpm_limit = if max_soft_rpm <= 0. {
            max_rpm
        } else {
            max_rpm.min(max_soft_rpm)
        };

        let max_current = MotorSamples::get_max_current(&motor_samples);
        let max_soft_current = parameters.soft_current_limit;
        // hangisi daha küçükse onu limit olarak kullanacağız
        let current_limit = if max_soft_current <= 0. {
            max_current
        } else {
            max_current.min(max_soft_current)
        };

        // hız ve akım sınırlarını belirle
        speed_pid.set_output_limits(
            -current_limit, 
            current_limit, 
        );
        speed_pid.set_target_limits(
            rpm_limit, 
            -rpm_limit,
        );

        let max_torque = MotorSamples::get_max_tnm(&motor_samples) * parameters.gearbox_ratio;
        let max_force =  max_torque * parameters.output_shaft_radius;

        // I am unwrapping here because i know it will not panic
        let current_properties = MotorSamples::simulate_properties_from_current(&motor_samples, 0.)
            .unwrap();

        // line plotterı yarat
        let line_plotter = if parameters.enable_debug_plotting {
            let plot_rv = LinePlotter::new(parameters.plot_path);
            match plot_rv {
                Ok(line_plotter) => {
                    println!("Plotter created");
                    Some(line_plotter)
                },
                Err(e) => {
                    println!("Error creating plotter: {}", e);
                    None
                }
            }
        } else {
            None
        };

        Ok(
            Self {
                motor_samples,
                gearbox_ratio: parameters.gearbox_ratio,
                current_properties,
                speed_pid,
                current_current: 0.0,
                total_energy_used: 0.0,
                max_force,
                line_plotter,
            }
        )
    }

    fn get_rpm(&self) -> f32 {
        self.current_properties.rpm
    }

    pub fn get_current_speed(&self) -> f32 {
        // this function gives the speed of the output shaft of the gear box
        self.get_rpm() / self.gearbox_ratio
    }

    pub fn set_target_speed(&mut self, target: f32) -> bool {
        // this function sets the speed of the output shaft of the gear box
        let motor_target = target*self.gearbox_ratio;

        // rpm limit is applied in pid controller
        self.speed_pid.set_target(motor_target)
    }

    pub fn get_total_energy_used(&self) -> f32 {
        self.total_energy_used
    }

    pub fn has_reached_target(&self) -> bool {
        self.speed_pid.has_reached_target(self.get_rpm())
    }

    fn give_current(&mut self, current: f32) {
        self.current_properties = MotorSamples::simulate_properties_from_current(&self.motor_samples, current)
            .expect("Motor current limit exceeded");
    }

    fn plot(&mut self) {
        if let Some(line_plotter) = &mut self.line_plotter {
            line_plotter.add_point(self.current_properties.rpm);
            line_plotter.update().unwrap();
        }
    }

    pub fn update(&mut self, delta_time: f32) {
        self.total_energy_used += self.current_properties.kwp_in * delta_time;

        let current_change = self.speed_pid.update(self.get_rpm(), delta_time);
        self.current_current += current_change;

        self.give_current(self.current_current);
        self.plot();
    }
}


#[cfg(test)]
mod tests {
    // Import the outer module's functions
    use super::*;
    use std::time::Instant;

    #[test]
    fn give_current() {
        let mut motor = ElevatorMotor::from_file("param/motor_test_parameters.yaml").unwrap();
        let max_current = MotorSamples::get_max_current(&motor.motor_samples);
        motor.give_current(max_current-5.);
        assert!(motor.current_properties.rpm > 0.);
    }

    #[test]
    fn give_negative_current() {
        let mut motor = ElevatorMotor::from_file("param/motor_test_parameters.yaml").unwrap();
        let max_current = MotorSamples::get_max_current(&motor.motor_samples);
        motor.give_current(-max_current+5.);
        assert!(motor.current_properties.rpm < 0.);
    }

    #[test]
    #[should_panic]
    fn overcurrent() {
        let mut motor = ElevatorMotor::from_file("param/motor_test_parameters.yaml").unwrap();
        let max_current = MotorSamples::get_max_current(&motor.motor_samples);
        motor.give_current(max_current+5.);
    }

    #[test]
    #[should_panic]
    fn negative_overcurrent() {
        let mut motor = ElevatorMotor::from_file("param/motor_test_parameters.yaml").unwrap();
        let max_current = MotorSamples::get_max_current(&motor.motor_samples);
        motor.give_current(-max_current-5.);
    }

    #[test]
    fn set_speed() {
        let mut motor = ElevatorMotor::from_file("param/motor_test_parameters.yaml").unwrap();
        let max_rpm = MotorSamples::get_max_rpm(&motor.motor_samples);
        let target_speed: f32 = max_rpm/2.;

        motor.set_target_speed(target_speed);
        
        let start = Instant::now();
        loop {
            motor.update(start.elapsed().as_secs_f32());
            if motor.has_reached_target() {
                break;
            }

            // time limit
            let elapsed = start.elapsed().as_secs_f32();
            if elapsed >= 6. {
                panic!("Timeout");
            }
        }
        assert!((motor.current_properties.rpm - target_speed).abs() < 10.);
    }

    #[test]
    fn set_negative_speed() {
        let mut motor = ElevatorMotor::from_file("param/motor_test_parameters.yaml").unwrap();
        let max_rpm = MotorSamples::get_max_rpm(&motor.motor_samples);
        let target_speed: f32 = -max_rpm/2.;

        motor.set_target_speed(target_speed);
        
        let start = Instant::now();
        loop {
            motor.update(start.elapsed().as_secs_f32());
            if motor.has_reached_target() {
                break;
            }

            // time limit
            let elapsed = start.elapsed().as_secs_f32();
            if elapsed >= 6. {
                panic!("Timeout");
            }
        }
        assert!((motor.current_properties.rpm - target_speed).abs() < 10.);
    }

    #[test]
    fn for_tuning() {
        let mut motor = ElevatorMotor::from_file("param/motor_test_parameters.yaml").unwrap();
        let target_speed: f32 = 200.;

        motor.set_target_speed(target_speed);
        
        let start = Instant::now();
        loop {
            motor.update(start.elapsed().as_secs_f32());
            // if motor.has_reached_target() {
            //     break;
            // }

            // time limit
            let elapsed = start.elapsed().as_secs_f32();
            if elapsed >= 3. {
                // panic!("Timeout");
                break;
            }
        }
        assert!((motor.current_properties.rpm - target_speed).abs() < 10.);
    }
}