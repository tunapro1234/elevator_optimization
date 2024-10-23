use std::error::Error;
use crate::machine::pid_controller::PIDController;
use super::MotorProperties;

pub struct ElevatorMotor {
    motor_properties: Vec<MotorProperties>,
    current_properties: MotorProperties,
    gear_ratio: f32,
    current_speed: f32,
    pub speed_pid: PIDController,
    pub total_energy_used: f32,
    pub max_force: f32, // max force of the gearbox output shaft
}

impl ElevatorMotor {
    pub fn new(
        properties_path: &str,
        gear_ratio: f32,
    ) -> Result<Self, Box<dyn Error>> {
        let motor_properties = MotorProperties::get(properties_path)?;
        let max_rpm = MotorProperties::get_max_rpm(&motor_properties);
        let max_force = MotorProperties::get_max_tnm(&motor_properties) * gear_ratio;
        let max_current = MotorProperties::get_max_current(&motor_properties);

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

        let current_properties = MotorProperties::simulate_properties_from_current(&motor_properties, 0.);

        Ok( 
            Self {
                motor_properties,
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
        
        // the limit is applied in pid controller
        // if motor_target > self.max_rpm {
        //     self.speed_pid.set_target(self.max_rpm);
        //     false
        // } 
        // else {
        //     self.speed_pid.set_target(motor_target);
        //     true
        // }

        self.speed_pid.set_target(motor_target)
    }

    pub fn get_total_energy_used(&self) -> f32 {
        self.total_energy_used
    }

    pub fn has_reached_target(&self) -> bool {
        self.speed_pid.has_reached_target(self.current_speed)
    }

    pub fn give_current(&mut self, current: f32) {
        self.current_properties = MotorProperties::simulate_properties_from_current(&self.motor_properties, current);
        self.current_speed = self.current_properties.rpm;
    }

    pub fn update(&mut self, delta_time: f32) {
        self.total_energy_used += self.current_properties.kwp_in * delta_time;

        let new_current = self.speed_pid.update(self.current_speed, delta_time);
        self.give_current(new_current);
    }
}

