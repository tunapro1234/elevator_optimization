pub struct ElevatorMotor {
    max_force: f32, // max force the motor can output
    energy_efficiency: f32, // energy efficiency of the motor
}

impl ElevatorMotor {
    pub fn new(
        max_force: f32,
        energy_efficiency: f32,
    ) -> Self {
        Self {
            max_force,
            energy_efficiency,
        }
    }

    fn limit_force(&self, force: f32) -> f32 {
        if force > self.max_force {
            self.max_force
        } else if force < -self.max_force {
            -self.max_force
        } else {
            force
        }
    }

    pub fn calculate_used_energy(&self, mut force: f32, speed: f32, accel: f32, time: f32) -> f32 {
        // this is probably unnecessary
        force = self.limit_force(force);
        
        let distance = speed * time + 0.5 * accel * time * time;
        let energy_output = force * distance;

        let energy_input = energy_output / self.energy_efficiency;

        energy_input
    }
}