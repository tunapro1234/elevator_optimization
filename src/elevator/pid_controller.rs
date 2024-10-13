pub struct PIDController {
    pub target: f32,
    kp: f32,
    ki: f32,
    kd: f32,
    prev_error: f32,
    integral: f32,
    prev_output: f32,
    accumulated_time: f32, // to keep at constant frequency
    update_freq: f32,
    tolerance: f32,
}

impl PIDController {
    pub fn new(
        kp: f32, 
        ki: f32, 
        kd: f32, 
        update_freq: f32,
        tolerance: f32,
    ) -> Self {
        Self {
            target: 0.0,
            kp,
            ki,
            kd,
            prev_error: 0.0,
            integral: 0.0,
            prev_output: 0.0,
            accumulated_time: 0.0,
            update_freq,
            tolerance,
        }
    }

    pub fn set_target(&mut self, target: f32) {
        self.target = target;
    }

    pub fn has_reached_target(&self, current_value: f32) -> bool {
        (self.target - current_value).abs() < self.tolerance
    }

    pub fn update(&mut self, current_value: f32, delta_time: f32) -> f32 {
        // Accumulate the time
        self.accumulated_time += delta_time;

        // Check if enough time has passed for an update (based on frequency)
        if self.accumulated_time < (1.0 / self.update_freq) {
            // If not enough time has passed, return the last output
            return self.prev_output;
        }

        // Reset accumulated time after enough time has passed
        self.accumulated_time = 0.0;

        let error = self.target - current_value;

        // Proportional term
        let proportional = self.kp * error;

        // Integral term
        self.integral += error * delta_time;
        let integral = self.ki * self.integral;

        // Derivative term
        let derivative = if delta_time > 0.0 {
            (error - self.prev_error) / delta_time
        } else {
            0.0
        };
        let derivative = self.kd * derivative;

        // Update previous error for the next cycle
        self.prev_error = error;

        // Calculate the output
        let output = proportional + integral + derivative;
        self.prev_output = output;

        // Return the output
        output
    }

}