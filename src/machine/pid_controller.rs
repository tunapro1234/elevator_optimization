use serde::Deserialize;


// this struct is for parsing the pid parameters from a yaml file
#[derive(Debug, Deserialize)]
pub struct PIDParameters {
    kp: f32,
    ki: f32,
    kd: f32,
    integral_limit: f32,
    update_freq: f32,
    tolerance: f32,

    #[serde(default = "default_enable_target_limits")]
    enable_target_limits: bool,
    #[serde(default = "default_max_target")]
    max_target: f32,
    #[serde(default = "default_min_target")]
    min_target: f32,
    #[serde(default = "default_enable_output_limits")]
    enable_output_limits: bool,
    #[serde(default = "default_max_output")]
    max_output: f32,
    #[serde(default = "default_min_output")]
    min_output: f32,
    #[serde(default = "default_change_limit")]
    change_limit: f32,
}

fn default_enable_target_limits() -> bool { false }
fn default_min_target() -> f32 { 0. }
fn default_max_target() -> f32 { 0. }
fn default_enable_output_limits() -> bool { false }
fn default_min_output() -> f32 { 0. }
fn default_max_output() -> f32 { 0. }
fn default_change_limit() -> f32 { 0. }


// this is the real thing
pub struct PIDController {
    pub target: f32,
    kp: f32,
    ki: f32,
    kd: f32,
    integral_limit: f32,
    enable_target_limits: bool,
    max_target: f32,
    min_target: f32,
    enable_output_limits: bool,
    max_output: f32,
    min_output: f32,
    change_limit: f32,
    prev_error: f32,
    integral: f32,
    prev_output: f32,
    accumulated_time: f32, // to keep at constant frequency
    update_freq: f32,
    tolerance: f32,
}

impl PIDController {
    pub fn from_parameters(
        parameters: PIDParameters,
    ) -> Self {
        Self::new(
            parameters.kp, 
            parameters.ki, 
            parameters.kd, 
            parameters.integral_limit,
            parameters.update_freq,
            parameters.tolerance,
            parameters.enable_target_limits,
            parameters.max_target,
            parameters.min_target,
            parameters.enable_output_limits,
            parameters.max_output,
            parameters.min_output,
            parameters.change_limit,
        )
    }

    pub fn new(
        kp: f32, 
        ki: f32, 
        kd: f32, 
        integral_limit: f32,
        update_freq: f32,
        tolerance: f32,
        enable_target_limits: bool,
        max_target: f32,
        min_target: f32,
        enable_output_limits: bool,
        max_output: f32,
        min_output: f32,
        change_limit: f32,
    ) -> Self {
        Self {
            target: 0.0,
            kp,
            ki,
            kd,
            integral_limit,
            enable_target_limits,
            max_target,
            min_target,
            enable_output_limits,
            max_output,
            min_output,
            change_limit,
            prev_error: 0.0,
            integral: 0.0,
            prev_output: 0.0,
            accumulated_time: 0.0,
            update_freq,
            tolerance,
        }
    }

    pub fn set_output_limits(&mut self, min_output: f32, max_output: f32) {
        self.enable_output_limits = true;
        self.min_output = min_output;
        self.max_output = max_output;
    }

    pub fn disable_output_limits(&mut self) {
        self.enable_output_limits = false;
    }

    pub fn set_target_limits(&mut self, min_target: f32, max_target: f32) {
        self.enable_output_limits = true;
        self.min_target = min_target ;
        self.max_target = max_target ;
    }

    pub fn disable_target_limits(&mut self) {
        self.enable_target_limits = false;
    }

    pub fn set_change_limit(&mut self, change_limit: f32) {
        self.change_limit = change_limit;
    }

    pub fn disable_change_limit(&mut self) {
        self.change_limit = 0.;
    }

    pub fn set_parameters(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    pub fn set_integral_limit(&mut self, integral_limit: f32) {
        self.integral_limit = integral_limit;
    }

    pub fn disable_integral_limit(&mut self, integral_limit: f32) {
        self.integral_limit = integral_limit;
    }

    pub fn set_target(&mut self, target: f32) -> bool{
        if self.enable_target_limits {
            if target < self.min_target {
                self.target = self.min_target;
                return false;
            } else if target > self.max_target {
                self.target = self.max_target;
                return false;
            } 
        }

        self.target = target;
        return true;
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

        // Integral term with clamping
        self.integral += error * delta_time;

        // Apply the integral limit
        if self.integral_limit != 0. {
            if self.integral > self.integral_limit {
                self.integral = self.integral_limit;
            } else if self.integral < -self.integral_limit {
                self.integral = -self.integral_limit;
            }
        }

        // Calculate the integral contribution to the output
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
        let mut output = proportional + integral + derivative;

        // Check if the output is within limits
        if self.enable_output_limits {
            if output < self.min_output {
                output = self.min_output;
            } else if output > self.max_output {
                output = self.max_output;
            }
        } 

        if self.change_limit != 0. {
            if (output - current_value).abs()/delta_time > self.change_limit {
                if output > current_value {
                    output = current_value + self.change_limit * delta_time;
                } else {
                    output = current_value - self.change_limit * delta_time;
                }
            }
        }

        // Return the output
        self.prev_output = output;
        output
    }

}