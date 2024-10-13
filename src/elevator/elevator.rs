use crate::elevator::pid_controller::PIDController;
use crate::elevator::elevator_motor::ElevatorMotor;
use std::time::Instant;

pub struct Elevator {
    floors: Vec<f32>, // floor heights, taken from elevator controller
    is_idle: bool,
    // pid-related
    current_height: f32,
    current_speed: f32,
    current_accel: f32,
    height_pid: PIDController,
    speed_pid: PIDController,
    // weigth and forces 
    max_speed: f32,
    max_accel: f32,
    elevator_mass: f32,
    elevator_counter_mass: f32,
    max_load: f32,
    current_load: f32,
    motor: ElevatorMotor,
    // simulation-related
    gravity: f32,
    last_update: Instant,
    time_multiplier: f32,
}


impl Elevator {
    pub fn new(
        floors: Vec<f32>,
        elevator_mass: f32,
        elevator_counter_mass: f32,
        max_speed: f32,
        max_accel: f32,
        max_load: f32,
        time_multiplier: f32,
    ) -> Self {
        let height_pid = PIDController::new(1., 0., 0., 10., 0.);
        let speed_pid = PIDController::new(1., 0., 0., 30., 0.);

        Self {
            floors,
            is_idle: true,
            current_height: 0.0,
            current_speed: 0.0,
            current_accel: 0.0,
            height_pid,
            speed_pid,
            max_speed,
            max_accel,
            elevator_mass,
            elevator_counter_mass,
            max_load,
            current_load: 0.0,
            motor: ElevatorMotor::new(1000., 0.8),
            gravity: 9.81,
            last_update: Instant::now(),
            time_multiplier,
        }
    }

    fn get_total_mass(&self) -> f32 {
        self.elevator_mass + self.current_load + self.elevator_counter_mass
    }

    fn get_delta_time(&mut self) -> f32 {
        // calculate delta time
        let now = Instant::now();
        let mut delta_time = now.duration_since(self.last_update).as_secs_f32();
        self.last_update = now;
        delta_time *= self.time_multiplier;
        
        delta_time
    }

    fn calculate_target_speed(&mut self, delta_time: f32) -> f32 {
        // calculate target speed
        let mut target_speed = self.height_pid.update(self.current_height, delta_time);
        if target_speed > self.max_speed {
            target_speed = self.max_speed;
        } else if target_speed < -self.max_speed {
            target_speed = -self.max_speed;
        }

        target_speed
    }

    fn calculate_target_accel(&mut self, target_speed: f32, delta_time: f32) -> f32 {
        // calculate target acceleration
        self.speed_pid.set_target(target_speed);
        let mut target_accel = self.speed_pid.update(self.current_speed, delta_time);
        // calculate the force required, this will be used on energy calculation
        if target_accel > self.max_accel {
            target_accel = self.max_accel;
        } else if target_accel < -self.max_accel {
            target_accel = -self.max_accel;
        }

        target_accel
    }

    fn calculate_motor_force(&self, target_accel: f32) -> f32 {
        let m = self.get_total_mass();
        let e = self.elevator_mass + self.current_load;

        // required force by the elevator motor
        let req_force = (e-self.elevator_counter_mass)*self.gravity - m*target_accel;
        req_force         
    }

    pub fn load(&mut self, weight: f32) {
        self.current_load += weight;
    }

    pub fn unload(&mut self, weight: f32) {
        self.current_load -= weight;
    }

    pub fn direction(&self) -> bool {
        self.height_pid.target > self.current_height
    }

    pub fn distance_to_floor(&self, floor_idx: usize) -> f32 {
        self.floors[floor_idx] - self.current_height
    }

    pub fn is_idle(&self) -> bool {
        self.is_idle
    }

    pub fn set_target(&mut self, floor_idx: usize) {
        self.height_pid.set_target(self.floors[floor_idx]);
        self.is_idle = false;
    }

    pub fn update(&mut self) -> f32 {
        let delta_time = self.get_delta_time();

        let force = self.calculate_motor_force(self.current_accel);
        let used_energy = self.motor.calculate_used_energy(force, self.current_speed, self.current_accel, delta_time);

        // calculate target speed
        let target_speed: f32 = self.calculate_target_speed(delta_time);
        let target_accel = self.calculate_target_accel(target_speed, delta_time);

        self.current_speed += target_accel * delta_time; 
        self.current_height += self.current_speed * delta_time;

        self.is_idle = self.height_pid.has_reached_target(self.current_height);

        used_energy
    }


}
