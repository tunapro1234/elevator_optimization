use super::pid_controller::PIDController;
use super::motor::ElevatorMotor;
use std::time::Instant;

pub struct Elevator {
    pub floors: Vec<f32>, // floor heights, taken from elevator controller
    pub is_idle: bool,
    // pid-related
    pub current_height: f32,
    // pub current_speed: f32,
    pub current_accel: f32,
    pub height_pid: PIDController,
    // pub speed_pid: PIDController,
    // weigth and forces 
    pub max_speed: f32,
    pub max_accel: f32,
    pub elevator_mass: f32,
    pub elevator_counter_mass: f32,
    pub max_load: f32,
    pub current_load: f32,
    pub motor: ElevatorMotor,
    // simulation-related
    pub gravity: f32,
    pub last_update: Instant,
    pub time_multiplier: f32,
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
        let height_pid = PIDController::new(
            1., 
            0., 
            0., 
            10., 
            0.,
            false,
            0.,
            0.,
            0.,
            0.,
        );
        // let speed_pid = PIDController::new(1., 0., 0., 30., 0.);
        let motor = ElevatorMotor::new(
            "motor_samples.json", 
            1.
        ).unwrap();

        Self {
            floors,
            is_idle: true,
            current_height: 0.0,
            // current_speed: 0.0,
            current_accel: 0.0,
            height_pid,
            // speed_pid,
            max_speed,
            max_accel,
            elevator_mass,
            elevator_counter_mass,
            max_load,
            current_load: 0.0,
            motor,
            gravity: 9.81,
            last_update: Instant::now(),
            time_multiplier,
        }
    }

    fn get_used_energy(&self) -> f32{
        self.motor.get_total_energy_used()
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
        let target_speed = self.height_pid.update(self.current_height, delta_time);

        // limits are applied in the motor
        // if target_speed > self.max_speed {
        //     target_speed = self.max_speed;
        // } else if target_speed < -self.max_speed {
        //     target_speed = -self.max_speed;
        // }

        target_speed
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

    pub fn get_current_speed(&self) -> f32 {
        self.motor.get_current_speed()
    }

    pub fn update(&mut self) {
        // Delta time ve geçmiş döngüyle hesaplama yapan işler fonksiyonun başında
        // yeni hesaplamalar aşağıda
        
        // geçen zamanı al
        let delta_time = self.get_delta_time();

        // geçen zamana bağlı yüksekliği güncelle
        self.current_height += self.motor.get_current_speed() * delta_time;

        // geçen zamana bağlı motor değerlerini güncelle (harcanılan enerji gibi)
        self.motor.update(delta_time);

        // yeni hesaplamalar
        // calculate target speed
        let target_speed: f32 = self.calculate_target_speed(delta_time);

        // get required force to reach the target speed
        // let target_accel = (target_speed - self.current_accel) / delta_time;
        // let required_force = self.calculate_motor_force(target_accel)
        // do stuff with required force idk

        // motora yeni hedefi ver
        self.motor.set_target_speed(target_speed);
        self.is_idle = self.height_pid.has_reached_target(self.current_height);
    }


}
