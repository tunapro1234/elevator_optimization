mod elevator;
use elevator::elevator_system::ElevatorSystem;

use std::thread;
use std::time::{Duration, Instant};

fn main() {
    println!("Started");
    // i will keep the update frequency low for debugging purposes
    let update_freq = 1;
    let mut system = ElevatorSystem::new(1, vec![0.0, 100.0, 200.0, 300.0]);

    let start_time = Instant::now();
    let mut has_target_set = false;
    loop {
        system.update();

        println!("Energy consumed so far: {}", system.total_energy_consumed);
        println!("Elevator height: {}", system.elevators[0].current_height);
        println!("Elevator speed: {}, target: {}", system.elevators[0].current_speed, system.elevators[0].speed_pid.target);


        if start_time.elapsed() >= Duration::from_secs(2) && !has_target_set {
            system.elevators[0].set_target(3);
            println!("2 seconds passed. Target height set to: {}", system.elevators[0].height_pid.target);
            has_target_set = true;
        }

        thread::sleep(Duration::from_secs(1/update_freq));
    }
}
