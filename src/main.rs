// elevator_optimization
// Copyright (C) 2024   Tuna Gül

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


mod machine;
mod control_algorithms;
mod population;

use machine::elevator_system::ElevatorSystem;

use std::thread;
use std::time::{Duration, Instant};

fn main() {
    println!("    
        elevator_optimization  Copyright (C) 2024  Tuna Gül
        This program comes with ABSOLUTELY NO WARRANTY;
        This is free software, and you are welcome to redistribute it
        under certain conditions;
    ");
    // i will keep the update frequency low for debugging purposes
    let update_freq = 1;
    let mut system = ElevatorSystem::new(1, vec![0.0, 100.0, 200.0, 300.0]);

    let start_time = Instant::now();
    let mut has_target_set = false;
    loop {
        system.update();

        println!("Energy consumed so far: {}", system.total_energy_consumed);
        println!("Elevator height: {}", system.elevators[0].current_height);
        // println!("Elevator speed: {}, target: {}", system.elevators[0].get_current_speed(), system.elevators[0].motor.speed_pid.target);


        if start_time.elapsed() >= Duration::from_secs(2) && !has_target_set {
            system.elevators[0].set_target(3);
            println!("2 seconds passed. Target height set to: {}", system.elevators[0].height_pid.target);
            has_target_set = true;
        }

        thread::sleep(Duration::from_secs(1/update_freq));
    }
}
