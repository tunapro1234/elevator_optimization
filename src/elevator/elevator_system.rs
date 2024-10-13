use std::time::{SystemTime, UNIX_EPOCH};
use crate::elevator::elevator::Elevator;

use super::elevator;

extern crate rand;
// use rand::Rng;

struct ElevatorSystem {
    floors: Vec<f32>,
    pub elevators: Vec<Elevator>,
    pub total_energy_consumed: f32,
}

impl ElevatorSystem {
    fn new(num_elevators: usize, floors: Vec<f32>) -> Self {
        let mut elevators = Vec::new();
        for _ in 0..num_elevators {
            elevators.push(Elevator::new(
                floors.clone(), 
                500.0, 
                300.0, 
                10.0, 
                5.0, 
                1000.0, 
                1.0,
            ));
        }

        Self {
            floors,
            elevators,
            total_energy_consumed: 0,
        }
    }

    fn new_call(&mut self, current_floor_idx: usize, target_floor_idx: bool) {
        let mut best_elevator_index = None;
        let mut min_distance = f32::MAX;

        for (i, elevator) in self.elevators.iter().enumerate() {
            if elevator.is_idle() {
                let distance = elevator.distance_to_floor(current_floor_idx);
                if distance < min_distance {
                    min_distance = distance;
                    best_elevator_index = Some(i);
                }
            }
        }

        if let Some(index) = best_elevator_index {
            let elevator = &mut self.elevators[index];
            elevator.set_target(target_floor_idx);
            elevator.load(70.0);
        }
    }

    fn update(&mut self) {
        for elevator in &mut self.elevators {
            let consumed_energy = elevator.update();
            self.total_energy_consumed += consumed_energy;
        }
    }

    fn run_simulation(&mut self) {

        if (start % 100) < (visitor_frequency * 100.0) as u64 {
            let from = rand::random::<i32>() % self.floors.len() as i32;
            let to = rand::random::<i32>() % self.floors.len() as i32;
            if from != to {
                self.new_call(from, to);
            }
        }
        self.update_elevators();

    }

    fn print_statistics(&self) {
        println!("Simulation Results:");
        println!("Total Energy Consumed: {} units", self.total_energy_consumed);
        println!(
            "Average Waiting Time: {} seconds",
            if self.num_visitors > 0 {
                self.total_waiting_time / self.num_visitors
            } else {
                0
            }
        );
    }
}
