// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2024 Tuna Gül

use super::elevator::Elevator;

extern crate rand;
// use rand::Rng;

pub struct ElevatorSystem {
    floors: Vec<f32>,
    pub elevators: Vec<Elevator>,
    pub total_energy_consumed: f32,
}

impl ElevatorSystem {
    pub fn new(num_elevators: usize, floors: Vec<f32>) -> Self {
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
            total_energy_consumed: 0.0,
        }
    }

    pub fn update(&mut self) {
        for elevator in &mut self.elevators {
            elevator.update();
        }
    }
}
