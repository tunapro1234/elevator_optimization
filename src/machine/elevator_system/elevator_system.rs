// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2024 Tuna Gül

use crate::machine::elevator::{ self, Elevator, ElevatorParameters };
use super::elevator_system_parameters::ElevatorSystemParameters;

use std::time::Instant;
use std::error::Error;


pub struct ElevatorSystem {
    floors: Vec<f32>,
    pub elevators: Vec<Elevator>,
    last_update: Instant,
    time_multiplier: f32,
    gravity: f32,
}


impl ElevatorSystem {
    pub fn from_file(
        file_path: &str,        
    ) -> Result<Self, Box<dyn Error>> {
        let parameters = ElevatorSystemParameters::from_file(file_path)?;
        Ok(Self::new(parameters))
    }

    pub fn new(
        parameters: ElevatorSystemParameters,
    ) -> Self {
        let mut elevators = Vec::new();
        for file in &parameters.elevators {
            elevators.push(
                Elevator::new(
                    parameters.floors.clone(),
                    parameters.gravity,
                    ElevatorParameters::from_file(file).unwrap()
                )
            );
        }

        Self {
            floors: parameters.floors,
            elevators,
            last_update: Instant::now(),
            time_multiplier: parameters.time_multiplier,
            gravity: parameters.gravity,
        }
    }

    pub fn get_used_energy(&self) -> f32 {
        let mut total = 0.0;
        for elevator in &self.elevators {
            total += elevator.get_used_energy();
        }
        total
    }

    fn get_delta_time(&mut self) -> f32 {
        // calculate delta time
        let now = Instant::now();
        let mut delta_time = now.duration_since(self.last_update).as_secs_f32();
        self.last_update = now;
        delta_time *= self.time_multiplier;
        
        delta_time
    }


    pub fn update(&mut self) {
        let delta_time = self.get_delta_time();
        for elevator in &mut self.elevators {
            elevator.update(delta_time);
        }
    }
}
