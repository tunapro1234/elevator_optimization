// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2024 Tuna Gül

// File for parsing the motor parameters like voltage or randomness or pid coefficients from a yaml file

use serde::Deserialize;
use std::error::Error;
use crate::machine::pid_controller::PIDParameters;

#[derive(Debug, Deserialize)]
pub struct MotorParameters {
    pub pid_parameters: PIDParameters,
    pub gearbox_ratio: f32,
    pub output_shaft_radius: f32,
    pub sample_path: String,
    pub soft_rpm_limit: f32,
    pub soft_current_limit: f32,
}

impl MotorParameters {
    pub fn from_file(file_path: &str) -> Result<Self, Box<dyn Error>> {
        let file = std::fs::File::open(file_path)?;
        let result = serde_yaml::from_reader(file)?;
        Ok(result)
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read() {
        let parameters = MotorParameters::from_file("parameters/motor_parameters.yaml").unwrap();
        println!("{:?}", parameters);
    }
}