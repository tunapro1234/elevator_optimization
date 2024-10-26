use std::error::Error;
use rand::distributions::weighted;
use serde::Deserialize;


#[derive(Debug, Deserialize)]
pub struct ElevatorSystemParameters {
    pub floors: Vec<f32>,
    pub gravity: f32,
    pub elevators: Vec<String>,
    pub time_multiplier: f32,
}


impl ElevatorSystemParameters {
    pub fn from_file(file_path: &str) -> Result<Self, Box<dyn Error>> {
        let file = std::fs::File::open(file_path)?;
        let result = serde_yaml::from_reader(file)?;
        Ok(result)
    }
}


