// File for parsing the motor parameters like voltage or randomness or pid coefficients from a yaml file
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct MotorParameters {
}
