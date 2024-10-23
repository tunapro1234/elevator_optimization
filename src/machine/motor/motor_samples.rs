// File for parsing the motor data from a CSV file

use serde::Deserialize;
use std::error::Error;
use std::fs::File;


#[derive(Debug, Deserialize, Copy, Clone)]
pub struct MotorSamples {
    pub _i: i32,
    pub kwp_in: f32,
    pub efficiency: f32,
    pub voltage: f32,
    pub current: f32,
    pub rpm: f32,
    pub tnm: f32,
    pub ih: f32,
    pub mo: f32,
}

impl MotorSamples {
    pub fn get(file_path: &str) -> Result<Vec<Self>, Box<dyn Error>> {
        // Open the CSV file
        let file = File::open(file_path)?;

        // Create a CSV reader
        let mut rdr = csv::Reader::from_reader(file);

        // Collect deserialized MotorProperties into a vector
        let mut motor_samples = Vec::new();

        for result in rdr.deserialize() {
            let record: MotorSamples = result?;
            motor_samples.push(record);
        }

        if motor_samples.len() == 0 {
            return Err("No motor samples found".into());
        }

        // Sort the data by current
        // motor_samples.sort_by(|a, b| a.current.partial_cmp(&b.current).unwrap());

        MotorSamples::check_properties(&motor_samples)?;

        Ok(motor_samples)
    }

    fn check_properties(samples: &Vec<Self>) -> Result<(), Box<dyn Error>> {
        // Check if the samples are sorted by current
        for i in 0..samples.len()-1 {
            if samples[i].current > samples[i+1].current {
                return Err("Motor samples should be sorted by current".into());
            }
        }

        // Check if the voltage and the tnm are the same
        for i in 0..samples.len()-1 {
            if samples[i].voltage != samples[i+1].voltage {
                // panic!("Voltage property of the motor should be the same. Check the data file"); 
                return Err("Voltage property of the motor should be the same. Check the data file".into());
            }
            if samples[i].tnm != samples[i+1].tnm {
                // panic!("TnM property of the motor should be the same. Check the data file"); 
                return Err("TnM property of the motor should be the same. Check the data file".into());
            }
        }

        Ok(())
    }

    pub fn get_max_rpm(samples: &Vec<Self>) -> f32 {
        let mut max_rpm = 0.0;
        for motor_property in samples {
            if motor_property.rpm > max_rpm {
                max_rpm = motor_property.rpm;
            }
        }
        max_rpm
    }

    pub fn get_max_tnm(samples: &Vec<Self>) -> f32 {
        let mut max_tnm = 0.0;
        for motor_property in samples {
            if motor_property.tnm > max_tnm {
                max_tnm = motor_property.tnm;
            }
        }
        max_tnm
    }

    pub fn get_max_current(samples: &Vec<Self>) -> f32 {
        let mut max_current = 0.0;
        for motor_property in samples {
            if motor_property.current > max_current {
                max_current = motor_property.current;
            }
        }
        max_current
    }

    pub fn find_smaller_closest(samples: &Vec<Self>, current: f32) -> usize {
        // verdiğimiz akımdaki datanın bir küçüğünün indexini verir

        let mut closest_current = 0.0;
        let mut closest_idx = 0;
        for (index, motor_property) in samples.iter().enumerate() {
            if index == samples.len()-1 {
                break;
            }

            if motor_property.current <= current {
                if current - motor_property.current < current - closest_current {
                    closest_current = motor_property.current;
                    closest_idx = index;
                }
            }
        }
        closest_idx
    }

    pub fn simulate_properties_from_current(samples: &Vec<Self>, current: f32) -> Option<MotorSamples> {
        if current > Self::get_max_current(samples) {
            return None;
        }

        let matching_idx = MotorSamples::find_smaller_closest(samples, current);
        let mc1 = samples[matching_idx].current;
        let mc2 = samples[matching_idx+1].current;

        // current 10 20 input 15 // rat 0.5
        // prop    40 45 // 60 // idx * (1+rat)

        let mult = (current - mc1) / (mc2 - mc1);

        let kwp_in = samples[matching_idx].kwp_in + (samples[matching_idx+1].kwp_in - samples[matching_idx].kwp_in) * mult;
        let voltage = samples[matching_idx].voltage + (samples[matching_idx+1].voltage - samples[matching_idx].voltage) * mult;
        let rpm = samples[matching_idx].rpm + (samples[matching_idx+1].rpm - samples[matching_idx].rpm) * mult;
        let tnm = samples[matching_idx].tnm + (samples[matching_idx+1].tnm - samples[matching_idx].tnm) * mult;
        let ih = samples[matching_idx].ih + (samples[matching_idx+1].ih - samples[matching_idx].ih) * mult;
        let mo = samples[matching_idx].mo + (samples[matching_idx+1].mo - samples[matching_idx].mo) * mult;

        // i was going to use this but then i realized that current could be 0
        // let efficiency = (rpm*2.*(PI as f32)/60. * tnm) / (voltage * current);

        // instead this is more appropriate
        let efficiency = samples[matching_idx].efficiency + (samples[matching_idx+1].efficiency - samples[matching_idx].efficiency) * mult;

        Some(
            Self {
                _i: 0,
                kwp_in,
                efficiency,
                voltage,
                current,
                rpm,
                tnm,
                ih,
                mo,
            }
        )
    }
}


#[cfg(test)]
mod tests {
    // Import the outer module's functions
    use super::*;

    // Write a test case
    #[test]
    fn read_motor_samples() {
        MotorSamples::get("data/motor_samples.csv").unwrap();
    }

    #[test]
    fn get_max_rpm() {
        let motor_samples = MotorSamples::get("data/motor_samples.csv").unwrap();
        let rpm = MotorSamples::get_max_rpm(&motor_samples) as i32;
        println!("Max RPM: {}", rpm);
        assert!(rpm == 299);
    }

    #[test]
    fn get_max_current() {
        let motor_samples = MotorSamples::get("data/motor_samples.csv").unwrap();
        let current = MotorSamples::get_max_current(&motor_samples) as i32;
        println!("Max Current: {}", current);
        assert!(current == 61);
    }

    #[test]
    fn get_max_tnm() {
        let motor_samples = MotorSamples::get("data/motor_samples.csv").unwrap();
        let tnm = MotorSamples::get_max_tnm(&motor_samples) as i32;
        println!("Max torque: {}", tnm);
        assert!(tnm == 785);
    }

    #[test]
    fn max_current_closest() {
        let motor_samples = MotorSamples::get("data/motor_samples.csv").unwrap();
        let current = MotorSamples::get_max_current(&motor_samples);
        let idx = MotorSamples::find_smaller_closest(&motor_samples, current);
        assert!(idx == motor_samples.len()-2);
    }

    #[test]
    fn min_current_closest() {
        let motor_samples = MotorSamples::get("data/motor_samples.csv").unwrap();
        let current = 0.;
        let idx = MotorSamples::find_smaller_closest(&motor_samples, current);
        assert!(idx == 0);
    }

    #[test]
    fn current_closest() {
        let motor_samples = MotorSamples::get("data/motor_samples.csv").unwrap();
        let current = 18.5;
        let idx = MotorSamples::find_smaller_closest(&motor_samples, current);
        println!("Closest index: {}", idx);
        assert!(idx == 5);
    }

    #[test]
    fn simulate_properties_rpm() {
        let motor_samples = MotorSamples::get("data/motor_samples.csv").unwrap();
        // currents -> 60.29266331658292, 61.51879396984925
        // rpm -> 293.6120603015075, 299.5417085427135
        // tnm -> 785, 785
        let current = 61.;
        let new_property = MotorSamples::simulate_properties_from_current(&motor_samples, current).unwrap();
        assert!(297. < new_property.rpm && new_property.rpm < 298.);
    }

    #[test]
    fn simulate_properties_tnm() {
        let motor_samples = MotorSamples::get("data/motor_samples.csv").unwrap();
        // currents -> 60.29266331658292, 61.51879396984925
        // rpm -> 293.6120603015075, 299.5417085427135
        // tnm -> 785, 785
        let current = 61.;
        let new_property = MotorSamples::simulate_properties_from_current(&motor_samples, current).unwrap();
        assert!(new_property.tnm == 785.);
        assert!(88.94 < new_property.efficiency && new_property.efficiency < 88.96);
    }

    #[test]
    fn simulate_properties_efficiency() {
        let motor_samples = MotorSamples::get("data/motor_samples.csv").unwrap();
        // currents -> 60.29266331658292, 61.51879396984925
        // rpm -> 293.6120603015075, 299.5417085427135
        // tnm -> 785, 785
        let current = 61.;
        let new_property = MotorSamples::simulate_properties_from_current(&motor_samples, current).unwrap();
        assert!(88.94 < new_property.efficiency && new_property.efficiency < 88.96);
    }
}