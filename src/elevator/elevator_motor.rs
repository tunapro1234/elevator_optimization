use serde::Deserialize;
use std::error::Error;
use std::fs::File;
use std::path::Path;
use super::pid_controller::PIDController;
use std::f64::consts::PI;

#[derive(Debug, Deserialize, Copy, Clone)]
struct MotorProperties {
    i: i32,
    kwp_in: f32,
    efficiency: f32,
    voltage: f32,
    current: f32,
    rpm: f32,
    tnm: f32,
    ih: f32,
    mo: f32,
}

impl MotorProperties {
    fn get(file_path: &str) -> Result<Vec<Self>, Box<dyn Error>> {
        // Open the CSV file
        let file = File::open(file_path)?;

        // Create a CSV reader
        let mut rdr = csv::Reader::from_reader(file);

        // Collect deserialized MotorProperties into a vector
        let mut motor_properties = Vec::new();

        for result in rdr.deserialize() {
            let record: MotorProperties = result?;
            motor_properties.push(record);
        }

        if motor_properties.len() == 0 {
            return Err("No motor properties found".into());
        }

        Ok(motor_properties)
    }

    fn get_max_rpm(properties: &Vec<Self>) -> f32 {
        let mut max_rpm = 0.0;
        for motor_property in properties {
            if motor_property.rpm > max_rpm {
                max_rpm = motor_property.rpm;
            }
        }
        max_rpm
    }

    fn get_max_tnm(properties: &Vec<Self>) -> f32 {
        let mut max_tnm = 0.0;
        for motor_property in properties {
            if motor_property.tnm > max_tnm {
                max_tnm = motor_property.tnm;
            }
        }
        max_tnm
    }

    fn get_max_current(properties: &Vec<Self>) -> f32 {
        let mut max_current = 0.0;
        for motor_property in properties {
            if motor_property.current > max_current {
                max_current = motor_property.current;
            }
        }
        max_current
    }

    fn find_smaller_closest(properties: &Vec<Self>, current: f32) -> usize {
        // verdiğimiz akımdaki datanın bir küçüğünün indexini verir

        let mut closest_current = 0.0;
        let mut closest_idx = 0;
        for (index, motor_property) in properties.iter().enumerate() {
            if index == properties.len()-1 {
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

    fn simulate_properties_from_current(properties: &Vec<Self>, current: f32) -> MotorProperties {
        let matching_idx = MotorProperties::find_smaller_closest(properties, current);
        let mc1 = properties[matching_idx].current;
        let mc2 = properties[matching_idx+1].current;

        // current 10 20 input 15 // rat 0.5
        // prop    40 45 // 60 // idx * (1+rat)

        let mult = (current - mc1) / (mc2 - mc1);

        let kwp_in = properties[matching_idx].kwp_in + (properties[matching_idx+1].kwp_in - properties[matching_idx].kwp_in) * mult;
        let voltage = properties[matching_idx].voltage + (properties[matching_idx+1].voltage - properties[matching_idx].voltage) * mult;
        let rpm = properties[matching_idx].rpm + (properties[matching_idx+1].rpm - properties[matching_idx].rpm) * mult;
        let tnm = properties[matching_idx].tnm + (properties[matching_idx+1].tnm - properties[matching_idx].tnm) * mult;
        let ih = properties[matching_idx].ih + (properties[matching_idx+1].ih - properties[matching_idx].ih) * mult;
        let mo = properties[matching_idx].mo + (properties[matching_idx+1].mo - properties[matching_idx].mo) * mult;

        // i was going to use this but then i realized that current could be 0
        // let efficiency = (rpm*2.*(PI as f32)/60. * tnm) / (voltage * current);

        // instead this is more appropriate
        let efficiency = properties[matching_idx].efficiency + (properties[matching_idx+1].efficiency - properties[matching_idx].efficiency) * mult;

        Self {
            i: 0,
            kwp_in,
            efficiency,
            voltage,
            current,
            rpm,
            tnm,
            ih,
            mo,
        }
    }
}


pub struct ElevatorMotor {
    motor_properties: Vec<MotorProperties>,
    current_properties: MotorProperties,
    gear_ratio: f32,
    current_speed: f32,
    pub speed_pid: PIDController,
    pub total_energy_used: f32,
    // pub max_rpm: f32,
    pub max_tnm: f32,
}

impl ElevatorMotor {
    pub fn new(
        properties_path: &str,
        gear_ratio: f32,
    ) -> Result<Self, Box<dyn Error>> {
        let motor_properties = MotorProperties::get(properties_path)?;
        let max_rpm = MotorProperties::get_max_rpm(&motor_properties);
        let max_tnm = MotorProperties::get_max_tnm(&motor_properties);
        let max_current = MotorProperties::get_max_current(&motor_properties);

        let speed_pid = PIDController::new(
            1., 
            0., 
            0., 
            30.,
            0.,
            true,
            max_rpm,
            -max_rpm,
            max_current,
            -max_current
        );

        let current_properties = MotorProperties::simulate_properties_from_current(&motor_properties, 0.);

        Ok( 
            Self {
                motor_properties,
                gear_ratio,
                speed_pid,
                current_properties,
                current_speed: 0.0,
                total_energy_used: 0.0,
                // max_rpm,
                max_tnm,
            }
        )
    }

    pub fn get_current_speed(&self) -> f32 {
        // this function gives the speed of the output shaft of the gear box
        self.current_speed / self.gear_ratio
    }

    pub fn set_target_speed(&mut self, target: f32) -> bool {
        // this function sets the speed of the output shaft of the gear box
        let motor_target = target*self.gear_ratio;
        
        // the limit is applied in pid controller
        // if motor_target > self.max_rpm {
        //     self.speed_pid.set_target(self.max_rpm);
        //     false
        // } 
        // else {
        //     self.speed_pid.set_target(motor_target);
        //     true
        // }

        self.speed_pid.set_target(motor_target)
    }

    pub fn get_total_energy_used(&self) -> f32 {
        self.total_energy_used
    }

    pub fn has_reached_target(&self) -> bool {
        self.speed_pid.has_reached_target(self.current_speed)
    }

    pub fn give_current(&mut self, current: f32) {
        self.current_properties = MotorProperties::simulate_properties_from_current(&self.motor_properties, current);
        self.current_speed = self.current_properties.rpm;
    }

    pub fn update(&mut self, delta_time: f32) {
        self.total_energy_used += self.current_properties.kwp_in * delta_time;

        let new_current = self.speed_pid.update(self.current_speed, delta_time);
        self.give_current(new_current);
    }
}



#[cfg(test)]
mod tests {
    // Import the outer module's functions
    use super::*;

    // Write a test case
    #[test]
    fn read_motor_data() {
        MotorProperties::get("data/motor_data.csv").unwrap();
    }

    #[test]
    fn get_max_rpm() {
        let motor_properties = MotorProperties::get("data/motor_data.csv").unwrap();
        let rpm = MotorProperties::get_max_rpm(&motor_properties) as i32;
        println!("Max RPM: {}", rpm);
        assert!(rpm == 299);
    }

    #[test]
    fn get_max_current() {
        let motor_properties = MotorProperties::get("data/motor_data.csv").unwrap();
        let current = MotorProperties::get_max_current(&motor_properties) as i32;
        println!("Max Current: {}", current);
        assert!(current == 61);
    }

    #[test]
    fn get_max_tnm() {
        let motor_properties = MotorProperties::get("data/motor_data.csv").unwrap();
        let tnm = MotorProperties::get_max_tnm(&motor_properties) as i32;
        println!("Max torque: {}", tnm);
        assert!(tnm == 785);
    }

    #[test]
    fn max_current_closest() {
        let motor_properties = MotorProperties::get("data/motor_data.csv").unwrap();
        let current = MotorProperties::get_max_current(&motor_properties);
        let idx = MotorProperties::find_smaller_closest(&motor_properties, current);
        assert!(idx == motor_properties.len()-2);
    }

    #[test]
    fn min_current_closest() {
        let motor_properties = MotorProperties::get("data/motor_data.csv").unwrap();
        let current = 0.;
        let idx = MotorProperties::find_smaller_closest(&motor_properties, current);
        assert!(idx == 0);
    }

    #[test]
    fn current_closest() {
        let motor_properties = MotorProperties::get("data/motor_data.csv").unwrap();
        let current = 18.5;
        let idx = MotorProperties::find_smaller_closest(&motor_properties, current);
        println!("Closest index: {}", idx);
        assert!(idx == 5);
    }

    #[test]
    fn simulate_properties_rpm() {
        let motor_properties = MotorProperties::get("data/motor_data.csv").unwrap();
        // currents -> 60.29266331658292, 61.51879396984925
        // rpm -> 293.6120603015075, 299.5417085427135
        // tnm -> 785, 785
        let current = 61.;
        let new_property = MotorProperties::simulate_properties_from_current(&motor_properties, current);
        assert!(297. < new_property.rpm && new_property.rpm < 298.);
    }

    #[test]
    fn simulate_properties_tnm() {
        let motor_properties = MotorProperties::get("data/motor_data.csv").unwrap();
        // currents -> 60.29266331658292, 61.51879396984925
        // rpm -> 293.6120603015075, 299.5417085427135
        // tnm -> 785, 785
        let current = 61.;
        let new_property = MotorProperties::simulate_properties_from_current(&motor_properties, current);
        assert!(new_property.tnm == 785.);
        assert!(88.94 < new_property.efficiency && new_property.efficiency < 88.96);
    }

    #[test]
    fn simulate_properties_efficiency() {
        let motor_properties = MotorProperties::get("data/motor_data.csv").unwrap();
        // currents -> 60.29266331658292, 61.51879396984925
        // rpm -> 293.6120603015075, 299.5417085427135
        // tnm -> 785, 785
        let current = 61.;
        let new_property = MotorProperties::simulate_properties_from_current(&motor_properties, current);
        assert!(88.94 < new_property.efficiency && new_property.efficiency < 88.96);
    }


}
