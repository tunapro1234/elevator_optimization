use super::boardable::Boardable;
use crate::machine::Elevator;

pub enum Gender {
    Male,
    Female,
}

pub struct Human {
    weight: f32,
    age: u8,
    area: f32,
    gender: Gender,
    destination_floor: usize,
    waiting_time: f32,
}

impl Boardable for Human {
    fn get_area(&self) -> f32 {
        self.area 
    }

    fn get_weight(&self) -> f32 {
        self.weight
    }

    fn calculate_boarding_time(&self, elevator: &Elevator) -> f32 {
        // Calculation logic based on elevator properties, e.g., door width
        // Placeholder value for boarding time
        2.0 
    }

    fn get_waiting_time(&self) -> f32 {
        self.waiting_time
    }

    fn increase_wait_time(&mut self, increment: f32) {
        self.waiting_time += increment;
    }

    fn get_destination(&self) -> usize {
        self.destination_floor
    }
}

impl Human {
    pub fn new(
        age: u8, 
        gender: Gender,
        destination_floor: usize,
    ) -> Self {
        let weight = Self::calc_weight(age, &gender);
        let area = Self::calc_area(weight);

        Self {
            weight,
            age,
            gender,
            area,
            destination_floor,
            waiting_time: 0.,
        }
    }

    fn calc_weight(age: u8, gender: &Gender) -> f32 {
        match &gender {
            Gender::Female => {
                age as f32
            }
            Gender::Male => {
                age as f32
            }
        }
    }
    
    fn calc_area(weight: f32, gender: &Gender) -> f32 {
        let (a, b) = match gender {
            Gender::Male => {
                let a = 0.6 + 0.004 * weight;
                let b = 0.3 + 0.003 * weight;
                (a, b)
            },
            Gender::Female => {
                let a = 0.5 + 0.004 * weight;
                let b = 0.3 + 0.003 * weight;
                (a, b)
            }
        };
    
        // Calculate elliptical area
        PI * a * b
    }
    
    fn generate_ellipse_points(weight: f32, gender: &Gender, num_points: usize) -> Vec<(f32, f32)> {
        let (a, b) = match gender {
            Gender::Male => {
                let a = 0.6 + 0.004 * weight;
                let b = 0.3 + 0.003 * weight;
                (a, b)
            },
            Gender::Female => {
                let a = 0.5 + 0.004 * weight;
                let b = 0.3 + 0.003 * weight;
                (a, b)
            }
        };
    
        // Generate ellipse points
        let mut points = Vec::new();
        for i in 0..num_points {
            let theta = 2.0 * PI * i as f32 / num_points as f32;
            let x = a * theta.cos();
            let y = b * theta.sin();
            points.push((x, y));
        }
        points
    }


    pub fn increment_waiting_time(&mut self, increment: f32) {
        self.waiting_time += increment;
    }
}


pub struct HumanGroup {
    members: Vec<Human>,
}

impl HumanGroup {
    pub fn new(members: Vec<Human>) -> Self {
        Self {
            members,
        }
    }
}

impl Boardable for HumanGroup {
    fn get_area(&self) -> f32 {
        // Sum up the area occupied by each human in the group
        self.members.iter().map(|h| h.area).sum()
    }

    fn get_weight(&self) -> f32 {
        // Sum up the weight of each human in the group
        self.members.iter().map(|h| h.weight).sum()
    }

    fn calculate_boarding_time(&self, elevator: &Elevator) -> f32 {
        // Calculate boarding time for the entire group
        // Placeholder value based on group size or other logic
        self.members.len() as f32 * 2.0 
    }

    fn get_waiting_time(&self) -> f32 {
        // Calculate average or total waiting time for the group if needed
        self.members.iter().map(|h| h.waiting_time).sum::<f32>() / self.members.len() as f32
    }

    fn increase_wait_time(&mut self, increment: f32) {
        // Increment the waiting time for each member of the group
        self.members.iter_mut().for_each(|h| h.waiting_time += increment); 
    }

    fn get_destination(&self) -> usize {
        self.members[0].destination_floor
    }
}
