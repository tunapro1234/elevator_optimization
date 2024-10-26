use std::time::Instant;
use plotters::prelude::*;
use std::error::Error;


pub struct LinePlotter {
    root: DrawingArea<BitMapBackend<'static>, plotters::coord::Shift>,
    start_time: Instant,
    points: Vec<(f32, f32)>,
    output_file: String,
} 

impl LinePlotter {
    pub fn new(output_file: String) -> Result<Self, Box<dyn Error>> {
        let output_file_static = Box::leak(Box::new(output_file.clone())); // Leak the string to get a 'static reference
        let backend = BitMapBackend::new(output_file_static, (1920, 1080));
        let root = backend.into_drawing_area();
        root.fill(&WHITE)?;

        Ok(
            Self {
                root,
                points: Vec::new(),
                start_time: Instant::now(),
                output_file,
            }
        )
    }

    pub fn add_point(&mut self, value: f32) {
        let elapsed = self.start_time.elapsed().as_secs_f32();
        self.points.push((elapsed, value));
        
        // Keep only the most recent 1000 points for a smooth plot
        if self.points.len() > 1000 {
            self.points.remove(0);
        }
    }

    pub fn update(&self) -> Result<(), Box<dyn Error>> {
        // Set up the output file
        self.root.fill(&WHITE)?;

        // Define x and y axis ranges based on data
        let x_min = self.points.iter().map(|(x, _)| *x).fold(f32::INFINITY, f32::min);
        let x_max = self.points.iter().map(|(x, _)| *x).fold(f32::NEG_INFINITY, f32::max);
        let y_min = self.points.iter().map(|(_, y)| *y).fold(f32::INFINITY, f32::min);
        let y_max = self.points.iter().map(|(_, y)| *y).fold(f32::NEG_INFINITY, f32::max);

        let mut chart = ChartBuilder::on(&self.root)
            .caption("Line Plotter", ("sans-serif", 40))
            .margin(10)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(x_min..x_max, y_min..y_max)?;

        // Configure the chart grid
        chart.configure_mesh().draw()?;

        // Plot the points as a line
        chart.draw_series(LineSeries::new(
            self.points.iter().map(|&(x, y)| (x, y)),
            &BLUE,
        ))?;

        Ok(())
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn test_initialization() {
        let output_file = "data/debug/line_plotter_test.png".to_string();
        let plotter = LinePlotter::new(output_file);
        assert!(plotter.is_ok(), "LinePlotter failed to initialize.");
    }

    #[test]
    fn test_add_point() {
        let output_file = "data/debug/line_plotter_test.png".to_string();
        let mut plotter = LinePlotter::new(output_file).expect("Failed to initialize LinePlotter");

        plotter.add_point(1.0);
        plotter.add_point(2.0);
    }

    #[test]
    fn test_update() {
        let output_file = "data/debug/line_plotter_test.png";
        let mut plotter = LinePlotter::new(output_file.to_string()).expect("Failed to initialize LinePlotter");

        // Add some points and update the plot
        plotter.add_point(1.0);
        plotter.add_point(2.0);
        plotter.add_point(3.0);

        // Ensure update runs without error
        let result = plotter.update();
        assert!(result.is_ok(), "Plot update failed.");

        // Verify that the output file is created
        assert!(fs::metadata(output_file).is_ok(), "Output file was not created.");

        // Clean up
        // let _ = fs::remove_file(output_file);
    }
}
