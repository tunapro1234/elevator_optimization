use egui::*;
use super::elevator::Elevator;  // Import the Elevator struct from elevator.rs
use super::elevator_system::ElevatorSystem;  // Import the Elevator struct from elevator.rs

fn ui_example(ctx: &CtxRef, system: &mut ElevatorSystem) {
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.label("Elevator System");

        if let Some(elevator) = system.elevators.get(0) {
            draw_elevator(elevator, ui, 50.0, 400.0);
        }
    });
}

pub fn draw_elevator(elevator: &Elevator, ui: &mut Ui, box_width: f32, box_height: f32) {
    // Draw the shaft (building)
    let (rect, _) = ui.allocate_exact_size(
        Vec2::new(box_width, box_height),
        Sense::hover(),
    );
    ui.painter().rect_stroke(rect, 0.0, Stroke::new(2.0, Color32::BLACK));

    // Calculate elevator position inside the shaft based on height
    let elevator_max_height = elevator.floors[elevator.floors.len()-1];
    let elevator_height = (box_height - (elevator.current_height / elevator_max_height * box_height)).clamp(0.0, box_height);

    // Draw the elevator as a rectangle moving up/down
    let elevator_rect = Rect::from_min_size(
        rect.min + vec2(0.0, elevator_height),
        Vec2::new(box_width, 40.0),  // Size of the elevator box
    );
    ui.painter().rect_filled(elevator_rect, 0.0, Color32::RED);
}
