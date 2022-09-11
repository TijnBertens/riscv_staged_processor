use eframe::{egui, emath};
use eframe::egui::{Painter, Shape, Pos2, Stroke, Color32, Vec2, Rect, Sense, Frame, Align2, FontId, Context};
use eframe::egui::epaint::PathShape;

pub struct ProcessorGUI {
    pc: usize,
    pos: egui::Pos2,
}

impl ProcessorGUI {
    pub fn new() -> Self {
        Self {
            pos: (50.0 , 50.0).into(),
            pc: 0
        }
    }

    fn build_table(&mut self, ui: &mut egui::Ui) {
        use egui_extras::{Size, TableBuilder};

        let program_string = "MVI x1, 50;\
        MVI x2, 33;\
        ADDI x2 3;\
        ADD x3 x2 x1;\
        BNE x2 -2;\
        STORE x2 x1 2;\
        LOAD x3 x1 2;";

        let instructions: Vec<&str> = program_string.split(';').into_iter()
            .map(|s| s.trim()).collect();

        ui.add(egui::Slider::new(&mut self.pc, 0..=(instructions.len() - 1)));

        let mut table = TableBuilder::new(ui)
            .striped(true)
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .column(Size::initial(20.0).at_least(20.0))
            .column(Size::initial(100.0).at_least(100.0))
            .column(Size::initial(20.0).at_least(20.0))
            .resizable(true);

        table
            .header(20.0, |mut header| {
                header.col(|ui| {
                    ui.heading("@");
                });
                header.col(|ui| {
                    ui.heading("instruction");
                });
                header.col(|ui| {
                    ui.heading("PC");
                });
            })
            .body(|mut body| {
                body.rows(15.0, instructions.len(), |row_idx, mut row| {
                    let address = row_idx * 4;
                    row.col(|ui| {
                       ui.monospace(address.to_string());
                    });
                    row.col(|ui| {
                        ui.monospace(instructions[row_idx]);
                    });
                    row.col(|ui| {
                        ui.monospace(
                            if address == (self.pc*4) { "←" } else { "" }
                        );
                    });
                });
            });
    }

    fn draw_mux(&self, painter: &mut Painter, center: Pos2, ui: &mut egui::Ui) {

        let width = 30.0;
        let half_width = width / 2.0;

        let in_height = 60.0;
        let in_half_height = in_height / 2.0;

        let out_height = 40.0;
        let out_half_height = out_height / 2.0;

        let top_left = Pos2::new(-half_width, in_half_height) + center.to_vec2();
        let top_right = Pos2::new(half_width, out_half_height) + center.to_vec2();
        let bot_right = Pos2::new(half_width, -out_half_height) + center.to_vec2();
        let bot_left = Pos2::new(-half_width, -in_half_height) + center.to_vec2();

        let shape = egui::Shape::Path(
            PathShape::closed_line(
                vec![top_left, top_right, bot_right, bot_left],
                Stroke::new(3.0, Color32::WHITE)
            )
        );

        painter.add(shape);
    }

    fn draw_thing(&mut self, ui: &mut egui::Ui) {
        Frame::canvas(ui.style()).show(ui, |ui| {
            let (response, mut painter) =
                ui.allocate_painter(Vec2::new(ui.available_width(), ui.available_height()), Sense::hover());

            let to_screen = emath::RectTransform::from_to(
                Rect::from_min_size(Pos2::ZERO, response.rect.size()),
                response.rect,
            );

            let center = to_screen.transform_pos((100.0, 100.0).into());
            self.draw_mux(&mut painter, center, ui);

            let center = to_screen.transform_pos((200.0, 300.0).into());
            self.draw_mux(&mut painter, center, ui);

            // let top_left = Pos2::new(5.0, 5.0) + self.pos.to_vec2();
            // let top_right = Pos2::new(65.0, 5.0) + self.pos.to_vec2();
            // let bot_left = Pos2::new(15.0, 35.0) + self.pos.to_vec2();
            // let bot_right = Pos2::new(55.0, 35.0) + self.pos.to_vec2();
            //
            // let top_left = to_screen.transform_pos(top_left);
            // let top_right = to_screen.transform_pos(top_right);
            // let bot_left = to_screen.transform_pos(bot_left);
            // let bot_right = to_screen.transform_pos(bot_right);
            //
            // let point_in_screen = to_screen.transform_pos(self.pos);
            // let point_rect = Rect::from_center_size(point_in_screen, Vec2::new(20.0, 20.0));
            // let point_response = ui.interact(point_rect, response.id, Sense::drag());
            //
            // self.pos += point_response.drag_delta();
            //
            // let shape = egui::Shape::Path(
            //     PathShape::closed_line(
            //         vec![top_left, top_right, bot_right, bot_left],
            //         ui.style().interact(&point_response).fg_stroke
            //     )
            // );
            //
            // painter.add(shape);

            response
        });
    }
}

impl eframe::App for ProcessorGUI {
    fn update(&mut self, ctx: &Context, frame: &mut eframe::Frame) {
        egui::SidePanel::left("Program Section").show(ctx, |ui| {
            self.build_table(ui);
        });

        egui::CentralPanel::default().show(&ctx, |ui| {
            self.draw_thing(ui);
        });
    }
}

pub fn run_gui() {
    let options = eframe::NativeOptions {
        drag_and_drop_support: true,
        initial_window_size: Some((1280.0, 720.0).into()),
        ..eframe::NativeOptions::default()
    };
    eframe::run_native(
        "5-Stage RISC-V CPU",
        options,
        Box::new(|_cc| Box::new(ProcessorGUI::new()))
    );
}

mod component_graphics {
    use eframe::egui;
    use crate::components::{ConstantRegister, Register};
    use eframe::egui::{Pos2, Painter, Vec2, Color32, FontId, Align2};
    use eframe::epaint::{RectShape, PathShape};

    pub const COL_DARK_BLUE: Color32 = Color32::from_rgb(38, 70, 83);
    pub const COL_LIGHT_BLUE: Color32 = Color32::from_rgb(42, 157, 143);
    pub const COL_SAND: Color32 = Color32::from_rgb(233, 196, 106);
    pub const COL_LIGHT_ORANGE: Color32 = Color32::from_rgb(244, 162, 97);
    pub const COL_DARK_ORANGE: Color32 = Color32::from_rgb(231, 111, 81);

    pub fn draw_constant_register(painter: &mut Painter, reg: &ConstantRegister, center: Pos2) {
        let width = 60.0;
        let height = 45.0;

        let rect = egui::Rect::from_center_size(center, Vec2::new(width, height));

        let rect_shape = egui::Shape::Rect(RectShape::filled(
            rect,
            egui::Rounding::same(5.0),
            COL_SAND
        ));

        painter.add(rect_shape);

        let pos_output_port = center + Vec2::new(width / 2.0, 0.0);

        let rect = egui::Rect::from_center_size(pos_output_port, Vec2::new(8.0, 8.0));

        let rect_shape = egui::Shape::Rect(RectShape::filled(
            rect,
            egui::Rounding::same(1.0),
            COL_DARK_ORANGE
        ));

        painter.add(rect_shape);

        let pos_name = center - Vec2::new(0.0, 8.0);
        let pos_val = center + Vec2::new(0.0, 8.0);

        painter.text(
            pos_name,
            Align2::CENTER_CENTER,
            &reg.name,
            FontId::monospace(13.0),
            Color32::BLACK
        );

        let value = reg.port_collection.borrow().get_port_data(reg.output_port);
        painter.text(
            pos_val,
            Align2::CENTER_CENTER,
            value.to_string(),
            FontId::monospace(13.0),
            Color32::BLACK
        );
    }

    pub fn draw_register(painter: &mut Painter, reg: &Register, center: Pos2) {
        let width = 40.0;
        let height = 30.0;

        let rect = egui::Shape::Rect(RectShape::filled(
            egui::Rect::from_center_size(center, Vec2::new(width, height)),
            egui::Rounding::none(),
            Color32::GRAY
        ));

        painter.add(rect);

        let value = reg.port_collection.borrow().get_port_data(reg.output_port);
        painter.text(
            center,
            Align2::CENTER_CENTER,
            value.to_string(),
            FontId::monospace(20.0),
            Color32::WHITE
        );
    }

    pub fn draw_connection(painter: &mut Painter, path: &[Pos2]) {
        let path = egui::Shape::Path(PathShape::line(
            path.into(),
            egui::Stroke::new(3.0, Color32::LIGHT_RED)
        ));

        painter.add(path);
    }
}

mod tests {
    use super::*;
    use super::component_graphics::*;
    use crate::components::{ConstantRegister, Component, Register};
    use crate::circuit::PortCollection;
    use std::cell::RefCell;
    use std::rc::Rc;

    pub struct TestGUI<D, F>
    where F: FnMut(&mut D, &mut egui::Ui) -> () {
        data: D,
        f: F
    }

    impl<D: 'static, F: FnMut(&mut D, &mut egui::Ui) -> () + 'static> TestGUI<D, F> {
        pub fn new(data: D, f: F) -> Self {
            Self {
                data,
                f
            }
        }

        fn create_and_run(data: D, f: F) {
            let options = eframe::NativeOptions {
                drag_and_drop_support: true,
                initial_window_size: Some((1280.0, 720.0).into()),
                ..eframe::NativeOptions::default()
            };
            eframe::run_native(
                "Test GUI",
                options,
                Box::new(|_cc| Box::new(Self::new(data, f)))
            );
        }
    }

    impl<D, F: FnMut(&mut D, &mut egui::Ui) -> ()> eframe::App for TestGUI<D, F> {
        fn update(&mut self, ctx: &Context, frame: &mut eframe::Frame) {
            egui::CentralPanel::default().show(&ctx, |ui| {
                (self.f)(&mut self.data, ui);
            });
        }
    }

    #[test]
    fn test_draw_constant_register() {
        struct TestData {
            reg_c: ConstantRegister,
            reg_a: Register,
        }

        let port_collection = Rc::new(RefCell::new(PortCollection::new()));

        let reg_c = ConstantRegister::new(port_collection.clone(), 3, String::from("reg_c"));
        let reg_a = Register::new(port_collection.clone(), reg_c.output_port, String::from("reg_a"));

        let data = TestData {
            reg_c,
            reg_a
        };

        TestGUI::create_and_run(data, |data, ui| {
            ui.vertical(|ui| {
                ui.add(egui::Slider::new(&mut data.reg_c.constant_value, 0..=999));
                if ui.button("Process Cycle").clicked() {
                    data.reg_c.process_cycle();
                    data.reg_a.process_cycle();
                }
            });

            Frame::canvas(ui.style()).fill(COL_DARK_BLUE).show(ui, |ui| {
                let (response, mut painter) =
                    ui.allocate_painter(Vec2::new(ui.available_width(), ui.available_height()), Sense::hover());

                let to_screen = emath::RectTransform::from_to(
                    Rect::from_min_size(Pos2::ZERO, response.rect.size()),
                    response.rect,
                );

                let center = to_screen.transform_pos((100.0, 100.0).into());
                draw_constant_register(&mut painter, &data.reg_c, center);

                let center = to_screen.transform_pos((200.0, 100.0).into());
                draw_register(&mut painter, &data.reg_a, center);

                draw_connection(&mut painter, &[
                   center - Vec2::new(70.0, 0.0),
                    center - Vec2::new(30.0, 0.0),
                ]);
            });
        });
    }

}