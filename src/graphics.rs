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
    use crate::components::{ConstantRegister, Mux, Register, RegisterFile, RMemory};
    use eframe::egui::{Pos2, Painter, Vec2, Color32, FontId, Align2, Shape, Stroke};
    use eframe::epaint::{RectShape, PathShape};

    /*
    Colors theme (https://coolors.co/palette/264653-2a9d8f-e9c46a-f4a261-e76f51)
     */

    pub const COL_DARK_BLUE: Color32 = Color32::from_rgb(38, 70, 83);
    pub const COL_LIGHT_BLUE: Color32 = Color32::from_rgb(42, 157, 143);
    pub const COL_SAND: Color32 = Color32::from_rgb(233, 196, 106);
    pub const COL_LIGHT_ORANGE: Color32 = Color32::from_rgb(244, 162, 97);
    pub const COL_DARK_ORANGE: Color32 = Color32::from_rgb(231, 111, 81);
    pub const COL_GREEN: Color32 = Color32::from_rgb(183, 232, 104);

    /*
    Constants
     */

    const PORT_SIZE: Vec2 = Vec2::new(8.0, 8.0);
    const PORT_ROUNDING: f32 = 1.0;

    /*
    Graphics for ConstantRegister
     */

    pub struct ConstantRegisterGraphic<'a> {
        reg: &'a ConstantRegister,
        position: Pos2,
        pub out_port_pos: Pos2,
    }

    impl<'a> ConstantRegisterGraphic<'a> {
        const BG_WIDTH: f32 = 60.0;
        const BG_HEIGHT: f32 = 45.0;
        const BG_ROUNDING: f32 = 5.0;

        pub fn new(reg: &'a ConstantRegister, position: Pos2) -> Self {
            Self {
                reg,
                position,
                out_port_pos: position + Vec2::new(Self::BG_WIDTH / 2.0, 0.0)
            }
        }

        pub fn draw(&self, painter: &mut Painter) {
            // Draw background
            let background = egui::Shape::Rect(RectShape::filled(
                egui::Rect::from_center_size(self.position, Vec2::new(Self::BG_WIDTH, Self::BG_HEIGHT)),
                egui::Rounding::same(Self::BG_ROUNDING),
                COL_SAND
            ));

            painter.add(background);

            // Draw output port
            draw_port(painter, self.out_port_pos, COL_DARK_ORANGE);

            // Draw text
            let pos_name = self.position - Vec2::new(0.0, 8.0);
            let pos_val = self.position + Vec2::new(0.0, 8.0);

            draw_text_basic(painter, &self.reg.name, pos_name, 13.0);

            let value = self.reg.port_collection.borrow().get_port_data(self.reg.output_port);
            draw_text_basic(painter, value, pos_val, 13.0);
        }
    }

    /*
    Graphics for Register
     */

    pub struct RegisterGraphic<'a> {
        reg: &'a Register,
        position: Pos2,
        pub in_port_pos: Pos2,
        pub out_port_pos: Pos2,
    }

    impl<'a> RegisterGraphic<'a> {
        const BG_WIDTH: f32 = 60.0;
        const BG_HEIGHT: f32 = 45.0;
        const BG_ROUNDING: f32 = 5.0;

        pub fn new(reg: &'a Register, position: Pos2) -> Self {
            Self {
                reg,
                position,
                in_port_pos: position - Vec2::new(Self::BG_WIDTH / 2.0, 0.0),
                out_port_pos: position + Vec2::new(Self::BG_WIDTH / 2.0, 0.0)
            }
        }

        pub fn draw(&self, painter: &mut Painter) {
            // Draw background
            let background = egui::Shape::Rect(RectShape::filled(
                egui::Rect::from_center_size(self.position, Vec2::new(Self::BG_WIDTH, Self::BG_HEIGHT)),
                egui::Rounding::same(Self::BG_ROUNDING),
                COL_SAND
            ));

            painter.add(background);

            // Draw ports
            draw_port(painter, self.in_port_pos, COL_GREEN);
            draw_port(painter, self.out_port_pos, COL_DARK_ORANGE);

            // Draw text
            let pos_name = self.position - Vec2::new(0.0, 8.0);
            let pos_val = self.position + Vec2::new(0.0, 8.0);

            draw_text_basic(painter, &self.reg.name, pos_name, 13.0);

            let value = self.reg.port_collection.borrow().get_port_data(self.reg.output_port);
            draw_text_basic(painter, value, pos_val, 13.0);
        }
    }

    /*
    Graphics for Mux
     */

    pub struct MuxGraphic<'a, const NUM_INPUTS: usize> {
        mux: &'a Mux<NUM_INPUTS>,
        position: Pos2,

        pub in_port_positions: [Pos2; NUM_INPUTS],
        pub select_port_pos: Pos2,
        pub out_port_pos: Pos2,
    }

    impl<'a, const NUM_INPUTS: usize> MuxGraphic<'a, NUM_INPUTS> {
        const BG_WIDTH: f32 = 40.0;
        const BG_HEIGHT_OUT: f32 = 45.0;
        const BG_HEIGHT_IN: f32 = 75.0;
        const BG_ROUNDING: f32 = 5.0;

        pub fn new(mux: &'a Mux<NUM_INPUTS>, position: Pos2) -> Self {
            let mut in_port_positions = [Pos2::ZERO; NUM_INPUTS];
            let in_port_x =  position.x - (Self::BG_WIDTH / 2.0);

            let height_per_segment = Self::BG_HEIGHT_IN / (NUM_INPUTS + 1) as f32;

            for i in 0..NUM_INPUTS {
                let y = (i + 1)  as f32 * height_per_segment + (position.y - (Self::BG_HEIGHT_IN / 2.0));
                in_port_positions[i] = Pos2::new(in_port_x, y);
            }

            let bot_left = position + 0.5 * Vec2::new(-Self::BG_WIDTH, Self::BG_HEIGHT_IN);
            let bot_right = position + 0.5 * Vec2::new(Self::BG_WIDTH, Self::BG_HEIGHT_OUT);
            let select_port_pos = bot_left + 0.5 * (bot_right - bot_left);

            Self {
                mux,
                position,
                in_port_positions,
                select_port_pos,
                out_port_pos: position + Vec2::new(Self::BG_WIDTH / 2.0, 0.0)
            }
        }

        pub fn draw(&self, painter: &mut Painter) {
            // Draw background
            let top_left = self.position + 0.5 * Vec2::new(-Self::BG_WIDTH, Self::BG_HEIGHT_IN);
            let bot_left = self.position + 0.5 * Vec2::new(-Self::BG_WIDTH, -Self::BG_HEIGHT_IN);
            let top_right = self.position + 0.5 * Vec2::new(Self::BG_WIDTH, Self::BG_HEIGHT_OUT);
            let bot_right = self.position + 0.5 * Vec2::new(Self::BG_WIDTH, -Self::BG_HEIGHT_OUT);

            let background = Shape::Path(PathShape::convex_polygon(
                vec![top_left, top_right, bot_right, bot_left],
                COL_SAND,
                Stroke::new(
                    5.0,
                    COL_SAND
                )
            ));

            painter.add(background);

            // Draw inner connection
            let selection_idx = self.mux.port_collection.borrow().get_port_data(self.mux.selection_input);

            let inner_connection = Shape::LineSegment {
                points: [self.in_port_positions[selection_idx as usize], self.out_port_pos],
                stroke: Stroke::new(3.0, COL_LIGHT_BLUE)
            };

            painter.add(inner_connection);

            // Draw ports
            draw_port(painter, self.out_port_pos, COL_DARK_ORANGE);
            draw_port(painter, self.select_port_pos, COL_GREEN);

            for i in 0..NUM_INPUTS {
                draw_port(painter, self.in_port_positions[i], COL_GREEN);
            }
        }
    }

    /*
    Graphics for Mux
    */

    pub struct RMemoryGraphic<'a> {
        mem: &'a RMemory,
        position: Pos2,

        pub in_address_pos: Pos2,
        pub in_length_pos: Pos2,
        pub out_port_pos: Pos2
    }

    impl<'a> RMemoryGraphic<'a> {
        const BG_WIDTH: f32 = 60.0;
        const BG_HEIGHT: f32 = 120.0;
        const BG_ROUNDING: f32 = 5.0;

        pub fn new(mem: &'a RMemory, position: Pos2) -> Self {
            Self {
                mem,
                position,
                in_address_pos: position + Vec2::new(-Self::BG_WIDTH / 2.0, -Self::BG_HEIGHT / 4.0),
                in_length_pos: position + Vec2::new(-Self::BG_WIDTH / 2.0, Self::BG_HEIGHT / 4.0),
                out_port_pos: position + Vec2::new(Self::BG_WIDTH / 2.0, -Self::BG_HEIGHT / 4.0)
            }
        }

        pub fn draw(&self, painter: &mut Painter) {
            // Draw background
            let background = Shape::Rect(RectShape::filled(
                egui::Rect::from_center_size(self.position, Vec2::new(Self::BG_WIDTH, Self::BG_HEIGHT)),
                egui::Rounding::same(Self::BG_ROUNDING),
                COL_SAND
            ));

            painter.add(background);

            // Draw ports
            draw_port(painter, self.in_address_pos, COL_GREEN);
            draw_port(painter, self.in_length_pos, COL_GREEN);
            draw_port(painter, self.out_port_pos, COL_DARK_ORANGE);

            // Draw name
            let pos_name = self.position;
            draw_text_basic(painter, &self.mem.name, pos_name, 15.0);
        }
    }

    /*
    Graphics for RegisterFile
    */

    pub struct RegisterFileGraphic<'a, const NUM_REGISTERS: usize> {
        rf: &'a RegisterFile<NUM_REGISTERS>,
        position: Pos2,

        pub in_read_a_pos: Pos2,
        pub in_read_b_pos: Pos2,

        pub in_write_data_pos: Pos2,
        pub in_write_select_pos: Pos2,
        pub in_write_enable_pos: Pos2,

        pub out_a_pos: Pos2,
        pub out_b_pos: Pos2,
    }

    impl<'a, const NUM_REGISTERS: usize> RegisterFileGraphic<'a, NUM_REGISTERS> {
        const BG_WIDTH: f32 = 60.0;
        const BG_HEIGHT: f32 = 109.0;
        const BG_ROUNDING: f32 = 5.0;

        pub fn new(rf: &'a RegisterFile<NUM_REGISTERS>, position: Pos2) -> Self {
            let half_width = Self::BG_WIDTH / 2.0;
            let half_height = Self::BG_HEIGHT / 2.0;

            let left_edge = position.x - half_width;
            let right_edge = position.x + half_width;

            let read_a_height = position.y - (0.70 * half_height);
            let read_b_height = position.y - (0.30 * half_height);

            let write_data_height = position.y + (0.30 * half_height);
            let write_select_height = position.y + (0.70 * half_height);
            let write_enable_height = position.y + (1.0 * half_height);

            Self {
                rf,
                position,
                in_read_a_pos: Pos2::new(left_edge, read_a_height),
                in_read_b_pos: Pos2::new(left_edge, read_b_height),
                in_write_data_pos: Pos2::new(left_edge, write_data_height),
                in_write_select_pos: Pos2::new(left_edge, write_select_height),
                in_write_enable_pos: Pos2::new(position.x, write_enable_height),
                out_a_pos: Pos2::new(right_edge, read_a_height),
                out_b_pos: Pos2::new(right_edge, read_a_height)
            }
        }

        pub fn draw(&self, painter: &mut Painter) {
            // Draw background
            let background = Shape::Rect(RectShape::filled(
                egui::Rect::from_center_size(self.position, Vec2::new(Self::BG_WIDTH, Self::BG_HEIGHT)),
                egui::Rounding::same(Self::BG_ROUNDING),
                COL_SAND
            ));

            painter.add(background);

            // Draw ports
            draw_port(painter, self.in_read_a_pos, COL_GREEN);
            draw_port(painter, self.in_read_b_pos, COL_GREEN);
            draw_port(painter, self.in_write_data_pos, COL_GREEN);
            draw_port(painter, self.in_write_select_pos, COL_GREEN);
            draw_port(painter, self.in_write_enable_pos, COL_GREEN);

            draw_port(painter, self.out_a_pos, COL_DARK_ORANGE);
            draw_port(painter, self.out_b_pos, COL_DARK_ORANGE);

            // Draw name
            let pos_name = self.position;
            draw_text_basic(painter, "RF", pos_name, 15.0);
        }
    }

    /// Draws the graphic for a port at a given position using a given color.
    pub fn draw_port(painter: &mut Painter, pos: Pos2, col: Color32) {
        let port = Shape::Rect(RectShape::filled(
            egui::Rect::from_center_size(pos, PORT_SIZE),
            egui::Rounding::same(PORT_ROUNDING),
            col
        ));

        painter.add(port);
    }

    /// Draws a string of black monospace text aligned CENTER-CENTER style.
    pub fn draw_text_basic(painter: &mut Painter, text: impl ToString, pos: Pos2, size: f32) {
        painter.text(
            pos,
            Align2::CENTER_CENTER,
            text,
            FontId::monospace(size),
            Color32::BLACK
        );
    }

    pub fn draw_connection_simple(painter: &mut Painter, start: Pos2, end: Pos2, jump_pos: Option<f32>) {
        let path = if start.y == end.y {
            // Go in a straight line
            vec![start, end]
        } else {
            // A jump is necessary
            let d = end.to_vec2() - start.to_vec2();

            let jump_pos = jump_pos.unwrap_or(0.5).clamp(0.0, 1.0);

            let jump_first = start + Vec2::new(jump_pos * d.x, 0.0);
            let jump_second = jump_first + Vec2::new(0.0, d.y);

            vec![start, jump_first, jump_second, end]
        };

        let path = Shape::Path(PathShape::line(
            path,
            Stroke::new(3.0, COL_LIGHT_BLUE)
        ));

        painter.add(path);
    }

    pub fn draw_connection(painter: &mut Painter, path: &[Pos2]) {
        let path = egui::Shape::Path(PathShape::line(
            path.into(),
            egui::Stroke::new(3.0, COL_LIGHT_BLUE)
        ));

        painter.add(path);
    }
}

mod tests {
    use super::*;
    use super::component_graphics::*;
    use crate::components::{ConstantRegister, Component, Register, Mux, RMemory, RegisterFile};
    use crate::circuit::{PORT_NULL_ID, PortCollection};
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
            reg_s: ConstantRegister,
            mux: Mux<2>,
            imem: RMemory,
            rf: RegisterFile<32>
        }

        let port_collection = Rc::new(RefCell::new(PortCollection::new()));

        let reg_c = ConstantRegister::new(port_collection.clone(), 3, String::from("reg_c"));
        let reg_a = Register::new(port_collection.clone(), reg_c.output_port, String::from("reg_a"));
        let reg_s = ConstantRegister::new(port_collection.clone(), 1, String::from("reg_s"));

        let mux = Mux::<2>::new(port_collection.clone(), &[reg_c.output_port, reg_a.output_port], reg_s.output_port, String::from("mux"));
        let imem = RMemory::new(port_collection.clone(), mux.output_port, PORT_NULL_ID, String::from("imem"));
        let rf = RegisterFile::<32>::new(
            port_collection.clone(),
            PORT_NULL_ID,
            PORT_NULL_ID,
            PORT_NULL_ID,
            PORT_NULL_ID,
            PORT_NULL_ID,
        );

        let data = TestData {
            reg_c,
            reg_a,
            reg_s,
            mux,
            imem,
            rf
        };

        TestGUI::create_and_run(data, |data, ui| {
            ui.vertical(|ui| {
                ui.add(egui::Slider::new(&mut data.reg_c.constant_value, 0..=999));
                if ui.button("Process Cycle").clicked() {
                    data.reg_a.process_cycle();
                    data.reg_s.process_cycle();
                    data.reg_c.process_cycle();
                    data.mux.process_cycle();
                    data.imem.process_cycle();
                    data.rf.process_cycle();
                }

                if ui.button("Switch Mux").clicked() {
                    data.reg_s.constant_value ^= 1;
                    data.reg_s.process_cycle();
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
                let reg_c_graphic = ConstantRegisterGraphic::new(&data.reg_c, center);

                let center = to_screen.transform_pos((200.0, 100.0).into());
                let reg_a_graphic = RegisterGraphic::new(&data.reg_a, center);

                let center = to_screen.transform_pos((100.0, 250.0).into());
                let reg_s_graphic = ConstantRegisterGraphic::new(&data.reg_s, center);

                let center = to_screen.transform_pos((350.0, 150.0).into());
                let mux_graphic = MuxGraphic::new(&data.mux, center);

                let center = to_screen.transform_pos((450.0, 180.0).into());
                let imem_graphic = RMemoryGraphic::new(&data.imem, center);

                let center = to_screen.transform_pos((450.0, 350.0).into());
                let rf_graphic = RegisterFileGraphic::new(&data.rf, center);

                draw_connection_simple(
                    &mut painter,
                    reg_c_graphic.out_port_pos,
                    reg_a_graphic.in_port_pos,
                    None
                );

                draw_connection_simple(
                    &mut painter,
                    reg_a_graphic.out_port_pos,
                    mux_graphic.in_port_positions[0],
                    None
                );

                draw_connection_simple(
                    &mut painter,
                    reg_c_graphic.out_port_pos,
                    mux_graphic.in_port_positions[1],
                    Some(0.1)
                );

                draw_connection_simple(
                    &mut painter,
                    reg_s_graphic.out_port_pos,
                    mux_graphic.select_port_pos,
                    Some(1.0)
                );

                draw_connection_simple(
                    &mut painter,
                    mux_graphic.out_port_pos,
                    imem_graphic.in_address_pos,
                    None
                );

                reg_c_graphic.draw(&mut painter);
                reg_a_graphic.draw(&mut painter);
                reg_s_graphic.draw(&mut painter);
                mux_graphic.draw(&mut painter);
                imem_graphic.draw(&mut painter);
                rf_graphic.draw(&mut painter);
            });
        });
    }

}
