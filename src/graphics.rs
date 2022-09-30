use crate::assembler;
use crate::assembler::Program;
use crate::components::Component;
use crate::cpu::Processor;
use eframe::egui;
use eframe::egui::{Color32, Context, ScrollArea};

#[derive(Copy, Clone, PartialEq)]
pub enum Tab {
    Editor,
    Run,
}

pub struct ProcessorGUI {
    pc: usize,
    pos: egui::Pos2,

    selected_tab: Tab,
    code_editor: CodeEditor,
    run_environment: RunEnvironment,
}

impl ProcessorGUI {
    pub fn new() -> Self {
        Self {
            pos: (50.0, 50.0).into(),
            pc: 0,
            selected_tab: Tab::Editor,
            code_editor: CodeEditor::new(),
            run_environment: RunEnvironment::new(),
        }
    }
}

impl eframe::App for ProcessorGUI {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("Tabs").show(&ctx, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.selected_tab, Tab::Editor, "Editor");
                ui.selectable_value(&mut self.selected_tab, Tab::Run, "Run");
            });
        });

        match self.selected_tab {
            Tab::Editor => {
                self.code_editor.ui(ctx, |program| {
                    self.run_environment.load_program(program);
                });
            }
            Tab::Run => {
                self.run_environment.ui(ctx);
            }
        }
    }
}

#[derive(Debug)]
pub struct RunState {
    /// Program counter for IF stage
    pc_if: usize,

    /// Program counter for ID stage
    pc_id: usize,

    /// Program counter for EX stage
    pc_ex: usize,

    /// Program counter for MEM stage
    pc_mem: usize,

    /// Program counter for WB stage
    pc_wb: usize,
}

impl RunState {
    pub fn new() -> Self {
        Self {
            pc_if: 0,
            pc_id: 0,
            pc_ex: 0,
            pc_mem: 0,
            pc_wb: 0,
        }
    }

    pub fn propagate_pc(&mut self, is_stalled: bool) {
        self.pc_wb = self.pc_mem;
        self.pc_mem = self.pc_ex;

        if !is_stalled {
            self.pc_ex = self.pc_id;
            self.pc_id = self.pc_if;
        }
    }
}

pub struct RunEnvironment {
    program: Option<Program>,
    cpu: Processor,
    run_state: Option<RunState>,
}

impl RunEnvironment {
    pub fn new() -> Self {
        Self {
            program: None,
            cpu: Processor::new(),
            run_state: None,
        }
    }

    pub fn load_program(&mut self, program: Program) {
        self.cpu = Processor::new();
        self.cpu.load_program_memory(&program.to_mem());
        self.program = Some(program);

        self.run_state = Some(RunState::new());
    }

    pub fn build_register_table(&self, ui: &mut egui::Ui) {
        egui::Grid::new("register_state")
            .striped(true)
            .show(ui, |ui| {
                let register_contents = self.cpu.get_rf_contents();

                for y in 0..8 {
                    for x in 0..4 {
                        let reg_idx = y * 4 + x;

                        let reg_val = if reg_idx == 0 {
                            0
                        } else {
                            register_contents[reg_idx]
                        };

                        ui.monospace(format!("x{}:", reg_idx));
                        ui.monospace(reg_val.to_string());
                    }
                    ui.end_row();
                }
            });
    }

    pub fn build_program_table(&self, ui: &mut egui::Ui, program: &Program) {
        egui::ScrollArea::vertical()
            .id_source("code")
            .show(ui, |ui| {
                let run_state = self.run_state.as_ref().unwrap();

                // Resolve program counters to lines
                let current_program_line_if = program
                    .instruction_to_line(run_state.pc_if)
                    .unwrap_or(usize::MAX);
                let current_program_line_id = program
                    .instruction_to_line(run_state.pc_id)
                    .unwrap_or(usize::MAX);
                let current_program_line_ex = program
                    .instruction_to_line(run_state.pc_ex)
                    .unwrap_or(usize::MAX);
                let current_program_line_mem = program
                    .instruction_to_line(run_state.pc_mem)
                    .unwrap_or(usize::MAX);
                let current_program_line_wb = program
                    .instruction_to_line(run_state.pc_wb)
                    .unwrap_or(usize::MAX);

                egui::Grid::new("code_lines")
                    .striped(true)
                    .min_col_width(0.0)
                    .show(ui, |ui| {
                        // TODO: use the pre-built lines from the program here
                        for (idx, line) in program.raw_text().lines().into_iter().enumerate() {
                            ui.monospace(idx.to_string());
                            ui.add(egui::Separator::default().vertical());

                            // Print program counters

                            ui.monospace(if idx == current_program_line_if {
                                ">"
                            } else {
                                " "
                            });
                            ui.add(egui::Separator::default().vertical());

                            ui.monospace(if idx == current_program_line_id {
                                ">"
                            } else {
                                " "
                            });
                            ui.add(egui::Separator::default().vertical());

                            ui.monospace(if idx == current_program_line_ex {
                                ">"
                            } else {
                                " "
                            });
                            ui.add(egui::Separator::default().vertical());

                            ui.monospace(if idx == current_program_line_mem {
                                ">"
                            } else {
                                " "
                            });
                            ui.add(egui::Separator::default().vertical());

                            ui.monospace(if idx == current_program_line_wb {
                                ">"
                            } else {
                                " "
                            });
                            ui.add(egui::Separator::default().vertical());

                            ui.monospace(line);
                            ui.end_row();
                        }
                    });
            });
    }

    pub fn ui(&mut self, ctx: &Context) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Loaded Program");
            ui.separator();

            if let Some(program) = &self.program {
                if ui.button("Process Cycle").clicked() {
                    self.cpu.process_cycle();

                    if let Some(run_state) = &mut self.run_state {
                        run_state.propagate_pc(self.cpu.is_stalled());
                        run_state.pc_if = self.cpu.get_coming_instruction_idx() as usize;
                    }
                }
                self.build_program_table(ui, program);
            }
        });

        egui::SidePanel::right("state").show(ctx, |ui| {
            ui.heading("Register State");
            ui.separator();
            self.build_register_table(ui);
        });
    }
}

pub struct CodeEditor {
    code: String,
    assembler_output: String,
    font_size: f32,
}

impl CodeEditor {
    pub fn new() -> Self {
        Self {
            code: String::new(),
            assembler_output: String::from("Output from the assembler will show up here..."),
            font_size: 15.0,
        }
    }

    fn is_instruction(word: &str) -> bool {
        matches!(
            word,
            // NOP
            "NOP" |

            // Moves
            "MV" | "MVI" |

            // Ops
            "ADD" | "SLT" | "SLTU" | "AND" | "OR" | "XOR" | "SLL" | "SUB" | "SRL" | "SRA" |

            // OP-IMM
            "ADDI" | "SLTI" | "SLTIU" | "ANDI" | "ORI" | "XORI" | "SLLI" | "SRLI" | "SRAI" |

            // Branch
            "BEQ" | "BNE" | "BLT" | "BLTU" | "BGE" | "BGEU" | "BLE" | "BLEU" | "BGT" | "BGTU" |

            // Memory Access
            "LOAD" | "STORE" |

            // Extension
            "MUL"
        )
    }

    fn is_register(word: &str) -> bool {
        (word.len() >= 2)
            && (word.starts_with('x'))
            && (word[1..].chars().all(|c: char| c.is_numeric()))
    }

    fn is_literal(word: &str) -> bool {
        (!word.is_empty()) && (word.chars().all(|c: char| c.is_numeric()))
    }

    /// Creates a layout job for rendering highlighted code.
    fn highlight_code(mut code: &str, font_size: f32) -> egui::text::LayoutJob {
        let mut job = egui::text::LayoutJob::default();

        let font_id = egui::FontId::monospace(font_size);

        let format_normal = egui::TextFormat::simple(font_id.clone(), Color32::WHITE);
        let format_comment = egui::TextFormat::simple(font_id.clone(), Color32::LIGHT_BLUE);
        let format_keyword = egui::TextFormat::simple(font_id.clone(), Color32::LIGHT_RED);
        let format_register = egui::TextFormat::simple(font_id.clone(), Color32::LIGHT_GREEN);
        let format_literal = egui::TextFormat::simple(font_id.clone(), Color32::KHAKI);

        while !code.is_empty() {
            if code.starts_with(";") {
                let end = code.find('\n').unwrap_or(code.len());
                job.append(&code[..end], 0.0, format_comment.clone());
                code = &code[end..];
            } else if code.starts_with(|c: char| c.is_ascii_alphanumeric()) {
                let end = code[1..]
                    .find(|c: char| !c.is_ascii_alphanumeric())
                    .map_or_else(|| code.len(), |i| i + 1);

                let token = &code[..end];

                if Self::is_instruction(token) {
                    job.append(token, 0.0, format_keyword.clone());
                } else if Self::is_register(token) {
                    job.append(token, 0.0, format_register.clone());
                } else if Self::is_literal(token) {
                    job.append(token, 0.0, format_literal.clone());
                } else {
                    job.append(token, 0.0, format_normal.clone());
                }
                code = &code[end..];
            } else if code.starts_with(|c: char| c.is_ascii_whitespace()) {
                let end = code[1..]
                    .find(|c: char| !c.is_ascii_whitespace())
                    .map_or_else(|| code.len(), |i| i + 1);
                job.append(&code[..end], 0.0, format_normal.clone());
                code = &code[end..];
            } else {
                let end = code[1..]
                    .find(|c: char| c.is_ascii_whitespace())
                    .map_or_else(|| code.len(), |i| i + 1);
                job.append(&code[..end], 0.0, format_normal.clone());
                code = &code[end..];
            }
        }

        return job;
    }

    pub fn ui(&mut self, ctx: &Context, on_program_load: impl FnOnce(Program)) {
        egui::TopBottomPanel::top("controls").show(&ctx, |ui| {
            ui.horizontal(|ui| {
                if ui.button("assemble").clicked() {
                    let assembler_result = assembler::Program::from_text(self.code.clone());
                    if let Err(error) = assembler_result {
                        self.assembler_output = error;
                    } else {
                        self.assembler_output = String::from("Program assembled correctly!");
                    }
                }
                if ui.button("load").clicked() {
                    let assembler_result = assembler::Program::from_text(self.code.clone());
                    if let Err(error) = assembler_result {
                        self.assembler_output = error;
                    } else {
                        self.assembler_output = String::from("Program assembled correctly!");
                        on_program_load(assembler_result.unwrap());
                    }
                }
            });
        });

        egui::SidePanel::right("console_panel")
            .default_width(400.0)
            .show(&ctx, |ui| {
                ScrollArea::vertical().id_source("console").show(ui, |ui| {
                    ui.add(
                        egui::TextEdit::multiline(&mut self.assembler_output.as_str())
                            .desired_width(f32::INFINITY)
                            .code_editor(),
                    )
                });
            });

        egui::CentralPanel::default().show(&ctx, |ui| {
            ScrollArea::vertical().id_source("source").show(ui, |ui| {
                // TODO: cache highlighting result
                let mut layouter = |ui: &egui::Ui, string: &str, wrap_width: f32| {
                    let mut layout_job: egui::text::LayoutJob =
                        Self::highlight_code(string, self.font_size);
                    layout_job.wrap.max_width = wrap_width;
                    ui.fonts().layout_job(layout_job)
                };

                ui.add(
                    egui::TextEdit::multiline(&mut self.code)
                        .code_editor()
                        .font(egui::FontId::monospace(self.font_size))
                        .desired_width(f32::INFINITY)
                        .desired_rows((ui.available_height() / self.font_size) as usize)
                        .layouter(&mut layouter),
                )
            });
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
        Box::new(|_cc| Box::new(ProcessorGUI::new())),
    );
}

mod component_graphics {
    use crate::components::{ConstantRegister, Mux, RMemory, Register, RegisterFile};
    use eframe::egui;
    use eframe::egui::{Align2, Color32, FontId, Painter, Pos2, Shape, Stroke, Vec2};
    use eframe::epaint::{PathShape, RectShape};

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
                out_port_pos: position + Vec2::new(Self::BG_WIDTH / 2.0, 0.0),
            }
        }

        pub fn draw(&self, painter: &mut Painter) {
            // Draw background
            let background = egui::Shape::Rect(RectShape::filled(
                egui::Rect::from_center_size(
                    self.position,
                    Vec2::new(Self::BG_WIDTH, Self::BG_HEIGHT),
                ),
                egui::Rounding::same(Self::BG_ROUNDING),
                COL_SAND,
            ));

            painter.add(background);

            // Draw output port
            draw_port(painter, self.out_port_pos, COL_DARK_ORANGE);

            // Draw text
            let pos_name = self.position - Vec2::new(0.0, 8.0);
            let pos_val = self.position + Vec2::new(0.0, 8.0);

            draw_text_basic(painter, &self.reg.name, pos_name, 13.0);

            let value = self
                .reg
                .port_collection
                .borrow()
                .get_port_data(self.reg.output_port);
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
                out_port_pos: position + Vec2::new(Self::BG_WIDTH / 2.0, 0.0),
            }
        }

        pub fn draw(&self, painter: &mut Painter) {
            // Draw background
            let background = egui::Shape::Rect(RectShape::filled(
                egui::Rect::from_center_size(
                    self.position,
                    Vec2::new(Self::BG_WIDTH, Self::BG_HEIGHT),
                ),
                egui::Rounding::same(Self::BG_ROUNDING),
                COL_SAND,
            ));

            painter.add(background);

            // Draw ports
            draw_port(painter, self.in_port_pos, COL_GREEN);
            draw_port(painter, self.out_port_pos, COL_DARK_ORANGE);

            // Draw text
            let pos_name = self.position - Vec2::new(0.0, 8.0);
            let pos_val = self.position + Vec2::new(0.0, 8.0);

            draw_text_basic(painter, &self.reg.name, pos_name, 13.0);

            let value = self
                .reg
                .port_collection
                .borrow()
                .get_port_data(self.reg.output_port);
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
            let in_port_x = position.x - (Self::BG_WIDTH / 2.0);

            let height_per_segment = Self::BG_HEIGHT_IN / (NUM_INPUTS + 1) as f32;

            for i in 0..NUM_INPUTS {
                let y =
                    (i + 1) as f32 * height_per_segment + (position.y - (Self::BG_HEIGHT_IN / 2.0));
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
                out_port_pos: position + Vec2::new(Self::BG_WIDTH / 2.0, 0.0),
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
                Stroke::new(5.0, COL_SAND),
            ));

            painter.add(background);

            // Draw inner connection
            let selection_idx = self
                .mux
                .port_collection
                .borrow()
                .get_port_data(self.mux.selection_input);

            let inner_connection = Shape::LineSegment {
                points: [
                    self.in_port_positions[selection_idx as usize],
                    self.out_port_pos,
                ],
                stroke: Stroke::new(3.0, COL_LIGHT_BLUE),
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
        pub out_port_pos: Pos2,
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
                out_port_pos: position + Vec2::new(Self::BG_WIDTH / 2.0, -Self::BG_HEIGHT / 4.0),
            }
        }

        pub fn draw(&self, painter: &mut Painter) {
            // Draw background
            let background = Shape::Rect(RectShape::filled(
                egui::Rect::from_center_size(
                    self.position,
                    Vec2::new(Self::BG_WIDTH, Self::BG_HEIGHT),
                ),
                egui::Rounding::same(Self::BG_ROUNDING),
                COL_SAND,
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
                out_b_pos: Pos2::new(right_edge, read_a_height),
            }
        }

        pub fn draw(&self, painter: &mut Painter) {
            // Draw background
            let background = Shape::Rect(RectShape::filled(
                egui::Rect::from_center_size(
                    self.position,
                    Vec2::new(Self::BG_WIDTH, Self::BG_HEIGHT),
                ),
                egui::Rounding::same(Self::BG_ROUNDING),
                COL_SAND,
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
            col,
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
            Color32::BLACK,
        );
    }

    pub fn draw_connection_simple(
        painter: &mut Painter,
        start: Pos2,
        end: Pos2,
        jump_pos: Option<f32>,
    ) {
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

        let path = Shape::Path(PathShape::line(path, Stroke::new(3.0, COL_LIGHT_BLUE)));

        painter.add(path);
    }

    pub fn draw_connection(painter: &mut Painter, path: &[Pos2]) {
        let path = egui::Shape::Path(PathShape::line(
            path.into(),
            egui::Stroke::new(3.0, COL_LIGHT_BLUE),
        ));

        painter.add(path);
    }
}

#[allow(unused_imports)]
mod tests {
    use super::component_graphics::*;
    use super::*;
    use crate::circuit::{PortCollection, PORT_NULL_ID};
    use crate::components::{Component, ConstantRegister, Mux, RMemory, Register, RegisterFile};
    use std::cell::RefCell;
    use std::rc::Rc;

    use egui;
    use egui::{emath, Frame, Pos2, Rect, Sense, Vec2};

    pub struct TestGUI<D, F>
    where
        F: FnMut(&mut D, &mut egui::Ui) -> (),
    {
        data: D,
        f: F,
    }

    impl<D: 'static, F: FnMut(&mut D, &mut egui::Ui) -> () + 'static> TestGUI<D, F> {
        pub fn new(data: D, f: F) -> Self {
            Self { data, f }
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
                Box::new(|_cc| Box::new(Self::new(data, f))),
            );
        }
    }

    impl<D, F: FnMut(&mut D, &mut egui::Ui) -> ()> eframe::App for TestGUI<D, F> {
        fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
            egui::CentralPanel::default().show(&ctx, |ui| {
                (self.f)(&mut self.data, ui);
            });
        }
    }

    #[test]
    #[ignore]
    fn test_draw_constant_register() {
        struct TestData {
            reg_c: ConstantRegister,
            reg_a: Register,
            reg_s: ConstantRegister,
            mux: Mux<2>,
            imem: RMemory,
            rf: RegisterFile<32>,
        }

        let port_collection = Rc::new(RefCell::new(PortCollection::new()));

        let reg_c = ConstantRegister::new(port_collection.clone(), 3, String::from("reg_c"));
        let reg_a = Register::new(
            port_collection.clone(),
            reg_c.output_port,
            String::from("reg_a"),
        );
        let reg_s = ConstantRegister::new(port_collection.clone(), 1, String::from("reg_s"));

        let mux = Mux::<2>::new(
            port_collection.clone(),
            &[reg_c.output_port, reg_a.output_port],
            reg_s.output_port,
            String::from("mux"),
        );
        let imem = RMemory::new(
            port_collection.clone(),
            mux.output_port,
            PORT_NULL_ID,
            String::from("imem"),
        );
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
            rf,
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

            Frame::canvas(ui.style())
                .fill(COL_DARK_BLUE)
                .show(ui, |ui| {
                    let (response, mut painter) = ui.allocate_painter(
                        Vec2::new(ui.available_width(), ui.available_height()),
                        Sense::hover(),
                    );

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
                        None,
                    );

                    draw_connection_simple(
                        &mut painter,
                        reg_a_graphic.out_port_pos,
                        mux_graphic.in_port_positions[0],
                        None,
                    );

                    draw_connection_simple(
                        &mut painter,
                        reg_c_graphic.out_port_pos,
                        mux_graphic.in_port_positions[1],
                        Some(0.1),
                    );

                    draw_connection_simple(
                        &mut painter,
                        reg_s_graphic.out_port_pos,
                        mux_graphic.select_port_pos,
                        Some(1.0),
                    );

                    draw_connection_simple(
                        &mut painter,
                        mux_graphic.out_port_pos,
                        imem_graphic.in_address_pos,
                        None,
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
