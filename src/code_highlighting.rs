use eframe::egui;
use eframe::egui::Color32;

/// Checks whether a given string slice matches any instruction.
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

/// Checks whether a given string slice matches a register.
fn is_register(word: &str) -> bool {
    (word.len() >= 2)
        && (word.starts_with('x'))
        && (word[1..].chars().all(|c: char| c.is_numeric()))
}

/// Checks whether a given string slice constitutes a literal.
fn is_literal(word: &str) -> bool {
    (!word.is_empty()) && (word.chars().all(|c: char| c.is_numeric()))
}

/// Creates a layout job for rendering highlighted code.
pub fn highlight_code(mut code: &str, font_size: f32) -> egui::text::LayoutJob {
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

            if is_instruction(token) {
                job.append(token, 0.0, format_keyword.clone());
            } else if is_register(token) {
                job.append(token, 0.0, format_register.clone());
            } else if is_literal(token) {
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
