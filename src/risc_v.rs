use std::mem;

/// OP codes
pub mod op_code {
    use crate::risc_v::Word;

    pub const OP: Word = 0;
    pub const OP_IMM: Word = 1;

    pub const LOAD: Word = 2;
    pub const STORE: Word = 3;

    pub const JAL: Word = 4;
    pub const JALR: Word = 5;
    pub const BRANCH: Word = 6;

    pub const LUI: Word = 7;
    pub const AUIPC: Word = 8;
}

pub mod func_code_3 {
    use crate::risc_v::Word;

    // Conditional branches
    // First bit determines unsigned vs signed versions, last 2 bit determine the condition.
    // Other branch conditions (e.g. BGT and BLE) are synthesized by swapping operands.
    pub const BEQ: Word = 0b_000;
    pub const BNE: Word = 0b_001;
    pub const BLT: Word = 0b_010;
    pub const BGE: Word = 0b_011;
    pub const BLTU: Word = 0b_110;      // Unsigned version (highest bit set to 1)
    pub const BGEU: Word = 0b_111;      // Unsigned version (highest bit set to 1)

    // Register-register ops
    pub const ADD: Word = 0b_000;
    pub const SLT: Word = 0b_001;
    pub const SLTU: Word = 0b_010;
    pub const AND: Word = 0b_011;
    pub const OR: Word = 0b_100;
    pub const XOR: Word = 0b_101;
    pub const SLL_SUB: Word = 0b_110;
    pub const SRL_SRA: Word = 0b_111;

    // Register-register ops MUL extension
    pub const MUL: Word = 0b_000;

    // Register-Immediate ops
    pub const ADDI: Word = 0b_000;
    pub const SLTI: Word = 0b_001;
    pub const SLTIU: Word = 0b_010;
    pub const ANDI: Word = 0b_011;
    pub const ORI: Word = 0b_100;
    pub const XORI: Word = 0b_101;
    pub const SLLI: Word = 0b_110;
    pub const SRLI_SRAI: Word = 0b_111;
}

pub mod func_code_7 {
    use crate::risc_v::Word;

    pub const BASE: Word = 0b_0000000;
    pub const BASE_ALT: Word = 0b_0100000;
    pub const MULDIV: Word = 0b_0000001;
}

// CPU data width
pub const CPU_WIDTH: usize = 32;
pub type Word = u32;
pub type SWord = i32;

/*
Instruction Shortcuts
 */

// ADDI x0, x0, 0
pub const NOP: Word = 0 | (func_code_3::ADDI << 12) | (op_code::OP_IMM as Word);

/*
Masks
 */

// Opcode
pub const MASK_OPCODE: Word = 0b_00000000_00000000_00000000_01111111;

// Registers
pub const MASK_RD: Word = 0b_00000000_00000000_00001111_10000000;
pub const MASK_RS1: Word = 0b_00000000_00001111_10000000_00000000;
pub const MASK_RS2: Word = 0b_00000001_11110000_00000000_00000000;

// Function codes
pub const MASK_FUNCT_3: Word = 0b_00000000_00000000_01110000_00000000;
pub const MASK_FUNCT_7: Word = 0b_11111110_00000000_00000000_00000000;

// Imm fields
pub const MASK_IMM_I_TYPE: Word = MASK_FUNCT_7 | MASK_RS2;
pub const MASK_IMM_U_TYPE: Word = MASK_FUNCT_7 | MASK_RS2 | MASK_RS1 | MASK_FUNCT_3;

pub const MASK_IMM_S_TYPE_LOWER: Word = MASK_RD;
pub const MASK_IMM_S_TYPE_UPPER: Word = MASK_FUNCT_7;

/// Extracts and sign-extends the immediate field from a given I-type instruction word.
pub fn extract_imm_i_type(instruction: Word) -> Word {
    let imm_value = (instruction & MASK_IMM_I_TYPE) >> 20;

    // The sign bit is always bit 31
    let is_negative = ((instruction >> 31) & 1) == 1;

    if is_negative {
        const PADDING_NEG: Word = 0b_11111111_11111111_11110000_00000000;
        imm_value | PADDING_NEG
    } else {
        imm_value
    }
}

/// Extracts and sign-extends the immediate field from a given S-type instruction word.
pub fn extract_imm_s_type(instruction: Word) -> Word {
    let imm_value_lower = (instruction & MASK_IMM_S_TYPE_LOWER) >> 7;
    let imm_value_upper = (instruction & MASK_IMM_S_TYPE_UPPER) >> 25;
    let imm_value = (imm_value_upper << 5) | imm_value_lower;

    // The sign bit is always bit 31
    let is_negative = ((instruction >> 31) & 1) == 1;


    if is_negative {
        const PADDING_NEG: Word = 0b_11111111_11111111_11110000_00000000;
        imm_value | PADDING_NEG
    } else {
        imm_value
    }
}

/// Extracts and sign-extends the immediate field from a given B-type instruction word.
pub fn extract_imm_b_type(instruction: Word) -> Word {
    extract_imm_s_type(instruction) << 2
}

/// Extracts and sign-extends the immediate field from a given U-type instruction word.
pub fn extract_imm_u_type(instruction: Word) -> Word {
    instruction & (MASK_IMM_U_TYPE)
}

/// Extracts and sign-extends the immediate field from a given J-type instruction word.
pub fn extract_imm_j_type(instruction: Word) -> Word {
    let imm_value = (instruction & MASK_IMM_U_TYPE) >> 12;

    // The sign bit is always bit 31
    let is_negative = ((instruction >> 31) & 1) == 1;

    if is_negative {
        const PADDING_NEG: Word = 0b_11111111_11110000_00000000_00000000;
        (imm_value | PADDING_NEG) << 1
    } else {
        (imm_value) << 1
    }
}

/// Extracts the op code from a given instruction word
pub fn extract_op_code(instruction: Word) -> Word {
    instruction & MASK_OPCODE
}

/// Extracts the funct 3 code from a given instruction word
pub fn extract_funct_3(instruction: Word) -> Word {
    (instruction & MASK_FUNCT_3) >> 12
}

/// Extracts the funct 7 code from a given instruction word
pub fn extract_funct_7(instruction: Word) -> Word {
    (instruction & MASK_FUNCT_7) >> 25
}

/// Extracts the rs1 number from a given instruction word
pub fn extract_rs1(instruction: Word) -> Word {
    (instruction & MASK_RS1) >> 15
}

/// Extracts the rs2 number from a given instruction word
pub fn extract_rs2(instruction: Word) -> Word {
    (instruction & MASK_RS2) >> 20
}

pub fn extract_rd(instruction: Word) -> Word {
    (instruction & MASK_RD) >> 7
}

/// Builds an R-type instruction from its given components.
pub const fn build_instruction_r_type(op_code: Word, func_7: Word, func_3: Word,
                                      dest: Word, s1: Word, s2: Word) -> Word {
    op_code |
        (dest << 7) |
        (func_3 << 12) |
        (s1 << 15) |
        (s2 << 20) |
        (func_7 << 25)
}

/// Builds an I-type instruction from its given components.
pub const fn build_instruction_i_type(op_code: Word, func_3: Word,
                                      dest: Word, s1: Word, imm: Word) -> Word {
    op_code |
        (dest << 7) |
        (func_3 << 12) |
        (s1 << 15) |
        (imm << 20)
}

/// Builds an S-type instruction from its given components.
pub const fn build_instruction_s_type(op_code: Word, func_3: Word,
                                      s1: Word, s2: Word, imm: Word) -> Word {
    op_code |
        ((imm & 0b_011111) << 7) |
        (func_3 << 12) |
        (s1 << 15) |
        (s2 << 20) |
        ((imm & 0b_0111111100000) << (25 - 5))
}

/// Builds a B-type instruction from its given components.
pub const fn build_instruction_b_type(op_code: Word, func_3: Word,
                                      s1: Word, s2: Word, imm: Word) -> Word {
    build_instruction_s_type(op_code, func_3, s1, s2, imm)
}