use crate::isa::*;
use std::collections::HashMap;
use std::fmt::Formatter;
use std::ops::Range;
use std::{fmt, mem};

/*


2-Pass Parser:
 - First pass:
   Collect all jump labels
   Assign an instruction address to all instructions

 - Second pass:
   Encode all instructions

Data:
 - Hashmap mapping each label to an instruction address
 - Array mapping line number to instruction number

 */

macro_rules! check_num_params {
    ($tokens:expr, $expectation:literal) => {
        if $tokens.len() != ($expectation + 1) {
            return Err(format!(
                "Invalid number of parameters. Expected {} but got {}.",
                $expectation,
                $tokens.len() - 1
            ));
        }
    };
}

macro_rules! parse_param {
    ($val:expr => REG $name:ident) => {
        let $name = token_to_register($val);
        if $name == None {
            return Err(format!("Invalid register identifier: '{}' !", $val));
        }
        let $name = $name.unwrap();
    };
    ($val:expr => IMM $name:ident) => {
        let $name = token_to_literal($val);
        if $name == None {
            return Err(format!("Invalid immediate: '{}' !", $val));
        }
        let $name = $name.unwrap().as_word();
    };
    ($val:expr => IMM $name:ident ($max_len:expr)) => {
        let $name = token_to_literal($val);
        if $name == None {
            return Err(format!("Invalid immediate: '{}' !", $val));
        }
        let $name = $name.unwrap();
        if !$name.fits_in($max_len) {
            return Err(format!(
                "Immediate '{}' does not fit in '{}' bits!",
                $val, $max_len
            ));
        }
        let $name = $name.as_word();
    };
    ($val:expr => BRANCH $name:ident) => {
        let literal_conversion = token_to_literal($val);
        let $name = if let Some(literal) = literal_conversion {
            BranchTarget::Literal(literal)
        } else {
            BranchTarget::Label($val)
        };
    };
}

/// Parses a token into a register. Registers are expected in the form "xn", where n is the register
/// number.
fn token_to_register(token: &str) -> Option<Word> {
    let reg_number_str = token.strip_prefix("x")?;
    let reg_number = reg_number_str.parse::<Word>();

    return if let Ok(reg_number) = reg_number {
        if reg_number > 31 {
            return None;
        }

        Some(reg_number)
    } else {
        None
    };
}

// Wrapper for signed or unsigned literals. Provides some utilities for performing checks.
#[derive(Debug, PartialEq)]
pub enum Literal {
    Signed(SWord),
    Unsigned(Word),
}

impl Literal {
    pub fn as_word(&self) -> Word {
        match self {
            Self::Signed(val) => *val as Word,
            Self::Unsigned(val) => *val,
        }
    }

    pub fn is_signed(&self) -> bool {
        match self {
            Self::Signed(_) => true,
            Self::Unsigned(_) => false,
        }
    }

    pub fn is_unsigned(&self) -> bool {
        match self {
            Self::Signed(_) => false,
            Self::Unsigned(_) => true,
        }
    }

    /// Does this literal fit in a given number of bits?
    pub fn fits_in(&self, num_bits: u32) -> bool {
        match self {
            Self::Signed(val) => *val < (1 << (num_bits - 1)) && -(*val) <= (1 << (num_bits - 1)),
            Self::Unsigned(val) => *val < (1 << num_bits),
        }
    }
}

/// Parses a token into a literal.
fn token_to_literal(token: &str) -> Option<Literal> {
    if token.starts_with("-") {
        token.parse::<SWord>().ok().map(|i| Literal::Signed(i))
    } else {
        token.parse::<Word>().ok().map(|i| Literal::Unsigned(i))
    }
}

/// Target of a branch instruction can either be a label or an offset given as a literal.
pub enum BranchTarget<'a> {
    Label(&'a str),
    Literal(Literal),
}

impl BranchTarget<'_> {
    /// Computes the offset associated with a BranchTarget given both an instruction index and a
    /// map of possible labels.
    ///
    /// Returns None if the label cannot be resolved.
    pub fn resolve_offset(
        &self,
        instruction_idx: usize,
        labels: &HashMap<&str, usize>,
    ) -> Option<Word> {
        match self {
            BranchTarget::Literal(literal) => Some(literal.as_word()),
            BranchTarget::Label(label) => {
                if let Some(target_idx) = labels.get(label) {
                    let offset = *target_idx as SWord - (instruction_idx + 1) as SWord;
                    Some(offset as Word)
                } else {
                    None
                }
            }
        }
    }
}

/// Given raw text describing an instruction, this function returns the encoded instruction
/// word. No labels can be used as branch targets.
///
/// If the tokens do not constitute a valid instruction, an error string is returned.
pub fn encode_instruction_str(instruction: &str) -> Result<Word, String> {
    let instruction = instruction.trim();
    let tokens: Vec<&str> = instruction.split_whitespace().collect();

    encode_instruction_no_labels(&tokens)
}

/// Given a series of tokens that form an instruction, this function returns the encoded instruction
/// word. No labels can be used as branch targets.
///
/// If the tokens do not constitute a valid instruction, an error string is returned.
pub fn encode_instruction_no_labels(tokens: &Vec<&str>) -> Result<Word, String> {
    let instruction_token = tokens.first().unwrap();

    let instruction_word = match *instruction_token {
        /*
        NOP
         */
        "NOP" => {
            // Compiles to ADDI x0 x0 0
            check_num_params!(tokens, 0);
            build_instruction_i_type(op_code::OP_IMM, func_code_3::ADDI, 0, 0, 0)
        }

        /*
        Moves
         */
        "MV" => {
            // Compiles to ADDI dst src 0
            check_num_params!(tokens, 2);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src);
            build_instruction_i_type(op_code::OP_IMM, func_code_3::ADDI, dst, src, 0)
        }
        "MVI" => {
            // Compiles to ADDI dst x0 imm
            check_num_params!(tokens, 2);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::ADDI, dst, 0, imm)
        }

        /*
        OP
         */
        "ADD" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::ADD,
                dst,
                src1,
                src2,
            )
        }
        "SLT" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::SLT,
                dst,
                src1,
                src2,
            )
        }
        "SLTU" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::SLTU,
                dst,
                src1,
                src2,
            )
        }
        "AND" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::AND,
                dst,
                src1,
                src2,
            )
        }
        "OR" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::OR,
                dst,
                src1,
                src2,
            )
        }
        "XOR" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::XOR,
                dst,
                src1,
                src2,
            )
        }
        "SLL" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::SLL_SUB,
                dst,
                src1,
                src2,
            )
        }
        "SUB" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE_ALT,
                func_code_3::SLL_SUB,
                dst,
                src1,
                src2,
            )
        }
        "SRL" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::SRL_SRA,
                dst,
                src1,
                src2,
            )
        }
        "SRA" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE_ALT,
                func_code_3::SRL_SRA,
                dst,
                src1,
                src2,
            )
        }

        /*
        OP-IMM
         */
        "ADDI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::ADDI, dst, src1, imm)
        }
        "SLTI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::SLTI, dst, src1, imm)
        }
        "SLTIU" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::SLTIU, dst, src1, imm)
        }
        "ANDI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::ANDI, dst, src1, imm)
        }
        "ORI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::ORI, dst, src1, imm)
        }
        "XORI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::XORI, dst, src1, imm)
        }
        "SLLI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (5));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::SLLI, dst, src1, imm)
        }
        "SRLI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (5));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::SRLI_SRAI, dst, src1, imm)
        }
        "SRAI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (5));
            build_instruction_i_type(
                op_code::OP_IMM,
                func_code_3::SRLI_SRAI,
                dst,
                src1,
                imm | 0b0100000_00000,
            )
        }

        /*
        BRANCH
         */
        "BEQ" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => IMM offset (12));
            build_instruction_b_type(op_code::BRANCH, func_code_3::BEQ, src1, src2, offset)
        }
        "BNE" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => IMM offset (12));
            build_instruction_b_type(op_code::BRANCH, func_code_3::BNE, src1, src2, offset)
        }
        "BLT" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => IMM offset (12));
            build_instruction_b_type(op_code::BRANCH, func_code_3::BLT, src1, src2, offset)
        }
        "BLTU" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => IMM offset (12));
            build_instruction_b_type(op_code::BRANCH, func_code_3::BLTU, src1, src2, offset)
        }
        "BGE" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => IMM offset (12));
            build_instruction_b_type(op_code::BRANCH, func_code_3::BGE, src1, src2, offset)
        }
        "BGEU" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => IMM offset (12));
            build_instruction_b_type(op_code::BRANCH, func_code_3::BGEU, src1, src2, offset)
        }
        "BLE" => {
            // Uses BGE with swapped operands
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => IMM offset (12));
            build_instruction_b_type(op_code::BRANCH, func_code_3::BGE, src2, src1, offset)
        }
        "BLEU" => {
            // Uses BGEU with swapped operands
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => IMM offset (12));
            build_instruction_b_type(op_code::BRANCH, func_code_3::BGEU, src2, src1, offset)
        }
        "BGT" => {
            // Uses BLT with swapped operands
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => IMM offset (12));
            build_instruction_b_type(op_code::BRANCH, func_code_3::BLT, src2, src1, offset)
        }
        "BGTU" => {
            // Uses BLTU with swapped operands
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => IMM offset (12));
            build_instruction_b_type(op_code::BRANCH, func_code_3::BLTU, src2, src1, offset)
        }

        /*
        Memory Access
         */
        "LOAD" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG base);
            parse_param!(tokens[3] => IMM offset);
            build_instruction_i_type(op_code::LOAD, 0, dst, base, offset)
        }
        "STORE" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src);
            parse_param!(tokens[2] => REG base);
            parse_param!(tokens[3] => IMM offset);
            build_instruction_s_type(op_code::STORE, 0, base, src, offset)
        }

        /*
        Extension
         */
        "MUL" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::MULDIV,
                func_code_3::MUL,
                dst,
                src1,
                src2,
            )
        }

        _ => {
            return Err(format!("Unrecognized instruction: {}", *instruction_token));
        }
    };

    return Ok(instruction_word);
}

/// Given a series of tokens that form an instruction, the index of the instruction in the program,
/// and a map of valid labels in the program, this function returns the encoded instruction
/// word.
///
/// If the tokens do not constitute a valid instruction, an error string is returned.
pub fn encode_instruction(
    tokens: &Vec<&str>,
    instruction_idx: usize,
    labels: &HashMap<&str, usize>,
) -> Result<Word, String> {
    let instruction_token = tokens.first().unwrap();

    /// Used to validate a branch target and resolve it to an offset.
    macro_rules! validate_and_resolve {
        ($target:expr => $name:ident) => {
            let $name = $target.resolve_offset(instruction_idx, labels);
            if $name.is_none() {
                let label = if let BranchTarget::Label(label) = $target {
                    label
                } else {
                    unreachable!()
                };
                return Err(format!("Label '{}' does not exist!", label));
            }
            let $name = $name.unwrap();
            if !Literal::Signed($name as SWord).fits_in(12) {
                return Err(format!(
                    "Branch target is too far away and does not fit in 12 bits!"
                ));
            }
        };
    }

    let instruction_word = match *instruction_token {
        /*
        NOP
         */
        "NOP" => {
            // Compiles to ADDI x0 x0 0
            check_num_params!(tokens, 0);
            build_instruction_i_type(op_code::OP_IMM, func_code_3::ADDI, 0, 0, 0)
        }

        /*
        Moves
         */
        "MV" => {
            // Compiles to ADDI dst src 0
            check_num_params!(tokens, 2);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src);
            build_instruction_i_type(op_code::OP_IMM, func_code_3::ADDI, dst, src, 0)
        }
        "MVI" => {
            // Compiles to ADDI dst x0 imm
            check_num_params!(tokens, 2);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::ADDI, dst, 0, imm)
        }

        /*
        OP
         */
        "ADD" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::ADD,
                dst,
                src1,
                src2,
            )
        }
        "SLT" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::SLT,
                dst,
                src1,
                src2,
            )
        }
        "SLTU" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::SLTU,
                dst,
                src1,
                src2,
            )
        }
        "AND" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::AND,
                dst,
                src1,
                src2,
            )
        }
        "OR" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::OR,
                dst,
                src1,
                src2,
            )
        }
        "XOR" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::XOR,
                dst,
                src1,
                src2,
            )
        }
        "SLL" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::SLL_SUB,
                dst,
                src1,
                src2,
            )
        }
        "SUB" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE_ALT,
                func_code_3::SLL_SUB,
                dst,
                src1,
                src2,
            )
        }
        "SRL" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE,
                func_code_3::SRL_SRA,
                dst,
                src1,
                src2,
            )
        }
        "SRA" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::BASE_ALT,
                func_code_3::SRL_SRA,
                dst,
                src1,
                src2,
            )
        }

        /*
        OP-IMM
         */
        "ADDI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::ADDI, dst, src1, imm)
        }
        "SLTI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::SLTI, dst, src1, imm)
        }
        "SLTIU" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::SLTIU, dst, src1, imm)
        }
        "ANDI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::ANDI, dst, src1, imm)
        }
        "ORI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::ORI, dst, src1, imm)
        }
        "XORI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (12));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::XORI, dst, src1, imm)
        }
        "SLLI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (5));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::SLLI, dst, src1, imm)
        }
        "SRLI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (5));
            build_instruction_i_type(op_code::OP_IMM, func_code_3::SRLI_SRAI, dst, src1, imm)
        }
        "SRAI" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => IMM imm (5));
            build_instruction_i_type(
                op_code::OP_IMM,
                func_code_3::SRLI_SRAI,
                dst,
                src1,
                imm | 0b0100000_00000,
            )
        }

        /*
        BRANCH
         */
        "BEQ" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => BRANCH target);
            validate_and_resolve!(target => offset);
            build_instruction_b_type(op_code::BRANCH, func_code_3::BEQ, src1, src2, offset)
        }
        "BNE" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => BRANCH target);
            validate_and_resolve!(target => offset);
            build_instruction_b_type(op_code::BRANCH, func_code_3::BNE, src1, src2, offset)
        }
        "BLT" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => BRANCH target);
            validate_and_resolve!(target => offset);
            build_instruction_b_type(op_code::BRANCH, func_code_3::BLT, src1, src2, offset)
        }
        "BLTU" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => BRANCH target);
            validate_and_resolve!(target => offset);
            build_instruction_b_type(op_code::BRANCH, func_code_3::BLTU, src1, src2, offset)
        }
        "BGE" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => BRANCH target);
            validate_and_resolve!(target => offset);
            build_instruction_b_type(op_code::BRANCH, func_code_3::BGE, src1, src2, offset)
        }
        "BGEU" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => BRANCH target);
            validate_and_resolve!(target => offset);
            build_instruction_b_type(op_code::BRANCH, func_code_3::BGEU, src1, src2, offset)
        }
        "BLE" => {
            // Uses BGE with swapped operands
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => BRANCH target);
            validate_and_resolve!(target => offset);
            build_instruction_b_type(op_code::BRANCH, func_code_3::BGE, src2, src1, offset)
        }
        "BLEU" => {
            // Uses BGEU with swapped operands
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => BRANCH target);
            validate_and_resolve!(target => offset);
            build_instruction_b_type(op_code::BRANCH, func_code_3::BGEU, src2, src1, offset)
        }
        "BGT" => {
            // Uses BLT with swapped operands
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => BRANCH target);
            validate_and_resolve!(target => offset);
            build_instruction_b_type(op_code::BRANCH, func_code_3::BLT, src2, src1, offset)
        }
        "BGTU" => {
            // Uses BLTU with swapped operands
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src1);
            parse_param!(tokens[2] => REG src2);
            parse_param!(tokens[3] => BRANCH target);
            validate_and_resolve!(target => offset);
            build_instruction_b_type(op_code::BRANCH, func_code_3::BLTU, src2, src1, offset)
        }

        /*
        Memory Access
         */
        "LOAD" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG base);
            parse_param!(tokens[3] => IMM offset);
            build_instruction_i_type(op_code::LOAD, 0, dst, base, offset)
        }
        "STORE" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG src);
            parse_param!(tokens[2] => REG base);
            parse_param!(tokens[3] => IMM offset);
            build_instruction_s_type(op_code::STORE, 0, base, src, offset)
        }

        /*
        Extension
         */
        "MUL" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(
                op_code::OP,
                func_code_7::MULDIV,
                func_code_3::MUL,
                dst,
                src1,
                src2,
            )
        }

        _ => {
            return Err(format!(
                "Unrecognized instruction: '{}'",
                *instruction_token
            ));
        }
    };

    return Ok(instruction_word);
}

pub fn parse_label_definition(code_line: &str) -> Result<&str, String> {
    // Trim trailing whitespace
    let code_line = code_line.trim_end();

    // The label definition must end with a ':' character
    if !code_line.ends_with(':') {
        return Err(format!(
            "Expected ':' character at the end of label definition!"
        ));
    }

    // Remove the ':' at the end of the line
    let (label_identifier, _) = code_line.split_at(code_line.len() - 1);
    let label_identifier = label_identifier.trim();

    fn is_valid_identifier(text: &str) -> bool {
        (!text.is_empty()) && text.chars().all(|c: char| c.is_alphabetic() || (c == '_'))
    }

    // Test if the label identifier is valid
    if !is_valid_identifier(label_identifier) {
        return Err(format!(
            "Invalid label identifier! No whitespace or ':' characters allowed!"
        ));
    }

    return Ok(label_identifier);
}

pub struct ExplorationPassResult<'a> {
    /// A hashmap mapping labels to instruction indexes
    labels: HashMap<&'a str, usize>,

    /// An array of instructions. A tuple is stored for each instruction, capturing the line number
    /// at which the instruction is defined, together with the text defining the instruction.
    instructions: Vec<(usize, &'a str)>,
}

pub struct ExplorationPassError<'a> {
    /// The line number of the original program at which the error occurred.
    line_number: usize,
    /// The raw line at which the error occurred.
    raw_line: &'a str,
    /// An error message.
    message: String,
}

impl<'a> ExplorationPassError<'a> {
    pub fn new(line_number: usize, raw_line: &'a str, message: String) -> Self {
        Self {
            line_number,
            raw_line,
            message,
        }
    }
}

impl fmt::Display for ExplorationPassError<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Cause: {} \n\n Occurred on line {}:\n {} \n",
            self.message,
            self.line_number + 1,
            self.raw_line
        )
    }
}

impl fmt::Debug for ExplorationPassError<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Cause: {} \n\n Occurred on line {}:\n {}",
            self.message,
            self.line_number + 1,
            self.raw_line
        )
    }
}

/// Strips a possible comment from a raw line of program text.
pub fn strip_comment(raw_line: &str) -> &str {
    if let Some((code_part, _)) = raw_line.split_once(';') {
        code_part
    } else {
        raw_line
    }
}

/// Makes an exploratory first pass through raw program text (lines). During this pass:
/// (1) instructions are identified and located, but not parsed, and
/// (2) all labels are identified and their corresponding addresses are determined.
pub fn parse_exploration_pass<'a>(
    raw_lines: &Vec<&'a str>,
) -> Result<ExplorationPassResult<'a>, ExplorationPassError<'a>> {
    // Variables used in the result
    let mut labels: HashMap<&str, usize> = HashMap::new();
    let mut instructions: Vec<(usize, &'a str)> = Vec::new();

    // Variables for bookkeeping
    let mut last_label: Option<(usize, &str)> = None; // (line_number, identifier)
    let mut instruction_counter: usize = 0;

    for (line_idx, line) in raw_lines.iter().enumerate() {
        // Check for comment and separate it from the code
        let code = strip_comment(line);

        if code.trim().is_empty() {
            // This code line is empty (or only contains whitespace)
        } else if code.contains(':') {
            // This code line defines a label
            let parse_result = parse_label_definition(code);

            if let Err(error_message) = parse_result {
                // An error occurred while parsing
                return Err(ExplorationPassError::new(line_idx, *line, error_message));
            }

            // Parsing was successful, so we can unwrap the identifier
            let identifier = parse_result.unwrap();

            if labels.contains_key(identifier) {
                // Error, label already exists
                return Err(ExplorationPassError::new(
                    line_idx,
                    *line,
                    format!("Label identifier '{}' is already defined!", identifier),
                ));
            }

            if last_label.is_some() {
                // Error, two labels defined consecutively
                return Err(ExplorationPassError::new(
                    line_idx,
                    *line,
                    format!("Label '{}' points to the same instruction as the previously defined label '{}' !", identifier, last_label.unwrap().1)
                ));
            }

            // Label is OK, so store it to be matched to an instruction
            last_label = Some((line_idx, identifier));
        } else {
            // This code line should describe an instruction
            if let Some((_, identifier)) = last_label {
                // Set the address corresponding to the last label to the address of this instruction
                labels.insert(identifier, instruction_counter);
                last_label = None;
            }

            // Store the instruction and its location
            instructions.push((line_idx, *line));
            instruction_counter += 1;
        }
    }

    if let Some((line_idx, identifier)) = last_label {
        // There is a label that does not point to any instruction
        return Err(ExplorationPassError::new(
            line_idx,
            raw_lines[line_idx],
            format!("Label '{}' does not point to an instruction!", identifier),
        ));
    }

    // All is Ok, return results
    Ok(ExplorationPassResult {
        labels,
        instructions,
    })
}

pub struct Program {
    /// The raw program text.
    raw_text: String,

    /// Slices from the raw program text expressed in ranges.
    raw_text_lines: Vec<Range<usize>>,

    /// Maps all labels in the program to their associated instruction index.
    labels: HashMap<String, usize>,

    /// Maps instruction indices to lines in the original program text.
    instruction_index: Vec<usize>,

    /// Encoded instruction words that make up the program.
    instructions: Vec<Word>,
}

impl Program {
    pub fn from_text(text: String) -> Result<Program, String> {
        let raw_lines: Vec<&str> = text.lines().collect();
        let exploration_pass_result = parse_exploration_pass(&raw_lines);

        // An error occurred in the first pass
        if let Err(error) = exploration_pass_result {
            return Err(format!("{}", error));
        }

        let exploration_pass_result = exploration_pass_result.unwrap();
        let mut encoded_instructions: Vec<Word> = Vec::new();

        for (instruction_idx, (line_idx, instruction_line)) in
            exploration_pass_result.instructions.iter().enumerate()
        {
            let instruction_code = strip_comment(*instruction_line);
            let tokens: Vec<&str> = instruction_code.split_whitespace().collect();

            let encoded_instruction =
                encode_instruction(&tokens, instruction_idx, &exploration_pass_result.labels);

            if let Err(error) = encoded_instruction {
                return Err(format!(
                    "Error: {}\n\nOccurred on line {}:\n {}",
                    error,
                    *line_idx + 1,
                    *instruction_line
                ));
            }

            encoded_instructions.push(encoded_instruction.unwrap());
        }

        // Convert data structures using slices to avoid Program becoming a self-referencing struct

        let lines_as_ranges = raw_lines
            .into_iter()
            .map(|s: &str| {
                let start = s.as_ptr() as usize - text.as_ptr() as usize;
                let end = start + s.len();
                Range { start, end }
            })
            .collect();

        let labels_as_strings = exploration_pass_result
            .labels
            .into_iter()
            .map(|(s, l)| (s.to_string(), l))
            .collect();

        return Ok(Program {
            raw_text_lines: lines_as_ranges,
            labels: labels_as_strings,
            instruction_index: exploration_pass_result
                .instructions
                .iter()
                .map(|(idx, _)| *idx)
                .collect(),
            instructions: encoded_instructions,
            raw_text: text,
        });
    }

    pub fn to_mem<const MEM_SIZE: usize>(&self) -> [u8; MEM_SIZE] {
        let mut memory = [0; MEM_SIZE];

        // Copy program
        for (i, instruction) in self.instructions.iter().enumerate() {
            let mem_offset = i * mem::size_of::<Word>();

            let instruction_bytes = instruction.to_be_bytes();
            memory[mem_offset + 0] = instruction_bytes[0];
            memory[mem_offset + 1] = instruction_bytes[1];
            memory[mem_offset + 2] = instruction_bytes[2];
            memory[mem_offset + 3] = instruction_bytes[3];
        }

        // Fill the rest with NOP instructions
        let start_idx = self.instructions.len();
        let end_idx = MEM_SIZE / std::mem::size_of::<Word>();

        for i in start_idx..end_idx {
            let mem_offset = i * mem::size_of::<Word>();

            let instruction_bytes = NOP.to_be_bytes();
            memory[mem_offset + 0] = instruction_bytes[0];
            memory[mem_offset + 1] = instruction_bytes[1];
            memory[mem_offset + 2] = instruction_bytes[2];
            memory[mem_offset + 3] = instruction_bytes[3];
        }

        return memory;
    }

    pub fn raw_text<'a>(&self) -> &str {
        self.raw_text.as_str()
    }

    pub fn get_line_slice(&self, idx: usize) -> &str {
        &self.raw_text[self.raw_text_lines[idx].clone()]
    }

    pub fn get_lines_as_slices(&self) -> Vec<&str> {
        self.raw_text_lines
            .iter()
            .map(|r| &self.raw_text[r.clone()])
            .collect()
    }

    pub fn instruction_to_line(&self, instr_idx: usize) -> Option<usize> {
        self.instruction_index.get(instr_idx).map(|i| *i)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_label_definition() {
        macro_rules! create_test {
            // Expect success
            ($code:literal -> $exp:literal) => {
                let result = parse_label_definition($code);

                if let Ok(ident) = result {
                    assert_eq!(ident, $exp);
                } else {
                    let error = result.unwrap_err();
                    assert!(false, "Expected success, but found fail: \n {}", error);
                }
            };

            // Expect failure
            ($code:literal FAILS) => {
                let result = parse_label_definition($code);

                if let Ok(ident) = result {
                    assert!(
                        false,
                        "Expected fail, but found parsed identifier: {}",
                        ident
                    );
                } else {
                    assert!(true);
                }
            };
        }

        create_test!("balab:" -> "balab");
        create_test!("   balab: " -> "balab");
        create_test!("   balab:     \n" -> "balab");

        create_test!("balab\n" FAILS);
        create_test!("   balab     \n" FAILS);
        create_test!("ba lab:\n" FAILS);
        create_test!("ba\tlab:\n" FAILS);
        create_test!("ba:lab:\n" FAILS);
    }

    #[test]
    pub fn test_token_to_reg() {
        assert_eq!(token_to_register("pp0"), None);
        assert_eq!(token_to_register("x33"), None);
        assert_eq!(token_to_register("x 15"), None);
        assert_eq!(token_to_register("15x"), None);
        assert_eq!(token_to_register("x"), None);

        assert_eq!(token_to_register("x15"), Some(15));
        assert_eq!(token_to_register("x3"), Some(3));
    }

    #[test]
    pub fn test_token_to_literal() {
        assert_eq!(token_to_literal("2x2"), None);
        assert_eq!(token_to_literal(""), None);
        assert_eq!(token_to_literal("a"), None);

        assert_eq!(token_to_literal("0022"), Some(Literal::Unsigned(22)));
        assert_eq!(token_to_literal("-22"), Some(Literal::Signed(-22)));
        assert_eq!(token_to_literal("8192"), Some(Literal::Unsigned(8192)));
    }

    #[test]
    pub fn test_literal_fits_in() {
        assert_eq!(Literal::Unsigned(1 << 10).fits_in(10), false);
        assert_eq!(Literal::Unsigned((1 << 10) - 1).fits_in(10), true);

        assert_eq!(Literal::Signed(1 << 9).fits_in(10), false);
        assert_eq!(Literal::Signed((1 << 9) - 1).fits_in(10), true);
        assert_eq!(Literal::Signed(-(1 << 9) - 1).fits_in(10), false);
        assert_eq!(Literal::Signed(-(1 << 9)).fits_in(10), true);
    }
}
