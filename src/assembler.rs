use crate::isa::*;
use std::mem;

macro_rules! check_num_params {
    ($tokens:expr, $expectation:literal) => {
        if $tokens.len() != ($expectation + 1) { return Err(format!("Invalid number of parameters. Expected {} but got {}.", $expectation, $tokens.len() - 1)); }
    }
}

macro_rules! parse_param {
    ($val:expr => REG $name:ident) => {
        let $name = token_to_register($val);
        if $name == None { return Err(format!("Invalid register identifier: {}", $val)); }
        let $name = $name.unwrap();
    };
    ($val:expr => IMM $name:ident) => {
        let $name = token_to_literal($val);
        if $name == None { return Err(format!("Invalid immediate: {}", $val)); }
        let $name = $name.unwrap().as_word();
    };
    ($val:expr => IMM $name:ident ($max_len:expr)) => {
        let $name = token_to_literal($val);
        if $name == None { return Err(format!("Invalid immediate: {}", $val)); }
        let $name = $name.unwrap();
        if !$name.fits_in($max_len) { return Err(format!("Immediate {} does not fit in {} bits!", $val, $max_len)); }
        let $name = $name.as_word();
    }
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
    }
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

// Wrapper for signed or unsigned literals. Provides some utilities for performing checks.
#[derive(Debug, PartialEq)]
pub enum Literal {
    Signed(SWord),
    Unsigned(Word)
}

impl Literal {
    pub fn as_word(&self) -> Word {
        match self {
            Self::Signed(val) => *val as Word,
            Self::Unsigned(val) => *val
        }
    }

    pub fn is_signed(&self) -> bool {
        match self {
            Self::Signed(_) => true,
            Self::Unsigned(_) => false
        }
    }

    pub fn is_unsigned(&self) -> bool {
        match self {
            Self::Signed(_) => false,
            Self::Unsigned(_) => true
        }
    }

    /// Does this literal fit in a given number of bits?
    pub fn fits_in(&self, num_bits: u32) -> bool {
        match self {
            Self::Signed(val) => {
                *val < (1 << (num_bits - 1)) && -(*val) <= (1 << (num_bits - 1))
            },
            Self::Unsigned(val) => {
                *val < (1 << num_bits)
            }
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

pub fn encode_instruction_str(instruction: &str) -> Result<Word, String> {
    let instruction = instruction.trim();
    let tokens: Vec<&str> = instruction.split_whitespace().collect();

    encode_instruction(&tokens)
}

pub fn encode_instruction(tokens: &Vec<&str>) -> Result<Word, String> {
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
            build_instruction_r_type(op_code::OP, func_code_7::BASE, func_code_3::ADD, dst, src1, src2)
        }
        "SLT" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(op_code::OP, func_code_7::BASE, func_code_3::SLT, dst, src1, src2)
        }
        "SLTU" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(op_code::OP, func_code_7::BASE, func_code_3::SLTU, dst, src1, src2)
        }
        "AND" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(op_code::OP, func_code_7::BASE, func_code_3::AND, dst, src1, src2)
        }
        "OR" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(op_code::OP, func_code_7::BASE, func_code_3::OR, dst, src1, src2)
        }
        "XOR" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(op_code::OP, func_code_7::BASE, func_code_3::XOR, dst, src1, src2)
        }
        "SLL" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(op_code::OP, func_code_7::BASE, func_code_3::SLL_SUB, dst, src1, src2)
        }
        "SUB" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(op_code::OP, func_code_7::BASE_ALT, func_code_3::SLL_SUB, dst, src1, src2)
        }
        "SRL" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(op_code::OP, func_code_7::BASE, func_code_3::SRL_SRA, dst, src1, src2)
        }
        "SRA" => {
            check_num_params!(tokens, 3);
            parse_param!(tokens[1] => REG dst);
            parse_param!(tokens[2] => REG src1);
            parse_param!(tokens[3] => REG src2);
            build_instruction_r_type(op_code::OP, func_code_7::BASE_ALT, func_code_3::SRL_SRA, dst, src1, src2)
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
            build_instruction_i_type(op_code::OP_IMM, func_code_3::SRLI_SRAI, dst, src1, imm | 0b0100000_00000)
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
            build_instruction_r_type(op_code::OP, func_code_7::MULDIV, func_code_3::MUL, dst, src1, src2)
        }


        _ => { return Err(format!("Unrecognized instruction: {}", *instruction_token)); }
    };

    return Ok(instruction_word);
}

pub fn assemble_program(program_text: &str) -> Result<Vec<Word>, String> {
    let mut program: Vec<Word> = Vec::new();
    let instructions = program_text.split(';');

    for instruction in instructions {
        let instruction = instruction.trim();

        let tokens: Vec<&str> = instruction.split_whitespace().collect();

        // Skip empty instruction line rather than panicking.
        // Occurs when a double semi-colon is inserted.
        if tokens.len() == 0 {
            continue;
        }

        // Encode and push
        let instruction_word = encode_instruction(&tokens)?;
        program.push(instruction_word);
    }

    return Ok(program);
}

pub fn program_to_mem<const MEM_SIZE: usize>(program: &Vec<Word>) -> [u8; MEM_SIZE] {
    let mut memory = [0; MEM_SIZE];

    // Copy program
    for (i, instruction) in program.iter().enumerate() {
        let mem_offset = i * mem::size_of::<Word>();

        let instruction_bytes = instruction.to_be_bytes();
        memory[mem_offset + 0] = instruction_bytes[0];
        memory[mem_offset + 1] = instruction_bytes[1];
        memory[mem_offset + 2] = instruction_bytes[2];
        memory[mem_offset + 3] = instruction_bytes[3];
    }

    // Fill the rest with NOP instructions
    let start_idx = program.len();
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

#[cfg(test)]
mod tests {
    use super::*;

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
