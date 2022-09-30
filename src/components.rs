use crate::circuit::*;
use crate::isa::*;
use std::cell::RefCell;
use std::mem::MaybeUninit;
use std::rc::Rc;

/*
Traits
 */

pub trait Component {
    fn process_cycle(&mut self);
}

/*
Registers
 */

/// Standard register that stores its current input every clock cycle.
pub struct Register {
    pub input: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,
}

/// Register that only commits input when an enable signal is given.
pub struct GuardedRegister {
    pub input_enable: PortID,
    pub input: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,
}

/// Register outputting a constant value.
pub struct ConstantRegister {
    pub constant_value: Word,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,
}

/// Standard register that selects only a contiguous subset of bits from its input, and outputs
/// these bits shifted to the right.
///
/// These registers are used to emulate connections in hardware, where only a selection of output
/// bits are connected rather than the whole output WORD.
pub struct BitSelectionRegister<const START_BIT: usize, const LEN: usize> {
    pub input: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,
}

impl Component for Register {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        let input_value = port_collection.get_port_data(self.input);
        port_collection.set_port_data(self.output_port, input_value);
    }
}

impl Component for GuardedRegister {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        if port_collection.get_port_data(self.input_enable) != 0 {
            let input_value = port_collection.get_port_data(self.input);
            port_collection.set_port_data(self.output_port, input_value);
        }
    }
}

impl Component for ConstantRegister {
    fn process_cycle(&mut self) {
        // First half of the cycle performs STORE and the the second half performs LOAD
        // , so we store before we load
        let mut port_collection = self.port_collection.borrow_mut();
        port_collection.set_port_data(self.output_port, self.constant_value);
    }
}

impl<const START_BIT: usize, const LEN: usize> Component for BitSelectionRegister<START_BIT, LEN> {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        let input_value = port_collection.get_port_data(self.input);
        let output = (input_value & Self::BIT_MASK) >> START_BIT;

        port_collection.set_port_data(self.output_port, output);
    }
}

impl ConstantRegister {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, value: Word, name: String) -> Self {
        let port_id = port_collection
            .borrow_mut()
            .register_port(value, name.clone() + ".out");

        Self {
            constant_value: value,
            output_port: port_id,
            port_collection,
            name,
        }
    }
}

impl Register {
    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        input_port: PortID,
        name: String,
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input: input_port,
            output_port: output_port_id,
            port_collection,
            name,
        }
    }

    pub fn new_with_initial(
        port_collection: Rc<RefCell<PortCollection>>,
        input_port: PortID,
        initial_value: Word,
        name: String,
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(initial_value, name.clone() + ".out");

        Self {
            input: input_port,
            output_port: output_port_id,
            port_collection,
            name,
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let input_value = port_collection.get_port_data(self.input);
        let output_value = port_collection.get_port_data(self.output_port);

        println!(
            "Register '{}': input_value: {},  output_value: {}",
            self.name, input_value, output_value
        );
    }
}

impl GuardedRegister {
    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        input_port: PortID,
        input_enable_port: PortID,
        name: String,
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input_enable: input_enable_port,
            input: input_port,
            output_port: output_port_id,
            port_collection,
            name,
        }
    }

    pub fn new_with_initial(
        port_collection: Rc<RefCell<PortCollection>>,
        input_port: PortID,
        input_enable_port: PortID,
        initial_value: Word,
        name: String,
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(initial_value, name.clone() + ".out");

        Self {
            input_enable: input_enable_port,
            input: input_port,
            output_port: output_port_id,
            port_collection,
            name,
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let input_value = port_collection.get_port_data(self.input);
        let enabled_value = port_collection.get_port_data(self.input_enable);
        let output_value = port_collection.get_port_data(self.output_port);

        println!(
            "Guarded Register '{}': input_value: {}, input_enabled_value: {}, output_value: {}",
            self.name, input_value, enabled_value, output_value
        );
    }
}

impl<const START_BIT: usize, const LEN: usize> BitSelectionRegister<START_BIT, LEN> {
    const BIT_MASK: Word = ((1 << LEN) - 1) << START_BIT;

    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        input_port: PortID,
        name: String,
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input: input_port,
            output_port: output_port_id,
            port_collection,
            name,
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let input_value = port_collection.get_port_data(self.input);
        let output_value = port_collection.get_port_data(self.output_port);

        println!(
            "Register '{}': input_value: {},  output_value: {}",
            self.name, input_value, output_value
        );
    }
}

/*
Memory
 */

pub const MEM_SIZE: usize = 65536;

pub const MEM_LEN_BYTE: Word = 0;
pub const MEM_LEN_SHORT: Word = 1;
pub const MEM_LEN_WORD: Word = 2;

/// Read-only memory consisting of MEM_SIZE individually addressable bytes. The memory takes an
/// address as input, along with a length input that determines the addressing mode (byte, short, or
/// word).
///
/// Reads are performed unconditionally every cycle.
pub struct RMemory {
    pub address_input: PortID,
    pub length_input: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,

    pub content: [u8; MEM_SIZE],
}

/// Read/Write memory consisting of MEM_SIZE individually addressable bytes. Like a read-only memory,
/// the r/w memory takes an address and length input, where the latter determines the type of addressing
/// (byte, short, or word). Additionally, the r/w memory takes a data input and write enable input.
/// If the write enable input is high, the data input is written to the memory at the input address.
///
/// Writes are performed on the first half of the cycle, while reads are performed on the second half.
/// Reads are performed unconditionally every cycle.
pub struct RWMemory {
    pub address_input: PortID,
    pub data_input: PortID,
    pub length_input: PortID,

    pub write_enable_input: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,

    pub content: [u8; MEM_SIZE],
}

fn mem_fetch(mem_content: &[u8; MEM_SIZE], address: Word, length_input: Word) -> Word {
    // NOTE: Make sure the address is valid. Really, there should be some kind of error that occurs
    // when fetching an illegal memory address. However, we perform reads on every cycle, which
    // means there can be garbage in the address without there being anything wrong. Sanitizing
    // the address prevents an index-out-of-bounds error in this case.
    let address = address % (MEM_SIZE as Word);

    if length_input == MEM_LEN_WORD {
        assert_eq!(
            address % 4,
            0,
            "Unaligned memory access for MEM_LEN_WORD! address: {}",
            address
        );

        // Big Endian
        let b_3 = mem_content[address as usize];
        let b_2 = mem_content[(address + 1) as usize];
        let b_1 = mem_content[(address + 2) as usize];
        let b_0 = mem_content[(address + 3) as usize];

        Word::from_be_bytes([b_3, b_2, b_1, b_0])
    } else if length_input == MEM_LEN_SHORT {
        assert_eq!(
            address % 2,
            0,
            "Unaligned memory access for MEM_LEN_SHORT! address: {}",
            address
        );

        // Big Endian
        let b_1 = mem_content[address as usize];
        let b_0 = mem_content[(address + 1) as usize];

        u16::from_be_bytes([b_1, b_0]) as Word
    } else {
        mem_content[address as usize] as Word
    }
}

fn mem_write(mem_content: &mut [u8; MEM_SIZE], address: Word, length_input: Word, data: Word) {
    // Big Endian
    let data_bytes = data.to_be_bytes();

    if length_input == MEM_LEN_WORD {
        assert_eq!(
            address % 4,
            0,
            "Unaligned memory access for MEM_LEN_WORD! address: {}",
            address
        );

        mem_content[address as usize + 0] = data_bytes[0];
        mem_content[address as usize + 1] = data_bytes[1];
        mem_content[address as usize + 2] = data_bytes[2];
        mem_content[address as usize + 3] = data_bytes[3];
    } else if length_input == MEM_LEN_SHORT {
        assert_eq!(
            address % 2,
            0,
            "Unaligned memory access for MEM_LEN_SHORT! address: {}",
            address
        );

        mem_content[address as usize + 0] = data_bytes[0];
        mem_content[address as usize + 1] = data_bytes[1];
    } else {
        mem_content[address as usize] = data_bytes[0];
    }
}

impl Component for RMemory {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        let address = port_collection.get_port_data(self.address_input);
        let length_input = port_collection.get_port_data(self.length_input) & 0b_11;

        let value = mem_fetch(&self.content, address, length_input);
        port_collection.set_port_data(self.output_port, value);
    }
}

impl Component for RWMemory {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        // Reading -- read is performed before write!
        let address = port_collection.get_port_data(self.address_input);
        let length_input = port_collection.get_port_data(self.length_input) & 0b_11;

        let read_value = mem_fetch(&self.content, address, length_input);
        port_collection.set_port_data(self.output_port, read_value);

        // Writing
        let write_enable = port_collection.get_port_data(self.write_enable_input);
        let data = port_collection.get_port_data(self.data_input);

        if write_enable != 0 {
            mem_write(&mut self.content, address, length_input, data);
        }
    }
}

impl RMemory {
    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        address_port: PortID,
        length_port: PortID,
        name: String,
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            address_input: address_port,
            length_input: length_port,
            output_port: output_port_id,
            port_collection,
            name,
            content: [0u8; MEM_SIZE],
        }
    }

    pub fn new_with_mem(
        port_collection: Rc<RefCell<PortCollection>>,
        address_port: PortID,
        length_port: PortID,
        name: String,
        mem: &[u8; MEM_SIZE],
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            address_input: address_port,
            length_input: length_port,
            output_port: output_port_id,
            port_collection,
            name,
            content: mem.clone(),
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let address_value = port_collection.get_port_data(self.address_input);
        let length_value = port_collection.get_port_data(self.length_input) & 1;
        let output_value = port_collection.get_port_data(self.output_port);

        println!(
            "Read-only memory '{}': address_value: {}, length_value: {}, output_value: {:#010X}",
            self.name, address_value, length_value, output_value
        );
    }
}

impl RWMemory {
    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        address_port: PortID,
        length_port: PortID,
        data_port: PortID,
        write_enable_port: PortID,
        name: String,
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            address_input: address_port,
            data_input: data_port,
            length_input: length_port,
            write_enable_input: write_enable_port,
            output_port: output_port_id,
            port_collection,
            name,
            content: [0u8; MEM_SIZE],
        }
    }

    pub fn new_with_mem(
        port_collection: Rc<RefCell<PortCollection>>,
        address_port: PortID,
        length_port: PortID,
        data_port: PortID,
        write_enable_port: PortID,
        name: String,
        mem: &[u8; MEM_SIZE],
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            address_input: address_port,
            data_input: data_port,
            length_input: length_port,
            write_enable_input: write_enable_port,
            output_port: output_port_id,
            port_collection,
            name,
            content: mem.clone(),
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let address_value = port_collection.get_port_data(self.address_input);
        let length_value = port_collection.get_port_data(self.length_input) & 1;
        let write_enable_value = port_collection.get_port_data(self.write_enable_input);
        let data_value = port_collection.get_port_data(self.data_input);

        let output_value = port_collection.get_port_data(self.output_port);

        println!("Read/Write memory '{}': address_value: {}, length_value: {}, output_value: {:#010X}, write_enable_value: {}, data_value: {}",
                 self.name, address_value, length_value, output_value, write_enable_value, data_value);
    }
}

/*
Functional units
 */

/// Simple adder. An adder takes two inputs and provides their sum as output.
pub struct Adder {
    pub input_a: PortID,
    pub input_b: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,
}

/// Simple comparator unit. A comparator takes two numerical inputs, along with a mode input that switches
/// between signed and unsigned comparisons. Two flags are output: whether the inputs are equal,
/// and whether the first input is strictly smaller than the other. All different branch conditions
/// can be built out of these two simple flags.
pub struct Comparator {
    pub input_a: PortID,
    pub input_b: PortID,
    pub input_mode: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,
}

/// Used for testing branch conditions. A branch tester takes the output flags from a comparator unit,
/// along with a function code describing the branch condition to be tested on the comparator output.
///
/// This branch tester only supports the conditions described in the RISC V spec, which are: BEQ,
/// BNE, BLT, BLTU, BGE, BGEU. Other conditions (BLE and BGT) can be simulated by swapping the
/// operands of a BGE or BLT condition respectively.
pub struct BranchTester {
    pub input_comp: PortID,
    pub input_func: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,
}

/// Arithmetic logic unit, used for all arithmetic operations. An ALU takes two numeric inputs, along
/// with a function code describing which operations should be applied to the numeric inputs.
pub struct ALU {
    pub input_a: PortID,
    pub input_b: PortID,
    pub input_func: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,
}

impl Component for Adder {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        let in_a = port_collection.get_port_data(self.input_a);
        let in_b = port_collection.get_port_data(self.input_b);

        port_collection.set_port_data(self.output_port, in_a + in_b);
    }
}

impl Component for Comparator {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        let in_mode = port_collection.get_port_data(self.input_mode) & 0b_01;

        if in_mode == Self::MODE_U {
            // Interpret operands as unsigned
            let in_a = port_collection.get_port_data(self.input_a);
            let in_b = port_collection.get_port_data(self.input_b);

            let eq = in_a == in_b;
            let lt = in_a < in_b;

            let mut flags: Word = 0;
            flags |= if eq { Self::EQ_BIT } else { 0 };
            flags |= if lt { Self::LT_BIT } else { 0 };

            port_collection.set_port_data(self.output_port, flags);
        } else {
            // Interpret operands as signed
            let in_a = port_collection.get_port_data(self.input_a) as SWord;
            let in_b = port_collection.get_port_data(self.input_b) as SWord;

            let eq = in_a == in_b;
            let lt = in_a < in_b;

            let mut flags: Word = 0;
            flags |= if eq { Self::EQ_BIT } else { 0 };
            flags |= if lt { Self::LT_BIT } else { 0 };

            port_collection.set_port_data(self.output_port, flags);
        }
    }
}

impl Component for BranchTester {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        let in_func = port_collection.get_port_data(self.input_func);
        let in_comp = port_collection.get_port_data(self.input_comp);

        let take = match in_func {
            func_code_3::BEQ => in_comp & Comparator::EQ_BIT != 0,
            func_code_3::BNE => in_comp & Comparator::EQ_BIT == 0,
            func_code_3::BLT | func_code_3::BLTU => in_comp & Comparator::LT_BIT != 0,
            func_code_3::BGE | func_code_3::BGEU => in_comp & Comparator::LT_BIT == 0,
            _ => {
                unreachable!("Found invalid func code: {}", in_comp)
            }
        };

        let output = if take {
            Self::BRANCH_TAKE
        } else {
            Self::BRANCH_REJECT
        };
        port_collection.set_port_data(self.output_port, output);
    }
}

impl Component for ALU {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        let in_a = port_collection.get_port_data(self.input_a);
        let in_b = port_collection.get_port_data(self.input_b);

        // Signed versions of inputs
        let in_as = in_a as SWord;
        let in_bs = in_b as SWord;

        let in_func = port_collection.get_port_data(self.input_func);

        let output_value = match in_func {
            Self::OP_ADD => in_a.wrapping_add(in_b),
            Self::OP_SUB => in_a.wrapping_sub(in_b),
            Self::OP_AND => in_a & in_b,
            Self::OP_OR => in_a | in_b,
            Self::OP_XOR => in_a ^ in_b,
            Self::OP_SLT => {
                if in_as < in_bs {
                    1
                } else {
                    0
                }
            }
            Self::OP_SLTU => {
                if in_a < in_b {
                    1
                } else {
                    0
                }
            }
            Self::OP_SLL => in_a << in_b,
            Self::OP_SRL => in_a >> in_b,
            Self::OP_SRA => (in_as >> in_b) as Word,
            Self::OP_MUL => in_a.wrapping_mul(in_b),
            Self::OP_BYPASS_A => in_a,
            Self::OP_BYPASS_B => in_b,
            _ => {
                unreachable!()
            }
        };

        port_collection.set_port_data(self.output_port, output_value);
    }
}

impl Adder {
    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        input_a: PortID,
        input_b: PortID,
        name: String,
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input_a,
            input_b,
            output_port: output_port_id,
            port_collection,
            name,
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let input_a = port_collection.get_port_data(self.input_a);
        let input_b = port_collection.get_port_data(self.input_b);
        let output_value = port_collection.get_port_data(self.output_port);

        println!(
            "Adder '{}': input_a: {}, input_b: {}, output_value: {}",
            self.name, input_a, input_b, output_value
        );
    }
}

impl Comparator {
    // Output flag bits
    const EQ_BIT: Word = 0b_01;
    const LT_BIT: Word = 0b_10;

    // Input mode types
    const MODE_S: Word = 0;
    const MODE_U: Word = 1;

    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        in_a: PortID,
        in_b: PortID,
        in_mode: PortID,
        name: String,
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input_a: in_a,
            input_b: in_b,
            input_mode: in_mode,
            output_port: output_port_id,
            port_collection,
            name,
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let input_a = port_collection.get_port_data(self.input_a);
        let input_b = port_collection.get_port_data(self.input_b);
        let input_mode = port_collection.get_port_data(self.input_mode);
        let output_value = port_collection.get_port_data(self.output_port);

        println!(
            "Comparator '{}': input_a: {}, input_b: {}, input_mode: {}, EQ flag: {}, LT flag: {}",
            self.name,
            input_a,
            input_b,
            input_mode,
            output_value & Self::EQ_BIT != 0,
            output_value & Self::LT_BIT != 0
        );
    }
}

impl BranchTester {
    pub const BRANCH_REJECT: Word = 0;
    pub const BRANCH_TAKE: Word = 1;

    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        in_comp: PortID,
        in_func: PortID,
        name: String,
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input_comp: in_comp,
            input_func: in_func,
            output_port: output_port_id,
            port_collection,
            name,
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let in_comp = port_collection.get_port_data(self.input_comp);
        let in_func = port_collection.get_port_data(self.input_func) & 7;
        let output_value = port_collection.get_port_data(self.output_port);

        println!(
            "BranchTester '{}': in_EQ: {}, in_LT: {}, in_func: {}, out: {}",
            self.name,
            in_comp & Comparator::EQ_BIT != 0,
            in_comp & Comparator::LT_BIT != 0,
            in_func,
            output_value
        );
    }
}

impl ALU {
    pub const OP_ADD: Word = 0;
    pub const OP_SUB: Word = 1;

    pub const OP_SLT: Word = 2;
    pub const OP_SLTU: Word = 3;

    pub const OP_AND: Word = 4;
    pub const OP_OR: Word = 5;
    pub const OP_XOR: Word = 6;

    pub const OP_SLL: Word = 7;
    pub const OP_SRL: Word = 8;
    pub const OP_SRA: Word = 9;

    pub const OP_MUL: Word = 10;
    // pub const OP_MULH: Word = 11;
    // pub const OP_MULHS: Word = 12;
    // pub const OP_MULHSU: Word = 13;

    pub const OP_BYPASS_A: Word = 11;
    pub const OP_BYPASS_B: Word = 12;

    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        in_a: PortID,
        in_b: PortID,
        in_func: PortID,
        name: String,
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input_a: in_a,
            input_b: in_b,
            input_func: in_func,
            output_port: output_port_id,
            port_collection,
            name,
        }
    }
}

/*
Switching
 */

/// A multiplexer that switches between NUM_INPUTS inputs, based on a secondary selection input. Let
/// *i* be the number that is encoded by the selection input, then the MUX outputs the value of the
/// *i^th* input.
pub struct Mux<const NUM_INPUTS: usize> {
    pub selection_input: PortID,
    pub inputs: [PortID; NUM_INPUTS],
    pub output_port: PortID,
    pub input_mask: Word,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,
}

/// A de-multiplexer is the opposite of a multiplexer. It takes a single input, and forwards it to
/// one of NUM_OUTPUTS output channels, while forwarding *0* to the rest. A secondary selection input
/// is used switch between outputs. Let *i* be the number that is encoded by the selection input,
/// then the DeMUX forwards the input to the *i^th* output.
pub struct DeMux<const NUM_OUTPUTS: usize> {
    pub selection_input: PortID,
    pub input: PortID,
    pub outputs: [PortID; NUM_OUTPUTS],
    pub input_mask: Word,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,
}

impl<const NUM_INPUTS: usize> Component for Mux<NUM_INPUTS> {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        let selected_input = port_collection.get_port_data(self.selection_input) & self.input_mask;
        assert!((selected_input as usize) < NUM_INPUTS);

        let input_value = port_collection.get_port_data(self.inputs[selected_input as usize]);

        port_collection.set_port_data(self.output_port, input_value);
    }
}

impl<const NUM_OUTPUTS: usize> Component for DeMux<NUM_OUTPUTS> {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        let selected_output = port_collection.get_port_data(self.selection_input) & self.input_mask;
        assert!((selected_output as usize) < NUM_OUTPUTS);

        let input_value = port_collection.get_port_data(self.input);

        for i in 0..NUM_OUTPUTS {
            if i == selected_output as usize {
                port_collection.set_port_data(self.outputs[i], input_value);
            } else {
                port_collection.set_port_data(self.outputs[i], PORT_DEFAULT_VALUE);
            }
        }
    }
}

impl<const NUM_INPUTS: usize> Mux<NUM_INPUTS> {
    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        inputs: &[PortID; NUM_INPUTS],
        selection_input: PortID,
        name: String,
    ) -> Self {
        let output_port_id = port_collection
            .borrow_mut()
            .register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        let num_selection_bits = (NUM_INPUTS as f64).log2().ceil() as u32;
        let input_mask = (2_u32.pow(num_selection_bits) as Word) - 1;

        Self {
            selection_input,
            inputs: inputs.clone(),
            output_port: output_port_id,
            input_mask,
            port_collection,
            name,
        }
    }
}

impl<const NUM_OUTPUTS: usize> DeMux<NUM_OUTPUTS> {
    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        input: PortID,
        selection_input: PortID,
        name: String,
    ) -> Self {
        let mut output_ports = [PORT_NULL_ID; NUM_OUTPUTS];

        for i in 0..NUM_OUTPUTS {
            output_ports[i] = port_collection
                .borrow_mut()
                .register_port(PORT_DEFAULT_VALUE, format!("{}.out_{}", name, i));
        }

        let num_selection_bits = (NUM_OUTPUTS as f64).log2().ceil() as u32;
        let input_mask = (2_u32.pow(num_selection_bits) as Word) - 1;

        Self {
            selection_input,
            input,
            outputs: output_ports,
            input_mask,
            port_collection,
            name,
        }
    }
}

/*
Register File
 */

/// A collection of NUM_REGISTERS register, with provided switching hardware for reading from/
/// writing to specific registers. This specific design of register file has two inputs for reading,
/// such that two registers can be read every cycle. It also has three inputs for writing:
/// (1) a data input,
/// (2) a selection input, and
/// (3) a write enable input.
/// When the write enable input is high, the data input is written to the register selected by the
/// selection input, allowing a single write per cycle.
///
/// Finally, writes are performed on the first half of the cycle, while reads are performed on the
/// second half of the cycle. Consequently, written values are available for reads on the same cycle.
pub struct RegisterFile<const NUM_REGISTERS: usize> {
    pub registers: [GuardedRegister; NUM_REGISTERS],

    // Mux for switching output values a and b
    pub mux_out_a: Mux<NUM_REGISTERS>,
    pub mux_out_b: Mux<NUM_REGISTERS>,

    // Demux for writes to a specific register
    pub de_mux_in: DeMux<NUM_REGISTERS>,

    pub in_read_a: PortID,
    pub in_read_b: PortID,

    pub in_write_data: PortID,
    pub in_write_select: PortID,
    pub in_write_enable: PortID,

    pub out_a: PortID,
    pub out_b: PortID,
}

impl<const NUM_REGISTERS: usize> Component for RegisterFile<NUM_REGISTERS> {
    fn process_cycle(&mut self) {
        self.de_mux_in.process_cycle();

        for reg in self.registers.iter_mut() {
            reg.process_cycle();
        }

        self.mux_out_a.process_cycle();
        self.mux_out_b.process_cycle();
    }
}

impl<const NUM_REGISTERS: usize> RegisterFile<NUM_REGISTERS> {
    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        read_a: PortID,
        read_b: PortID,
        write_data: PortID,
        write_select: PortID,
        write_enable: PortID,
    ) -> Self {
        // Demux for propagating input enable signal to the gate of the proper register
        let de_mux_in = DeMux::<NUM_REGISTERS>::new(
            port_collection.clone(),
            write_enable,
            write_select,
            String::from("rf_demux_in"),
        );

        // Issue relating to the initialization of arrays with const generic size
        // https://github.com/rust-lang/rust/issues/61956

        // Register arrays starts uninitialized because the size is const generic
        let mut registers_uninit: [MaybeUninit<GuardedRegister>; NUM_REGISTERS] =
            unsafe { MaybeUninit::uninit().assume_init() };

        for i in 0..NUM_REGISTERS {
            let reg = GuardedRegister::new(
                port_collection.clone(),
                write_data,
                de_mux_in.outputs[i],
                String::from(format!("rf_reg_{}", i)),
            );

            registers_uninit[i].write(reg);
        }

        // Gruesome hack to initialize const generic size array
        // More info: https://github.com/rust-lang/rust/issues/61956
        let ptr = &mut registers_uninit as *mut _ as *mut [GuardedRegister; NUM_REGISTERS];
        let registers = unsafe { ptr.read() };
        core::mem::forget(registers_uninit);

        // Create array of all register output ports
        let mut register_outputs: [PortID; NUM_REGISTERS] = [PORT_NULL_ID; NUM_REGISTERS];
        for i in 0..NUM_REGISTERS {
            register_outputs[i] = registers[i].output_port;
        }

        let mux_out_a = Mux::<NUM_REGISTERS>::new(
            port_collection.clone(),
            &register_outputs,
            read_a,
            String::from("rf_mux_out_a"),
        );

        let mux_out_b = Mux::<NUM_REGISTERS>::new(
            port_collection.clone(),
            &register_outputs,
            read_b,
            String::from("rf_mux_out_b"),
        );

        let out_a = mux_out_a.output_port;
        let out_b = mux_out_b.output_port;

        Self {
            registers,
            mux_out_a,
            mux_out_b,
            de_mux_in,
            in_read_a: read_a,
            in_read_b: read_b,
            in_write_data: write_data,
            in_write_select: write_select,
            in_write_enable: write_enable,
            out_a,
            out_b,
        }
    }

    pub fn set_write_inputs(
        &mut self,
        write_data: PortID,
        write_select: PortID,
        write_enable: PortID,
    ) {
        self.de_mux_in.selection_input = write_select;
        self.de_mux_in.input = write_enable;

        for register in &mut self.registers {
            register.input = write_data;
        }
    }
}

/*
Misc
 */

/// Used for extracting and sign-extending immediate values directly from instructions. Since the
/// instruction is not yet decoded, values for all different immediate encodings are provided in
/// the output.
///
/// TODO: It would be possible to determine which immediate type is required by simply looking at the opcode of an instruction, which can be done in ID.
pub struct ImmSignExtender {
    pub input: PortID,
    pub out_i_type: PortID,
    pub out_s_type: PortID,
    pub out_b_type: PortID,
    pub out_u_type: PortID,
    pub out_j_type: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,
}

/// Used for detecting Read-after-Write hazards in the pipeline. If either the EX stage, or MEM stage
/// is busy producing a value that is needed by the instruction currently in the ID stage, the CPU
/// must stall the IF and ID stages until the value reaches the WB stage.
///
/// The interlock unit takes the instruction words that are going to be processed in the the ID, EX,
/// and MEM stages in the coming cycle. It outputs the inverted version of a stall flag, signalling
/// that the IF and ID stages must stall. This inversion is chosen such that the output can be used
/// as an input to guarded pipeline registers directly.
///
/// Internally, the unit uses a timer that keeps track of how many cycles a stall should take, and
/// when it is cleared.
pub struct InterlockUnit {
    pub in_id_instr: PortID,
    pub in_ex_instr: PortID,
    pub in_mem_instr: PortID,

    pub out_not_stall: PortID,
    pub stall_timer: Word,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,
}

impl Component for ImmSignExtender {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        let input_value = port_collection.get_port_data(self.input);

        let imm_i_type = extract_imm_i_type(input_value);
        let imm_s_type = extract_imm_s_type(input_value);
        let imm_b_type = extract_imm_b_type(input_value);
        let imm_u_type = extract_imm_u_type(input_value);
        let imm_j_type = extract_imm_j_type(input_value);

        port_collection.set_port_data(self.out_i_type, imm_i_type);
        port_collection.set_port_data(self.out_s_type, imm_s_type);
        port_collection.set_port_data(self.out_b_type, imm_b_type);
        port_collection.set_port_data(self.out_u_type, imm_u_type);
        port_collection.set_port_data(self.out_j_type, imm_j_type);
    }
}

/// Checks whether a RaW hazard exists between two given instructions.
fn check_raw_hazard(id_instr: Word, other_instr: Word) -> bool {
    let op_code_id = extract_op_code(id_instr);
    let op_code_other = extract_op_code(other_instr);

    let conflict = match op_code_id {
        op_code::OP | op_code::BRANCH | op_code::STORE => {
            // ID reads two registers
            let rs1 = extract_rs1(id_instr);
            let rs2 = extract_rs2(id_instr);

            match op_code_other {
                op_code::OP
                | op_code::OP_IMM
                | op_code::LOAD
                | op_code::JAL
                | op_code::JALR
                | op_code::LUI
                | op_code::AUIPC => {
                    // Other instruction writes to register
                    let rd = extract_rd(other_instr);
                    // NOTE: No RaW conflict can exist on register 0
                    (rd != 0) && (rd == rs1 || rd == rs2)
                }
                _ => false,
            }
        }
        op_code::OP_IMM | op_code::JALR => {
            // ID reads one register
            let rs1 = extract_rs1(id_instr);

            match op_code_other {
                op_code::OP
                | op_code::OP_IMM
                | op_code::LOAD
                | op_code::JAL
                | op_code::JALR
                | op_code::LUI
                | op_code::AUIPC => {
                    // Other instruction writes to register
                    let rd = extract_rd(other_instr);
                    // NOTE: No RaW conflict can exist on register 0
                    (rd != 0) && (rd == rs1)
                }
                _ => false,
            }
        }
        _ => {
            // No read is performed
            false
        }
    };

    return conflict;
}

impl Component for InterlockUnit {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        let id_instr = port_collection.get_port_data(self.in_id_instr);
        let ex_instr = port_collection.get_port_data(self.in_ex_instr);
        let mem_instr = port_collection.get_port_data(self.in_mem_instr);

        // NOTE: The stall timer is used to keep track of how many cycles the CPU should stall for,
        // as well as to guard against false hazards generated as a consequence of stalling. When
        // the CPU detects a hazard, and an instruction is stalled in the ID stage (so before being
        // issued), the previously issued instruction is repeated in the EX stage as long as the
        // stalled instruction is not ready to be issued. These repeated 'phantom' instructions can
        // cause new hazards for future instructions, though their result is already in the register
        // file.
        //
        // To prevent stalling in this case, the stall_timer keeps track of where the phantom
        // instructions are. When the timer is 2, there are still phantom instructions in the EX and
        // MEM stages. When the timer hits 1, there is still a phantom instruction in the MEM stage.

        if self.stall_timer == 0 {
            if check_raw_hazard(id_instr, ex_instr) {
                // Must stall for 2 cycles
                self.stall_timer = 3;
            } else if check_raw_hazard(id_instr, mem_instr) {
                // Only stall for a single cycle
                self.stall_timer = 2
            }
        } else if self.stall_timer == 1 {
            if check_raw_hazard(id_instr, ex_instr) {
                // Must stall for 2 cycles
                self.stall_timer = 3;
            } else {
                self.stall_timer = self.stall_timer - 1;
            }
        } else {
            self.stall_timer = self.stall_timer - 1;
        }

        let not_stall = !(self.stall_timer >= 2);
        port_collection.set_port_data(self.out_not_stall, not_stall.into());
    }
}

impl ImmSignExtender {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, input: PortID, name: String) -> Self {
        let out_i_type = port_collection.borrow_mut().register_port(
            PORT_DEFAULT_VALUE,
            format!("{}_{}", name, String::from("out_i")),
        );
        let out_s_type = port_collection.borrow_mut().register_port(
            PORT_DEFAULT_VALUE,
            format!("{}_{}", name, String::from("out_s")),
        );
        let out_b_type = port_collection.borrow_mut().register_port(
            PORT_DEFAULT_VALUE,
            format!("{}_{}", name, String::from("out_b")),
        );
        let out_u_type = port_collection.borrow_mut().register_port(
            PORT_DEFAULT_VALUE,
            format!("{}_{}", name, String::from("out_u")),
        );
        let out_j_type = port_collection.borrow_mut().register_port(
            PORT_DEFAULT_VALUE,
            format!("{}_{}", name, String::from("out_j")),
        );

        Self {
            input,
            out_i_type,
            out_s_type,
            out_b_type,
            out_u_type,
            out_j_type,
            port_collection,
            name,
        }
    }
}

impl InterlockUnit {
    pub fn new(
        port_collection: Rc<RefCell<PortCollection>>,
        in_id_instr: PortID,
        in_ex_instr: PortID,
        in_mem_instr: PortID,
        name: String,
    ) -> Self {
        let out_not_stall = port_collection.borrow_mut().register_port(
            PORT_DEFAULT_VALUE,
            format!("{}_{}", name, String::from("not_stall")),
        );

        Self {
            in_id_instr,
            in_ex_instr,
            in_mem_instr,
            out_not_stall,
            stall_timer: 0,
            port_collection,
            name,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    mod registers {
        use super::*;

        #[test]
        ///
        /// Tests a constant register, normal register, and guarded register (a, b, c) in series.
        /// One extra constant register (e) is added for the enable line to the guarded register.
        ///
        fn test_registers() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let a_value: Word = 88;
            let e_value: Word = 0;

            let mut a =
                ConstantRegister::new(Rc::clone(&port_collection), a_value, String::from("a"));
            let mut b = Register::new(
                Rc::clone(&port_collection),
                a.output_port,
                String::from("b"),
            );

            let mut e =
                ConstantRegister::new(Rc::clone(&port_collection), e_value, String::from("e"));
            let mut c = GuardedRegister::new(
                Rc::clone(&port_collection),
                b.output_port,
                e.output_port,
                String::from("c"),
            );

            a.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(a.output_port), a_value);

                assert_eq!(port_collection.get_port_data(b.input), a_value);
                assert_eq!(
                    port_collection.get_port_data(b.output_port),
                    PORT_DEFAULT_VALUE
                );

                assert_eq!(port_collection.get_port_data(c.input), PORT_DEFAULT_VALUE);
                assert_eq!(
                    port_collection.get_port_data(c.input_enable),
                    PORT_DEFAULT_VALUE
                );
                assert_eq!(
                    port_collection.get_port_data(c.output_port),
                    PORT_DEFAULT_VALUE
                );

                assert_eq!(port_collection.get_port_data(e.output_port), e_value);
            }

            b.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(a.output_port), a_value);

                assert_eq!(port_collection.get_port_data(b.input), a_value);
                assert_eq!(port_collection.get_port_data(b.output_port), a_value);

                assert_eq!(port_collection.get_port_data(c.input), a_value);
                assert_eq!(
                    port_collection.get_port_data(c.input_enable),
                    PORT_DEFAULT_VALUE
                );
                assert_eq!(
                    port_collection.get_port_data(c.output_port),
                    PORT_DEFAULT_VALUE
                );

                assert_eq!(port_collection.get_port_data(e.output_port), e_value);
            }

            c.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(a.output_port), a_value);

                assert_eq!(port_collection.get_port_data(b.input), a_value);
                assert_eq!(port_collection.get_port_data(b.output_port), a_value);

                assert_eq!(port_collection.get_port_data(c.input), a_value);
                assert_eq!(port_collection.get_port_data(c.input_enable), e_value);
                assert_eq!(
                    port_collection.get_port_data(c.output_port),
                    PORT_DEFAULT_VALUE
                );

                assert_eq!(port_collection.get_port_data(e.output_port), e_value);
            }

            let e_value = 1;
            e.constant_value = 1;
            e.process_cycle();
            c.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(a.output_port), a_value);

                assert_eq!(port_collection.get_port_data(b.input), a_value);
                assert_eq!(port_collection.get_port_data(b.output_port), a_value);

                assert_eq!(port_collection.get_port_data(c.input), a_value);
                assert_eq!(port_collection.get_port_data(c.input_enable), e_value);
                assert_eq!(port_collection.get_port_data(c.output_port), a_value);

                assert_eq!(port_collection.get_port_data(e.output_port), e_value);
            }
        }

        #[test]
        /// Tests whether a bit selection register correctly selects and shifts bits from its input.
        pub fn test_bit_selection_register() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            const CONST_VALUE: Word = 0b_101101;
            const NUM_VAL_BITS: usize = 6;
            const OFFSET: usize = 5;

            let mut c = ConstantRegister::new(
                Rc::clone(&port_collection),
                CONST_VALUE << OFFSET,
                String::from("c"),
            );
            let mut s = BitSelectionRegister::<OFFSET, NUM_VAL_BITS>::new(
                Rc::clone(&port_collection),
                c.output_port,
                String::from("c"),
            );

            c.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();
                assert_eq!(
                    PORT_DEFAULT_VALUE,
                    port_collection.get_port_data(s.output_port)
                );
            }

            s.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();
                assert_eq!(CONST_VALUE, port_collection.get_port_data(s.output_port));
            }
        }
    }

    mod memories {
        use super::*;

        #[test]
        /// Tests length modes and valid return values of an isolated read-only memory.
        fn test_read_only_memory() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let mut mem: [u8; MEM_SIZE] = [0u8; MEM_SIZE];
            mem[0] = 0x0A;
            mem[1] = 0x0B;
            mem[2] = 0x0C;
            mem[3] = 0x0D;
            mem[4] = 0x02;
            mem[5] = 0x04;
            mem[6] = 0x06;
            mem[7] = 0x08;

            let address = 2;
            let len_mode = MEM_LEN_BYTE;

            let mut reg_addr =
                ConstantRegister::new(port_collection.clone(), address, String::from("reg_addr"));
            let mut reg_len =
                ConstantRegister::new(port_collection.clone(), len_mode, String::from("reg_len"));

            let mut memory = RMemory::new_with_mem(
                port_collection.clone(),
                reg_addr.output_port,
                reg_len.output_port,
                String::from("m"),
                &mem,
            );

            reg_addr.process_cycle();
            reg_len.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(memory.address_input), address);
                assert_eq!(port_collection.get_port_data(memory.length_input), len_mode);
                assert_eq!(
                    port_collection.get_port_data(memory.output_port),
                    PORT_DEFAULT_VALUE
                );
            }

            memory.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(memory.address_input), address);
                assert_eq!(port_collection.get_port_data(memory.length_input), len_mode);
                assert_eq!(port_collection.get_port_data(memory.output_port), 0x000C);
            }

            let len_mode = MEM_LEN_SHORT;
            reg_len.constant_value = len_mode;

            reg_len.process_cycle();
            memory.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(memory.address_input), address);
                assert_eq!(port_collection.get_port_data(memory.length_input), len_mode);
                assert_eq!(port_collection.get_port_data(memory.output_port), 0x0C0D);
            }

            let len_mode = MEM_LEN_WORD;
            let address = 4;

            reg_len.constant_value = len_mode;
            reg_addr.constant_value = address;

            reg_len.process_cycle();
            reg_addr.process_cycle();
            memory.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(memory.address_input), address);
                assert_eq!(port_collection.get_port_data(memory.length_input), len_mode);
                assert_eq!(
                    port_collection.get_port_data(memory.output_port),
                    0x02040608
                );
            }
        }

        #[test]
        #[should_panic]
        /// Tests whether miss-aligned short memory access throws a panic.
        fn test_read_only_memory_short_unaligned() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let address = 3;
            let len_mode = MEM_LEN_SHORT;

            let mut reg_addr =
                ConstantRegister::new(port_collection.clone(), address, String::from("reg_addr"));
            let mut reg_len =
                ConstantRegister::new(port_collection.clone(), len_mode, String::from("reg_len"));

            let mut memory = RMemory::new(
                port_collection.clone(),
                reg_addr.output_port,
                reg_len.output_port,
                String::from("m"),
            );

            reg_addr.process_cycle();
            reg_len.process_cycle();
            memory.process_cycle();
        }

        #[test]
        #[should_panic]
        /// Tests whether miss-aligned word memory access throws a panic.
        fn test_read_only_memory_word_unaligned() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let address = 3;
            let len_mode = MEM_LEN_WORD;

            let mut reg_addr =
                ConstantRegister::new(port_collection.clone(), address, String::from("reg_addr"));
            let mut reg_len =
                ConstantRegister::new(port_collection.clone(), len_mode, String::from("reg_len"));

            let mut memory = RMemory::new(
                port_collection.clone(),
                reg_addr.output_port,
                reg_len.output_port,
                String::from("m"),
            );

            reg_addr.process_cycle();
            reg_len.process_cycle();
            memory.process_cycle();
        }

        #[test]
        pub fn test_read_write_memory() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let mut mem: [u8; MEM_SIZE] = [0u8; MEM_SIZE];
            mem[0] = 0x0A;
            mem[1] = 0x0B;
            mem[2] = 0x0C;
            mem[3] = 0x0D;
            mem[4] = 0x02;
            mem[5] = 0x04;
            mem[6] = 0x06;
            mem[7] = 0x08;

            let address = 4;
            let data = 0x12344321;
            let len_mode = MEM_LEN_WORD;
            let write_enable = 1;

            let mut reg_addr =
                ConstantRegister::new(port_collection.clone(), address, String::from("reg_addr"));
            let mut reg_len =
                ConstantRegister::new(port_collection.clone(), len_mode, String::from("reg_len"));

            let mut reg_data =
                ConstantRegister::new(port_collection.clone(), data, String::from("reg_data"));
            let mut reg_w_enable = ConstantRegister::new(
                port_collection.clone(),
                write_enable,
                String::from("reg_w_enable"),
            );

            let mut mem = RWMemory::new_with_mem(
                port_collection.clone(),
                reg_addr.output_port,
                reg_len.output_port,
                reg_data.output_port,
                reg_w_enable.output_port,
                String::from("mem"),
                &mem,
            );

            reg_addr.process_cycle();
            reg_len.process_cycle();
            reg_data.process_cycle();
            reg_w_enable.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(mem.length_input), len_mode);
                assert_eq!(port_collection.get_port_data(mem.address_input), address);
                assert_eq!(port_collection.get_port_data(mem.data_input), data);
                assert_eq!(
                    port_collection.get_port_data(mem.write_enable_input),
                    write_enable
                );
                assert_eq!(
                    port_collection.get_port_data(mem.output_port),
                    PORT_DEFAULT_VALUE
                );
            }

            mem.process_cycle();

            {
                // Read should be the initial value
                let port_collection = port_collection.borrow_mut();
                assert_eq!(port_collection.get_port_data(mem.output_port), 0x02040608);

                // Test Big Endianness
                let data_bytes = data.to_be_bytes();
                assert_eq!(mem.content[address as usize + 0], data_bytes[0]);
                assert_eq!(mem.content[address as usize + 1], data_bytes[1]);
                assert_eq!(mem.content[address as usize + 2], data_bytes[2]);
                assert_eq!(mem.content[address as usize + 3], data_bytes[3]);
            }

            mem.process_cycle();

            {
                // Read should be the previously written value
                let port_collection = port_collection.borrow_mut();
                assert_eq!(port_collection.get_port_data(mem.output_port), data);
            }

            // Test write enable

            let new_data = 0x11223344;
            let write_enable = 0;

            reg_data.constant_value = new_data;
            reg_w_enable.constant_value = write_enable;

            reg_data.process_cycle();
            reg_w_enable.process_cycle();

            mem.process_cycle();
            mem.process_cycle();

            {
                // Read should be the previously written value
                let port_collection = port_collection.borrow_mut();
                assert_eq!(port_collection.get_port_data(mem.output_port), data);
            }
        }
    }

    mod functional_units {
        use super::*;

        #[test]
        fn test_adder() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let in_a = 8;
            let in_b = 80;

            let mut reg_a =
                ConstantRegister::new(port_collection.clone(), in_a, String::from("reg_a"));
            let mut reg_b =
                ConstantRegister::new(port_collection.clone(), in_b, String::from("reg_b"));

            let mut adder = Adder::new(
                port_collection.clone(),
                reg_a.output_port,
                reg_b.output_port,
                String::from("add"),
            );

            reg_a.process_cycle();
            reg_b.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(adder.input_a), in_a);
                assert_eq!(port_collection.get_port_data(adder.input_b), in_b);
                assert_eq!(
                    port_collection.get_port_data(adder.output_port),
                    PORT_DEFAULT_VALUE
                );
            }

            adder.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(adder.input_a), in_a);
                assert_eq!(port_collection.get_port_data(adder.input_b), in_b);
                assert_eq!(
                    port_collection.get_port_data(adder.output_port),
                    in_a + in_b
                );
            }
        }

        #[test]
        pub fn test_comparator_unsigned() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            // Test LT

            let in_a = 8;
            let in_b = 80;
            let in_mode = Comparator::MODE_U;

            let mut reg_a =
                ConstantRegister::new(port_collection.clone(), in_a, String::from("reg_a"));
            let mut reg_b =
                ConstantRegister::new(port_collection.clone(), in_b, String::from("reg_b"));
            let reg_mode =
                ConstantRegister::new(port_collection.clone(), in_mode, String::from("reg_mode"));

            let mut comp = Comparator::new(
                port_collection.clone(),
                reg_a.output_port,
                reg_b.output_port,
                reg_mode.output_port,
                String::from("comp"),
            );

            reg_a.process_cycle();
            reg_b.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(comp.input_a), in_a);
                assert_eq!(port_collection.get_port_data(comp.input_b), in_b);
                assert_eq!(port_collection.get_port_data(comp.input_mode), in_mode);
                assert_eq!(
                    port_collection.get_port_data(comp.output_port),
                    PORT_DEFAULT_VALUE
                );
            }

            comp.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(comp.input_a), in_a);
                assert_eq!(port_collection.get_port_data(comp.input_b), in_b);
                assert_eq!(port_collection.get_port_data(comp.input_mode), in_mode);
                assert_eq!(
                    port_collection.get_port_data(comp.output_port),
                    Comparator::LT_BIT
                );
            }

            // Test EQ

            let in_a = 80;
            let in_b = 80;

            reg_a.constant_value = in_a;
            reg_b.constant_value = in_b;

            reg_a.process_cycle();
            reg_b.process_cycle();
            comp.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(comp.input_a), in_a);
                assert_eq!(port_collection.get_port_data(comp.input_b), in_b);
                assert_eq!(port_collection.get_port_data(comp.input_mode), in_mode);
                assert_eq!(
                    port_collection.get_port_data(comp.output_port),
                    Comparator::EQ_BIT
                );
            }

            // Test GT

            let in_a = 88;
            let in_b = 80;

            reg_a.constant_value = in_a;
            reg_b.constant_value = in_b;

            reg_a.process_cycle();
            reg_b.process_cycle();
            comp.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(comp.input_a), in_a);
                assert_eq!(port_collection.get_port_data(comp.input_b), in_b);
                assert_eq!(port_collection.get_port_data(comp.input_mode), in_mode);
                assert_eq!(port_collection.get_port_data(comp.output_port), 0);
            }
        }

        #[test]
        pub fn test_comparator_signed() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            // Test LT

            let in_a: SWord = -88;
            let in_b: SWord = 80;
            let in_mode = Comparator::MODE_S;

            let mut reg_a =
                ConstantRegister::new(port_collection.clone(), in_a as Word, String::from("reg_a"));
            let mut reg_b =
                ConstantRegister::new(port_collection.clone(), in_b as Word, String::from("reg_b"));
            let reg_mode =
                ConstantRegister::new(port_collection.clone(), in_mode, String::from("reg_mode"));

            let mut comp = Comparator::new(
                port_collection.clone(),
                reg_a.output_port,
                reg_b.output_port,
                reg_mode.output_port,
                String::from("comp"),
            );

            reg_a.process_cycle();
            reg_b.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(comp.input_a), in_a as Word);
                assert_eq!(port_collection.get_port_data(comp.input_b), in_b as Word);
                assert_eq!(port_collection.get_port_data(comp.input_mode), in_mode);
                assert_eq!(
                    port_collection.get_port_data(comp.output_port),
                    PORT_DEFAULT_VALUE
                );
            }

            comp.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(comp.input_a), in_a as Word);
                assert_eq!(port_collection.get_port_data(comp.input_b), in_b as Word);
                assert_eq!(port_collection.get_port_data(comp.input_mode), in_mode);
                assert_eq!(
                    port_collection.get_port_data(comp.output_port),
                    Comparator::LT_BIT
                );
            }

            // Test EQ

            let in_a: SWord = -80;
            let in_b: SWord = -80;

            reg_a.constant_value = in_a as Word;
            reg_b.constant_value = in_b as Word;

            reg_a.process_cycle();
            reg_b.process_cycle();
            comp.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(comp.input_a), in_a as Word);
                assert_eq!(port_collection.get_port_data(comp.input_b), in_b as Word);
                assert_eq!(port_collection.get_port_data(comp.input_mode), in_mode);
                assert_eq!(
                    port_collection.get_port_data(comp.output_port),
                    Comparator::EQ_BIT
                );
            }

            // Test GT

            let in_a: SWord = 8;
            let in_b: SWord = -80;

            reg_a.constant_value = in_a as Word;
            reg_b.constant_value = in_b as Word;

            reg_a.process_cycle();
            reg_b.process_cycle();
            comp.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(comp.input_a), in_a as Word);
                assert_eq!(port_collection.get_port_data(comp.input_b), in_b as Word);
                assert_eq!(port_collection.get_port_data(comp.input_mode), in_mode);
                assert_eq!(port_collection.get_port_data(comp.output_port), 0);
            }
        }

        #[test]
        pub fn test_branch_tester_signed() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let mut reg_a = ConstantRegister::new(
                port_collection.clone(),
                PORT_DEFAULT_VALUE,
                String::from("reg_a"),
            );
            let mut reg_b = ConstantRegister::new(
                port_collection.clone(),
                PORT_DEFAULT_VALUE,
                String::from("reg_b"),
            );
            let mut reg_func = ConstantRegister::new(
                port_collection.clone(),
                PORT_DEFAULT_VALUE,
                String::from("reg_func"),
            );
            let mut reg_mode = BitSelectionRegister::<2, 1>::new(
                port_collection.clone(),
                reg_func.output_port,
                String::from("reg_mode"),
            );

            let mut comp = Comparator::new(
                port_collection.clone(),
                reg_a.output_port,
                reg_b.output_port,
                reg_mode.output_port,
                String::from("comp"),
            );
            let mut bt = BranchTester::new(
                port_collection.clone(),
                comp.output_port,
                reg_func.output_port,
                String::from("bt"),
            );

            let a_b_pairs = [(-80, 80), (-8, -8), (80, -80)];

            let branch_matrix = [
                // BEQ, BNE, BLT, BGE
                [0, 1, 1, 0],
                [1, 0, 0, 1],
                [0, 1, 0, 1],
            ];

            // No unsigned versions
            let func_codes = [
                func_code_3::BEQ,
                func_code_3::BNE,
                func_code_3::BLT,
                func_code_3::BGE,
            ];

            for i in 0..a_b_pairs.len() {
                for j in 0..func_codes.len() {
                    let (in_a, in_b): (SWord, SWord) = a_b_pairs[i];
                    let in_func = func_codes[j];
                    let expectation = branch_matrix[i][j];

                    reg_a.constant_value = in_a as Word;
                    reg_b.constant_value = in_b as Word;
                    reg_func.constant_value = in_func;

                    reg_a.process_cycle();
                    reg_b.process_cycle();
                    reg_func.process_cycle();
                    reg_mode.process_cycle();
                    comp.process_cycle();
                    bt.process_cycle();

                    let port_collection = port_collection.borrow_mut();
                    assert_eq!(port_collection.get_port_data(bt.output_port), expectation);
                }
            }
        }

        #[test]
        pub fn test_branch_tester_unsigned() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let mut reg_a = ConstantRegister::new(
                port_collection.clone(),
                PORT_DEFAULT_VALUE,
                String::from("reg_a"),
            );
            let mut reg_b = ConstantRegister::new(
                port_collection.clone(),
                PORT_DEFAULT_VALUE,
                String::from("reg_b"),
            );
            let mut reg_func = ConstantRegister::new(
                port_collection.clone(),
                PORT_DEFAULT_VALUE,
                String::from("reg_func"),
            );
            let mut reg_mode = BitSelectionRegister::<2, 1>::new(
                port_collection.clone(),
                reg_func.output_port,
                String::from("reg_mode"),
            );

            let mut comp = Comparator::new(
                port_collection.clone(),
                reg_a.output_port,
                reg_b.output_port,
                reg_mode.output_port,
                String::from("comp"),
            );
            let mut bt = BranchTester::new(
                port_collection.clone(),
                comp.output_port,
                reg_func.output_port,
                String::from("bt"),
            );

            let a_b_pairs = [(80, 88), (88, 88), (88, 80)];

            let branch_matrix = [
                // BEQ, BNE, BLTU, BGEU
                [0, 1, 1, 0],
                [1, 0, 0, 1],
                [0, 1, 0, 1],
            ];

            // No signed versions
            let func_codes = [
                func_code_3::BEQ,
                func_code_3::BNE,
                func_code_3::BLTU,
                func_code_3::BGEU,
            ];

            for i in 0..a_b_pairs.len() {
                for j in 0..func_codes.len() {
                    let (in_a, in_b): (SWord, SWord) = a_b_pairs[i];
                    let in_func = func_codes[j];
                    let expectation = branch_matrix[i][j];

                    reg_a.constant_value = in_a as Word;
                    reg_b.constant_value = in_b as Word;
                    reg_func.constant_value = in_func;

                    reg_a.process_cycle();
                    reg_b.process_cycle();
                    reg_func.process_cycle();
                    reg_mode.process_cycle();
                    comp.process_cycle();
                    bt.process_cycle();

                    let port_collection = port_collection.borrow_mut();
                    assert_eq!(port_collection.get_port_data(bt.output_port), expectation);
                }
            }
        }

        #[test]
        pub fn test_alu_signed() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let mut reg_a = ConstantRegister::new(
                port_collection.clone(),
                PORT_DEFAULT_VALUE,
                String::from("reg_a"),
            );
            let mut reg_b = ConstantRegister::new(
                port_collection.clone(),
                PORT_DEFAULT_VALUE,
                String::from("reg_b"),
            );
            let mut reg_func = ConstantRegister::new(
                port_collection.clone(),
                PORT_DEFAULT_VALUE,
                String::from("reg_func"),
            );

            let mut alu = ALU::new(
                port_collection.clone(),
                reg_a.output_port,
                reg_b.output_port,
                reg_func.output_port,
                String::from("alu"),
            );

            let tests = [
                // FUNC, A, B, EXPECTATION
                (ALU::OP_ADD, 80 as SWord, 8 as SWord, 88 as SWord),
                (ALU::OP_ADD, -88, 8, -80),
                (ALU::OP_SUB, -80, 8, -88),
                (ALU::OP_SUB, -88, -8, -80),
                (ALU::OP_BYPASS_A, -88, -8, -88),
                (ALU::OP_BYPASS_B, -88, -8, -8),
                (ALU::OP_AND, 0b_101010, 0b_001110, 0b_001010),
                (ALU::OP_OR, 0b_101010, 0b_001110, 0b_101110),
                (ALU::OP_XOR, 0b_101010, 0b_001110, 0b_100100),
                (ALU::OP_SLTU, 88, 80, 0),
                (ALU::OP_SLTU, 88, 88, 0),
                (ALU::OP_SLTU, 88, 888, 1),
                (ALU::OP_MUL, -8, -8, 64),
                (ALU::OP_MUL, -8, 8, -64),
                (ALU::OP_SLL, 0b_00001100, 2, 0b_00110000),
                (ALU::OP_SRL, 0b_00001100, 2, 0b_00000011),
                (ALU::OP_SRA, -8, 2, -2),
                (ALU::OP_SRA, 8, 2, 2),
            ];

            for (func, in_a, in_b, expectation) in tests {
                reg_a.constant_value = in_a as Word;
                reg_b.constant_value = in_b as Word;
                reg_func.constant_value = func;

                reg_a.process_cycle();
                reg_b.process_cycle();
                reg_func.process_cycle();
                alu.process_cycle();

                let port_collection = port_collection.borrow_mut();
                assert_eq!(
                    port_collection.get_port_data(alu.output_port),
                    expectation as Word
                );
            }
        }

        #[test]
        pub fn test_alu_unsigned() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let mut reg_a = ConstantRegister::new(
                port_collection.clone(),
                PORT_DEFAULT_VALUE,
                String::from("reg_a"),
            );
            let mut reg_b = ConstantRegister::new(
                port_collection.clone(),
                PORT_DEFAULT_VALUE,
                String::from("reg_b"),
            );
            let mut reg_func = ConstantRegister::new(
                port_collection.clone(),
                PORT_DEFAULT_VALUE,
                String::from("reg_func"),
            );

            let mut alu = ALU::new(
                port_collection.clone(),
                reg_a.output_port,
                reg_b.output_port,
                reg_func.output_port,
                String::from("alu"),
            );

            let tests = [
                // FUNC, A, B, EXPECTATION
                (ALU::OP_ADD, 80, 8, 88),
                (ALU::OP_SUB, 88, 8, 80),
                (ALU::OP_BYPASS_A, 88, 8, 88),
                (ALU::OP_BYPASS_B, 88, 8, 8),
                (ALU::OP_AND, 0b_101010, 0b_001110, 0b_001010),
                (ALU::OP_OR, 0b_101010, 0b_001110, 0b_101110),
                (ALU::OP_XOR, 0b_101010, 0b_001110, 0b_100100),
                (ALU::OP_SLTU, 88, 80, 0),
                (ALU::OP_SLTU, 88, 88, 0),
                (ALU::OP_SLTU, 88, 888, 1),
                (ALU::OP_MUL, 8, 8, 64),
                (ALU::OP_SLL, 0b_00001100, 2, 0b_00110000),
                (ALU::OP_SRL, 0b_00001100, 2, 0b_00000011),
                (ALU::OP_SUB, 10, 20, (-10 as SWord) as Word), // Test underflow
                (ALU::OP_ADD, Word::MAX, 1, 0),                // Test overflow
            ];

            for (func, in_a, in_b, expectation) in tests {
                reg_a.constant_value = in_a;
                reg_b.constant_value = in_b;
                reg_func.constant_value = func;

                reg_a.process_cycle();
                reg_b.process_cycle();
                reg_func.process_cycle();
                alu.process_cycle();

                let port_collection = port_collection.borrow_mut();
                assert_eq!(port_collection.get_port_data(alu.output_port), expectation);
            }
        }
    }

    mod switching {
        use super::*;

        #[test]
        /// Tests an isolated mux with 2 inputs and a single bit selection line. Tests valid input selection,
        /// as well as application of the input mask.
        fn test_binary_mux() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let input_value_a = 8;
            let input_value_b = 88;

            let selection_value = 0;

            let mut reg_a = ConstantRegister::new(
                port_collection.clone(),
                input_value_a,
                String::from("reg_a"),
            );
            let mut reg_b = ConstantRegister::new(
                port_collection.clone(),
                input_value_b,
                String::from("reg_b"),
            );
            let mut reg_s = ConstantRegister::new(
                port_collection.clone(),
                selection_value,
                String::from("reg_c"),
            );

            let mut mux = Mux::<2>::new(
                port_collection.clone(),
                &[reg_a.output_port, reg_b.output_port],
                reg_s.output_port,
                String::from("mux"),
            );

            reg_a.process_cycle();
            reg_b.process_cycle();
            reg_s.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(mux.inputs[0]), input_value_a);
                assert_eq!(port_collection.get_port_data(mux.inputs[1]), input_value_b);
                assert_eq!(
                    port_collection.get_port_data(mux.output_port),
                    PORT_DEFAULT_VALUE
                );
            }

            mux.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(mux.inputs[0]), input_value_a);
                assert_eq!(port_collection.get_port_data(mux.inputs[1]), input_value_b);
                assert_eq!(
                    port_collection.get_port_data(mux.output_port),
                    input_value_a
                );
            }

            let selection_value = 1;
            reg_s.constant_value = selection_value;

            reg_s.process_cycle();
            mux.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(mux.inputs[0]), input_value_a);
                assert_eq!(port_collection.get_port_data(mux.inputs[1]), input_value_b);
                assert_eq!(
                    port_collection.get_port_data(mux.output_port),
                    input_value_b
                );
            }

            let selection_value = 2;
            reg_s.constant_value = selection_value;

            reg_s.process_cycle();
            mux.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(mux.inputs[0]), input_value_a);
                assert_eq!(port_collection.get_port_data(mux.inputs[1]), input_value_b);
                assert_eq!(
                    port_collection.get_port_data(mux.output_port),
                    input_value_a
                );
            }
        }

        #[test]
        /// Tests an isolated demux with 2 outputs and a single bit selection line. Tests valid output selection,
        /// as well as application of the input mask.
        fn test_binary_de_mux() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let input_value = 88;
            let selection_value = 0;

            let mut reg_a =
                ConstantRegister::new(port_collection.clone(), input_value, String::from("reg_a"));
            let mut reg_s = ConstantRegister::new(
                port_collection.clone(),
                selection_value,
                String::from("reg_s"),
            );

            let mut de_mux = DeMux::<2>::new(
                port_collection.clone(),
                reg_a.output_port,
                reg_s.output_port,
                String::from("demux"),
            );

            reg_a.process_cycle();
            reg_s.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(de_mux.input), input_value);
                assert_eq!(
                    port_collection.get_port_data(de_mux.selection_input),
                    selection_value
                );
                assert_eq!(
                    port_collection.get_port_data(de_mux.outputs[0]),
                    PORT_DEFAULT_VALUE
                );
                assert_eq!(
                    port_collection.get_port_data(de_mux.outputs[1]),
                    PORT_DEFAULT_VALUE
                );
            }

            de_mux.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(de_mux.input), input_value);
                assert_eq!(
                    port_collection.get_port_data(de_mux.selection_input),
                    selection_value
                );
                assert_eq!(
                    port_collection.get_port_data(de_mux.outputs[0]),
                    input_value
                );
                assert_eq!(
                    port_collection.get_port_data(de_mux.outputs[1]),
                    PORT_DEFAULT_VALUE
                );
            }

            let selection_value = 1;
            reg_s.constant_value = selection_value;

            reg_s.process_cycle();
            de_mux.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(de_mux.input), input_value);
                assert_eq!(
                    port_collection.get_port_data(de_mux.selection_input),
                    selection_value
                );
                assert_eq!(
                    port_collection.get_port_data(de_mux.outputs[0]),
                    PORT_DEFAULT_VALUE
                );
                assert_eq!(
                    port_collection.get_port_data(de_mux.outputs[1]),
                    input_value
                );
            }

            let selection_value = 2;
            reg_s.constant_value = selection_value;

            reg_s.process_cycle();
            de_mux.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(de_mux.input), input_value);
                assert_eq!(
                    port_collection.get_port_data(de_mux.selection_input),
                    selection_value
                );
                assert_eq!(
                    port_collection.get_port_data(de_mux.outputs[0]),
                    input_value
                );
                assert_eq!(
                    port_collection.get_port_data(de_mux.outputs[1]),
                    PORT_DEFAULT_VALUE
                );
            }
        }
    }

    mod register_file {
        use super::*;

        #[test]
        /// Tests writing to and reading from a register file.
        fn test_rf() {
            let port_collection = Rc::new(RefCell::new(PortCollection::new()));

            let val_read_a = 0;
            let val_read_b = 3;

            let val_write_data = 88;
            let val_write_select = 0;
            let val_write_enable = 0;

            let mut reg_read_a = ConstantRegister::new(
                port_collection.clone(),
                val_read_a,
                String::from("reg_read_a"),
            );
            let mut reg_read_b = ConstantRegister::new(
                port_collection.clone(),
                val_read_b,
                String::from("reg_read_b"),
            );

            let mut reg_write_data = ConstantRegister::new(
                port_collection.clone(),
                val_write_data,
                String::from("reg_write_data"),
            );
            let mut reg_write_select = ConstantRegister::new(
                port_collection.clone(),
                val_write_select,
                String::from("reg_write_select"),
            );
            let mut reg_write_enable = ConstantRegister::new(
                port_collection.clone(),
                val_write_enable,
                String::from("reg_write_enable"),
            );

            let mut rf = RegisterFile::<8>::new(
                port_collection.clone(),
                reg_read_a.output_port,
                reg_read_b.output_port,
                reg_write_data.output_port,
                reg_write_select.output_port,
                reg_write_enable.output_port,
            );

            reg_read_a.process_cycle();
            reg_read_b.process_cycle();
            reg_write_data.process_cycle();
            reg_write_select.process_cycle();
            reg_write_enable.process_cycle();
            rf.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(rf.out_a), PORT_DEFAULT_VALUE);
                assert_eq!(port_collection.get_port_data(rf.out_b), PORT_DEFAULT_VALUE);
            }

            // Write data to register 0
            let val_write_enable = 1;
            reg_write_enable.constant_value = val_write_enable;

            reg_write_enable.process_cycle();
            rf.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(rf.out_a), val_write_data);
                assert_eq!(port_collection.get_port_data(rf.out_b), PORT_DEFAULT_VALUE);
            }

            // Write data to register 3
            let val_write_data = 33;
            let val_write_select = 3;

            reg_write_data.constant_value = val_write_data;
            reg_write_select.constant_value = val_write_select;

            reg_write_data.process_cycle();
            reg_write_select.process_cycle();
            rf.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(rf.out_a), 88);
                assert_eq!(port_collection.get_port_data(rf.out_b), val_write_data);
            }

            // Change read registers

            reg_read_a.constant_value = val_read_b;
            reg_read_b.constant_value = 7;

            reg_write_enable.constant_value = 0;

            reg_read_a.process_cycle();
            reg_read_b.process_cycle();
            reg_write_enable.process_cycle();

            rf.process_cycle();

            {
                let port_collection = port_collection.borrow_mut();

                assert_eq!(port_collection.get_port_data(rf.out_a), val_write_data);
                assert_eq!(port_collection.get_port_data(rf.out_b), PORT_DEFAULT_VALUE);
            }
        }
    }

    mod misc {
        use super::*;
        use crate::assembler;

        #[test]
        pub fn test_check_raw_hazard() {
            /// Creates a test case expecting no RaW hazard
            macro_rules! expect_no_hazard {
                ($instr_r:literal, $instr_w:literal) => {
                    assert_eq!(
                        check_raw_hazard(
                            assembler::encode_instruction_str($instr_r).unwrap(),
                            assembler::encode_instruction_str($instr_w).unwrap()
                        ),
                        false
                    );
                };
            }

            /// Creates a test case expecting a RaW hazard
            macro_rules! expect_hazard {
                ($instr_r:literal, $instr_w:literal) => {
                    assert_eq!(
                        check_raw_hazard(
                            assembler::encode_instruction_str($instr_r).unwrap(),
                            assembler::encode_instruction_str($instr_w).unwrap()
                        ),
                        true
                    );
                };
            }

            // Only different registers
            expect_no_hazard!("ADD x4 x5 x6", "ADD x1 x2 x3");
            expect_no_hazard!("ADDI x2 x3 100", "ADDI x1 x0 50");
            expect_no_hazard!("STORE x2 x3 100", "LOAD x1 x0 50");

            // No RaW hazards on register 0
            expect_no_hazard!("ADD x2 x0 x0", "ADDI x0 x1 50");
            expect_no_hazard!("NOP", "NOP");

            // RaW hazard
            expect_hazard!("STORE x22 x0 50", "ADDI x22 x22 50");
            expect_hazard!("ADD x1 x3 x4", "ADD x3 x4 x4");
            expect_hazard!("ADD x1 x3 x4", "LOAD x3 x0 10");
        }
    }
}
