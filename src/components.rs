use crate::circuit::{PortID, PortCollection, PORT_NULL_ID, PORT_DEFAULT_VALUE, PORT_NULL_VALUE, Port};
use std::cell::RefCell;
use std::rc::Rc;
use std::ops::Deref;
use std::mem::MaybeUninit;
use std::ptr::write;
use crate::risc_v::*;
use crate::assembler;

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
    pub name: String
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
    pub name: String
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
        let port_id = port_collection.borrow_mut().register_port(value, name.clone() + ".out");

        Self {
            constant_value: value,
            output_port: port_id,
            port_collection,
            name
        }
    }
}

impl Register {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, input_port: PortID, name: String) -> Self {
        let output_port_id = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input: input_port,
            output_port: output_port_id,
            port_collection,
            name
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let input_value = port_collection.get_port_data(self.input);
        let output_value = port_collection.get_port_data(self.output_port);

        println!("Register '{}': input_value: {},  output_value: {}", self.name, input_value, output_value);
    }
}

impl GuardedRegister {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, input_port: PortID, input_enable_port: PortID, name: String) -> Self {
        let output_port_id = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input_enable: input_enable_port,
            input: input_port,
            output_port: output_port_id,
            port_collection,
            name
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let input_value = port_collection.get_port_data(self.input);
        let enabled_value = port_collection.get_port_data(self.input_enable);
        let output_value = port_collection.get_port_data(self.output_port);

        println!("Guarded Register '{}': input_value: {}, input_enabled_value: {}, output_value: {}", self.name, input_value, enabled_value, output_value);
    }
}

impl<const START_BIT: usize, const LEN: usize> BitSelectionRegister<START_BIT, LEN> {
    const BIT_MASK: Word = ((1 << LEN) - 1) << START_BIT;

    pub fn new(port_collection: Rc<RefCell<PortCollection>>, input_port: PortID, name: String) -> Self {
        let output_port_id = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input: input_port,
            output_port: output_port_id,
            port_collection,
            name
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let input_value = port_collection.get_port_data(self.input);
        let output_value = port_collection.get_port_data(self.output_port);

        println!("Register '{}': input_value: {},  output_value: {}", self.name, input_value, output_value);
    }
}

#[test]
///
/// Tests a constant register, normal register, and guarded register (a, b, c) in series.
/// One extra constant register (e) is added for the enable line to the guarded register.
///
fn test_registers() {
    let port_collection = Rc::new(RefCell::new(PortCollection::new()));

    let a_value: Word = 88;
    let e_value: Word = 0;

    let mut a = ConstantRegister::new(Rc::clone(&port_collection), a_value, String::from("a"));
    let mut b = Register::new(Rc::clone(&port_collection), a.output_port, String::from("b"));

    let mut e = ConstantRegister::new(Rc::clone(&port_collection), e_value, String::from("e"));
    let mut c = GuardedRegister::new(Rc::clone(&port_collection), b.output_port, e.output_port, String::from("c"));

    a.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(a.output_port), a_value);

        assert_eq!(port_collection.get_port_data(b.input), a_value);
        assert_eq!(port_collection.get_port_data(b.output_port), PORT_DEFAULT_VALUE);

        assert_eq!(port_collection.get_port_data(c.input), PORT_DEFAULT_VALUE);
        assert_eq!(port_collection.get_port_data(c.input_enable), PORT_DEFAULT_VALUE);
        assert_eq!(port_collection.get_port_data(c.output_port), PORT_DEFAULT_VALUE);

        assert_eq!(port_collection.get_port_data(e.output_port), e_value);
    }

    b.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(a.output_port), a_value);

        assert_eq!(port_collection.get_port_data(b.input), a_value);
        assert_eq!(port_collection.get_port_data(b.output_port), a_value);

        assert_eq!(port_collection.get_port_data(c.input), a_value);
        assert_eq!(port_collection.get_port_data(c.input_enable), PORT_DEFAULT_VALUE);
        assert_eq!(port_collection.get_port_data(c.output_port), PORT_DEFAULT_VALUE);

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
        assert_eq!(port_collection.get_port_data(c.output_port), PORT_DEFAULT_VALUE);

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

    let mut c = ConstantRegister::new(Rc::clone(&port_collection), CONST_VALUE << OFFSET,String::from("c"));
    let mut s = BitSelectionRegister::<OFFSET, NUM_VAL_BITS>::new(Rc::clone(&port_collection), c.output_port, String::from("c"));

    c.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();
        assert_eq!(PORT_DEFAULT_VALUE, port_collection.get_port_data(s.output_port));
    }

    s.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();
        assert_eq!(CONST_VALUE, port_collection.get_port_data(s.output_port));
    }
}

/*
Memory
 */

const MEM_SIZE: usize = 65536;

pub const MEM_LEN_BYTE: Word = 0;
pub const MEM_LEN_SHORT: Word = 1;
pub const MEM_LEN_WORD: Word = 2;

pub struct RMemory {
    pub address_input: PortID,
    pub length_input: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String,

    pub content: [u8; MEM_SIZE],
}

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
    if length_input == MEM_LEN_WORD {
        assert_eq!(address % 4, 0, "Unaligned memory access for MEM_LEN_WORD! address: {}", address);

        // Big Endian
        let b_3 = mem_content[address as usize];
        let b_2 = mem_content[(address + 1) as usize];
        let b_1 = mem_content[(address + 2) as usize];
        let b_0 = mem_content[(address + 3) as usize];

        Word::from_be_bytes([b_3, b_2, b_1, b_0])
    } else if length_input == MEM_LEN_SHORT {
        assert_eq!(address % 2, 0, "Unaligned memory access for MEM_LEN_SHORT! address: {}", address);

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
        assert_eq!(address % 4, 0, "Unaligned memory access for MEM_LEN_WORD! address: {}", address);

        mem_content[address as usize + 0] = data_bytes[0];
        mem_content[address as usize + 1] = data_bytes[1];
        mem_content[address as usize + 2] = data_bytes[2];
        mem_content[address as usize + 3] = data_bytes[3];
    } else if length_input == MEM_LEN_SHORT {
        assert_eq!(address % 2, 0, "Unaligned memory access for MEM_LEN_SHORT! address: {}", address);

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
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, address_port: PortID, length_port: PortID, name: String) -> Self {
        let output_port_id = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            address_input: address_port,
            length_input: length_port,
            output_port: output_port_id,
            port_collection,
            name,
            content: [0u8; MEM_SIZE]
        }
    }

    pub fn new_with_mem(port_collection: Rc<RefCell<PortCollection>>, address_port: PortID, length_port: PortID, name: String, mem: &[u8; MEM_SIZE]) -> Self {
        let output_port_id = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            address_input: address_port,
            length_input: length_port,
            output_port: output_port_id,
            port_collection,
            name,
            content: mem.clone()
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let address_value = port_collection.get_port_data(self.address_input);
        let length_value = port_collection.get_port_data(self.length_input) & 1;
        let output_value = port_collection.get_port_data(self.output_port);

        println!("Read-only memory '{}': address_value: {}, length_value: {}, output_value: {:#010X}", self.name, address_value, length_value, output_value);
    }
}

impl RWMemory {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, address_port: PortID, length_port: PortID, data_port: PortID, write_enable_port: PortID, name: String) -> Self {
        let output_port_id = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            address_input: address_port,
            data_input: data_port,
            length_input: length_port,
            write_enable_input: write_enable_port,
            output_port: output_port_id,
            port_collection,
            name,
            content: [0u8; MEM_SIZE]
        }
    }

    pub fn new_with_mem(port_collection: Rc<RefCell<PortCollection>>, address_port: PortID, length_port: PortID, data_port: PortID, write_enable_port: PortID, name: String, mem: &[u8; MEM_SIZE]) -> Self {
        let output_port_id = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            address_input: address_port,
            data_input: data_port,
            length_input: length_port,
            write_enable_input: write_enable_port,
            output_port: output_port_id,
            port_collection,
            name,
            content: mem.clone()
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

    let mut reg_addr = ConstantRegister::new(port_collection.clone(), address, String::from("reg_addr"));
    let mut reg_len = ConstantRegister::new(port_collection.clone(), len_mode, String::from("reg_len"));

    let mut memory = RMemory::new_with_mem(port_collection.clone(), reg_addr.output_port, reg_len.output_port, String::from("m"), &mem);

    reg_addr.process_cycle();
    reg_len.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(memory.address_input), address);
        assert_eq!(port_collection.get_port_data(memory.length_input), len_mode);
        assert_eq!(port_collection.get_port_data(memory.output_port), PORT_DEFAULT_VALUE);
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
        assert_eq!(port_collection.get_port_data(memory.output_port), 0x02040608);
    }
}

#[test]
#[should_panic]
/// Tests whether miss-aligned short memory access throws a panic.
fn test_read_only_memory_short_unaligned() {
    let port_collection = Rc::new(RefCell::new(PortCollection::new()));

    let address = 3;
    let len_mode = MEM_LEN_SHORT;

    let mut reg_addr = ConstantRegister::new(port_collection.clone(), address, String::from("reg_addr"));
    let mut reg_len = ConstantRegister::new(port_collection.clone(), len_mode, String::from("reg_len"));

    let mut memory = RMemory::new(port_collection.clone(), reg_addr.output_port, reg_len.output_port, String::from("m"));

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

    let mut reg_addr = ConstantRegister::new(port_collection.clone(), address, String::from("reg_addr"));
    let mut reg_len = ConstantRegister::new(port_collection.clone(), len_mode, String::from("reg_len"));

    let mut memory = RMemory::new(port_collection.clone(), reg_addr.output_port, reg_len.output_port, String::from("m"));

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

    let mut reg_addr = ConstantRegister::new(port_collection.clone(), address, String::from("reg_addr"));
    let mut reg_len = ConstantRegister::new(port_collection.clone(), len_mode, String::from("reg_len"));

    let mut reg_data = ConstantRegister::new(port_collection.clone(), data, String::from("reg_data"));
    let mut reg_w_enable = ConstantRegister::new(port_collection.clone(), write_enable, String::from("reg_w_enable"));

    let mut mem = RWMemory::new_with_mem(
        port_collection.clone(),
        reg_addr.output_port,
        reg_len.output_port,
        reg_data.output_port,
        reg_w_enable.output_port,
        String::from("mem"),
        &mem
    );

    reg_addr.process_cycle();
    reg_len.process_cycle();
    reg_data.process_cycle();
    reg_w_enable.process_cycle();

    {
        let mut port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(mem.length_input), len_mode);
        assert_eq!(port_collection.get_port_data(mem.address_input), address);
        assert_eq!(port_collection.get_port_data(mem.data_input), data);
        assert_eq!(port_collection.get_port_data(mem.write_enable_input), write_enable);
        assert_eq!(port_collection.get_port_data(mem.output_port), PORT_DEFAULT_VALUE);
    }

    mem.process_cycle();

    {
        // Read should be the initial value
        let mut port_collection = port_collection.borrow_mut();
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
        let mut port_collection = port_collection.borrow_mut();
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
        let mut port_collection = port_collection.borrow_mut();
        assert_eq!(port_collection.get_port_data(mem.output_port), data);
    }
}

/*
Functional units
 */

pub struct Adder {
    pub input_a: PortID,
    pub input_b: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String
}

pub struct Comparator {
    pub input_a: PortID,
    pub input_b: PortID,
    pub input_mode: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String
}

pub struct BranchTester {
    pub input_comp: PortID,
    pub input_func: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String
}

pub struct ALU {
    pub input_a: PortID,
    pub input_b: PortID,
    pub input_func: PortID,
    pub output_port: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String
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
            func_code_3::BEQ => {
                in_comp & Comparator::EQ_BIT != 0
            }
            func_code_3::BNE => {
                in_comp & Comparator::EQ_BIT == 0
            }
            func_code_3::BLT | func_code_3::BLTU => {
                in_comp & Comparator::LT_BIT != 0
            }
            func_code_3::BGE | func_code_3::BGEU => {
                in_comp & Comparator::LT_BIT == 0
            }
            _ => { unreachable!("Found invalid func code: {}", in_comp) }
        };

        let output = if take { Self::BRANCH_TAKE } else { Self::BRANCH_REJECT };
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
            Self::OP_ADD => {
                in_a.wrapping_add(in_b)
            }
            Self::OP_SUB => {
                in_a.wrapping_sub(in_b)
            }
            Self::OP_AND => {
                in_a & in_b
            }
            Self::OP_OR => {
                in_a | in_b
            }
            Self::OP_XOR => {
                in_a ^ in_b
            }
            Self::OP_SLT => {
                if in_as < in_bs { 1 } else { 0 }
            }
            Self::OP_SLTU => {
                if in_a < in_b { 1 } else { 0 }
            }
            Self::OP_SLL => {
                in_a << in_b
            }
            Self::OP_SRL => {
                in_a >> in_b
            }
            Self::OP_SRA => {
                (in_as >> in_b) as Word
            }
            Self::OP_MUL => {
                in_a.wrapping_mul(in_b)
            }
            _ => { unreachable!() }
        };

        port_collection.set_port_data(self.output_port, output_value);
    }
}

impl Adder {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, input_a: PortID, input_b: PortID, name: String) -> Self {
        let output_port_id = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input_a,
            input_b,
            output_port: output_port_id,
            port_collection,
            name
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let input_a = port_collection.get_port_data(self.input_a);
        let input_b = port_collection.get_port_data(self.input_b);
        let output_value = port_collection.get_port_data(self.output_port);

        println!("Adder '{}': input_a: {}, input_b: {}, output_value: {}", self.name, input_a, input_b, output_value);
    }
}

impl Comparator {
    // Output flag bits
    const EQ_BIT: Word = 0b_01;
    const LT_BIT: Word = 0b_10;

    // Input mode types
    const MODE_S: Word = 0;
    const MODE_U: Word = 1;

    pub fn new(port_collection: Rc<RefCell<PortCollection>>, in_a: PortID, in_b: PortID, in_mode: PortID,  name: String) -> Self {
        let output_port_id = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input_a: in_a,
            input_b: in_b,
            input_mode: in_mode,
            output_port: output_port_id,
            port_collection,
            name
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let input_a = port_collection.get_port_data(self.input_a);
        let input_b = port_collection.get_port_data(self.input_b);
        let input_mode = port_collection.get_port_data(self.input_mode);
        let output_value = port_collection.get_port_data(self.output_port);

        println!("Comparator '{}': input_a: {}, input_b: {}, input_mode: {}, EQ flag: {}, LT flag: {}",
                 self.name, input_a, input_b, input_mode,
                 output_value & Self::EQ_BIT != 0,
                 output_value & Self::LT_BIT != 0);
    }
}

impl BranchTester {
    pub const BRANCH_REJECT: Word = 0;
    pub const BRANCH_TAKE: Word = 1;

    pub fn new(port_collection: Rc<RefCell<PortCollection>>, in_comp: PortID, in_func: PortID,  name: String) -> Self {
        let output_port_id = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input_comp: in_comp,
            input_func: in_func,
            output_port: output_port_id,
            port_collection,
            name
        }
    }

    pub fn print_state(&self) {
        let port_collection = self.port_collection.borrow();

        let in_comp = port_collection.get_port_data(self.input_comp);
        let in_func = port_collection.get_port_data(self.input_func) & 7;
        let output_value = port_collection.get_port_data(self.output_port);

        println!("BranchTester '{}': in_EQ: {}, in_LT: {}, in_func: {}, out: {}",
                 self.name,
                 in_comp & Comparator::EQ_BIT != 0,
                 in_comp & Comparator::LT_BIT != 0,
                 in_func,
                 output_value);
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

    pub fn new(port_collection: Rc<RefCell<PortCollection>>, in_a: PortID, in_b: PortID, in_func: PortID, name: String) -> Self {
        let output_port_id = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        Self {
            input_a: in_a,
            input_b: in_b,
            input_func: in_func,
            output_port: output_port_id,
            port_collection,
            name
        }
    }
}

#[test]
fn test_adder() {
    let port_collection = Rc::new(RefCell::new(PortCollection::new()));

    let in_a = 8;
    let in_b = 80;

    let mut reg_a = ConstantRegister::new(port_collection.clone(), in_a, String::from("reg_a"));
    let mut reg_b = ConstantRegister::new(port_collection.clone(), in_b, String::from("reg_b"));

    let mut adder = Adder::new(port_collection.clone(), reg_a.output_port, reg_b.output_port, String::from("add"));

    reg_a.process_cycle();
    reg_b.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(adder.input_a), in_a);
        assert_eq!(port_collection.get_port_data(adder.input_b), in_b);
        assert_eq!(port_collection.get_port_data(adder.output_port), PORT_DEFAULT_VALUE);
    }

    adder.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(adder.input_a), in_a);
        assert_eq!(port_collection.get_port_data(adder.input_b), in_b);
        assert_eq!(port_collection.get_port_data(adder.output_port), in_a + in_b);
    }
}

#[test]
pub fn test_comparator_unsigned() {
    let port_collection = Rc::new(RefCell::new(PortCollection::new()));

    // Test LT

    let in_a = 8;
    let in_b = 80;
    let in_mode = Comparator::MODE_U;

    let mut reg_a = ConstantRegister::new(port_collection.clone(), in_a, String::from("reg_a"));
    let mut reg_b = ConstantRegister::new(port_collection.clone(), in_b, String::from("reg_b"));
    let mut reg_mode = ConstantRegister::new(port_collection.clone(), in_mode, String::from("reg_mode"));

    let mut comp = Comparator::new(port_collection.clone(), reg_a.output_port, reg_b.output_port, reg_mode.output_port, String::from("comp"));

    reg_a.process_cycle();
    reg_b.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(comp.input_a), in_a);
        assert_eq!(port_collection.get_port_data(comp.input_b), in_b);
        assert_eq!(port_collection.get_port_data(comp.input_mode), in_mode);
        assert_eq!(port_collection.get_port_data(comp.output_port), PORT_DEFAULT_VALUE);
    }

    comp.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(comp.input_a), in_a);
        assert_eq!(port_collection.get_port_data(comp.input_b), in_b);
        assert_eq!(port_collection.get_port_data(comp.input_mode), in_mode);
        assert_eq!(port_collection.get_port_data(comp.output_port), Comparator::LT_BIT);
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
        assert_eq!(port_collection.get_port_data(comp.output_port), Comparator::EQ_BIT);
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

    let mut reg_a = ConstantRegister::new(port_collection.clone(), in_a as Word, String::from("reg_a"));
    let mut reg_b = ConstantRegister::new(port_collection.clone(), in_b as Word, String::from("reg_b"));
    let mut reg_mode = ConstantRegister::new(port_collection.clone(), in_mode, String::from("reg_mode"));

    let mut comp = Comparator::new(port_collection.clone(), reg_a.output_port, reg_b.output_port, reg_mode.output_port, String::from("comp"));

    reg_a.process_cycle();
    reg_b.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(comp.input_a), in_a as Word);
        assert_eq!(port_collection.get_port_data(comp.input_b), in_b as Word);
        assert_eq!(port_collection.get_port_data(comp.input_mode), in_mode);
        assert_eq!(port_collection.get_port_data(comp.output_port), PORT_DEFAULT_VALUE);
    }

    comp.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(comp.input_a), in_a as Word);
        assert_eq!(port_collection.get_port_data(comp.input_b), in_b as Word);
        assert_eq!(port_collection.get_port_data(comp.input_mode), in_mode);
        assert_eq!(port_collection.get_port_data(comp.output_port), Comparator::LT_BIT);
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
        assert_eq!(port_collection.get_port_data(comp.output_port), Comparator::EQ_BIT);
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

    let mut reg_a = ConstantRegister::new(port_collection.clone(), PORT_DEFAULT_VALUE, String::from("reg_a"));
    let mut reg_b = ConstantRegister::new(port_collection.clone(), PORT_DEFAULT_VALUE, String::from("reg_b"));
    let mut reg_func = ConstantRegister::new(port_collection.clone(), PORT_DEFAULT_VALUE, String::from("reg_func"));
    let mut reg_mode = BitSelectionRegister::<2, 1>::new(port_collection.clone(), reg_func.output_port, String::from("reg_mode"));

    let mut comp = Comparator::new(port_collection.clone(), reg_a.output_port, reg_b.output_port, reg_mode.output_port, String::from("comp"));
    let mut bt = BranchTester::new(port_collection.clone(), comp.output_port, reg_func.output_port, String::from("bt"));

    let a_b_pairs = [
        (-80, 80),
        (-8, -8),
        (80, -80)
    ];

    let branch_matrix = [
        // BEQ, BNE, BLT, BGE
        [0, 1, 1, 0],
        [1, 0, 0, 1],
        [0, 1, 0, 1],
    ];

    // No unsigned versions
    let func_codes = [
        func_code_3::BEQ, func_code_3::BNE, func_code_3::BLT, func_code_3::BGE
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

    let mut reg_a = ConstantRegister::new(port_collection.clone(), PORT_DEFAULT_VALUE, String::from("reg_a"));
    let mut reg_b = ConstantRegister::new(port_collection.clone(), PORT_DEFAULT_VALUE, String::from("reg_b"));
    let mut reg_func = ConstantRegister::new(port_collection.clone(), PORT_DEFAULT_VALUE, String::from("reg_func"));
    let mut reg_mode = BitSelectionRegister::<2, 1>::new(port_collection.clone(), reg_func.output_port, String::from("reg_mode"));

    let mut comp = Comparator::new(port_collection.clone(), reg_a.output_port, reg_b.output_port, reg_mode.output_port, String::from("comp"));
    let mut bt = BranchTester::new(port_collection.clone(), comp.output_port, reg_func.output_port, String::from("bt"));

    let a_b_pairs = [
        (80, 88),
        (88, 88),
        (88, 80)
    ];

    let branch_matrix = [
        // BEQ, BNE, BLTU, BGEU
        [0, 1, 1, 0],
        [1, 0, 0, 1],
        [0, 1, 0, 1],
    ];

    // No signed versions
    let func_codes = [
        func_code_3::BEQ, func_code_3::BNE, func_code_3::BLTU, func_code_3::BGEU
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

    let mut reg_a = ConstantRegister::new(port_collection.clone(), PORT_DEFAULT_VALUE, String::from("reg_a"));
    let mut reg_b = ConstantRegister::new(port_collection.clone(), PORT_DEFAULT_VALUE, String::from("reg_b"));
    let mut reg_func = ConstantRegister::new(port_collection.clone(), PORT_DEFAULT_VALUE, String::from("reg_func"));

    let mut alu = ALU::new(
        port_collection.clone(),
        reg_a.output_port,
        reg_b.output_port,
        reg_func.output_port,
        String::from("alu")
    );

    let tests = [
        // FUNC, A, B, EXPECTATION
        (ALU::OP_ADD, 80 as SWord, 8 as SWord, 88 as SWord),
        (ALU::OP_ADD, -88, 8, -80),
        (ALU::OP_SUB, -80, 8, -88),
        (ALU::OP_SUB, -88, -8, -80),

        (ALU::OP_AND, 0b_101010, 0b_001110, 0b_001010),
        (ALU::OP_OR, 0b_101010, 0b_001110, 0b_101110),
        (ALU::OP_XOR, 0b_101010, 0b_001110, 0b_100100),

        (ALU::OP_SLTU, 88, 80, 0),
        (ALU::OP_SLTU, 88, 88, 0),
        (ALU::OP_SLTU, 88, 888, 1),

        (ALU::OP_MUL, -8, -8, 64),
        (ALU::OP_MUL, -8, 8, -64),

        (ALU::OP_SLL, 0b_00001100, 2, 0b_00110000),
        (ALU::OP_SRL,  0b_00001100, 2, 0b_00000011),
        (ALU::OP_SRA,  -8, 2, -2),
        (ALU::OP_SRA,  8, 2, 2),
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
        assert_eq!(port_collection.get_port_data(alu.output_port), expectation as Word);
    }
}

#[test]
pub fn test_alu_unsigned() {
    let port_collection = Rc::new(RefCell::new(PortCollection::new()));

    let mut reg_a = ConstantRegister::new(port_collection.clone(), PORT_DEFAULT_VALUE, String::from("reg_a"));
    let mut reg_b = ConstantRegister::new(port_collection.clone(), PORT_DEFAULT_VALUE, String::from("reg_b"));
    let mut reg_func = ConstantRegister::new(port_collection.clone(), PORT_DEFAULT_VALUE, String::from("reg_func"));

    let mut alu = ALU::new(
        port_collection.clone(),
        reg_a.output_port,
        reg_b.output_port,
        reg_func.output_port,
        String::from("alu")
    );

    let tests = [
        // FUNC, A, B, EXPECTATION
        (ALU::OP_ADD, 80, 8, 88),
        (ALU::OP_SUB, 88, 8, 80),

        (ALU::OP_AND, 0b_101010, 0b_001110, 0b_001010),
        (ALU::OP_OR, 0b_101010, 0b_001110, 0b_101110),
        (ALU::OP_XOR, 0b_101010, 0b_001110, 0b_100100),

        (ALU::OP_SLTU, 88, 80, 0),
        (ALU::OP_SLTU, 88, 88, 0),
        (ALU::OP_SLTU, 88, 888, 1),

        (ALU::OP_MUL, 8, 8, 64),

        (ALU::OP_SLL, 0b_00001100, 2, 0b_00110000),
        (ALU::OP_SRL,  0b_00001100, 2, 0b_00000011),

        (ALU::OP_SUB,  10, 20, (-10 as SWord) as Word),  // Test underflow
        (ALU::OP_ADD,  Word::MAX, 1, 0),                 // Test overflow
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

/*
Switching
 */

pub struct Mux<const NUM_INPUTS: usize> {
    pub selection_input: PortID,
    pub inputs: [PortID; NUM_INPUTS],
    pub output_port: PortID,
    pub input_mask: Word,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String
}

pub struct DeMux<const NUM_INPUTS: usize> {
    pub selection_input: PortID,
    pub input: PortID,
    pub outputs: [PortID; NUM_INPUTS],
    pub input_mask: Word,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String
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

impl<const NUM_INPUTS: usize> Component for DeMux<NUM_INPUTS> {
    fn process_cycle(&mut self) {
        let mut port_collection = self.port_collection.borrow_mut();

        let selected_output = port_collection.get_port_data(self.selection_input) & self.input_mask;
        assert!((selected_output as usize) < NUM_INPUTS);

        let input_value = port_collection.get_port_data(self.input);

        for i in 0..NUM_INPUTS {
            if i == selected_output as usize {
                port_collection.set_port_data(self.outputs[i], input_value);
            } else {
                port_collection.set_port_data(self.outputs[i], PORT_DEFAULT_VALUE);
            }
        }
    }
}

impl<const NUM_INPUTS: usize> Mux<NUM_INPUTS> {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, inputs: &[PortID; NUM_INPUTS], selection_input: PortID, name: String) -> Self {
        let output_port_id = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, name.clone() + ".out");

        let num_selection_bits = (NUM_INPUTS as f64).log2().ceil() as u32;
        let input_mask = (2_u32.pow(num_selection_bits) as Word) - 1;

        Self {
            selection_input,
            inputs: inputs.clone(),
            output_port: output_port_id,
            input_mask,
            port_collection,
            name
        }
    }
}

impl<const NUM_INPUTS: usize> DeMux<NUM_INPUTS> {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, input: PortID, selection_input: PortID, name: String) -> Self {
        let mut output_ports = [PORT_NULL_ID; NUM_INPUTS];

        for i in 0..NUM_INPUTS {
            output_ports[i] = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, format!("{}.out_{}", name, i));
        }

        let num_selection_bits = (NUM_INPUTS as f64).log2().ceil() as u32;
        let input_mask = (2_u32.pow(num_selection_bits) as Word) - 1;

        Self {
            selection_input,
            input,
            outputs: output_ports,
            input_mask,
            port_collection,
            name
        }
    }
}

#[test]
/// Tests an isolated mux with 2 inputs and a single bit selection line. Tests valid input selection,
/// as well as application of the input mask.
fn test_binary_mux() {
    let port_collection = Rc::new(RefCell::new(PortCollection::new()));

    let input_value_a = 8;
    let input_value_b = 88;

    let selection_value = 0;

    let mut reg_a = ConstantRegister::new(port_collection.clone(), input_value_a, String::from("reg_a"));
    let mut reg_b = ConstantRegister::new(port_collection.clone(), input_value_b, String::from("reg_b"));
    let mut reg_s = ConstantRegister::new(port_collection.clone(), selection_value, String::from("reg_c"));

    let mut mux = Mux::<2>::new(port_collection.clone(), &[reg_a.output_port, reg_b.output_port], reg_s.output_port, String::from("mux"));

    reg_a.process_cycle();
    reg_b.process_cycle();
    reg_s.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(mux.inputs[0]), input_value_a);
        assert_eq!(port_collection.get_port_data(mux.inputs[1]), input_value_b);
        assert_eq!(port_collection.get_port_data(mux.output_port), PORT_DEFAULT_VALUE);
    }

    mux.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(mux.inputs[0]), input_value_a);
        assert_eq!(port_collection.get_port_data(mux.inputs[1]), input_value_b);
        assert_eq!(port_collection.get_port_data(mux.output_port), input_value_a);
    }

    let selection_value = 1;
    reg_s.constant_value = selection_value;

    reg_s.process_cycle();
    mux.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(mux.inputs[0]), input_value_a);
        assert_eq!(port_collection.get_port_data(mux.inputs[1]), input_value_b);
        assert_eq!(port_collection.get_port_data(mux.output_port), input_value_b);
    }

    let selection_value = 2;
    reg_s.constant_value = selection_value;

    reg_s.process_cycle();
    mux.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(mux.inputs[0]), input_value_a);
        assert_eq!(port_collection.get_port_data(mux.inputs[1]), input_value_b);
        assert_eq!(port_collection.get_port_data(mux.output_port), input_value_a);
    }
}

#[test]
/// Tests an isolated demux with 2 outputs and a single bit selection line. Tests valid output selection,
/// as well as application of the input mask.
fn test_binary_de_mux() {
    let port_collection = Rc::new(RefCell::new(PortCollection::new()));

    let input_value = 88;
    let selection_value = 0;

    let mut reg_a = ConstantRegister::new(port_collection.clone(), input_value, String::from("reg_a"));
    let mut reg_s = ConstantRegister::new(port_collection.clone(), selection_value, String::from("reg_s"));

    let mut de_mux = DeMux::<2>::new(port_collection.clone(), reg_a.output_port, reg_s.output_port, String::from("demux"));

    reg_a.process_cycle();
    reg_s.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(de_mux.input), input_value);
        assert_eq!(port_collection.get_port_data(de_mux.selection_input), selection_value);
        assert_eq!(port_collection.get_port_data(de_mux.outputs[0]), PORT_DEFAULT_VALUE);
        assert_eq!(port_collection.get_port_data(de_mux.outputs[1]), PORT_DEFAULT_VALUE);
    }

    de_mux.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(de_mux.input), input_value);
        assert_eq!(port_collection.get_port_data(de_mux.selection_input), selection_value);
        assert_eq!(port_collection.get_port_data(de_mux.outputs[0]), input_value);
        assert_eq!(port_collection.get_port_data(de_mux.outputs[1]), PORT_DEFAULT_VALUE);
    }

    let selection_value = 1;
    reg_s.constant_value = selection_value;

    reg_s.process_cycle();
    de_mux.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(de_mux.input), input_value);
        assert_eq!(port_collection.get_port_data(de_mux.selection_input), selection_value);
        assert_eq!(port_collection.get_port_data(de_mux.outputs[0]), PORT_DEFAULT_VALUE);
        assert_eq!(port_collection.get_port_data(de_mux.outputs[1]), input_value);
    }

    let selection_value = 2;
    reg_s.constant_value = selection_value;

    reg_s.process_cycle();
    de_mux.process_cycle();

    {
        let port_collection = port_collection.borrow_mut();

        assert_eq!(port_collection.get_port_data(de_mux.input), input_value);
        assert_eq!(port_collection.get_port_data(de_mux.selection_input), selection_value);
        assert_eq!(port_collection.get_port_data(de_mux.outputs[0]), input_value);
        assert_eq!(port_collection.get_port_data(de_mux.outputs[1]), PORT_DEFAULT_VALUE);
    }
}

/*
Register File
 */

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
    pub out_b: PortID
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
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, read_a: PortID, read_b: PortID,
               write_data: PortID, write_select: PortID, write_enable: PortID) -> Self {

        // Demux for propagating input enable signal to the gate of the proper register
        let mut de_mux_in = DeMux::<NUM_REGISTERS>::new(port_collection.clone(), write_enable, write_select, String::from("rf_demux_in"));

        // Issue relating to the initialization of arrays with const generic size
        // https://github.com/rust-lang/rust/issues/61956

        // Register arrays starts uninitialized because the size is const generic
        let mut registers_uninit: [MaybeUninit<GuardedRegister>; NUM_REGISTERS] = unsafe {
            MaybeUninit::uninit().assume_init()
        };

        for i in 0..NUM_REGISTERS {
            let reg = GuardedRegister::new(
                port_collection.clone(),
                write_data,
                de_mux_in.outputs[i],
                String::from(format!("rf_reg_{}", i))
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
            String::from("rf_mux_out_a")
        );

        let mux_out_b = Mux::<NUM_REGISTERS>::new(
            port_collection.clone(),
            &register_outputs,
            read_b,
            String::from("rf_mux_out_b")
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
            out_b
        }
    }

    pub fn set_write_inputs(&mut self, write_data: PortID, write_select: PortID, write_enable: PortID) {
        self.de_mux_in.selection_input = write_select;
        self.de_mux_in.input = write_enable;

        for register in &mut self.registers {
            register.input = write_data;
        }
    }
}

#[test]
/// Tests writing to and reading from a register file.
fn test_rf() {
    let mut port_collection = Rc::new(RefCell::new(PortCollection::new()));

    let val_read_a = 0;
    let val_read_b = 3;

    let val_write_data = 88;
    let val_write_select = 0;
    let val_write_enable = 0;

    let mut reg_read_a = ConstantRegister::new(port_collection.clone(), val_read_a, String::from("reg_read_a"));
    let mut reg_read_b = ConstantRegister::new(port_collection.clone(), val_read_b, String::from("reg_read_b"));

    let mut reg_write_data = ConstantRegister::new(port_collection.clone(), val_write_data, String::from("reg_write_data"));
    let mut reg_write_select = ConstantRegister::new(port_collection.clone(), val_write_select, String::from("reg_write_select"));
    let mut reg_write_enable = ConstantRegister::new(port_collection.clone(), val_write_enable, String::from("reg_write_enable"));

    let mut rf = RegisterFile::<8>::new(
        port_collection.clone(),
        reg_read_a.output_port,
        reg_read_b.output_port,
        reg_write_data.output_port,
        reg_write_select.output_port,
        reg_write_enable.output_port
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

/*
Misc
 */

pub struct ImmSignExtender {
    pub input: PortID,
    pub out_i_type: PortID,
    pub out_s_type: PortID,
    pub out_b_type: PortID,
    pub out_u_type: PortID,
    pub out_j_type: PortID,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String
}

pub struct InterlockUnit {
    pub in_id_instr: PortID,
    pub in_ex_instr: PortID,
    pub in_mem_instr: PortID,

    pub out_not_stall: PortID,
    stall_timer: Word,

    pub port_collection: Rc<RefCell<PortCollection>>,
    pub name: String
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
                op_code::OP | op_code::OP_IMM | op_code::LOAD | op_code::JAL |
                op_code::JALR | op_code::LUI | op_code::AUIPC => {
                    // Other instruction writes to register
                    let rd = extract_rd(other_instr);
                    // NOTE: No RaW conflict can exist on register 0
                    (rd != 0) && (rd == rs1 || rd == rs2)
                }
                _ => false
            }
        }
        op_code::OP_IMM | op_code::JALR => {
            // ID reads one register
            let rs1 = extract_rs1(id_instr);

            match op_code_other {
                op_code::OP | op_code::OP_IMM | op_code::LOAD | op_code::JAL |
                op_code::JALR | op_code::LUI | op_code::AUIPC => {
                    // Other instruction writes to register
                    let rd = extract_rd(other_instr);
                    // NOTE: No RaW conflict can exist on register 0
                    (rd != 0) && (rd == rs1)
                }
                _ => false
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

        if self.stall_timer == 0 {
            if check_raw_hazard(id_instr, ex_instr) {
                // Must stall for 2 cycles
                self.stall_timer = 2;
            } else if check_raw_hazard(id_instr, mem_instr) {
                // Only stall for a single cycle
                self.stall_timer = 1
            }
        } else {
            self.stall_timer = self.stall_timer - 1;
        }

        let not_stall = !(self.stall_timer >= 1);

        port_collection.set_port_data(self.out_not_stall, not_stall.into());
    }
}

impl ImmSignExtender {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, input: PortID, name: String) -> Self {
        let out_i_type = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, format!("{}_{}", name, String::from("out_i")));
        let out_s_type = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, format!("{}_{}", name, String::from("out_s")));
        let out_b_type = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, format!("{}_{}", name, String::from("out_b")));
        let out_u_type = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, format!("{}_{}", name, String::from("out_u")));
        let out_j_type = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, format!("{}_{}", name, String::from("out_j")));

        Self {
            input,
            out_i_type,
            out_s_type,
            out_b_type,
            out_u_type,
            out_j_type,
            port_collection,
            name
        }
    }
}

impl InterlockUnit {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, in_id_instr: PortID,
               in_ex_instr: PortID, in_mem_instr: PortID, name: String) -> Self {
        let out_not_stall = port_collection.borrow_mut().register_port(PORT_DEFAULT_VALUE, format!("{}_{}", name, String::from("not_stall")));

        Self {
            in_id_instr,
            in_ex_instr,
            in_mem_instr,
            out_not_stall,
            stall_timer: 0,
            port_collection,
            name
        }
    }
}

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
        }
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
        }
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

/*
Processor -- IF Stage
 */

pub struct IFStage {
    /// Program counter register
    pub reg_pc: Register,

    /// Adder for PC increase
    pub addr: Adder,

    /// Mux for deciding branch vs next instruction
    pub mux: Mux<2>,

    /// Instruction memory
    pub imem: RMemory,

    /// Constant '2' for increasing program counter
    pub reg_c_4: ConstantRegister,

    /// Constant 'MEM_LEN_SHORT' for fetching 2-byte instructions
    pub reg_c_len_mode: ConstantRegister,

    /// NPC register storing next program counter
    pub reg_npc: Register,

    /// IR register for storing fetched instruction
    pub reg_ir: Register
}

impl IFStage {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>) -> Self {
        // First create all constant registers
        let reg_c_4 = ConstantRegister::new(port_collection.clone(), 4, String::from("if_reg_c_4"));
        let reg_c_len_mode = ConstantRegister::new(port_collection.clone(), MEM_LEN_WORD, String::from("if_reg_c_len_mode"));

        // Program counter register
        let mut reg_pc = Register::new(port_collection.clone(), PORT_NULL_ID, String::from("if_reg_pc"));

        // Adder, mux, and instruction memory (connecting second mux input requires EX stage)
        let addr = Adder::new(port_collection.clone(), reg_c_4.output_port, reg_pc.output_port, String::from("if_addr"));
        let mux = Mux::<2>::new(port_collection.clone(), &[addr.output_port, PORT_NULL_ID], PORT_NULL_ID, String::from("if_mux"));
        let imem = RMemory::new(port_collection.clone(), reg_pc.output_port, reg_c_len_mode.output_port, String::from("if_imem"));

        // Pipeline registers: IR and NPC
        let reg_npc = Register::new(port_collection.clone(), mux.output_port, String::from("if_npc"));
        let reg_ir = Register::new(port_collection.clone(), imem.output_port, String::from("if_ir"));

        // Set PC input to the MUX output
        reg_pc.input = mux.output_port;

        Self {
            reg_pc,
            addr,
            mux,
            imem,
            reg_c_4,
            reg_c_len_mode,
            reg_npc,
            reg_ir
        }
    }
}

impl Component for IFStage {
    fn process_cycle(&mut self) {
        // Constants
        self.reg_c_len_mode.process_cycle();
        self.reg_c_4.process_cycle();

        // Get PC
        self.reg_pc.process_cycle();

        // Increase PC
        self.addr.process_cycle();
        self.mux.process_cycle();

        // Instruction fetch
        self.imem.process_cycle();

        // Pipeline registers
        self.reg_npc.process_cycle();
        self.reg_ir.process_cycle();
    }
}

/*
Processor -- ID Stage
 */

pub struct IDStage {
    /*
    Registers for extracting register numbers
     */

    /// Destination register number
    pub reg_shift_rd: BitSelectionRegister<7, 5>,

    /// Source 1 register number
    pub reg_shift_rs1: BitSelectionRegister<15, 5>,

    /// Source 2 register number
    pub reg_shift_rs2: BitSelectionRegister<20, 5>,

    /*
    Register file and sign extend
    */

    /// Constant register for returning 0 when reading from x0
    pub reg_c_x0: ConstantRegister,

    /// Register file (32 registers)
    pub rf: RegisterFile<32>,

    /// Sign extension unit for processing immediate values
    pub sign_extend: ImmSignExtender,

    /*
    Pipeline registers
    */

    /// IR register for storing current instruction
    pub reg_ir: Register,

    /// Next program counter
    pub reg_npc: Register,

    /// Destination register number
    pub reg_rd: Register,

    /// Source 1 register number
    pub reg_ra: Register,

    /// Source 2 register number
    pub reg_rb: Register,

    /// I-type immediate
    pub reg_imm_i: Register,

    /// S-type immediate
    pub reg_imm_s: Register,

    /// B-type immediate
    pub reg_imm_b: Register,

    /// U-type immediate
    pub reg_imm_u: Register,

    /// J-type immediate
    pub reg_imm_j: Register,
}

impl IDStage {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, if_stage: &IFStage) -> Self {
        // Sign extension unit used for exracting and assembling immediate values from instructions
        let sign_extend = ImmSignExtender::new(port_collection.clone(), if_stage.reg_ir.output_port, String::from("id_imm"));

        // Registers used for extracting register numbers from instruction
        let reg_shift_rs1 = BitSelectionRegister::<15, 5>::new(port_collection.clone(), if_stage.reg_ir.output_port, String::from("id_reg_shift_rs1"));
        let reg_shift_rs2 = BitSelectionRegister::<20, 5>::new(port_collection.clone(), if_stage.reg_ir.output_port, String::from("id_reg_shift_rs2"));
        let reg_shift_rd = BitSelectionRegister::<7, 5>::new(port_collection.clone(), if_stage.reg_ir.output_port, String::from("id_reg_shift_rsd"));

        // Register file
        let mut rf = RegisterFile::<32>::new(
            port_collection.clone(),
            reg_shift_rs1.output_port,
            reg_shift_rs2.output_port,
            PORT_NULL_ID,                    // Originates from WB stage
            PORT_NULL_ID,                   // Originates from WB stage
            PORT_NULL_ID
        );

        // Overwrite input 0 on the output muxes of the register file. Reading x0 should always
        // return 0, which we will read from a constant register.
        let reg_c_x0 = ConstantRegister::new(port_collection.clone(), 0, String::from("id_reg_c_x0"));

        rf.mux_out_a.inputs[0] = reg_c_x0.output_port;
        rf.mux_out_b.inputs[0] = reg_c_x0.output_port;

        // Pipeline registers
        let reg_npc = Register::new(port_collection.clone(), if_stage.reg_npc.output_port, String::from("id_reg_npc"));

        let reg_ra = Register::new(port_collection.clone(), rf.out_a, String::from("id_reg_ra"));
        let reg_rb = Register::new(port_collection.clone(), rf.out_b, String::from("id_reg_rb"));

        let reg_imm_i = Register::new(port_collection.clone(), sign_extend.out_i_type, String::from("id_reg_imm_i"));
        let reg_imm_s = Register::new(port_collection.clone(), sign_extend.out_s_type, String::from("id_reg_imm_s"));
        let reg_imm_b = Register::new(port_collection.clone(), sign_extend.out_b_type, String::from("id_reg_imm_b"));
        let reg_imm_u = Register::new(port_collection.clone(), sign_extend.out_u_type, String::from("id_reg_imm_u"));
        let reg_imm_j = Register::new(port_collection.clone(), sign_extend.out_j_type, String::from("id_reg_imm_j"));

        let reg_rd = Register::new(port_collection.clone(), reg_shift_rd.output_port, String::from("id_reg_rd"));
        let reg_ir = Register::new(port_collection.clone(), if_stage.reg_ir.output_port, String::from("id_ir"));

        Self {
            reg_shift_rd,
            reg_shift_rs1,
            reg_shift_rs2,
            reg_c_x0,
            rf,
            sign_extend,
            reg_ir,
            reg_npc,
            reg_rd,
            reg_ra,
            reg_rb,
            reg_imm_i,
            reg_imm_s,
            reg_imm_b,
            reg_imm_u,
            reg_imm_j
        }
    }
}

impl Component for IDStage {
    fn process_cycle(&mut self) {
        // Constants
        self.reg_c_x0.process_cycle();

        // Main cycle
        self.reg_shift_rd.process_cycle();
        self.reg_shift_rs1.process_cycle();
        self.reg_shift_rs2.process_cycle();

        self.rf.process_cycle();
        self.sign_extend.process_cycle();

        // Pipeline registers
        self.reg_ir.process_cycle();
        self.reg_npc.process_cycle();
        self.reg_rd.process_cycle();
        self.reg_ra.process_cycle();
        self.reg_rb.process_cycle();
        self.reg_imm_i.process_cycle();
        self.reg_imm_s.process_cycle();
        self.reg_imm_b.process_cycle();
        self.reg_imm_u.process_cycle();
        self.reg_imm_j.process_cycle();
    }
}

/*
Processor -- EX Stage
 */

pub struct EXStage {
    /*
    Control registers
     */

    /// Selects which IMM version should be used
    pub ctrl_reg_imm_select: ConstantRegister,

    /// Selects whether ra or NPC should be forwarded to the ALU
    pub ctrl_reg_ra_select: ConstantRegister,

    /// Selects whether rb or imm should be forwarded to the ALU
    pub ctrl_reg_rb_select: ConstantRegister,

    /// Passes the function code to the ALU
    pub ctrl_reg_alu_func: ConstantRegister,

    /// Passes the compare mode (signes vs unsigned) to the comparator
    pub ctrl_reg_comp_mode: ConstantRegister,

    /// Passes the type of branch condition to the branch tester
    pub ctrl_reg_branch_cond: ConstantRegister,

    /// Used to switch between the evaluated branch condition and constant rejection (if the
    /// instruction is not a branch)
    pub ctrl_reg_branch_taken: ConstantRegister,

    /*
    Constant registers
     */

    /// Holds the branch rejection value
    pub reg_c_reject: ConstantRegister,

    /*
    Muxes
     */

    /// Switches between immediate types
    pub mux_imm_select: Mux<5>,

    /// Switches between ra and NPC before passing to the ALU
    pub mux_ra_select: Mux<2>,

    /// Switches between rb and imm before passing to the ALU
    pub mux_rb_select: Mux<2>,

    /// Switches between the evaluated branch condition and constant rejection
    pub mux_branch_taken: Mux<2>,

    /*
    Functional Units
     */

    /// Used to compare values for testing branch conditions
    pub comp: Comparator,

    /// Takes output from the comparator and evaluates branch conditions
    pub branch_test: BranchTester,

    /// Performs arithmetic
    pub alu: ALU,

    /*
    Pipeline Registers
     */

    /// IR register for storing current instruction
    pub reg_ir: Register,

    /// Stores the evaluation of the branch condition
    pub reg_bt: Register,

    /// Stores the result coming from the ALU
    pub reg_alu: Register,

    /// Stores the rb value
    pub reg_rb: Register,

    /// Stores the destination register
    pub reg_rd: Register,

    /*
    Pointer to port collection
     */

    pub port_collection: Rc<RefCell<PortCollection>>,
}

impl EXStage {
    const MUX_IMM_I_TYPE: Word = 0;
    const MUX_IMM_S_TYPE: Word = 1;
    const MUX_IMM_B_TYPE: Word = 2;
    const MUX_IMM_U_TYPE: Word = 3;
    const MUX_IMM_J_TYPE: Word = 4;

    const MUX_RA_NPC: Word = 0;
    const MUX_RA_RA: Word = 1;

    const MUX_RB_RB: Word = 0;
    const MUX_RB_IMM: Word = 1;

    const MUX_BT_REJECT: Word = 0;
    const MUX_BT_BRANCH: Word = 1;

    pub fn new(port_collection: Rc<RefCell<PortCollection>>, id_stage: &IDStage) -> Self {
        let ctrl_reg_imm_select = ConstantRegister::new(port_collection.clone(), 0, String::from("ex_ctrl_reg_imm_select"));
        let ctrl_reg_ra_select = ConstantRegister::new(port_collection.clone(), 0, String::from("ex_ctrl_reg_ra_select"));
        let ctrl_reg_rb_select = ConstantRegister::new(port_collection.clone(), 0, String::from("ex_ctrl_reg_rb_select"));

        let ctrl_reg_alu_func = ConstantRegister::new(port_collection.clone(), 0, String::from("ex_ctrl_reg_alu_func"));
        let ctrl_reg_comp_mode = ConstantRegister::new(port_collection.clone(), 0, String::from("ex_ctrl_reg_comp_mode"));
        let ctrl_reg_branch_cond = ConstantRegister::new(port_collection.clone(), 0, String::from("ex_ctrl_reg_branch_cond"));

        let ctrl_reg_branch_taken = ConstantRegister::new(port_collection.clone(), 0, String::from("ex_ctrl_reg_branch_taken"));

        let mux_imm_select = Mux::<5>::new(
            port_collection.clone(),
            &[
                id_stage.reg_imm_i.output_port,
                id_stage.reg_imm_s.output_port,
                id_stage.reg_imm_b.output_port,
                id_stage.reg_imm_u.output_port,
                id_stage.reg_imm_j.output_port,
            ],
            ctrl_reg_imm_select.output_port,
            String::from("ex_mux_imm_select")
        );

        let mux_ra_select = Mux::<2>::new (
            port_collection.clone(),
            &[
                id_stage.reg_npc.output_port,
                id_stage.reg_ra.output_port
            ],
            ctrl_reg_ra_select.output_port,
            String::from("ex_mux_ra_select")
        );

        let mux_rb_select = Mux::<2>::new (
            port_collection.clone(),
            &[
                id_stage.reg_rb.output_port,
                mux_imm_select.output_port
            ],
            ctrl_reg_rb_select.output_port,
            String::from("ex_mux_rb_select")
        );

        let alu = ALU::new(
            port_collection.clone(),
            mux_ra_select.output_port,
            mux_rb_select.output_port,
            ctrl_reg_alu_func.output_port,
            String::from("ex_alu")
        );

        let comp = Comparator::new(
            port_collection.clone(),
            id_stage.reg_ra.output_port,
            id_stage.reg_rb.output_port,
            ctrl_reg_comp_mode.output_port,
            String::from("ex_comp")
        );

        let branch_test = BranchTester::new(
            port_collection.clone(),
            comp.output_port,
            ctrl_reg_branch_cond.output_port,
            String::from("ex_branch_test")
        );

        let reg_c_reject = ConstantRegister::new(port_collection.clone(), BranchTester::BRANCH_REJECT, String::from("ex_reg_c_reject"));

        let mux_branch_taken = Mux::<2>::new(
          port_collection.clone(),
            &[reg_c_reject.output_port, branch_test.output_port],
          ctrl_reg_branch_taken.output_port,
            String::from("ex_mux_branch_taken")
        );

        // Pipeline registers
        let reg_bt = Register::new(port_collection.clone(), mux_branch_taken.output_port, String::from("ex_reg_bt"));
        let reg_alu = Register::new(port_collection.clone(), alu.output_port, String::from("ex_reg_alu"));
        let reg_rb = Register::new(port_collection.clone(), id_stage.reg_rb.output_port, String::from("ex_reg_rb"));
        let reg_rd = Register::new(port_collection.clone(), id_stage.reg_rd.output_port, String::from("ex_reg_rd"));
        let reg_ir = Register::new(port_collection.clone(), id_stage.reg_ir.output_port, String::from("ex_ir"));

        Self {
            ctrl_reg_imm_select,
            ctrl_reg_ra_select,
            ctrl_reg_rb_select,
            ctrl_reg_alu_func,
            ctrl_reg_comp_mode,
            ctrl_reg_branch_cond,
            ctrl_reg_branch_taken,
            reg_c_reject,
            mux_imm_select,
            mux_ra_select,
            mux_rb_select,
            mux_branch_taken,
            comp,
            branch_test,
            alu,
            reg_ir,
            reg_bt,
            reg_alu,
            reg_rb,
            reg_rd,
            port_collection
        }
    }

    /// Match function codes from a register-register instruction to an ALU operation.
    fn match_alu_func_code_rr(func_3: Word, func_7: Word) -> Word {
        match func_7 {
            func_code_7::BASE => {
                match func_3 {
                    func_code_3::ADD        => ALU::OP_ADD,
                    func_code_3::SLT        => ALU::OP_SLT,
                    func_code_3::SLTU       => ALU::OP_SLTU,
                    func_code_3::AND        => ALU::OP_AND,
                    func_code_3::OR         => ALU::OP_OR,
                    func_code_3::XOR        => ALU::OP_XOR,
                    func_code_3::SLL_SUB    => ALU::OP_SLL,
                    func_code_3::SRL_SRA    => ALU::OP_SRL,
                    _                       => unreachable!()
                }
            }
            func_code_7::BASE_ALT => {
                match func_3 {
                    func_code_3::SLL_SUB    => ALU::OP_SUB,
                    func_code_3::SRL_SRA    => ALU::OP_SRA,
                    _                       => unreachable!()
                }
            }
            func_code_7::MULDIV => {
                match func_3 {
                    func_code_3::MUL => ALU::OP_MUL,
                    _                => unreachable!()
                }
            }
            _ => { unreachable!() }
        }
    }

    /// Match function code(s) from a register-imm instruction to an ALU operation.
    fn match_alu_func_code_ri(func_3: Word, func_7: Word) -> Word {
        match func_3 {
            func_code_3::ADDI        => ALU::OP_ADD,
            func_code_3::SLTI        => ALU::OP_SLT,
            func_code_3::SLTIU       => ALU::OP_SLTU,
            func_code_3::ANDI        => ALU::OP_AND,
            func_code_3::ORI         => ALU::OP_OR,
            func_code_3::XORI        => ALU::OP_XOR,
            func_code_3::SLLI        => ALU::OP_SLL,
            func_code_3::SRLI_SRAI   => {
                // A bit from the func 7 code determines the type of right shift
                if func_7 & func_code_7::BASE_ALT != 0 { ALU::OP_SRA } else { ALU::OP_SRL }
            },
            _                        => unreachable!()
        }
    }

    fn set_control_signals(&mut self, instruction: Word) {
        let op_code = extract_op_code(instruction);
        let (imm_select, ra_select, rb_select, alu_func, comp_mode, branch_cond, branch_taken) = match op_code {
            op_code::OP => {
                let funct_3 = extract_funct_3(instruction);
                let funct_7 = extract_funct_7(instruction);
                let alu_func_code = Self::match_alu_func_code_rr(funct_3, funct_7);

                (0, Self::MUX_RA_RA, Self::MUX_RB_RB, alu_func_code, 0, 0, Self::MUX_BT_REJECT)
            }
            op_code::OP_IMM => {
                let funct_3 = extract_funct_3(instruction);
                let funct_7 = extract_funct_7(instruction);
                let alu_func_code = Self::match_alu_func_code_ri(funct_3, funct_7);

                (Self::MUX_IMM_I_TYPE, Self::MUX_RA_RA, Self::MUX_RB_IMM, alu_func_code, 0, 0, Self::MUX_BT_REJECT)
            }
            op_code::LOAD => {
                (Self::MUX_IMM_I_TYPE, Self::MUX_RA_RA, Self::MUX_RB_IMM, 1, 0, 0, Self::MUX_BT_REJECT)
            }
            op_code::STORE => {
                (Self::MUX_IMM_S_TYPE, Self::MUX_RA_RA, Self::MUX_RB_IMM, 1, 0, 0, Self::MUX_BT_REJECT)
            }
            op_code::JAL => {
                (0, 0, 0, 0, 0, 0, 0)
            }
            op_code::JALR => {
                (0, 0, 0, 0, 0, 0, 0)
            }
            op_code::BRANCH => {
                (0, 0, 0, 0, 0, 0, 0)
            }
            op_code::LUI => {
                (0, 0, 0, 0, 0, 0, 0)
            }
            op_code::AUIPC => {
                (0, 0, 0, 0, 0, 0, 0)
            }
            _ => { unreachable!() }
        };

        self.ctrl_reg_imm_select.constant_value = imm_select;
        self.ctrl_reg_ra_select.constant_value = ra_select;
        self.ctrl_reg_rb_select.constant_value = rb_select;
        self.ctrl_reg_alu_func.constant_value = alu_func;
        self.ctrl_reg_comp_mode.constant_value = comp_mode;
        self.ctrl_reg_branch_cond.constant_value = branch_cond;
        self.ctrl_reg_branch_taken.constant_value = branch_taken;
    }
}

impl Component for EXStage {
    fn process_cycle(&mut self) {
        // Constants
        self.reg_c_reject.process_cycle();

        // Get current instruction
        self.reg_ir.process_cycle();

        // Set control signals
        let instruction = self.port_collection.borrow().get_port_data(self.reg_ir.output_port);
        self.set_control_signals(instruction);

        // Push control signals
        self.ctrl_reg_imm_select.process_cycle();
        self.ctrl_reg_ra_select.process_cycle();
        self.ctrl_reg_rb_select.process_cycle();
        self.ctrl_reg_alu_func.process_cycle();
        self.ctrl_reg_comp_mode.process_cycle();
        self.ctrl_reg_branch_cond.process_cycle();
        self.ctrl_reg_branch_taken.process_cycle();

        // Main cycle
        self.mux_imm_select.process_cycle();
        self.mux_ra_select.process_cycle();
        self.mux_rb_select.process_cycle();

        self.alu.process_cycle();
        self.comp.process_cycle();
        self.branch_test.process_cycle();
        self.mux_branch_taken.process_cycle();

        // Pipeline registers
        self.reg_bt.process_cycle();
        self.reg_bt.process_cycle();
        self.reg_alu.process_cycle();
        self.reg_rd.process_cycle();
        self.reg_rb.process_cycle();
    }
}

/*
Processor -- MEM Stage
 */

pub struct MEMStage {
    /// Enables writing to data memory
    pub ctrl_reg_write_enable: ConstantRegister,

    /// Sets the length mode for memory accesses
    pub ctrl_reg_len_mode: ConstantRegister,

    /// Data memory (read/write)
    pub dmem: RWMemory,

    /// Stores the result from memory fetch
    pub reg_mem: Register,

    /// Stores the result from the ALU
    pub reg_alu: Register,

    /// Stores the destination register
    pub reg_rd: Register,

    /// IR register for storing current instruction
    pub reg_ir: Register,

    /*
    Pointer to port collection
     */

    pub port_collection: Rc<RefCell<PortCollection>>,
}

impl MEMStage {
    pub fn new(port_collection: Rc<RefCell<PortCollection>>, ex_stage: &EXStage) -> Self {
        // Control registers
        let ctrl_reg_write_enable = ConstantRegister::new(port_collection.clone(), 0, String::from("mem_ctrl_reg_write_enable"));
        let ctrl_reg_len_mode = ConstantRegister::new(port_collection.clone(), 0, String::from("mem_ctrl_reg_len_mode"));

        // Data memory
        let dmem = RWMemory::new(
            port_collection.clone(),
            ex_stage.reg_alu.output_port,
            ctrl_reg_len_mode.output_port,
            ex_stage.reg_rb.output_port,
            ctrl_reg_write_enable.output_port,
            String::from("mem_dmem")
        );

        // Pipeline registers
        let reg_mem = Register::new(port_collection.clone(), dmem.output_port, String::from("mem_reg_mem"));
        let reg_alu = Register::new(port_collection.clone(), ex_stage.reg_alu.output_port, String::from("mem_reg_alu"));
        let reg_rd = Register::new(port_collection.clone(), ex_stage.reg_rd.output_port, String::from("mem_reg_rd"));
        let reg_ir = Register::new(port_collection.clone(), ex_stage.reg_ir.output_port, String::from("mem_ir"));

        Self {
            ctrl_reg_write_enable,
            ctrl_reg_len_mode,
            dmem,
            reg_mem,
            reg_alu,
            reg_rd,
            reg_ir,
            port_collection
        }
    }

    fn set_control_signals(&mut self, instruction: Word) {
        let op_code = extract_op_code(instruction);

        // We use MEM_LEN_BYTE when the memory read result is discarded to avoid
        // unaligned access errors from garbage inputs. (Byte addresses are always aligned)

        let (write_enable, len_mode) = match op_code {
            op_code::OP => {
                (0, MEM_LEN_BYTE)
            }
            op_code::OP_IMM => {
                (0, MEM_LEN_BYTE)
            }
            op_code::LOAD => {
                (0, MEM_LEN_WORD)
            }
            op_code::STORE => {
                (1, MEM_LEN_WORD)
            }
            op_code::JAL => {
                (0, MEM_LEN_BYTE)
            }
            op_code::JALR => {
                (0, MEM_LEN_BYTE)
            }
            op_code::BRANCH => {
                (0, MEM_LEN_BYTE)
            }
            op_code::LUI => {
                (0, MEM_LEN_BYTE)
            }
            op_code::AUIPC => {
                (0, MEM_LEN_BYTE)
            }
            _ => { unreachable!() }
        };

        // Set control signals
        self.ctrl_reg_write_enable.constant_value = write_enable;
        self.ctrl_reg_len_mode.constant_value = len_mode;
    }
}

impl Component for MEMStage {
    fn process_cycle(&mut self) {
        // Get current instruction
        self.reg_ir.process_cycle();

        // Set control signals
        let instruction = self.port_collection.borrow().get_port_data(self.reg_ir.output_port);
        self.set_control_signals(instruction);

        // Push control signals
        self.ctrl_reg_len_mode.process_cycle();
        self.ctrl_reg_write_enable.process_cycle();

        // Main cycle
        self.dmem.process_cycle();

        // Pipeline register
        self.reg_alu.process_cycle();
        self.reg_mem.process_cycle();
        self.reg_rd.process_cycle();
    }
}

/*
Processor -- WB Stage
 */

pub struct WBStage {
    /// Controls which value to use for write back
    pub ctrl_reg_value_select: ConstantRegister,

    /// Controls whether a write is actually committed to the register file
    pub ctrl_reg_write_enable: ConstantRegister,

    /// Switches between ALU output and MEM output
    pub mux_value: Mux<2>,

    /// Holds the destination register of the write back
    pub reg_rd: Register,

    /// IR register for storing current instruction
    pub reg_ir: Register,

    /*
    Pointer to port collection
     */

    pub port_collection: Rc<RefCell<PortCollection>>,
}

impl WBStage {
    pub const MUX_VALUE_MEM: Word = 0;
    pub const MUX_VALUE_ALU: Word = 1;

    pub fn new(port_collection: Rc<RefCell<PortCollection>>, mem_stage: &MEMStage) -> Self {

        let ctrl_reg_value_select = ConstantRegister::new(port_collection.clone(), 0, String::from("wb_ctrl_reg_value_select"));
        let ctrl_reg_write_enable = ConstantRegister::new(port_collection.clone(), 0, String::from("wb_ctrl_reg_write_enable"));

        let mux_value = Mux::<2>::new(
            port_collection.clone(),
            &[mem_stage.reg_mem.output_port, mem_stage.reg_alu.output_port],
            ctrl_reg_value_select.output_port,
            String::from("wb_mux_value")
        );

        let reg_rd = Register::new(port_collection.clone(), mem_stage.reg_rd.output_port, String::from("wb_rd"));
        let reg_ir = Register::new(port_collection.clone(), mem_stage.reg_ir.output_port, String::from("wb_ir"));

        Self {
            ctrl_reg_value_select,
            ctrl_reg_write_enable,
            mux_value,
            reg_rd,
            reg_ir,
            port_collection
        }
    }

    fn set_control_signals(&mut self, instruction: Word) {
        let op_code = extract_op_code(instruction);

        let (value_select, write_enable) = match op_code {
            op_code::OP => {
                (Self::MUX_VALUE_ALU, 1)
            }
            op_code::OP_IMM => {
                (Self::MUX_VALUE_ALU, 1)
            }
            op_code::LOAD => {
                (Self::MUX_VALUE_MEM, 1)
            }
            op_code::STORE => {
                (Self::MUX_VALUE_ALU, 0)
            }
            op_code::JAL => {
                todo!()
            }
            op_code::JALR => {
                todo!()
            }
            op_code::BRANCH => {
                todo!()
            }
            op_code::LUI => {
                todo!()
            }
            op_code::AUIPC => {
                todo!()
            }
            _ => { unreachable!() }
        };

        // Set control signals
        self.ctrl_reg_value_select.constant_value = value_select;
        self.ctrl_reg_write_enable.constant_value = write_enable;
    }
}

impl Component for WBStage {
    fn process_cycle(&mut self) {
        // Get current instruction
        self.reg_ir.process_cycle();

        // Set control signals
        let instruction = self.port_collection.borrow().get_port_data(self.reg_ir.output_port);
        self.set_control_signals(instruction);

        // Push control signals
        self.ctrl_reg_write_enable.process_cycle();
        self.ctrl_reg_value_select.process_cycle();

        // Main cycle
        self.reg_rd.process_cycle();
        self.mux_value.process_cycle();
    }
}

/*
Processor
 */

pub struct Processor {
    pub port_collection: Rc<RefCell<PortCollection>>,

    pub if_stage: IFStage,
    pub id_stage: IDStage,
    pub ex_stage: EXStage,
    pub mem_stage: MEMStage,
    pub wb_stage: WBStage,
}

impl Processor {
    pub fn new() -> Self {
        let port_collection = Rc::new(RefCell::new(PortCollection::new()));

        let mut if_stage = IFStage::new(port_collection.clone());
        let mut id_stage = IDStage::new(port_collection.clone(), &if_stage);
        let ex_stage = EXStage::new(port_collection.clone(), &id_stage);
        let mem_stage = MEMStage::new(port_collection.clone(), &ex_stage);
        let wb_stage = WBStage::new(port_collection.clone(), &mem_stage);

        // Make final loopback connections

        // Branch
        if_stage.mux.inputs[1] = ex_stage.reg_alu.output_port;
        if_stage.mux.selection_input = ex_stage.reg_bt.output_port;

        // Write back
        id_stage.rf.set_write_inputs(
            wb_stage.mux_value.output_port,
            wb_stage.reg_rd.output_port,
            wb_stage.ctrl_reg_write_enable.output_port
        );

        // Initialize instruction registers with NOP
        port_collection.borrow_mut().set_port_data(if_stage.reg_ir.output_port, NOP);
        port_collection.borrow_mut().set_port_data(id_stage.reg_ir.output_port, NOP);
        port_collection.borrow_mut().set_port_data(ex_stage.reg_ir.output_port, NOP);
        port_collection.borrow_mut().set_port_data(mem_stage.reg_ir.output_port, NOP);
        port_collection.borrow_mut().set_port_data(wb_stage.reg_ir.output_port, NOP);

        Self {
            port_collection,
            if_stage,
            id_stage,
            ex_stage,
            mem_stage,
            wb_stage
        }
    }
}

impl Component for Processor {
    fn process_cycle(&mut self) {
        self.wb_stage.process_cycle();
        self.mem_stage.process_cycle();
        self.ex_stage.process_cycle();
        self.id_stage.process_cycle();
        self.if_stage.process_cycle();
    }
}

#[test]
pub fn test_processor() {
    let mut processor = Processor::new();

    // let program = assembler::assemble_program(
    //     "\
    //     MVI x1 30;\
    //     MVI x2 20;\
    //     MVI x3 880;\
    //     MVI x4 80;\
    //     \
    //     ADD x1 x1 x2;\
    //     NOP;\
    //     SUB x10 x3 x4;\
    //     NOP;\
    //     NOP;\
    //     ADD x1 x1 x10;\
    //     "
    // ).expect("Error compiling program");

    let program = assembler::assemble_program(
        "\
        MVI x1 30;\
        MVI x2 80;\
        NOP;\
        NOP;\
        ADD x1 x1 x2;\
        NOP;\
        MVI x3 200;\
        STORE x1 x0 0;\
        LOAD x10 x0 0;\
        NOP;\
        NOP;\
        ADD x1 x10 x3;\
        "
    ).expect("Error compiling program");

    processor.if_stage.imem.content = assembler::program_to_mem::<MEM_SIZE>(&program);

    for i in 0..16 {
        processor.process_cycle();

        println!("----------- Cycle {} -----------", i);
        println!("if_stage: {}", processor.port_collection.borrow().get_port_data(processor.if_stage.reg_ir.output_port));
        println!("id_stage: {}", processor.port_collection.borrow().get_port_data(processor.id_stage.reg_ir.output_port));
        println!("ex_stage: {}", processor.port_collection.borrow().get_port_data(processor.ex_stage.reg_ir.output_port));
        println!("mem_stage: {}", processor.port_collection.borrow().get_port_data(processor.mem_stage.reg_ir.output_port));
        println!("wb_stage: {}", processor.port_collection.borrow().get_port_data(processor.wb_stage.reg_ir.output_port));

        println!();
        println!("Registers:");
        println!("reg_1: {}", processor.port_collection.borrow().get_port_data(processor.id_stage.rf.registers[1].output_port));
        println!("reg_10: {}", processor.port_collection.borrow().get_port_data(processor.id_stage.rf.registers[10].output_port));
        println!();
    }
}

#[test]
fn tmp() {
    const WRITE: Word = 2;
    const READ: Word = 1;
    const OTHER: Word = 0;

    let mut count = 0;
    let mut not_stall: bool = false;

    fn has_hazard(rd_instr: Word, wr_instr: Word) -> bool {
        rd_instr == READ && wr_instr == WRITE
    }

    let mut if_stage = OTHER;
    let mut id_stage = OTHER;
    let mut ex_stage = OTHER;
    let mut mem_stage = OTHER;
    let mut wb_stage = OTHER;

    let mut pc = 0;
    let program = vec![
        WRITE,
        READ,
        READ
    ];

    println!("{} \t {} \t {} \t {} \t {} \t\t {} \t {}", "IF", "ID", "EX", "MEM", "WB", "S", "C");

    for _ in 0..10 {
        wb_stage = mem_stage;
        mem_stage = ex_stage;
        ex_stage = if not_stall { id_stage } else { ex_stage };
        id_stage = if not_stall { if_stage } else { id_stage };

        pc = if not_stall { pc + 1 } else { pc };
        if_stage = *program.get(pc).unwrap_or((&OTHER));

        if count == 0 {
            if has_hazard(id_stage, ex_stage) {
                count = 3;
            } else if has_hazard(id_stage, mem_stage) {
                count = 2
            }
        } else if count == 1 {
            if has_hazard(id_stage, ex_stage) {
                count = 3;
            } else {
                count = count - 1
            }
        } else {
            count = count - 1;
        }

        not_stall = !(count >= 2);

        println!("{} \t {} \t {} \t {} \t {} \t\t {} \t {}", if_stage, id_stage, ex_stage, mem_stage, wb_stage, not_stall, count);
    }
}