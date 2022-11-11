use crate::circuit::*;
use crate::components::*;
use crate::isa::*;

/*
Processor -- IF Stage
 */

pub struct IFStage {
    /// Program counter register
    pub reg_pc: GuardedRegister,

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
}

impl IFStage {
    pub fn new(port_collection: &mut PortCollection) -> Self {
        // First create all constant registers
        let reg_c_4 = ConstantRegister::new(port_collection, 4, String::from("if_reg_c_4"));
        let reg_c_len_mode = ConstantRegister::new(
            port_collection,
            MEM_LEN_WORD,
            String::from("if_reg_c_len_mode"),
        );

        // Program counter register (still need connection to interlock unit and branch feedback)
        let mut reg_pc = GuardedRegister::new_with_initial(
            port_collection,
            PORT_NULL_ID,
            PORT_NULL_ID,
            0,
            String::from("if_reg_pc"),
        );

        // Adder, mux, and instruction memory (connecting second mux input requires EX stage)
        let addr = Adder::new(
            port_collection,
            reg_c_4.output_port,
            reg_pc.output_port,
            String::from("if_addr"),
        );
        let mux = Mux::<2>::new(
            port_collection,
            &[addr.output_port, PORT_NULL_ID],
            PORT_NULL_ID,
            String::from("if_mux"),
        );
        let imem = RMemory::new(
            port_collection,
            reg_pc.output_port,
            reg_c_len_mode.output_port,
            String::from("if_imem"),
        );

        // Set PC input to the MUX output
        reg_pc.input = mux.output_port;

        Self {
            reg_pc,
            addr,
            mux,
            imem,
            reg_c_4,
            reg_c_len_mode,
        }
    }

    /// Makes necessary connections for interlock unit.
    pub fn connect_interlock_unit(&mut self, iu_out: PortID) {
        self.reg_pc.input_enable = iu_out;
    }

    /// Cycles only the mux.
    ///
    /// The IF-stage mux takes branch-related inputs from the EX stage. These can be updated after the IF
    /// stage has completed its cycle. An extra update of the mux is required in this case, such that the
    /// CPU is not left in an invalid state.
    pub fn cycle_mux(&mut self, port_collection: &mut PortCollection) {
        self.mux.process_cycle(port_collection);
    }
}

impl Component for IFStage {
    fn process_cycle(&mut self, port_collection: &mut PortCollection) {
        // Constants
        self.reg_c_len_mode.process_cycle(port_collection);
        self.reg_c_4.process_cycle(port_collection);

        // Get PC for next instruction
        self.reg_pc.process_cycle(port_collection);

        // Compute NPC
        self.addr.process_cycle(port_collection);
        self.mux.process_cycle(port_collection);

        // Instruction fetch
        self.imem.process_cycle(port_collection);
    }
}

/*
Processor -- IF/ID pipeline registers
*/

pub struct IFIDPipelineRegisters {
    /// NPC register storing next program counter
    pub reg_npc: GuardedRegister,

    /// IR register for storing fetched instruction
    pub reg_ir: GuardedRegister,
}

impl Component for IFIDPipelineRegisters {
    fn process_cycle(&mut self, port_collection: &mut PortCollection) {
        // Process all registers
        self.reg_npc.process_cycle(port_collection);
        self.reg_ir.process_cycle(port_collection);
    }
}

impl IFIDPipelineRegisters {
    pub fn new(port_collection: &mut PortCollection, if_stage: &IFStage) -> Self {
        // Pipeline registers: IR and NPC (still need connection to interlock unit)
        let reg_npc = GuardedRegister::new(
            port_collection,
            if_stage.addr.output_port,
            PORT_NULL_ID,
            String::from("IFID_npc"),
        );
        let reg_ir = GuardedRegister::new(
            port_collection,
            if_stage.imem.output_port,
            PORT_NULL_ID,
            String::from("IFID_ir"),
        );

        Self { reg_npc, reg_ir }
    }

    pub fn connect_interlock_unit(&mut self, iu_out: PortID) {
        self.reg_ir.input_enable = iu_out;
        self.reg_npc.input_enable = iu_out;
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
}

impl IDStage {
    pub fn new(
        port_collection: &mut PortCollection,
        if_id_pipeline_regs: &IFIDPipelineRegisters,
    ) -> Self {
        // Sign extension unit used for exracting and assembling immediate values from instructions
        let sign_extend = ImmSignExtender::new(
            port_collection,
            if_id_pipeline_regs.reg_ir.output_port,
            String::from("id_imm"),
        );

        // Registers used for extracting register numbers from instruction
        let reg_shift_rs1 = BitSelectionRegister::<15, 5>::new(
            port_collection,
            if_id_pipeline_regs.reg_ir.output_port,
            String::from("id_reg_shift_rs1"),
        );
        let reg_shift_rs2 = BitSelectionRegister::<20, 5>::new(
            port_collection,
            if_id_pipeline_regs.reg_ir.output_port,
            String::from("id_reg_shift_rs2"),
        );
        let reg_shift_rd = BitSelectionRegister::<7, 5>::new(
            port_collection,
            if_id_pipeline_regs.reg_ir.output_port,
            String::from("id_reg_shift_rsd"),
        );

        // Register file
        let mut rf = RegisterFile::<32>::new(
            port_collection,
            reg_shift_rs1.output_port,
            reg_shift_rs2.output_port,
            PORT_NULL_ID, // Originates from WB stage
            PORT_NULL_ID, // Originates from WB stage
            PORT_NULL_ID,
        );

        // Overwrite input 0 on the output muxes of the register file. Reading x0 should always
        // return 0, which we will read from a constant register.
        let reg_c_x0 =
            ConstantRegister::new(port_collection, 0, String::from("id_reg_c_x0"));

        rf.mux_out_a.inputs[0] = reg_c_x0.output_port;
        rf.mux_out_b.inputs[0] = reg_c_x0.output_port;

        Self {
            reg_shift_rd,
            reg_shift_rs1,
            reg_shift_rs2,
            reg_c_x0,
            rf,
            sign_extend,
        }
    }
}

impl Component for IDStage {
    fn process_cycle(&mut self, port_collection: &mut PortCollection) {
        // Constants
        self.reg_c_x0.process_cycle(port_collection);

        // Main cycle
        self.reg_shift_rd.process_cycle(port_collection);
        self.reg_shift_rs1.process_cycle(port_collection);
        self.reg_shift_rs2.process_cycle(port_collection);

        self.rf.process_cycle(port_collection);
        self.sign_extend.process_cycle(port_collection);
    }
}

/*
Processor -- ID/EX pipeline registers
*/

pub struct IDEXPipelineRegisters {
    /// IR register for storing current instruction
    pub reg_ir: GuardedRegister,

    /// Next program counter
    pub reg_npc: GuardedRegister,

    /// Destination register number
    pub reg_rd: GuardedRegister,

    /// Source 1 register number
    pub reg_ra: GuardedRegister,

    /// Source 2 register number
    pub reg_rb: GuardedRegister,

    /// I-type immediate
    pub reg_imm_i: GuardedRegister,

    /// S-type immediate
    pub reg_imm_s: GuardedRegister,

    /// B-type immediate
    pub reg_imm_b: GuardedRegister,

    /// U-type immediate
    pub reg_imm_u: GuardedRegister,

    /// J-type immediate
    pub reg_imm_j: GuardedRegister,
}

impl Component for IDEXPipelineRegisters {
    fn process_cycle(&mut self, port_collection: &mut PortCollection) {
        // Process all registers
        self.reg_ir.process_cycle(port_collection);
        self.reg_npc.process_cycle(port_collection);
        self.reg_rd.process_cycle(port_collection);
        self.reg_ra.process_cycle(port_collection);
        self.reg_rb.process_cycle(port_collection);
        self.reg_imm_i.process_cycle(port_collection);
        self.reg_imm_s.process_cycle(port_collection);
        self.reg_imm_b.process_cycle(port_collection);
        self.reg_imm_u.process_cycle(port_collection);
        self.reg_imm_j.process_cycle(port_collection);
    }
}

impl IDEXPipelineRegisters {
    pub fn new(
        port_collection: &mut PortCollection,
        id_stage: &IDStage,
        if_id_pipeline_regs: &IFIDPipelineRegisters,
    ) -> Self {
        // Pipeline registers (still need connection to interlock unit)
        let reg_ir = GuardedRegister::new(
            port_collection,
            if_id_pipeline_regs.reg_ir.output_port,
            PORT_NULL_ID,
            String::from("IDEX_ir"),
        );

        let reg_npc = GuardedRegister::new(
            port_collection,
            if_id_pipeline_regs.reg_npc.output_port,
            PORT_NULL_ID,
            String::from("IDEX_npc"),
        );

        let reg_ra = GuardedRegister::new(
            port_collection,
            id_stage.rf.out_a,
            PORT_NULL_ID,
            String::from("IDEX_ra"),
        );
        let reg_rb = GuardedRegister::new(
            port_collection,
            id_stage.rf.out_b,
            PORT_NULL_ID,
            String::from("IDEX_rb"),
        );
        let reg_rd = GuardedRegister::new(
            port_collection,
            id_stage.reg_shift_rd.output_port,
            PORT_NULL_ID,
            String::from("IDEX_rd"),
        );

        let reg_imm_i = GuardedRegister::new(
            port_collection,
            id_stage.sign_extend.out_i_type,
            PORT_NULL_ID,
            String::from("IDEX_imm_i"),
        );
        let reg_imm_s = GuardedRegister::new(
            port_collection,
            id_stage.sign_extend.out_s_type,
            PORT_NULL_ID,
            String::from("IDEX_imm_s"),
        );
        let reg_imm_b = GuardedRegister::new(
            port_collection,
            id_stage.sign_extend.out_b_type,
            PORT_NULL_ID,
            String::from("IDEX_imm_b"),
        );
        let reg_imm_u = GuardedRegister::new(
            port_collection,
            id_stage.sign_extend.out_u_type,
            PORT_NULL_ID,
            String::from("IDEX_imm_u"),
        );
        let reg_imm_j = GuardedRegister::new(
            port_collection,
            id_stage.sign_extend.out_j_type,
            PORT_NULL_ID,
            String::from("IDEX_imm_j"),
        );

        Self {
            reg_ir,
            reg_npc,
            reg_rd,
            reg_ra,
            reg_rb,
            reg_imm_i,
            reg_imm_s,
            reg_imm_b,
            reg_imm_u,
            reg_imm_j,
        }
    }

    /// Makes necessary connections for interlock unit.
    pub fn connect_interlock_unit(&mut self, iu_out: PortID) {
        self.reg_ir.input_enable = iu_out;
        self.reg_npc.input_enable = iu_out;
        self.reg_rd.input_enable = iu_out;
        self.reg_ra.input_enable = iu_out;
        self.reg_rb.input_enable = iu_out;
        self.reg_imm_i.input_enable = iu_out;
        self.reg_imm_s.input_enable = iu_out;
        self.reg_imm_b.input_enable = iu_out;
        self.reg_imm_u.input_enable = iu_out;
        self.reg_imm_j.input_enable = iu_out;
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
    Save the PortID of the instruction register in the ID/EX pipeline registers
    */
    pub reg_ir_id: PortID,
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

    pub fn new(
        port_collection: &mut PortCollection,
        id_ex_pipeline_regs: &IDEXPipelineRegisters,
    ) -> Self {
        let ctrl_reg_imm_select = ConstantRegister::new(
            port_collection,
            0,
            String::from("ex_ctrl_reg_imm_select"),
        );
        let ctrl_reg_ra_select = ConstantRegister::new(
            port_collection,
            0,
            String::from("ex_ctrl_reg_ra_select"),
        );
        let ctrl_reg_rb_select = ConstantRegister::new(
            port_collection,
            0,
            String::from("ex_ctrl_reg_rb_select"),
        );

        let ctrl_reg_alu_func = ConstantRegister::new(
            port_collection,
            0,
            String::from("ex_ctrl_reg_alu_func"),
        );
        let ctrl_reg_comp_mode = ConstantRegister::new(
            port_collection,
            0,
            String::from("ex_ctrl_reg_comp_mode"),
        );
        let ctrl_reg_branch_cond = ConstantRegister::new(
            port_collection,
            0,
            String::from("ex_ctrl_reg_branch_cond"),
        );

        let ctrl_reg_branch_taken = ConstantRegister::new(
            port_collection,
            0,
            String::from("ex_ctrl_reg_branch_taken"),
        );

        let mux_imm_select = Mux::<5>::new(
            port_collection,
            &[
                id_ex_pipeline_regs.reg_imm_i.output_port,
                id_ex_pipeline_regs.reg_imm_s.output_port,
                id_ex_pipeline_regs.reg_imm_b.output_port,
                id_ex_pipeline_regs.reg_imm_u.output_port,
                id_ex_pipeline_regs.reg_imm_j.output_port,
            ],
            ctrl_reg_imm_select.output_port,
            String::from("ex_mux_imm_select"),
        );

        let mux_ra_select = Mux::<2>::new(
            port_collection,
            &[
                id_ex_pipeline_regs.reg_npc.output_port,
                id_ex_pipeline_regs.reg_ra.output_port,
            ],
            ctrl_reg_ra_select.output_port,
            String::from("ex_mux_ra_select"),
        );

        let mux_rb_select = Mux::<2>::new(
            port_collection,
            &[
                id_ex_pipeline_regs.reg_rb.output_port,
                mux_imm_select.output_port,
            ],
            ctrl_reg_rb_select.output_port,
            String::from("ex_mux_rb_select"),
        );

        let alu = ALU::new(
            port_collection,
            mux_ra_select.output_port,
            mux_rb_select.output_port,
            ctrl_reg_alu_func.output_port,
            String::from("ex_alu"),
        );

        let comp = Comparator::new(
            port_collection,
            id_ex_pipeline_regs.reg_ra.output_port,
            id_ex_pipeline_regs.reg_rb.output_port,
            ctrl_reg_comp_mode.output_port,
            String::from("ex_comp"),
        );

        let branch_test = BranchTester::new(
            port_collection,
            comp.output_port,
            ctrl_reg_branch_cond.output_port,
            String::from("ex_branch_test"),
        );

        let reg_c_reject = ConstantRegister::new(
            port_collection,
            BranchTester::BRANCH_REJECT,
            String::from("ex_reg_c_reject"),
        );

        let mux_branch_taken = Mux::<2>::new(
            port_collection,
            &[reg_c_reject.output_port, branch_test.output_port],
            ctrl_reg_branch_taken.output_port,
            String::from("ex_mux_branch_taken"),
        );

        let reg_ir_id = id_ex_pipeline_regs.reg_ir.output_port;

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
            reg_ir_id,
        }
    }

    /// Match function codes from a register-register instruction to an ALU operation.
    fn match_alu_func_code_rr(func_3: Word, func_7: Word) -> Word {
        match func_7 {
            func_code_7::BASE => match func_3 {
                func_code_3::ADD => ALU::OP_ADD,
                func_code_3::SLT => ALU::OP_SLT,
                func_code_3::SLTU => ALU::OP_SLTU,
                func_code_3::AND => ALU::OP_AND,
                func_code_3::OR => ALU::OP_OR,
                func_code_3::XOR => ALU::OP_XOR,
                func_code_3::SLL_SUB => ALU::OP_SLL,
                func_code_3::SRL_SRA => ALU::OP_SRL,
                _ => unreachable!(),
            },
            func_code_7::BASE_ALT => match func_3 {
                func_code_3::SLL_SUB => ALU::OP_SUB,
                func_code_3::SRL_SRA => ALU::OP_SRA,
                _ => unreachable!(),
            },
            func_code_7::MULDIV => match func_3 {
                func_code_3::MUL => ALU::OP_MUL,
                _ => unreachable!(),
            },
            _ => {
                unreachable!()
            }
        }
    }

    /// Match function code(s) from a register-imm instruction to an ALU operation.
    fn match_alu_func_code_ri(func_3: Word, func_7: Word) -> Word {
        match func_3 {
            func_code_3::ADDI => ALU::OP_ADD,
            func_code_3::SLTI => ALU::OP_SLT,
            func_code_3::SLTIU => ALU::OP_SLTU,
            func_code_3::ANDI => ALU::OP_AND,
            func_code_3::ORI => ALU::OP_OR,
            func_code_3::XORI => ALU::OP_XOR,
            func_code_3::SLLI => ALU::OP_SLL,
            func_code_3::SRLI_SRAI => {
                // A bit from the func 7 code determines the type of right shift
                if func_7 & func_code_7::BASE_ALT != 0 {
                    ALU::OP_SRA
                } else {
                    ALU::OP_SRL
                }
            }
            _ => unreachable!(),
        }
    }

    fn set_control_signals(&mut self, instruction: Word) {
        let op_code = extract_op_code(instruction);
        let (imm_select, ra_select, rb_select, alu_func, comp_mode, branch_cond, branch_taken) =
            match op_code {
                op_code::OP => {
                    let funct_3 = extract_funct_3(instruction);
                    let funct_7 = extract_funct_7(instruction);
                    let alu_func_code = Self::match_alu_func_code_rr(funct_3, funct_7);

                    (
                        0,
                        Self::MUX_RA_RA,
                        Self::MUX_RB_RB,
                        alu_func_code,
                        0,
                        0,
                        Self::MUX_BT_REJECT,
                    )
                }
                op_code::OP_IMM => {
                    let funct_3 = extract_funct_3(instruction);
                    let funct_7 = extract_funct_7(instruction);
                    let alu_func_code = Self::match_alu_func_code_ri(funct_3, funct_7);

                    (
                        Self::MUX_IMM_I_TYPE,
                        Self::MUX_RA_RA,
                        Self::MUX_RB_IMM,
                        alu_func_code,
                        0,
                        0,
                        Self::MUX_BT_REJECT,
                    )
                }
                op_code::LOAD => (
                    Self::MUX_IMM_I_TYPE,
                    Self::MUX_RA_RA,
                    Self::MUX_RB_IMM,
                    1,
                    0,
                    0,
                    Self::MUX_BT_REJECT,
                ),
                op_code::STORE => (
                    Self::MUX_IMM_S_TYPE,
                    Self::MUX_RA_RA,
                    Self::MUX_RB_IMM,
                    1,
                    0,
                    0,
                    Self::MUX_BT_REJECT,
                ),
                op_code::JAL => {
                    todo!()
                }
                op_code::JALR => {
                    todo!()
                }
                op_code::BRANCH => {
                    let funct_3 = extract_funct_3(instruction);
                    let comp_mode = funct_3 >> 2;
                    let condition = funct_3 & 0b_011;
                    (
                        Self::MUX_IMM_B_TYPE,
                        Self::MUX_RA_NPC,
                        Self::MUX_RB_IMM,
                        ALU::OP_ADD,
                        comp_mode,
                        condition,
                        Self::MUX_BT_BRANCH,
                    )
                }
                op_code::LUI => {
                    todo!()
                }
                op_code::AUIPC => {
                    todo!()
                }
                _ => {
                    unreachable!()
                }
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
    fn process_cycle(&mut self, port_collection: &mut PortCollection) {
        // Constants
        self.reg_c_reject.process_cycle(port_collection);

        // Get current instruction and set control signals
        let instruction = port_collection.get_port_data(self.reg_ir_id);
        self.set_control_signals(instruction);

        // Push control signals
        self.ctrl_reg_imm_select.process_cycle(port_collection);
        self.ctrl_reg_ra_select.process_cycle(port_collection);
        self.ctrl_reg_rb_select.process_cycle(port_collection);
        self.ctrl_reg_alu_func.process_cycle(port_collection);
        self.ctrl_reg_comp_mode.process_cycle(port_collection);
        self.ctrl_reg_branch_cond.process_cycle(port_collection);
        self.ctrl_reg_branch_taken.process_cycle(port_collection);

        // Main cycle
        self.mux_imm_select.process_cycle(port_collection);
        self.mux_ra_select.process_cycle(port_collection);
        self.mux_rb_select.process_cycle(port_collection);

        self.alu.process_cycle(port_collection);
        self.comp.process_cycle(port_collection);
        self.branch_test.process_cycle(port_collection);
        self.mux_branch_taken.process_cycle(port_collection);
    }
}

/*
Processor -- IF/ID pipeline registers
*/

pub struct EXMEMPipelineRegisters {
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
}

impl Component for EXMEMPipelineRegisters {
    fn process_cycle(&mut self, port_collection: &mut PortCollection) {
        // Process all registers
        self.reg_ir.process_cycle(port_collection);
        self.reg_bt.process_cycle(port_collection);
        self.reg_bt.process_cycle(port_collection);
        self.reg_alu.process_cycle(port_collection);
        self.reg_rd.process_cycle(port_collection);
        self.reg_rb.process_cycle(port_collection);
    }
}

impl EXMEMPipelineRegisters {
    pub fn new(
        port_collection: &mut PortCollection,
        ex_stage: &EXStage,
        id_ex_pipeline_regs: &IDEXPipelineRegisters,
    ) -> Self {
        let reg_bt = Register::new(
            port_collection,
            ex_stage.mux_branch_taken.output_port,
            String::from("EXMEM_reg_bt"),
        );
        let reg_alu = Register::new(
            port_collection,
            ex_stage.alu.output_port,
            String::from("EXMEM_reg_alu"),
        );
        let reg_rb = Register::new(
            port_collection,
            id_ex_pipeline_regs.reg_rb.output_port,
            String::from("EXMEM_reg_rb"),
        );
        let reg_rd = Register::new(
            port_collection,
            id_ex_pipeline_regs.reg_rd.output_port,
            String::from("EXMEM_reg_rd"),
        );
        let reg_ir = Register::new(
            port_collection,
            id_ex_pipeline_regs.reg_ir.output_port,
            String::from("EXMEM_ir"),
        );

        Self {
            reg_ir,
            reg_bt,
            reg_alu,
            reg_rb,
            reg_rd,
        }
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

    /*
    Save the PortID of the instruction register in the ID/EX pipeline registers
    */
    pub reg_ir_id: PortID,
}

impl MEMStage {
    pub fn new(
        port_collection: &mut PortCollection,
        ex_mem_pipeline_regs: &EXMEMPipelineRegisters,
    ) -> Self {
        // Control registers
        let ctrl_reg_write_enable = ConstantRegister::new(
            port_collection,
            0,
            String::from("mem_ctrl_reg_write_enable"),
        );
        let ctrl_reg_len_mode = ConstantRegister::new(
            port_collection,
            0,
            String::from("mem_ctrl_reg_len_mode"),
        );

        // Data memory
        let dmem = RWMemory::new(
            port_collection,
            ex_mem_pipeline_regs.reg_alu.output_port,
            ctrl_reg_len_mode.output_port,
            ex_mem_pipeline_regs.reg_rb.output_port,
            ctrl_reg_write_enable.output_port,
            String::from("mem_dmem"),
        );

        let reg_ir_id = ex_mem_pipeline_regs.reg_ir.output_port;

        Self {
            ctrl_reg_write_enable,
            ctrl_reg_len_mode,
            dmem,
            reg_ir_id,
        }
    }

    fn set_control_signals(&mut self, instruction: Word) {
        let op_code = extract_op_code(instruction);

        // We use MEM_LEN_BYTE when the memory read result is discarded to avoid
        // unaligned access errors from garbage inputs. (Byte addresses are always aligned)

        let (write_enable, len_mode) = match op_code {
            op_code::OP => (0, MEM_LEN_BYTE),
            op_code::OP_IMM => (0, MEM_LEN_BYTE),
            op_code::LOAD => (0, MEM_LEN_WORD),
            op_code::STORE => (1, MEM_LEN_WORD),
            op_code::JAL => (0, MEM_LEN_BYTE),
            op_code::JALR => (0, MEM_LEN_BYTE),
            op_code::BRANCH => (0, MEM_LEN_BYTE),
            op_code::LUI => (0, MEM_LEN_BYTE),
            op_code::AUIPC => (0, MEM_LEN_BYTE),
            _ => {
                unreachable!()
            }
        };

        // Set control signals
        self.ctrl_reg_write_enable.constant_value = write_enable;
        self.ctrl_reg_len_mode.constant_value = len_mode;
    }
}

impl Component for MEMStage {
    fn process_cycle(&mut self, port_collection: &mut PortCollection) {
        // Get current instruction and set control signals
        let instruction = port_collection.get_port_data(self.reg_ir_id);
        self.set_control_signals(instruction);

        // Push control signals
        self.ctrl_reg_len_mode.process_cycle(port_collection);
        self.ctrl_reg_write_enable.process_cycle(port_collection);

        // Main cycle
        self.dmem.process_cycle(port_collection);
    }
}

/*
Processor -- MEM/WB pipeline registers
*/

pub struct MEMWBPipelineRegisters {
    /// Stores the result from memory fetch
    pub reg_mem: Register,

    /// Stores the result from the ALU
    pub reg_alu: Register,

    /// Stores the destination register
    pub reg_rd: Register,

    /// IR register for storing current instruction
    pub reg_ir: Register,
}

impl Component for MEMWBPipelineRegisters {
    fn process_cycle(&mut self, port_collection: &mut PortCollection) {
        // Process all registers
        self.reg_ir.process_cycle(port_collection);
        self.reg_alu.process_cycle(port_collection);
        self.reg_mem.process_cycle(port_collection);
        self.reg_rd.process_cycle(port_collection);
    }
}

impl MEMWBPipelineRegisters {
    pub fn new(
        port_collection: &mut PortCollection,
        mem_stage: &MEMStage,
        ex_mem_pipeline_regs: &EXMEMPipelineRegisters,
    ) -> Self {
        let reg_mem = Register::new(
            port_collection,
            mem_stage.dmem.output_port,
            String::from("MEMWB_reg_mem"),
        );
        let reg_alu = Register::new(
            port_collection,
            ex_mem_pipeline_regs.reg_alu.output_port,
            String::from("MEMWB_reg_alu"),
        );
        let reg_rd = Register::new(
            port_collection,
            ex_mem_pipeline_regs.reg_rd.output_port,
            String::from("MEMWB_reg_rd"),
        );
        let reg_ir = Register::new(
            port_collection,
            ex_mem_pipeline_regs.reg_ir.output_port,
            String::from("MEMWB_ir"),
        );

        Self {
            reg_mem,
            reg_alu,
            reg_rd,
            reg_ir,
        }
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

    /// Save the PortID of the instruction register in the ID/EX pipeline registers
    pub reg_ir_id: PortID,
}

impl WBStage {
    pub const MUX_VALUE_MEM: Word = 0;
    pub const MUX_VALUE_ALU: Word = 1;

    pub fn new(
        port_collection: &mut PortCollection,
        mem_wb_pipeline_regs: &MEMWBPipelineRegisters,
    ) -> Self {
        let ctrl_reg_value_select = ConstantRegister::new(
            port_collection,
            0,
            String::from("wb_ctrl_reg_value_select"),
        );
        let ctrl_reg_write_enable = ConstantRegister::new(
            port_collection,
            0,
            String::from("wb_ctrl_reg_write_enable"),
        );

        let mux_value = Mux::<2>::new(
            port_collection,
            &[
                mem_wb_pipeline_regs.reg_mem.output_port,
                mem_wb_pipeline_regs.reg_alu.output_port,
            ],
            ctrl_reg_value_select.output_port,
            String::from("wb_mux_value"),
        );

        let reg_rd = Register::new(
            port_collection,
            mem_wb_pipeline_regs.reg_rd.output_port,
            String::from("wb_rd"),
        );

        let reg_ir_id = mem_wb_pipeline_regs.reg_ir.output_port;

        Self {
            ctrl_reg_value_select,
            ctrl_reg_write_enable,
            mux_value,
            reg_rd,
            reg_ir_id,
        }
    }

    fn set_control_signals(&mut self, instruction: Word) {
        let op_code = extract_op_code(instruction);

        let (value_select, write_enable) = match op_code {
            op_code::OP => (Self::MUX_VALUE_ALU, 1),
            op_code::OP_IMM => (Self::MUX_VALUE_ALU, 1),
            op_code::LOAD => (Self::MUX_VALUE_MEM, 1),
            op_code::STORE => (Self::MUX_VALUE_ALU, 0),
            op_code::JAL => {
                todo!()
            }
            op_code::JALR => {
                todo!()
            }
            op_code::BRANCH => (Self::MUX_VALUE_ALU, 0),
            op_code::LUI => {
                todo!()
            }
            op_code::AUIPC => {
                todo!()
            }
            _ => {
                unreachable!()
            }
        };

        // Set control signals
        self.ctrl_reg_value_select.constant_value = value_select;
        self.ctrl_reg_write_enable.constant_value = write_enable;
    }
}

impl Component for WBStage {
    fn process_cycle(&mut self, port_collection: &mut PortCollection) {
        // Get current instruction and set control signals
        let instruction = port_collection.get_port_data(self.reg_ir_id);
        self.set_control_signals(instruction);

        // Push control signals
        self.ctrl_reg_write_enable.process_cycle(port_collection);
        self.ctrl_reg_value_select.process_cycle(port_collection);

        // Main cycle
        self.reg_rd.process_cycle(port_collection);
        self.mux_value.process_cycle(port_collection);
    }
}

/*
Processor
 */

pub struct Processor {
    pub port_collection: PortCollection,

    pub if_stage: IFStage,
    pub id_stage: IDStage,
    pub ex_stage: EXStage,
    pub mem_stage: MEMStage,
    pub wb_stage: WBStage,

    pub if_id_pipeline_regs: IFIDPipelineRegisters,
    pub id_ex_pipeline_regs: IDEXPipelineRegisters,
    pub ex_mem_pipeline_regs: EXMEMPipelineRegisters,
    pub mem_wb_pipeline_regs: MEMWBPipelineRegisters,

    pub interlock_unit: InterlockUnit,
}

impl Processor {
    pub fn new() -> Self {
        let mut port_collection = PortCollection::new();

        let mut if_stage = IFStage::new(&mut port_collection);
        let mut if_id_pipeline_regs =
            IFIDPipelineRegisters::new(&mut port_collection, &if_stage);

        let mut id_stage = IDStage::new(&mut port_collection, &if_id_pipeline_regs);
        let mut id_ex_pipeline_regs =
            IDEXPipelineRegisters::new(&mut port_collection, &id_stage, &if_id_pipeline_regs);

        let ex_stage = EXStage::new(&mut port_collection, &id_ex_pipeline_regs);
        let ex_mem_pipeline_regs =
            EXMEMPipelineRegisters::new(&mut port_collection, &ex_stage, &id_ex_pipeline_regs);

        let mem_stage = MEMStage::new(&mut port_collection, &ex_mem_pipeline_regs);
        let mem_wb_pipeline_regs =
            MEMWBPipelineRegisters::new(&mut port_collection, &mem_stage, &ex_mem_pipeline_regs);

        let wb_stage = WBStage::new(&mut port_collection, &mem_wb_pipeline_regs);

        let interlock_unit = InterlockUnit::new(
            &mut port_collection,
            if_id_pipeline_regs.reg_ir.output_port,
            id_ex_pipeline_regs.reg_ir.output_port,
            ex_mem_pipeline_regs.reg_ir.output_port,
            String::from("IU"),
        );

        // Connect Branch
        if_stage.mux.inputs[1] = ex_mem_pipeline_regs.reg_alu.output_port;
        if_stage.mux.selection_input = ex_mem_pipeline_regs.reg_bt.output_port;

        // Connect Write back
        id_stage.rf.set_write_inputs(
            wb_stage.mux_value.output_port,
            wb_stage.reg_rd.output_port,
            wb_stage.ctrl_reg_write_enable.output_port,
        );

        // Connect interlock_unit
        if_stage.connect_interlock_unit(interlock_unit.out_not_stall);
        if_id_pipeline_regs.connect_interlock_unit(interlock_unit.out_not_stall);
        id_ex_pipeline_regs.connect_interlock_unit(interlock_unit.out_not_stall);

        // Initialize instruction registers with NOP
        port_collection
            .set_port_data(if_id_pipeline_regs.reg_ir.output_port, NOP);
        port_collection
            .set_port_data(id_ex_pipeline_regs.reg_ir.output_port, NOP);
        port_collection
            .set_port_data(ex_mem_pipeline_regs.reg_ir.output_port, NOP);
        port_collection
            .set_port_data(mem_wb_pipeline_regs.reg_ir.output_port, NOP);

        Self {
            port_collection,
            if_stage,
            id_stage,
            ex_stage,
            mem_stage,
            wb_stage,
            if_id_pipeline_regs,
            id_ex_pipeline_regs,
            ex_mem_pipeline_regs,
            mem_wb_pipeline_regs,
            interlock_unit,
        }
    }
    
    /// Processes a single clockcycle in the processor.
    pub fn process_cycle(&mut self) {
        // Check if processor must stall
        self.interlock_unit.process_cycle(&mut self.port_collection);

        // Update stages
        self.wb_stage.process_cycle(&mut self.port_collection);
        self.if_stage.process_cycle(&mut self.port_collection);
        self.id_stage.process_cycle(&mut self.port_collection);
        self.ex_stage.process_cycle(&mut self.port_collection);
        self.mem_stage.process_cycle(&mut self.port_collection);

        // Cache stage results in pipeline registers
        self.mem_wb_pipeline_regs.process_cycle(&mut self.port_collection);
        self.ex_mem_pipeline_regs.process_cycle(&mut self.port_collection);
        self.id_ex_pipeline_regs.process_cycle(&mut self.port_collection);
        self.if_id_pipeline_regs.process_cycle(&mut self.port_collection);

        // NOTE: The IF-stage mux takes branch-related inputs from the EX stage. These are updated after the IF
        // stage has completed its cycle. An extra update of the mux is required, such that the CPU is not
        // left in an invalid state.
        self.if_stage.cycle_mux(&mut self.port_collection);
    }

    /// Get the current contents of the register file.
    pub fn get_rf_contents(&self) -> [Word; 32] {
        let mut result = [0; 32];

        for i in 0..32 {
            result[i] = self.port_collection.get_port_data(self.id_stage.rf.registers[i].output_port);
        }

        result
    }

    /// Get the current value of the program counter (IF stage).
    pub fn get_current_pc(&self) -> Word {
        self.port_collection.get_port_data(self.if_stage.reg_pc.output_port)
    }

    /// Get the instruction index of the instruction currently in the IF stage.
    pub fn get_current_instruction_idx(&self) -> Word {
        self.get_current_pc() >> 2
    }

    /// Get the address of the instruction that will be processed in the IF stage in the coming cycle.
    pub fn get_coming_pc(&self) -> Word {
        self.port_collection.get_port_data(self.if_stage.reg_pc.input)
    }

    /// Get the index of the instruction that will be processed in the IF stage in the coming cycle.
    pub fn get_coming_instruction_idx(&self) -> Word {
        self.get_coming_pc() >> 2
    }

    /// Returns whether the cpu is currently stalled or not.
    pub fn is_stalled(&self) -> bool {
        self.port_collection.get_port_data(self.interlock_unit.out_not_stall) == 0
    }

    /// Loads a given program memory into the CPU
    pub fn load_program_memory(&mut self, program: &[u8; MEM_SIZE]) {
        self.if_stage.imem.content = program.clone();
    }
}
