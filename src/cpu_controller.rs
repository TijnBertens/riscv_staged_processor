use crate::{cpu::Processor, assembler::Program};

/// Keeps track of program counters for all stages of the processor.
pub struct PCState {
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

impl PCState {
    pub fn new() -> Self {
        Self {
            pc_if: 0,
            pc_id: 0,
            pc_ex: 0,
            pc_mem: 0,
            pc_wb: 0,
        }
    }

    pub fn push_pc(&mut self, pc_if: usize, is_stalled: bool) {
        self.pc_wb = self.pc_mem;
        self.pc_mem = self.pc_ex;

        if !is_stalled {
            self.pc_ex = self.pc_id;
            self.pc_id = self.pc_if;
        }
        
        self.pc_if = pc_if;
    }
    
    pub fn get_if(&self) -> usize {
        self.pc_if
    }
    
    pub fn get_id(&self) -> usize {
        self.pc_id
    }
    
    pub fn get_ex(&self) -> usize {
        self.pc_ex
    }
    
    pub fn get_mem(&self) -> usize {
        self.pc_mem
    }
    
    pub fn get_wb(&self) -> usize {
        self.pc_wb
    }
}

#[derive(Copy, Clone, PartialEq, PartialOrd)]
pub enum ExecutionMode {
    /// Execution is paused
    Paused,
    /// Execution runs for a single cycle
    SingleCycle,
    /// Execution runs a single step (untill a new instruction is reached)
    SingleStep,
    // TODO: /// Run untills a breakpoint is hit
    //RunTillBreak,
    /// Run the program to completion
    RunTillComplete,
    /// Execution has finished (program has completed)
    Finished,
}

pub struct CPUDebugController {
    execution_mode: ExecutionMode,
    pc_state: PCState,
    prev_wb_pc: usize,
    cpu: Processor,
    current_program: Option<Program>
}

impl CPUDebugController {
    pub fn new() -> Self {
        Self {
            execution_mode: ExecutionMode::Paused,
            pc_state: PCState::new(),
            prev_wb_pc: 0,
            cpu: Processor::new(),
            current_program: None
        }
    }
    
    fn cycle_cpu(&mut self) {
        self.prev_wb_pc = self.pc_state.pc_wb;
        self.cpu.process_cycle();
        self.pc_state.push_pc(
            self.cpu.get_coming_instruction_idx() as usize,
            self.cpu.is_stalled()
        );
    }
    
    fn is_at_exit_point(&self) -> bool {
        if let Some(program) = &self.current_program {
            program.get_exit_points().contains(&(self.pc_state.pc_wb << 2))
        } else {
            false
        }
    }
    
    pub fn tick(&mut self) {    
        match self.execution_mode {
            ExecutionMode::Paused => {}
            ExecutionMode::SingleCycle => {
                self.cycle_cpu();
                self.execution_mode = ExecutionMode::Paused;
                
                // Check for completion
                if self.is_at_exit_point() {
                    self.execution_mode = ExecutionMode::Finished;
                }
            }
            ExecutionMode::SingleStep => {
                self.cycle_cpu();
                if self.pc_state.pc_wb != self.prev_wb_pc {
                    self.execution_mode = ExecutionMode::Paused;
                }
                
                // Check for completion
                if self.is_at_exit_point() {
                    self.execution_mode = ExecutionMode::Finished;
                }
            }
            ExecutionMode::RunTillComplete => {
                self.cycle_cpu();
                
                // Check for completion
                if self.is_at_exit_point() {
                    self.execution_mode = ExecutionMode::Finished;
                }
            }
            ExecutionMode::Finished => {}
        }
    }
    
    pub fn set_exec_mode(&mut self, mode: ExecutionMode) {
        self.execution_mode = mode;
    }
    
    pub fn get_exec_mode(&self) -> ExecutionMode {
        self.execution_mode
    }
    
    pub fn reset_cpu(&mut self) {
        self.cpu = Processor::new();
        self.pc_state = PCState::new();
        self.prev_wb_pc = 0;
        self.current_program = None;
    }
    
    pub fn load_program(&mut self, program: Program) {
        self.reset_cpu();
        self.cpu.load_program_memory(&program.to_mem());
        self.current_program = Some(program);
    }
    
    pub fn is_finished(&self) -> bool {
        self.execution_mode == ExecutionMode::Finished
    }
    
    pub fn get_cpu(&self) -> &Processor {
        &self.cpu
    }
    
    pub fn get_program(&self) -> &Option<Program> {
        &self.current_program
    }
    
    pub fn get_pc_state(&self) -> &PCState {
        &self.pc_state
    }
}
