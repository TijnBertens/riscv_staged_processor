use rsp_core::{self, cpu_controller::{CPUDebugController, ExecutionMode}};

macro_rules! parse_reg {
    (x0) => { 0 };
    (x1) => { 1 };
    (x2) => { 2 };
    (x3) => { 3 };
    (x4) => { 4 };
    (x5) => { 5 };
    (x6) => { 6 };
    (x7) => { 7 };
    (x8) => { 8 };
    (x9) => { 9 };
    
    (x10) => { 10 };
    (x11) => { 11 };
    (x12) => { 12 };
    (x13) => { 13 };
    (x14) => { 14 };
    (x15) => { 15 };
    (x16) => { 16 };
    (x17) => { 17 };
    (x18) => { 18 };
    (x19) => { 19 };
    
    (x20) => { 20 };
    (x21) => { 21 };
    (x22) => { 22 };
    (x23) => { 23 };
    (x24) => { 24 };
    (x25) => { 25 };
    (x26) => { 26 };
    (x27) => { 27 };
    (x28) => { 28 };
    (x29) => { 29 };
    
    (x30) => { 30 };
    (x31) => { 31 };
}

macro_rules! assert_reg {
        ($cpu:ident, $reg:ident, $val:expr) => {
            {
                let reg_num = parse_reg!($reg);
                assert_eq!(
                    $cpu.get_rf_contents()[reg_num],
                    $val,
                    "Register x{} expected to have value {}.", reg_num, $val
                );
            }
        };
    }

fn setup_test_environment<S: Into<String>>(program_code: S) -> CPUDebugController {
    let program = rsp_core::assembler::Program::from_text(program_code.into().trim().into()).unwrap();
	let mut cpu_controller = rsp_core::cpu_controller::CPUDebugController::new();
	cpu_controller.load_program(program);
	return cpu_controller;
}

/// Runs the program loaded into a cpu (controller) to completion. Optionally takes a timeout value expressed in cycles.
/// A default timeout value of 1_000_000 cycles is used. This function returns whether the program finished within the given 
/// time.
fn run_program_complete(cpu_controller: &mut CPUDebugController, timeout: Option<u32>) -> bool {
    cpu_controller.reset_cpu_with_program();
    cpu_controller.set_exec_mode(ExecutionMode::RunTillComplete);
    
    const DEFAULT_TIMEOUT: u32 = 1_000_000;
    
    let mut cycle_timeout = timeout.unwrap_or(DEFAULT_TIMEOUT);
	while !cpu_controller.is_finished() {
	    if cycle_timeout == 0 {
	       return false;
	    }
	    cycle_timeout -= 1;
	    
	    cpu_controller.tick();
	}
	
	return true;
}

#[test]
/// Branches can jump to themselves.
pub fn test_self_referencing_branch() {
    let program_code = "
start:
	MVI x1 0
jmp:
	BEQ x1 x0 jmp
	ADDI x1 x1 1
	NOP
	EXIT
	";    
	
	let mut cpu_controller= setup_test_environment(program_code);
	
	if !run_program_complete(&mut cpu_controller, Some(100)) {
	    assert!(false, "Program timed out!");
	}
    
    let cpu = cpu_controller.get_cpu();
    assert_reg!(cpu, x1, 2);
}