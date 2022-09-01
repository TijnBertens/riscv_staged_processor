mod circuit;
mod components;
mod risc_v;
mod assembler;

use std::rc::Rc;
use std::cell::RefCell;
use crate::circuit::{PortCollection};
use crate::components::{Component, IFStage};

fn main() {


    //
    //
    // b bbb bbb bbb bbb bbb


    // let port_collection = Rc::new(RefCell::new(PortCollection::new()));
    //
    // let mut if_stage = IFStage::new(port_collection.clone());
    //
    // for _ in 0..3 {
    //     if_stage.process_cycle();
    //
    //     let port_collection = port_collection.deref().borrow();
    //
    //     let npc_value = port_collection.get_port_data(if_stage.reg_npc.output_port);
    //     let ir_value = port_collection.get_port_data(if_stage.reg_ir.output_port);
    //
    //     println!("NPC: {}     IR: {:04X}", npc_value, ir_value);
    // }
}
