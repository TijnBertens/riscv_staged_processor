// use crate::{components::Component, cpu::Processor};
// use std::{
//     borrow::Borrow,
//     sync::Arc,
//     sync::{atomic::AtomicBool, Mutex},
//     thread,
// };

// pub struct CPUController {
//     paused: Arc<AtomicBool>,
//     cpu: Arc<Mutex<Processor>>,
// }

// impl CPUController {
//     pub fn new() -> Self {
//         Self {
//             paused: Arc::new(AtomicBool::new(true)),
//             cpu: Arc::new(Mutex::new(Processor::new())),
//         }
//     }

//     pub fn run_loop(&self) {
//         // let paused = Arc::clone(&self.paused);
//         // let cpu = Arc::clone(&self.cpu);

//         // thread::spawn(move || {
//         //     while !paused.borrow() {
//         //         cpu.lock().unwrap().process_cycle();
//         //     }
//         // });
//     }
// }
