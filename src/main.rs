#![allow(dead_code)]

mod assembler;
mod circuit;
mod code_highlighting;
mod components;
mod cpu;
mod cpu_controller;
mod example_programs;
mod graphics;
mod isa;

fn main() {
    graphics::run_gui();
}
