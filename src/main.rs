#![allow(dead_code)]

mod assembler;
mod circuit;
mod code_highlighting;
mod components;
mod cpu;
mod graphics;
mod isa;

fn main() {
    graphics::run_gui();
}
