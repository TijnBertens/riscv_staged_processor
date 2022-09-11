#![allow(dead_code)]

mod circuit;
mod components;
mod isa;
mod assembler;
mod cpu;
mod graphics;


fn main() {
    graphics::run_gui();
}