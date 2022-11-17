#![allow(dead_code)]

mod code_highlighting;
mod example_programs;
mod graphics;

#[cfg(not(target_arch = "wasm32"))]
fn main() {
    graphics::run_gui();
}

// when compiling to web using trunk.
#[cfg(target_arch = "wasm32")]
fn main() {
    // // Make sure panics are logged using `console.error`.
    console_error_panic_hook::set_once();
    graphics::run_gui();
}