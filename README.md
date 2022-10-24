# The Project

This project contains a component-based emulator for a 5-stage pipelined RISC-V processor written in Rust. Its purpose is mostly to be an educational tool by showing how instructions flow
through a pipelined processor, but the project has also served as an exercise using the Rust programming language. The design of the processor is based on the excellent 2017 book "*Computer Architecture: A Quantitative Approach*" by *Henessy* and *Patterson* (Sixth Edition).

This project currently provides a functional CPU and accompanying assembler. Furthermore, a primitive interface is available for writing programs, assembling them, and executing them on the CPU, though it is in a very early stage. The UI is written in native Rust using [EGUI](https://github.com/emilk/egui).

# The Instruction Set
The instruction set implemented in this processor is based on the [RISC-V ISA](https://riscv.org/wp-content/uploads/2017/05/riscv-spec-v2.2.pdf). Or, more specifically, on the *RV32I Base Integer Instruction Set*. There are a number of key design choices that characterize RISC-V.

- All instructions in the ISA have **equal width**. Since we use RV32I, each instruction will be exactly 32 bits.
- The ISA defines a set of **general-purpose registers** that act as internal storage for the CPU. In the case of RV32I, a total of 32 registers are available for storing operands for/results of instructions.
- RISC-V implements a **load-store architecture**, meaning that no instructions have access to memory directly, except for dedicated `LOAD` and `STORE` instructions. Practically, this implies that, for example, arithmetic instructions only operate on data that is in a register, and that the result of such an instruction is again stored in a register.

This specific set of design choices is in line with the main goal of RISC-V: to build a **R**educed **I**nstruction **S**et **C**omputer. By providing only a small set of simple instructions, and avoiding memory accesses in arithmetic instructions, RISC-V lends itself to many optimization techniques in hardware implementations. It is true that RISC-V often requires more instructions than an ISA with complex instructions to perform the same computation. However, the idea is that the optimizations facilitated by the simplistic ISA design, allow RISC-V instructions to be executed much faster than their complex counterparts. Perhaps the most important such optimization is **pipelining**, which we explore in this project.

# Component-Based Emulation
There are multiple ways to 'emulate' a RISC-V machine in software. The most practical would be to directly parse an incoming stream of RISC-V instructions, decode them, and emulate their execution using mechanisms from the programming language of the emulator. For example, an `ADD` instruction may then be handled as follows.

```Rust
let (op_code, mut reg_dst, reg_a, reg_b) = decode_arithmatic_instruction(...);

match decoded_op_code {
    ...
    OP_CODE::ADD => {
        reg_dst.assign_value(reg_a + reg_b);
    }
    ...
}
```

The goal of this project, however, is to explore how instructions and data move through a pipelined processor. Hence, a more interesting approach is to recreate the **actual circuitry** of a CPU implementing the instruction set. Doing so on the level of individual logic gates would be slow and too complex for our educational goal. Instead, this project simulates the **high-level components** like *registers*, *memories*, and the *ALU*, which make up the processor. By also simulating the necessary connections between these components, along with a central clock signal, the functionality of the CPU will naturally **emerge** from the simulated components.


# TODO
There are several things left to do in this project:

-   To execute more complex programs (with function calls etc...), support should be added for `jump-and-link` instructions. For now, the current set of supported instructions suffices to write basic programs.
-   Create a more complete runtime environment in the UI, including debugging tools and visualizations.
-   Create a visualization in the UI showing components and connections, along with their state in the currently running program.
-   Have the UI run in the web.