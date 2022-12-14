macro_rules! add_program {
    ($name:literal, $content:literal) => {
        ($name, $content)
    };
}

pub const EXAMPLE_PROGRAMS: &'static [(&'static str, &'static str)] = &[
    add_program!(
        "Fibonacci",
"start:
	MVI x1 1		; x1 <- iteration counter (first 2 iterations are skipped)
	MVI x10 19	  ; x10 <- num iterations
	MVI x2 0		; x2 <- f_0
	MVI x3 1		; x3 <- f_1

loop:
	ADDI x1 x1 1	; i++
	ADD x4 x2 x3	; x4 <- f_t
	BLT x1 x10 loop
	MV x2 x3		; x2 <- f_t-2
	MV x3 x4		; x3 <- f_t-1
	
end:
	EXIT"),
	
	add_program!(
	    "Integer Division",
";
;   Somewhat computes 60 / 4 (a / b)
;

start:
    MVI x1 0    ; i = 0
    MVI x2 0    ; count = 0
    MVI x3 60   ; a = 60
loop:
    ADDI x2 x2 4       ; count += 4
    BLT x2 x3 loop     ; count < 60
    ADDI x1 x1 1       ; i++
    NOP
end:
    MV x4 x1    ; Store result in x4
    EXIT  ; Exit"
	)
];