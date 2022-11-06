;
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
    EXIT  ; Exit


start:
	NOP
	NOP
jmp:
	MVI x1 10
	NOP
	NOP
	BLT x0 x1 jmp
	NOP
	NOP
	
Fibonacci:

start:
	MVI x1 0		; x1 <- iteration counter
	MVI x10 10	  ; x10 <- num iterations

	MVI x2 0		; x2 <- f_t-2
	MVI x3 1		; x3 <- f_t-1
loop:
	NOP
	BLT x1 x10 loop
	ADD x4 x2 x3	; x4 <- f_t
	ADDI x1 x1 1	; i++
	
end:
	EXIT