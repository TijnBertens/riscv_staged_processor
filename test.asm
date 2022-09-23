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
    MVI x31 88  ; Exit
