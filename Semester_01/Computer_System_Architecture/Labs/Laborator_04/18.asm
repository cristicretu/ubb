bits 32 

global start        

extern exit               
import exit msvcrt.dll   

segment data use32 class=data
    a dw 1001110100110101b
    b dq 0

;18   
;Given the word A, compute the doubleword B as follows:
;the bits 0-3 of B have the value 0;
;the bits 4-7 of B are the same as the bits 8-11 of A
;the bits 8-9 and 10-11 of B are the invert of the bits 0-1 of A (so 2 times) ;
;the bits 12-15 of B have the value 1
;the bits 16-31 of B are the same as the bits 0-15 of B.

; result is 1111101011010000 1111101011010000 which is correct

segment code use32 class=code
    start:
        ; clear registers
        xor EAX, EAX
        xor EDX, EDX
        xor ECX, ECX
    
        xor EBX, EBX ; store the result in ebx

        ; single out bits 8 - 11 from A
        mov AX, word [a]
        and AX, 000111100000000b
        mov CL, 4
        ror AX, CL
        
        ; store result in BX
        or BX, AX
        
        ; single out bits 0 - 1 from A
        mov AX, word [a]
        and AX, 0000000000000011b
        
        ; bits 8 - 9 from b = not bits 0 - 1 from A
        mov CL, 8
        rol AX, CL
        ; in order to have the inverse of those bits, we will apply a XOR
        ; so we need to set the bits from 8-11 to 1
        or BX, 0000111100000000b
        xor BX, AX
        
        ; bits 10 - 11 from b = not bits 0 - 1 from A
        mov CL, 2
        rol AX, CL
        xor BX, AX
        
        ; bits 12 - 15 are 1
        or BX, 01111000000000000b
        
        ; store the low 16 bits from EBX
        push BX
        mov CL, 16
        ; shift the low 16 bits to the high 16 bits
        shl EBX, CL
        ; retrieve the value from the stack, so now both sides are equal
        pop BX
        
        ;result is in EBX
        
    
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

