bits 32 

global start        

extern exit               
import exit msvcrt.dll   

segment data use32 class=data
    a dw 1001110100110101b
    b dq 0

;19  
;Given the word A, compute the doubleword B as follows:
;the bits 28-31 of B have the value 1;  12-15
;the bits 24- 25 and 26-27 of B are the same as the bits 8-9 of A 8-9 10-11
;the bits 20-23 of B are the invert of the bits 0-3 of A ;  4-7
;the bits 16-19 of B have the value 0     0-3
;the bits 0-15 of B are the same as the bits 16-31 of B.

; result is 1111 0101 1010 0000 | 1111 0101 1010 0000

segment code use32 class=code
    start:
        ; clear registers
        xor EAX, EAX
        xor EDX, EDX
        xor ECX, ECX
    
        xor EBX, EBX ; store the result in ebx

        ; bits 12-15 are 1
        or BX, 1111000000000000b
        
        ; single out bits 8-9 from a
        mov AX, word [a]
        and AX, 0000001100000000b
        or BX, AX
        
        ; put them into 10-11 to b
        mov CL, 2
        rol AX, CL
        or BX, AX
        
        mov AX, word [a]
        and AX, 0000000000001111b
        or BX, 0000000011110000b
        mov CL, 4
        rol AX, CL
        
        ; bits 4-7 are inverted from bits 0-3 from a
        xor BX, AX
        
        ; move lower 16 bits into higher 16 bits
        push BX
        shl EBX, 16
        pop BX
        
        ; result is in BX
        
    
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

