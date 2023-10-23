bits 32 

global start        

extern exit               
import exit msvcrt.dll   

segment data use32 class=data
    a dw 1001110100110101b
    b dw 0010101110110110b
    c dq 0

;21
;Given the words A and B, compute the doubleword C as follows:
;the bits 0-3 of C are the same as the bits 5-8 of B - yes
;the bits 4-10 of C are the invert of the bits 0-6 of B - yes
;the bits 11-18 of C have the value 1
;the bits 19-31 of C are the same as the bits 3-15 of B

; result is 0010 1011 1011 0111 1111 1100 1001 1101 which is correct

segment code use32 class=code
    start:
        ; clear registers
        xor EAX, EAX
        xor EDX, EDX
        xor ECX, ECX  
    
        xor EBX, EBX ; store the result in ebx
        
        ; bits 16, 17, 18 are 1 (0, 1, 2)
        or BX, 0000000000000111b
        
        ; single out bits 3 - 15 from b
        mov AX, word [b]
        and AX, 1111111111111000b
        or BX, AX
        
        ; move those into the higher 16 bits
        shl EBX, 16
        xor BX, BX

        ; single out bits 5-8
        mov AX, word [b]
        and AX, 0000000111100000b
        mov CL, 5
        ror AX, CL
        
        or BX, AX
        
        ; single out bits 0 - 6
        mov AX, word [b]
        and AX, 0000000001111111b
        or BX, 0000011111110000b
        mov CL, 4
        rol AX, CL
        
        xor BX, AX
        
        ; bits 11 - 15 are 1
        or BX, 1111100000000000b
        
        ; result is in EBX
        
    
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

