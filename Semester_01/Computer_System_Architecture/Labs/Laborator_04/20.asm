bits 32 

global start        

extern exit               
import exit msvcrt.dll   

segment data use32 class=data
    a dw 1001110100110101b
    b dw 0010101110110110b
    c dq 0

;20    
;Given the words A and B, compute the doubleword C as follows:
;the bits 0-5 of C are the same as the bits 3-8 of A
;the bits 6-8 of C are the same as the bits 2-4 of B
;the bits 9-15 of C are the same as the bits 6-12 of A
;the bits 16-31 of C have the value 0

segment code use32 class=code
    start:
        ; clear registers
        mov EAX, 0
        mov EDX, 0
        mov ECX, 0
    
        mov EBX, 0 ; store the result in ebx

        ; single out bits 3-8 from A
        mov AX, word [a]
        and AX, 0000000111111000b ; we get the 3-8 bits from A
        mov CL, 3
        ror AX, CL ; map 0 -> 0
        
        ; store the result in BX
        or BX, AX
        
        ; single out  bits 2-4 from B
        mov AX, word [b]
        and AX, 000000000011100b
        mov CL, 4
        rol AX, CL
        
        or BX, AX
        
        mov AX, word [a]
        and AX, 0001111111000000b
        mov CL, 3
        rol AX, CL
        
        or BX, AX
        
        ; bits from 16-31 are already 0
        ; result is in EBX
    
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

