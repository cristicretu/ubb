bits 32 

global start        
extern exit               
import exit msvcrt.dll   

; (a+b*c+2/c)/(2+a)+e+x; a,b-byte; c-word; e-doubleword; x-qword
segment data use32 class=data
    a db 1
    b db 2
    c dw 2
    e dd 4
    x dq 8
; for this input, the result should be
; (1 + 2 * 2 + 2 / 2) / (2 + 1) + 4 + 8 = 0xE = 14

segment code use32 class=code
    start:
        ; we clear the registers
        mov EAX, 0
        mov EBX, 0
        mov EDX, 0
        
        ; Calculate b * c
        mov AL, byte [b]
        mov BX, word [c]
        mul BX ; result is in AX
        mov CX, AX ; store the result in CX
        
        ; Calculate 2 / c
        mov AX, 2   
        div word [c]
        
        ; Add a + b * c + 2 / c
        add AL, byte [a] ; AL already has 2 / c, so we can add a to that
        adc AH, 0 ; if it overflows, add the carry
        add AX, CX ; CX stores the b * c, so we add it
        adc EDX, 0
        
        ; Calculate 2 + a
        mov BL, byte [a]
        add BL, 2
        div BL ; divide (a+b*c+2/c) by (2 + a)
                
        ; Add e to that result
        add EAX, [e]
        adc EDX, 0 ; if it's bigger than 32 bits, we put the carry into EDX
        
        ; Add lower 32 bits of x into EAX
        add EAX, DWORD [x]
        adc EDX, 0
                
        ; Add higher 32 bits of x into EDX
        add EDX, DWORD [x + 4]
        
               
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

