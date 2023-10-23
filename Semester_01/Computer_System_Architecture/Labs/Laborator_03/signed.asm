bits 32 

global start        
extern exit               
import exit msvcrt.dll   

; (a+b*c+2/c)/(2+a)+e+x; a,b-byte; c-word; e-doubleword; x-qword
segment data use32 class=data
    a db -1 
    b db 2  
    c dw -2 
    e dd 4  
    x dq 8
; (-1 - 4 - 1) / 1 + 4 + 8 = -6 + 4 + 8 = 6

segment code use32 class=code
    start:
        ; we clear the registers 
        mov EAX, 0
        mov EBX, 0
        mov EDX, 0
        
        ; Calculate b * c
        mov AL, byte [b]
        mov BL, byte [c]
        imul BX ; result is in AX, using signed multiplication
        mov CX, AX ; store the result in CX
        
        ; Calculate 2 / c
        mov AX, 2  
        cwd            ; Sign-extend ax into dx:ax
        idiv word [c]  
        
        ; Add a + b * c + 2 / c
        add AL, byte [a] ; AL already has 2 / c, so we can add a to that
        adc AH, 0 ; if it overflows add the carry
        add AX, CX ; CX stores the b * c, so we add it
        adc EDX, 0
        
        ; Calculate 2 + a
        mov BL, byte [a]
        add BL, 2
        cbw
        idiv BL ; Using signed division
        
        ; Add e to that result
        add EAX, [e]
        adc EDX, 0
        
        ; Add lower 32 bits of x into EAX
        add EAX, DWORD [x]
        adc EDX, 0
                
        ; Add higher 32 bits of x into EDX
        add EDX, DWORD [x + 4]
        
        push    dword 0
        call    [exit]

