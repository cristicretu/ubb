bits 32 

global start        
extern exit               
import exit msvcrt.dll   


; !!!!for hw a lot of comments

; 18. f+(c-2)*(3+a)/(d-4)
; a,b,c,d-byte, e,f,g,h-word

segment data use32 class=data
    ; a is an address (an offset)
    a db 1
    b db 10
    c db 3
    d db 8
    ;e dw 13
    f dw 2

segment code use32 class=code
    start:
    
        ; we add c to al
        mov al, byte [c]
        ; we substract (2)
        sub al, 2
        
     
        mov bl, byte [a]
        add bl, 3
        
        mov cl, byte [d]
        sub cl, 4
        
        mul bl
        div cl
        
        add ax, [f]
               
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

