bits 32 ; assembling for the 32 bits architecture

; declare the EntryPoint (a label defining the very first instruction of the program)
global start        

; declare external functions needed by our program
extern exit               ; tell nasm that exit exists even if we won't be defining it
import exit msvcrt.dll    ; exit is a function that ends the calling process. It is defined in msvcrt.dll
                          ; msvcrt.dll contains exit, printf and all the other important C-runtime specific functions

; our data is declared here (the variables needed by our program)
segment data use32 class=data
    a db 10
    b dw 0A1D7h
    c db 5
    d dd 900
    ; a + 3 - c
  
    ; ...

; our code starts here==--
segment code use32 class=code
    start:
    
        mov BH, [a]
        add BH, 3
        sub BH, [c]
        
        ;mov AL, 32
        ;mov byte [c], 15
        
        ;add AL, 10 ; AL = AL + 10 = 32 + 10 = 42
        ;add AL, byte [c]; AL = AL + c = 42 + 15 = 57
        ;add byte [c], 1
        ;sub AL, [c] ; AL = AL - c = 32 - 15
        
        ;add dword [d], 1
        ;sub dword [d], 1000
        
        
        
        ; ...
    
        ; exit(0)
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

