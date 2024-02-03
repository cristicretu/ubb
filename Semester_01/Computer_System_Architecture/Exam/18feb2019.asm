bits 32 ; assembling for the 32 bits architecture

; declare the EntryPoint (a label defining the very first instruction of the program)
global start        
; declare external functions needed by our program
extern exit, printf        ; tell nasm that exit exists even if we won't be defining it
import exit msvcrt.dll    ; exit is a function that ends the calling process. It is defined in msvcrt.dll
                          ; msvcrt.dll contains exit, printf and all the other important C-runtime specific functions
          
import printf msvcrt.dll

; our data is declared here (the variables needed by our program)
segment data use32 class=data
    s1 dd 12345678h
       dd 12785634h
       dd 1a4d3c2bh
    len equ ($-s1)/4 ; 4 bytes generated
    format db 'no. of ones is %d', 13, 10, 0
    s2 resw 100

; our code starts here
segment code use32 class=code
    start:
        mov esi, s1
        mov edi, s2
        mov ecx, len
        
        ; get the high bytes from each doubleword
        
        parcurge:
            lodsd
            push word AX
            shr EAX, 8
            pop word AX
            shr EAX, 8
            stosw
        loop parcurge
        
        xor ebx, ebx
        
        mov esi, s2
        mov ecx, len
        repeta:
            lodsw
            
            ; we have it in AX
            
            count_ones:
                test ax, ax
                jz afara
                
                test AL, 1
                jnz odd_element
                
                inc ebx
                
                odd_element:
                shr ax, 1
                jmp count_ones
                
            afara:
            
            loop repeta
            
        push dword ebx
        push dword format
        call [printf]
        add esp, 4 * 2
        
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

