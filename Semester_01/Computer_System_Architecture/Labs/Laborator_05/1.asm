bits 32 ; assembling for the 32 bits architecture

; declare the EntryPoint (a label defining the very first instruction of the program)
global start        

; declare external functions needed by our program
extern exit               ; tell nasm that exit exists even if we won't be defining it
import exit msvcrt.dll    ; exit is a function that ends the calling process. It is defined in msvcrt.dll
                          ; msvcrt.dll contains exit, printf and all the other important C-runtime specific functions

; our data is declared here (the variables needed by our program)
segment data use32 class=data
    s db 1, 2, 3, 4
    len_s equ $-s
    d times (len_s - 1) dw 0

; CMP [a], [b] = [a] - [b]
; jz jump if ZF = 0
; je jump if ZF = 1
; jnz jum[ if ZF = 1
; jc jump if CF = 1
; jnc jump if CF = 0

; attention on comparing signed and unsigned
; ja ; jump if  above (if a > b - unsigned)
; jb ; jump if below (if a < b - unsigned)

; jnae ; jump if not above or equal (if a <= b)


;  jg ; jump if greater (if a > b - signed)
;  jl ; jump if less (if a < b - signed)

; jcxz ; jump if CX = 0
; jecxz ; jump if ECX = 0

; cmp [a], [b]
; jb else
;     inc [a]
;     jmp endif
; else:
;     dec [b]
; endif:

; 1. Given a byte string S of length l, obtain the string D of length l-1 as D(i) = S(i) * S(i+1) (each element of D is the product of two consecutive elements of S).
; Example:
; S: 1, 2, 3, 4
; D: 2, 6, 12

segment code use32 class=code
    start:
        mov ecx, len_s
        mov esi, 1
        mov bl, byte [s]
        dec ecx

        jcxz end
        loop_a:
            mov ah, 0
            mov al, byte [s+esi]
            mul bl
            ; multiply the previous element with the 
            ; current one

            mov bl, byte [s+esi] ; store the current one as the previous el for next operation
            mov [d+esi-1], ax ; store the result in the destination array
        
            inc esi
        loop loop_a
        
        end: 
        ; exit(0)
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

