bits 32

global start        

extern exit               
import exit msvcrt.dll   

;Two byte strings A and B are given. Obtain the string R that contains only the odd positive elements of the two strings.
;Example:
;A: 2, 1, 3, -3
; B: 4, 5, -5, 7
; R: 1, 3, 5, 7

segment data use32 class=data
    a db 2, 1, 3, -3
    len_a equ $-a ; length of a
    b db 4, 5, -5, 7
    len_b equ $-b ; length of b
    r times (len_a+len_b) db 0 ; possible maximum length of r

segment code use32 class=code
    start:
        mov ecx, len_a ; number of elements in a
        mov esi, 0 ; index in a
        mov edi, 0 ; index in r

        test ecx, ecx ; check if ecx is 0   
        jz end_a ; if ecx is 0, we are done

        loop_a:
            mov al, [a+esi] ; get the current element of a
            test al, 1 ; test if the element is odd
            jz skip_a ; if it is even, skip it
            test al, al ; test if the element is positive
            js skip_a ; if it is negative, skip it

            mov [r+edi], al ; copy the element to r
            inc edi ; increment the index in r
        skip_a:
            inc esi ; increment the index in a
            dec ecx ; decrement the loop counter
            jnz loop_a ; if ecx is not 0, continue the loop
        end_a:

        mov ecx, len_b ; number of elements in b
        mov esi, 0 ; index in b

        test ecx, ecx ; check if ecx is 0
        jz end_b ; if ecx is 0, we are done

        loop_b:
            mov al, [b+esi]
            test al, 1
            jz skip_b
            test al, al
            js skip_b

            mov [r+edi], al
            inc edi
        skip_b:
            inc esi
            dec ecx
            jnz loop_b
        end_b:
       
    
        ; exit(0)
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

