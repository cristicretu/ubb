bits 32 ;assembling for the 32 bits architecture
global start

; we ask the assembler to give global visibility to the symbol called start 
;(the start label will be the entry point in the program) 
extern exit ; we inform the assembler that the exit symbol is foreign; it exists even if we won't be defining it
import exit msvcrt.dll  ; we specify the external library that defines the symbol
		; msvcrt.dll contains exit, printf and all the other important C-runtime functions

; our variables are declared here (the segment is called data) 
segment data use32 class=data
    s DD 12345607h, 1A2B3C15h, 13A33412h
    len_s equ ($ - s) / 4
    d times len_s db 0
    sapte db 7
; ... 

; the program code will be part of a segment called code
segment code use32 class=code
start:
    mov esi, s
    mov edi, d

    mov ecx, len_s

    jecxz end_loop

    repeta:
        lodsb
        mov bl, al ; store the result

        ; we now have the least significant 8 bits
        ; check if it's multiple of 7

        mov ah, 0
        div byte [sapte]
        cmp ah, 0
        jne not_multiple

        mov al, bl
        stosb

    not_multiple:
        lodsb
        lodsw
        loop repeta
; ... 

end_loop:
	; call exit(0) ), 0 represents status code: SUCCESS
	push dword 0 ; saves on stack the parameter of the function exit
	call [exit] ; function exit is called in order to end the execution of the program
