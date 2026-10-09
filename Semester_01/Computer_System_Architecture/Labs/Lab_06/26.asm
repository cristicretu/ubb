bits 32 ;assembling for the 32 bits architecture
global start

; we ask the assembler to give global visibility to the symbol called start 
;(the start label will be the entry point in the program) 
extern exit ; we inform the assembler that the exit symbol is foreign; it exists even if we won't be defining it
import exit msvcrt.dll  ; we specify the external library that defines the symbol
		; msvcrt.dll contains exit, printf and all the other important C-runtime functions

; our variables are declared here (the segment is called data) 
segment data use32 class=data
    s DD 12345678h, 1A2B3C4Dh, 98FEDC76h
    len_s equ ($-s)/4
    ten db 10
    d times len_s db 0


; A string of doublewords is given. Compute the string formed by the high bytes of the low words from the elements of the doubleword string and these bytes should be multiple of 10.
; Example:
; given the doublewords string:
; s DD 12345678h, 1A2B3C4Dh, FE98DC76h 
; obtain the string
; d DB 3Ch, DCh.

; the program code will be part of a segment called code
segment code use32 class=code
    start:
        mov esi, s
        cld

        mov ecx, len_s
        mov edi, d

        repeat:
            LODSW
            mov BH, AH
            LODSW
            mov AL, BH
            mov AH, 0
            div byte [ten]
            cmp AH, 0

            jnz again
            mov AL, BH ; move the high byte of ax into al
            STOSB ; store the byte from al into the address in edi

        again:
            loop repeat
; ... 

	; call exit(0) ), 0 represents status code: SUCCESS
	push dword 0 ; saves on stack the parameter of the function exit
	call [exit] ; function exit is called in order to end the execution of the program
