bits 32 ;assembling for the 32 bits architecture
global start

; we ask the assembler to give global visibility to the symbol called start 
;(the start label will be the entry point in the program) 
extern exit, printf, scanf ; we inform the assembler that the exit symbol is foreign; it exists even if we won't be defining it
import exit msvcrt.dll  ; we specify the external library that defines the symbol
import printf msvcrt.dll
import scanf msvcrt.dll
		; msvcrt.dll contains exit, printf and all the other important C-runtime functions

; read a number from base 16 and display it in base 10

; our variables are declared here (the segment is called data) 
segment data use32 class=data
    n dd 0
    message db "Introduceti un numar in baza 16: ", 0
    format_read db "%x", 0
    format_write db "%d", 10, 0
; ... 

; the program code will be part of a segment called code
segment code use32 class=code
start:
    push message
    call [printf]
    add esp, 4

    push n
    push format_read
    call [scanf]
    add esp, 8

    push dword [n]
    push format_write
    call [printf]
    add esp, 8

    
	push dword 0 
	call [exit] 
