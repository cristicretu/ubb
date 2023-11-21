bits 32
global start

extern exit, scanf, printf
import exit msvcrt.dll
import printf msvcrt.dll
import scanf msvcrt.dll

segment data use32 class=data
    a dd 0
    b dd 0
    format_printf db "%c = ", 0
    format_scanf db "%x", 0
    format_result db "%s = %x", 0


segment code use32 class=code
start:
    ; print a msg
    push dword 'a'
    push dword format_printf
    call [printf]
    add esp, 8

    ; read a
    push dword a
    push dword format_scanf
    call [scanf]
    add esp, 8

    ; print b msg
    push dword 'b'
    push dword format_printf
    call [printf]
    add esp, 8

    ; read b
    push dword b
    push dword format_scanf
    call [scanf]
    add esp, 8

    mov eax, dword [a]
    mov ebx, dword [b]

    and eax, 0x0000FFFF
    and ebx, 0x0000FFFF

    add eax, ebx

    push dword 'suma'
    push eax
    push dword format_result
    call [printf]
    add esp, 12

    mov eax, dword [a]
    mov ebx, dword [b]

    shr eax, 16
    shr ebx, 16

    sub ebx, eax

    push dword 'diferenta'
    push ebx
    push dword format_result
    call [printf]
    add esp, 12


	push dword 0 
	call [exit] 
