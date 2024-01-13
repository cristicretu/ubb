bits 32

global start

extern exit, printf, scanf, fread, fopen, fclose
import exit msvcrt.dll
import scanf msvcrt.dll
import printf msvcrt.dll
import fopen msvcrt.dll
import fread msvcrt.dll
import fclose msvcrt.dll

segment data use32 class=data
    text resb 100
    len equ 100
    format_read db "%s", 0
    format_write db "%d ", 0
    read_access db "r", 0
    fd dd -1
    input db "afara.txt", 0

segment code use32 class=code
start:
    push dword read_access
    push dword input
    call [fopen]
    add esp, 4 * 2
    
    cmp eax, 0
    je afara
    
    mov [fd], eax
    
    push dword [fd]
    push dword len
    push dword 1
    push dword text
    call [fread]
    add esp, 4 * 4
    
    mov esi, text
    xor ebx, ebx
    repeta:
        lodsb
        
        cmp al, 0
        je afara
        
        cmp al, ' '
        jne reset
                
        push dword ebx
        push dword format_write
        call [printf]
        add esp, 4 * 2
        
        xor ebx, ebx
        jmp repeta
        
        reset:
        
        inc bl
        jmp repeta
       
        
    push dword [fd]
    call [fclose]
    add esp, 4
        
    afara:

    push dword 0
    call [exit]
