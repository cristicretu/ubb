bits 32
global start

extern exit, fscanf, fprintf, fopen, fclose, printf, scanf
import exit msvcrt.dll
import fscanf msvcrt.dll
import fprintf msvcrt.dll
import fopen msvcrt.dll
import fclose msvcrt.dll
import printf msvcrt.dll
import scanf msvcrt.dll


segment data use32 class=data
    file_descriptor dd 0
    file_name db "output24.txt", 0
    write_format db "%c", 0
    access db "w", 0
    text db "ana are 7 mere si 5 cai frumosi in 4 case mici.", 0


segment code use32 class=code
start:
    push dword access
    push dword file_name
    call [fopen]
    add esp, 8

    mov [file_descriptor], eax
    cmp eax, 0
    je final
    
    mov esi, text

    repeat:
        xor eax, eax
        lodsb
        
        cmp al, 0
        je final
        
        cmp al, '0'
        jb store
        
        cmp al, '9'
        ja store
        
        mov al, 'C'
        
        store:
        push dword eax
        push dword write_format
        push dword [file_descriptor]
        call [fprintf]
        add esp, 12
        
    loop repeat
        

    close_file:
        push dword [file_descriptor]
        call [fclose]
        add esp, 4

    final:

        push dword 0 
        call [exit] 
