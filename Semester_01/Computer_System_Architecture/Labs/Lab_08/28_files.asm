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

; A file name (defined in data segment) is given. 
; Create a file with the given name, then read 
; words from the keyboard until character '$' is 
; read. Write only the words that contain at least
;  one lowercase letter to file.

segment data use32 class=data
    file_descriptor dd 0
    file_name db "output.txt", 0
    read_format db "%s", 0
    write_format db "%s ", 0
    access db "w", 0
    message db "Enter a word: ", 0
    n resb 100


segment code use32 class=code
start:
    push dword access
    push dword file_name
    call [fopen]
    add esp, 8

    mov [file_descriptor], eax
    cmp eax, 0
    je final

    repeat:
        push dword message
        call [printf]
        add esp, 4

        push dword n
        push dword read_format
        call [scanf]

        mov al, [n]
        mov bl, '$'
        
        cmp al, bl
        je close_file

        mov ESI, n
        check:
            lodsb
            cmp al, 0
            jz repeat

            cmp al, 'a'
            jb check

            cmp al, 'z'
            ja check

            push dword n
            push dword write_format
            push dword [file_descriptor]
            call [fprintf]
            add esp, 12

            jmp repeat

    close_file:
        push dword [file_descriptor]
        call [fclose]
        add esp, 4

    final:

        push dword 0 
        call [exit] 
