bits 32
global start

extern exit, fopen, fclose, fread, fprintf
import exit msvcrt.dll
import fopen msvcrt.dll
import fclose msvcrt.dll
import fread msvcrt.dll
import fprintf msvcrt.dll

; Open file "input.txt"
; Replace " " with "X"
; Count the number of *words*
; Write the new text into "output1.txt"
; Newline, write the number of *words*

segment data use32 class=data
    input_fd dd -1
    output_fd dd -1
    
    input_name db "input.txt", 0
    output_name db "output1.txt", 0
    
    write_access db "w", 0
    read_access db "r", 0
    
    format db "%c", 0
    answer db "The number of chars is %d.", 0
    n_format db "%s", 0
    newline db 10, 0
    digit db "%d", 0
    
    text resb 100
    buffer resb 200
    len equ 100


segment code use32 class=code
    start:
    ; open the input file
    push dword read_access
    push dword input_name
    call [fopen]
    add esp, 4 * 2
    cmp eax, 0
    je finish
    mov [input_fd], eax
    
    ; open the output file
    push dword write_access
    push dword output_name
    call [fopen]
    add esp, 4 * 2
    cmp eax, 0
    je close_input
    mov [output_fd], eax
    
    
    ; read the text
    push dword [input_fd]
    push dword len
    push dword 1
    push dword text
    call [fread]
    add esp, 4 * 4
    ;mov ecx, eax
    mov esi, text
   ; mov ecx, eax
    
    xor eax, eax
    xor ebx, ebx
    xor edx, edx
    
    ; bh - flag
    ; bl - counter
    
    repeta:
        xor eax, eax
        
        lodsb ; we load into al
        
        cmp al, 0
        je end_loop
        
        push eax
        
        cmp al, ' '
        jne store
        
        cmp bh, 1
        jne below
        
        inc bl
        
        below:
        
        mov al, 'x'
        mov bh, 0
        
        
        store:
        push dword eax
        push dword format
        push dword [output_fd]
        call [fprintf]
        add esp, 12
        
        pop edx
        
        or dl, 0x20
        sub dl, 'a'
        cmp dl, 'z'-'a'
        ja skip_char
        
        mov bh, 1
        
        skip_char:
        
        jmp repeta
        
    end_loop:
    
    cmp bh, 1
    jne close_files
    
    inc bl
    
    xor bh, bh
        
    close_files:
    
    ; print the number of chars
    push dword newline
    push dword n_format
    push dword [output_fd]
    call [fprintf]
    add esp, 4 * 3
    
    push dword ebx
    push dword digit
    push dword [output_fd]
    call [fprintf]
    add esp, 4 * 3
    
    close_output:
        push dword [output_fd]
        call [fclose]
        add esp, 4

    close_input:
        push dword [input_fd]
        call [fclose]
        add esp, 4

    finish:
        push dword 0
        call [exit]
    
        
