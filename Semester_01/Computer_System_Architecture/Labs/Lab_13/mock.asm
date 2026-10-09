bits 32
global start

extern exit, fscanf, fprintf, fopen, fclose, printf, scanf, fread
import exit msvcrt.dll
import fscanf msvcrt.dll
import fprintf msvcrt.dll
import fopen msvcrt.dll
import fclose msvcrt.dll
import printf msvcrt.dll
import scanf msvcrt.dll
import fread msvcrt.dll

; our data is declared here (the variables needed by our program)
segment data use32 class=data
    input_file_descriptor dd -1
    output_file_descriptor dd -1
    input_file db "input.txt", 0
    output_file db "output.txt", 0
    read_format db "%s", 0
    write_format db "%c", 0
    read_access db "r", 0
    write_access db "w", 0
    file_text resb 100
    len equ 100
    
    ; ...

; our code starts here
segment code use32 class=code
    start:
        push dword read_access
        push dword input_file
        call [fopen]
        add esp, 4 * 2
        
        cmp eax, 0
        je final
        
        mov [input_file_descriptor], eax
    
        ;push dword file_text
        ;push dword read_format
        ;push dword [input_file_descriptor]
        ;call [fscanf]
        ;add esp, 4 * 2
        
        push dword [input_file_descriptor]
        push dword len
        push dword 1
        push dword file_text
        call [fread]
        add esp, 4 * 4
   
        mov esi, file_text
        
        push dword write_access
        push dword output_file
        call [fopen]
        add esp, 4 * 2
        
        xor ebx, ebx
        
        cmp eax, 0
        je final
        
        mov [output_file_descriptor], eax

        repeta:
            xor eax, eax
            lodsb
            
            cmp al, 0
            je iesi
            
            cmp al, ' '
            jne store
            
            mov al, 'x'
            
            store:
            push dword eax
            push dword write_format
            push dword [output_file_descriptor]
            call [fprintf]
            add esp, 12
            
        loop repeta
        
        iesi:
        push dword [input_file_descriptor]
        call [fclose]
        add esp, 4
        
        push dword [output_file_descriptor]
        call [fclose]
        add esp, 4
        
        final:
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

