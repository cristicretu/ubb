bits 32
global start

extern exit, fscanf, fprintf, fopen, fclose, printf, scanf, fread, fwrite
import exit msvcrt.dll
import fscanf msvcrt.dll
import fprintf msvcrt.dll
import fopen msvcrt.dll
import fclose msvcrt.dll
import printf msvcrt.dll
import scanf msvcrt.dll
import fread msvcrt.dll
import fwrite msvcrt.dll


segment data use32 class=data
    file_descriptor dd -1
    file_descriptor_output dd -1
    file_name db "messi.txt", 0
    output_name db "ronaldo.txt", 0
    read_access db "r", 0
    append_access db "a", 0
    print_format db "%c", 0
    rezultat db 0
    s times 100 db 0
    osuta db 99
 
segment code use32 class=code
start:
    push dword read_access
    push dword file_name
    call [fopen]
    add esp, 4 * 2
    
    cmp eax, 0
    je final
    
    mov [file_descriptor], eax
    
    push dword append_access
    push dword output_name
    call [fopen]
    add esp, 4 * 2
    
    cmp eax, 0
    je final
    
    mov [file_descriptor_output], eax
    
    repeta:
        mov ecx, [osuta]
        mov edi, s
        
        jecxz continua
        clear_s:
            xor al, al
            stosb
            
        loop clear_s
            
        continua:
    
        push dword [file_descriptor]
        push dword [osuta]
        push dword 1
        push dword s
        call [fread]
        add esp, 4 * 4
        
        cmp eax, 0
        je iesi
        
        mov esi, s

        repeta_xor:
            xor eax, eax
            lodsb
            
            cmp al, 0
            je end_repeta
            
            xor al, 5
            mov byte [rezultat], al      
            
            push dword [rezultat]
            push dword print_format
            push dword [file_descriptor_output]
            call [fprintf]
            add esp, 4 * 3
            
        jmp repeta_xor
            
    end_repeta:    
        jmp repeta
        
iesi:    
    push dword [file_descriptor]
    call [fclose]
    add esp, 4
    
    push dword [file_descriptor_output]
    call [fclose]
    add esp, 4
    
final:

    push dword 0 
    call [exit] 
