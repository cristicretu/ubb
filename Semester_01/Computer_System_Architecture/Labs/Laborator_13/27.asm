bits 32
global start

import exit msvcrt.dll
import scanf msvcrt.dll
import printf msvcrt.dll
import fopen msvcrt.dll
import fclose msvcrt.dll
import fread msvcrt.dll
extern exit, scanf, printf, fopen, fclose, fread      

segment data use32
    text resb 100
    len equ 100
    fd dd -1
    file_name db "input.txt", 0
    format_read db "%s", 0
    read_access db "r", 0
    buffer resb 100
    buffer1 resb 100

segment code use32 public code
	start:
        push dword read_access
        push dword file_name
        call [fopen]
        add esp, 4 * 2
        
        cmp eax, 0
        je exit_program
        
        mov [fd], eax
        
        push dword [fd]
        push dword len
        push dword 1
        push dword text
        call [fread]
        add esp, 4 * 4
       
        
        mov esi, text
        mov edi, buffer
        
        repeta:
            lodsb
            
            cmp al, 0
            je iese
            
            cmp al, 'a'
            jb skip_char1
            
            cmp al, 'z'
            ja skip_char1
            
            stosb
            
            skip_char1:
            jmp repeta
            
        iese:
        
        mov byte [edi], 0 ; null terminate string
        
        push dword buffer
        push dword format_read
        call [printf]
        add esp, 4 * 2
        
        mov esi, text
        mov edi, buffer1
        
        repeta1:
            lodsb
            
            cmp al, 0
            je iese1
            
            cmp al, 'A'
            jb skip_char2
            
            cmp al, 'Z'
            ja skip_char2
            
            stosb
            
            skip_char2:
            jmp repeta1
            
        iese1:
        
        mov byte [edi], 0
        
         push dword buffer1
         push dword format_read
        call [printf]
        add esp, 4 * 2
        
     

     
    close_file:
        
        push dword [fd]
        call [fclose]
        add esp, 4
       
        
    exit_program:
    
        push    dword 0      
        call    [exit] 

