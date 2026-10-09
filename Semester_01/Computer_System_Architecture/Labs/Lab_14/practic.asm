bits 32 ; assembling for the 32 bits architecture

; declare the EntryPoint (a label defining the very first instruction of the program)
global start        


extern exit, fopen, fclose, fread, fprintf       
import exit msvcrt.dll    
import fopen msvcrt.dll
import fclose msvcrt.dll
import fread msvcrt.dll
import fprintf msvcrt.dll

; our data is declared here (the variables needed by our program)
segment data use32 class=data
    input_fd dd -1
    output_fd dd -1
    
    input_name db "input.txt", 0
    output_name db "output_cretu_cristian.txt", 0
    
    read db "r", 0
    write db "w", 0
    
    string_format db "%s", 0
    char_format db "%c", 0
    digit_format db "%d", 0
    newline db 10, 0
    
    text resb 100
    len equ 100
    

; our code starts here
segment code use32 class=code
    start:
        ; open input file
        push dword read
        push dword input_name
        call [fopen]
        add esp, 4 * 2
       
        cmp eax, 0
        je iesire
        
        mov [input_fd], eax
        
        ; open output file
        push dword write
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
        
        mov esi, text
        
        mov ebx, eax ; we store the number of chars read
        
        push dword text
        push dword [output_fd]
        call [fprintf]
        add esp, 4 * 2
        
        
        ; this is ex 2
        
        push dword newline
        push dword string_format
        push dword [output_fd]
        call [fprintf]
        add esp, 4 * 3
        
        push dword ebx
        push dword digit_format
        push dword [output_fd]
        call [fprintf]
        add esp, 4 * 3
        
        ; this is ex 3
        
        xor ebx, ebx
        repeta:
            lodsb
            
            cmp al, 0
            je iesi
            
            cmp al, ' '
            jne repeta
            
            inc bl
            
            jmp repeta
        

        iesi:
        
        push dword newline
        push dword string_format
        push dword [output_fd]
        call [fprintf]
        add esp, 4 * 3
        
        push dword ebx
        push dword digit_format
        push dword [output_fd]
        call [fprintf]
        add esp, 4 * 3
        
        ; this is ex 3
        
        mov esi, text
        
        push dword newline
        push dword string_format
        push dword [output_fd]
        call [fprintf]
        add esp, 4 * 3
        
        ; print a newline
        
        patru:
            lodsb
            
            cmp al, 0
            je iesi_patru
            
            mov bl, al
            sub bl, '*'
            cmp bl, '/' - '*'
            ja next_char
            
            mov al, '+'
            
            
            next_char:
            
            push dword eax
            push dword char_format
            push dword [output_fd]
            call [fprintf]
            add esp, 4 * 3
            
            ; print al then go to next char
            
            jmp patru
            
        iesi_patru:
        
        ; this is ex 4
        
        mov esi, text
        
        push dword newline
        push dword string_format
        push dword [output_fd]
        call [fprintf]
        add esp, 4 * 3
        
        ; print a newline
        
        xor ebx, ebx
        
        cinci:
            lodsb
            
            cmp al, 0
            je iesi_cinci
            
            mov bh, al
            
            sub bh, '0'
            cmp bh, '9'-'0'
            ja next_char5
            
            add bl, bh
            
            next_char5:
            
            jmp cinci
            
        iesi_cinci:
        
        
        xor bh, bh
        
        push dword ebx
        push dword digit_format
        push dword [output_fd]
        call [fprintf]
        add esp, 4 * 3
        
        ; this is ex 6
        
        push dword newline
        push dword string_format
        push dword [output_fd]
        call [fprintf]
        add esp, 4 * 3
        
        ; print a newline
        mov esi, text
        
        sase:
            lodsb
            
            cmp al, 0
            je iesi_sase
            
            mov bl, al
            sub bl, '*'
            cmp bl, '/' - '*'
            ja next_char6
            
            mov al, '+'
            
            next_char6:
            
            cmp al,' '
            je sase
            
            push dword eax
            push dword char_format
            push dword [output_fd]
            call [fprintf]
            add esp, 4 * 3
            
            ; print al then go to next char
            
            jmp sase
            
        iesi_sase:
        
        
    close_output:
        push dword [output_fd]
        call [fclose]
        add esp, 4
        
    close_input:

        push dword [input_fd]
        call [fclose]
        add esp, 4
       
    iesire:   
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

