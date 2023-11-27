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

; Read a file name and a text from the keyboard. Create a file with that name in the current folder and write the text that has been read to file. Observations: The file name has maximum 30 characters. The text has maximum 120 characters.


segment data use32 class=data
    file_name resb 30
    file_text resb 120
    file_descriptor dd -1
    hint_message db "Enter some text: ", 0
    read_format db "%s", 0
    access db "w", 0
    print_format db "%s", 0


segment code use32 class=code
start:
    ; print read fle name
    push dword hint_message
    push dword print_format
    call [printf]
    add esp, 4 * 2
    
    ; read file name
    push dword file_name
    push dword read_format
    call [scanf]
    add esp, 4 * 2
    
    ; create file
    push dword access
    push dword file_name
    call [fopen]
    add esp, 4 * 2
    
    cmp eax, 0
    je final
    
    ; store file descriptor
    mov [file_descriptor], eax
    
    ; print read some text
    push dword hint_message
    push dword print_format
    call [printf]
    add esp, 4 * 2
    
    ; read the text
    push dword file_text
    push dword read_format
    call [scanf]
    add esp, 4 * 2
    
    ; print the text
    push dword file_text
    push dword print_format
    push dword [file_descriptor]
    call [fprintf]
    add esp, 4 * 3
    
    ; close the file
    push dword [file_descriptor]
    call [fclose]
    add esp, 1
    
    final:

    push dword 0 
    call [exit] 
