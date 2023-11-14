; Codul de mai jos va afisa mesajul "Ana are 17 mere"
bits 32
global start        

; declararea functiilor externe folosite de program
extern exit, printf, scanf    ; adaugam printf ca functie externa            
import exit msvcrt.dll    
import printf msvcrt.dll    ; indicam asamblorului ca functia printf se gaseste in libraria msvcrt.dll
import scanf msvcrt.dll     ; similar pentru scanf
                        
                        
segment data use32 class=data
    a dd 0
    b dd 0
    nr_a dd 'a', 0
    nr_b dd 'b', 0
    message  dd "%s=", 0
    format  dd "%d", 0  ; %d <=> un numar decimal (baza 10)
    print_message dd "%s * %s = %d", 0

segment  code use32 class=code
    start:
        ; print a = 
        push dword nr_a
        push dword message
        call [printf]
        add esp, 4 * 2
        
        ; scanf a
        push dword a
        push dword format
        call [scanf]
        add esp, 4 * 2
        
        ; print b =
        push dword nr_b
        push dword message
        call [printf]
        add esp, 4 * 2
        
         ; scanf b
        push dword b
        push dword format
        call [scanf]
        add esp, 4 * 2
        
        ; multiply a * b
        
        mov eax, [a]
        mov ebx, [b]
        mul ebx
        
        push dword eax
        push dword nr_b
        push dword nr_a
        push  dword print_message
        call [printf]
        add esp, 4 * 3
        
        
        
        
        
     
        
        
        
        
        
        
        ; exit(0)
        push dword 0      ; punem pe stiva parametrul pentru exit
        call [exit]       
