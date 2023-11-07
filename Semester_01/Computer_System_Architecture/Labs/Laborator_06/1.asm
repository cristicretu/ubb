bits 32 ; assembling for the 32 bits architecture

; declare the EntryPoint (a label defining the very first instruction of the program)
global start        

; declare external functions needed by our program
extern exit               ; tell nasm that exit exists even if we won't be defining it
import exit msvcrt.dll    ; exit is a function that ends the calling process. It is defined in msvcrt.dll
                          ; msvcrt.dll contains exit, printf and all the other important C-runtime specific functions
; our data is declared here (the variables needed by our program)
segment data use32 class=data
    s dd 127F56h, 0ABCDABCDh
    len_s equ $-s
    d times len_s dd 0

; An array with doublewords containing packed data (4 bytes written as a single doubleword) is given. 
; Write an asm program in order to obtain a new array of doublewords, where each doubleword will be 
; composed by the rule: the sum of the bytes from an odd position will be written on the word from the 
; odd position and the sum of the bytes from an even position will be written on the word from the even position. 
; The bytes are considered to represent signed numbers, thus the extension of the sum on a word will be 
; performed according to the signed arithmetic.

; Example:
; for the initial array:
; 127F5678h, 0ABCDABCDh, ...
; The following should be obtained:
; 006800F7h, 0FF56FF9Ah, ... 

segment code use32 class=code
    start:
        mov   esi, s       ; esi will point to the source array
        mov   edi, d       ; edi will point to the destination array

        mov ecx, len_s     ; ecx will hold the number of elements in the array

    process_array:
        test ecx, ecx
        jz end

        xor eax, eax     ; eax will hold the sum of the bytes from an odd position
        xor ebx, ebx     ; ebx will hold the sum of the bytes from an even positiono
        
        mov edx, [esi]   ; edx will hold the current doubleword from the source array
        add esi, 4       ; advance the source pointer to the next doubleword

        mov al, dl       ; al will hold the first byte from the current doubleword
        cbw              ; extend the sign of al to ah
        cwd              ; extend the sign of ax to dx

        add ebx, eax     ; add the first byte from the current doubleword to ebx

        mov edx, [esi]   ; edx will hold the current doubleword from the source array
        shr edx, 8       ; shift the current doubleword to the right by 8 bits

        mov al, dl       ; al will hold the second byte from the current doubleword
        cbw              ; extend the sign of al to ah
        cwd              ; extend the sign of ax to dx
        add eax, ebx     ; add the second byte from the current doubleword to ebx

        mov edx, [esi]   ; edx will hold the current doubleword from the source array
        shr edx, 16    ; shift the current doubleword to the right by 8 bits

        mov al, dl       ; al will hold the third byte from the current doubleword
        cbw              ; extend the sign of al to ah
        cwd              ; extend the sign of ax to dx
        add ebx, eax     ; add the third byte from the current doubleword to ebx

        mov edx, [esi]   ; edx will hold the current doubleword from the source array
        shr edx, 24       ; shift the current doubleword to the right by 8 bits
        mov al, dl       ; al will hold the fourth byte from the current doubleword
        cbw              ; extend the sign of al to ah
        cwd              ; extend the sign of ax to dx
        add eax, ebx     ; add the fourth byte from the current doubleword to ebx

        shl ebx, 16      ; shift the sum of the bytes from an even position to the left by 16 bits
        or eax, ebx      ; combine the sum of the bytes from an even position with the sum of the bytes from an odd position

        mov [edi], eax   ; store the result in the destination array
        add edi, 4       ; advance the destination pointer to the next doubleword

        loop process_array

    end:
       
        int 3               ; breakpoint
        ; exit(0)
        push    dword 0      ; push the parameter for exit onto the stack
        call    [exit]       ; call exit to terminate the program

