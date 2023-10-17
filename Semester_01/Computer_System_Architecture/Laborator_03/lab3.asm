; mov AH, 0
; cwd -> only on signed
; cwd ax -> dx:ax
; cwde ax -> dx:eax

; push EAX 
; is equal to this
; sub ESP ,4
; add [ESP], EAX

; pop EAX
; is equal to this
; mov EAX, [ESP]
; add ESP, 4

; to swap variables
; push EAX
; push EBX
; pop EAX -> takes the top of stack and puts it on EAX (top is EBX)
; pop EBX -> -||- top is (EAX)
; now they are swapped

; if you do push AX, then pop EAX, then your stack will be missaligned
; the stack has 32 bits

; push FD -> you put all the flags in the stack (save the state of the flags)
; push AD -> save the state of EAX, EBX, etc
