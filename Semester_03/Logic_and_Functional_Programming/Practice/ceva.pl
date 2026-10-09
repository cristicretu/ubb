f([], 0).
f([H|T], Res):- 
    f(T, S1),
    f_aux(H, S1, Res).

f_aux(H, S1, H) :- S1 < H, !.
f_aux(H, S1, S1).

