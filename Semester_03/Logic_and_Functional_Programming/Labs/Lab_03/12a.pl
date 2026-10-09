% addDiv(i, i, o)
addDiv([], _, []).

addDiv([H|T], D, Res) :-
    D > H,
    addDiv(T, 1, Res).

addDiv([H|T], 1, [H, 1|Res]) :-
    addDiv([H|T], 2, Res).

addDiv([H|T], D, [D|Res]) :-
    D =< H,
    H mod D =:= 0,
    D1 is D + 1,
    addDiv([H|T], D1, Res).

addDiv([H|T], D, Res) :-
    D =< H,
    H mod D =\= 0,
    D1 is D + 1,
    addDiv([H|T], D1, Res).
