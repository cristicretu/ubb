# addDiv(i, i, o)
addDiv([], _, []).

# base case, our div got past the element that we are looking for divs
addDiv([H|T], D, Res) :-
    D > H,
    addDiv(T, 1, Res).

# base case, first div is always true since 1 divs all numbers, add both
addDiv([H|T], 1, [H, 1|Res]) :-
    addDiv([H|T], 2, Res).

# add the new div we found, we already have the H in the list
addDiv([H|T], D, [D|Res]) :-
    D =< H,
    H mod D =:= 0,
    D1 is D + 1,
    addDiv([H|T], D1, Res).

# keep looking for the next divizor
addDiv([H|T], D, Res) :-
    D =< H,
    H mod D =\= 0,
    D1 is D + 1,
    addDiv([H|T], D1, Res).
