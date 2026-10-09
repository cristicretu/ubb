findDivs(N, D, []) :-
    D >= N.
findDivs(N, D, [D|Res]) :-
    D < N,
    N mod D =:= 0,
    D1 is D + 1,
    findDivs(N, D1, Res).
findDivs(N, D, Res) :-
    D < N,
    N mod D =\= 0,
    D1 is D + 1,
    findDivs(N, D1, Res).

singleList([], []).
singleList([H|T], Res) :-
    findDivs(H, 2, Divs),
    merge([H|Divs], TailRes, Res),
    singleList(T, TailRes).

processHeteroList([], []).
processHeteroList([[H|T]|TailList], [Res|ResList]) :-
    singleList([H|T], Res),
    processHeteroList(TailList, ResList), !.
processHeteroList([H|TailList], [H|ResList]) :-  
    processHeteroList(TailList, ResList).

merge([], L, L).
merge([H|T], L, [H|R]) :-
    merge(T, L, R).