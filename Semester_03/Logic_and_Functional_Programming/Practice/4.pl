arrange(0, _, []).

arrange(K, Y, [y|Rest]) :-
    K > 0,
    Y < 3,
    K1 is K - 1,
    Y1 is Y + 1,
    arrange(K1, Y1, Rest).

arrange(K, Y, [r|Rest]) :-
    K > 0,
    K1 is K - 1,
    arrange(K1, Y, Rest).

find_all_arrangements(Arrangements) :-
    findall(A, arrange(5, 0, A), Arrangements).