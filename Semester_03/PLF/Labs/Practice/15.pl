prec(List, Rez) :-
    prec(List, [], 1, Res),
    remove_leading_zeros(Res, Rez).

% base case
prec([], [], _, []).
% build the accumulator array
prec([H|T], Acc, CF, Res) :-
    prec(T, [H|Acc], CF, Res).
prec([], [H|T], CF, Rez) :-
    N is H - CF,
    (N >= 0 ->
        CF1 = 0,
        N1 = N
    ;
        CF1 = 1,
        N1 = 9
    ),
    prepend(Res, N1, Rez),
    prec([], T, CF1, Res).

prepend([], El, [El]).
prepend([H|T], El, [H|Res]) :-
    prepend(T, El, Res).

remove_leading_zeros([0|T], Res) :-
    T \= [],
    remove_leading_zeros(T, Res).
remove_leading_zeros(List, List).