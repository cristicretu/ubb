pairs([], []).
pairs([_], []).
pairs([H|T], Result) :-
    pair_with_rest(H, T, Pairs),
    pairs(T, RestPairs),
    append(Pairs, RestPairs, Result).

pair_with_rest(_, [], []).
pair_with_rest(X, [H|T], [[X,H]|Pairs]) :-
    pair_with_rest(X, T, Pairs).

append([], L, L).
append([H|T], L2, [H|L3]) :- 
    append(T, L2, L3).