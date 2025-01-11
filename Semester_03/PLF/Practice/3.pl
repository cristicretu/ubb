subsets_n(L, N, Result) :- 
    findall(Aux, subsets_aux(L, N, [], Aux), Result).

subsets_aux([], N, Aux, Aux) :- 
    length(Aux, M), 
    M >= N,
    sum_list(Aux, Sum),
    0 is Sum mod 3.
subsets_aux([H|T], N, Aux, Result) :- 
    subsets_aux(T, N, [H|Aux], Result).
subsets_aux([_|T], N, Aux, Result) :- 
    subsets_aux(T, N, Aux, Result).

sum_list([], 0).
sum_list([H|T], Sum) :- 
    sum_list(T, Rest), 
    Sum is H + Rest.