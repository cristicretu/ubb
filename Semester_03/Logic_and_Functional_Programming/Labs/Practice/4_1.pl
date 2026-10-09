set_difference([], _, []).
set_difference([H|T], Set2, Result) :- 
    member(H, Set2), 
    set_difference(T, Set2, Result), !.
set_difference([H|T], Set2, [H|Result]) :- 
    set_difference(T, Set2, Result).

member(X, [X|_]).
member(X, [_|T]) :- member(X, T).