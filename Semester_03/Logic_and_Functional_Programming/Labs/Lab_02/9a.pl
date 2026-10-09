% flow -> (i, i, i, o)

insert([], _, _, _).

insert(List, 1, E, [E|List]).

insert([H|T], K, E, [H|Result]) :-
      K > 0,
      K1 is K - 1,
      insert(T, K1, E, Result).

