% base case: if K is out of bounds, return the list
insert(List, K, _, List) :-
      length(List, N),
      (K > N ; K < 0).

% base case k = 0, or empty list
insert(List, 0, E, [E|List]).

% recursive case
insert([H|T], K, E, [H|Result]) :-
      K > 0,
      K1 is K - 1,
      insert(T, K1, E, Result).

