myabs(X, -X) :- X < 0, !.
myabs(X, X).

myvalid([_]).
myvalid([X,Y|T]) :- 
  Dif is X - Y,
  myabs(Dif, R),
  R =< 2,
  myvalid([Y|T]).

permutation([], []).
permutation(L, [X|Perm]) :-
  append(Left, [X|Right], L),
  append(Left, Right, Rest),
  permutation(Rest, Perm).

permutation_helper(L, Res) :-
  permutation(L, Res),
  myvalid(Res).

mysolution(N, Res) :-
  linspeis(N, L),
  findall(Res, permutation_helper(L, Res), Res).

linspeis(N, Res) :-
  linspeis(N, N, Res).

linspeis(0, _, 0).

linspeis(N, Initial, [N|Res]) :-
  N =< Initial * 2,
  N1 is N + 1,
  linspeis(N1, Initial, Res).

linspeis(N, Initial, []) :-
  N > Initial * 2.
