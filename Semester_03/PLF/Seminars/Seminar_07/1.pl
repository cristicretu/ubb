countOcc(List, X, N) :- countOcc(List, X, 0, N).

countOcc([], _, Acc, Acc) :- !.
countOcc([H|T], X, Acc, N) :- H = X, !, NextAcc is Acc + 1, countOcc(T, X, NextAcc, N).
countOcc([H|T], X, Acc, N) :- countOcc(T, X, Acc, N).

mein([], _, []).
mein([H|T], X, Res) :-
  countOcc(X, H, N),
  N > 1,
  mein(T, X, Res), !.
mein([H|T], X, [H|Res]) :-
  countOcc(X, H, N),
  N = 1,
  mein(T, X, Res), !.

mein(List, Res) :-
  mein(List, List, Res).
