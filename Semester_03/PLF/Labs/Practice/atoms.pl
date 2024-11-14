% Define a predicate to produce a list of pairs (atom n) from an initial list of atoms. In this initial list atom has 
% n occurrences.
% Eg.: numberatom([1, 2, 1, 2, 1, 3, 1], X) => X = [[1, 4], [2, 2], [3, 1]].

countOcc(List, El, Res) :-
  countOcc(List, El, 0, Res).

countOcc([], _, Count, Count).
countOcc([H|T], El, Count, Res) :-
  H =:= El,
  Count1 is Count + 1,
  countOcc(T, El, Count1, Res), !.
countOcc([_|T], El, Count, Res) :-
  countOcc(T, El, Count, Res).


removeEl([], _, []).
removeEl([H|T], El, Res) :-
  H =:= El,
  removeEl(T, El, Res), !.
removeEl([H|T], El, [H|Res]) :-
  removeEl(T, El, Res).

numberatom([], []).
numberatom([H|T], [[H, Occ]|Res]) :-
    countOcc([H|T], H, Occ),
    removeEl([H|T], H, NewList),
    numberatom(NewList, Res).
