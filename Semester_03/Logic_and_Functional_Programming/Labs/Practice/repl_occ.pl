% replace all occurences of an element E in a list L with the elementes of another list
% example: substitute_elem([1, 2, 1, 3, 1, 4], 1, [10, 11]) -> [10, 11, 2, 10, 11, 3, 10, 11, 4]

% flow model repl(i, i, i, i, o)
repl([], _, _, _, []).

repl([H|T], El, [H1|T1], AlistCp, [H1|Res]):-
  H =:= El,
  repl([H|T], El, T1, AlistCp, Res), !.

repl([_|T], El, [], AlistCp, Res):-
  repl(T, El, AlistCp, AlistCp, Res).

repl([H|T], El, _, AlistCp, [H|Res]):-
  repl(T, El, AlistCp, AlistCp, Res).

% wrapper
repl(List, El, AList, Result) :-
    repl(List, El, AList, AList, Result).
