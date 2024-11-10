rem([], []).

rem([H], [H]).

rem([H1, H2], []):-
  1 =:= H2 - H1, !.

rem([H1, H2], [H1, H2]) :-
  1 =\= H2 - H1, !.

rem([H1, H2|T], Res) :-
  1 =:= H2 - H1,
  !,
  rem(T, Res).

rem([H1, H2|T], [H1|Res]) :-
  rem([H2|T], Res).