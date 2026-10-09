rem(List, Res) :-
  rem(List, 1, 1, Res).

rem([], _, _, []).
rem([_|T], Curr, Nxt, Res) :-
  Curr =:= Nxt,
  Curr1 is Curr + 1,
  Nxt1 is Nxt + Curr1,
  rem(T, Curr1, Nxt1, Res).
rem([H|T], Curr, Nxt, [H|Res]) :-
  Curr =\= Nxt,
  Curr1 is Curr + 1,
  rem(T, Curr1, Nxt, Res).
