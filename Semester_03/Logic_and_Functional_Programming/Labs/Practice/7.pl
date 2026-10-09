maxpos(List, Res):-
  maxpos(List, -100, 1, [],  Res).

maxpos([], _, _, Acc, Acc).
maxpos([H|T], MaxSoFar, Idx, _, Res) :-
  H > MaxSoFar,
  NextIdx is Idx + 1,
  maxpos(T, H, NextIdx, [Idx], Res).

maxpos([H|T], MaxSoFar, Idx, Acc, Res) :-
  H =:= MaxSoFar,
  NextIdx is Idx + 1,
  prepend(Acc, Idx, NewAcc),
  maxpos(T, MaxSoFar, NextIdx,NewAcc,Res).

maxpos([H|T], MaxSoFar, Idx, Acc, Rez) :-
  H < MaxSoFar,
  NextIdx is Idx + 1,
  maxpos(T, MaxSoFar, NextIdx, Acc, Rez).

prepend([], El, [El]).
prepend([H|T], El, [H|Res]) :-
  prepend(T, El, Res).

main([], []).
main([[H|T]|Tailist], [Rez|Res]) :-
  maxpos([H|T], Rez),
  main(Tailist, Res).
main([H|T], [H|Res]) :-
  main(T, Res).