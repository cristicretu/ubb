succ(List, Res):-
  succ(List, [], 1, Res).

succ([], [], CF, [CF]):-
  CF =\= 0, !.
succ([], [], 0, []).

succ([H|T], Acc, CF, Res):-
  succ(T, [H|Acc], CF, Res).

succ([], [H|T], CF, Rez):-
  Numb is H + CF,
  Ul is Numb mod 10,
  CF1 is Numb div 10,
  succ([], T, CF1, Res),
  app(Res, Ul, Rez).

app([], El, [El]).
app([H|T], El, [H|Res]):-
  app(T, El, Res).

