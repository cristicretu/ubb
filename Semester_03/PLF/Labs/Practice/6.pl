% prod(List, Accumulator, Remainder, Res)
prod(List, Digit, Rez) :-
  prod(List, [], 0, Digit, Res),
  remove_leading_zeros(Res, Rez).

prod([], [], Remainder, _, [Remainder]).
prod([H|T], Acc, Remainder, Digit, Res) :-
  prod(T, [H|Acc], Remainder, Digit, Res).
prod([], [H|T], Remainder, Digit, Rez) :-
  Numb is H * Digit + Remainder,
  Ul is Numb mod 10,
  Remainder1 is Numb div 10,
  prepend(Res, Ul, Rez),
  prod([], T, Remainder1, Digit, Res).

% prepend(List, Element, Res)
prepend([], El, [El]).
prepend([H|T], El, [H|Res]) :-
  prepend(T, El, Res).

remove_leading_zeros([0|T], Res) :-
  remove_leading_zeros(T, Res).
remove_leading_zeros(List, List).

