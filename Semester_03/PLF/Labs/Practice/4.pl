% b. For a heterogeneous list, formed from integer numbers and list of digits, write a predicate to compute the
% um of all numbers represented as sublists.
% Eg.: [1, [2, 31, 4, 5, [6, 7, 9], 10, 11, [1, 2, 0], 6] => [8, 2, 2].

getNumber(List, Res) :-
  getNumber(List, [], 1, Res).

getNumber([],[], _, 0).
getNumber([H|T], Acc, Cnt, Res) :-
  getNumber(T, [H|Acc], Cnt, Res).
getNumber([], [H|T], Cnt, Res) :-
  Number is H * Cnt,
  Cnt1 is Cnt * 10,
  getNumber([], T, Cnt1, Rez),
  Res is Number + Rez.

getDigits(0, []) :- !.
getDigits(N, Res) :-
  D is N mod 10,
  N1 is N div 10,
  getDigits(N1, D1),
  prepend(D1, D, Res).

prepend([], El, [El]).
prepend([H|T], El, [H|Res]) :-
  prepend(T, El, Res).

final(List, Result) :-
  main(List, Numb),
  getDigits(Numb, Result).

main([], 0).
main([[H|T]|Taillist], Res) :-
  getNumber([H|T], Numb),
  main(Taillist, Rez),
  Res is Numb + Rez.
main([_|T], Res) :-
  main(T, Res).