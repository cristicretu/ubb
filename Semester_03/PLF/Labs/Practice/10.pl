% a. For a list of integer numbers, define a predicate to write twice in list every prime number.
% b. For a heterogeneous list, formed from integer numbers and list of numbers, define a predicate to write in
% every sublist twice every prime number.
% Eg.: [1, [2, 31, 4, 5, [1, 4, 6], 3, [1, 3, 7, 9, 10], 5] =>
% [1, 12, 2, 3, 31, 4, 5, [1, 4, 6], 3, [1, 3, 3, 7, 7, 9, 10], 5]

isPrime(X):-
  X >= 2,
  isPrime(X, 2).

isPrime(2, _).

isPrime(X, D):-
  D * D > X, !.

isPrime(X, D):-
  X mod D =\= 0,
  D1 is D + 1,
  isPrime(X, D1).

twice([], []).
twice([H|T], [H,H|Res]):-
  isPrime(H),
  twice(T, Res), !.
twice([H|T], [H|Res]):-
  twice(T, Res).

main([], []).
main([[H|T]|Taillist], [Res|Rez]):-
  twice([H|T], Res), 
  main(Taillist, Rez), !.
main([H|T], [H|Res]):-
  main(T, Res).
  