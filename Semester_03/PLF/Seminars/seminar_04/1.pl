% gwenerate all the sublists of elements that have a valley aspect from a list of numbers.
% ex for [5,3,4,2,7, 11,1,8,6]

subset([], []).
subset([H|T], [H|R]) :- subset(T, R).
subset([_|T], R) :- subset(T, R).

valley([_], 1):-!.
valley([H1,H2|T], 0) :- H1 > H2, !, valley([H2|T], 0).
valley([H1,H2|T], 0) :- H1 < H2, !, valley([H2|T], 1).
valley([H1,H2|T], 1) :- H1 < H2, valley([H2|T], 1).

valley([H1,H2,H3|T]) :- H1 > H2, valley([H1, H2, H3|T], 0).

% solve(L, R) :- subset(L, R), valley(R).

getElem([E|_], E, []).
getElem([H|T], E, [H|R]) :- getElem(T, E, R).

buildValley(_, 1, Col, Col).
buildValley(L, 0, [Elem|Col], Res) :-
  getElem(L, NewE, Rest),
  NewE < Elem,
  buildValley(Rest, 0, [NewE, Elem|Col], Res).
buildValley(L, 0, [Elem|Col], Res) :-
  getElem(L, NewE, Rest),
  NewE > Elem,
  buildValley(Rest, 1, [NewE, Elem|Col], Res).
buildValley(L, 1, [Elem|Col], Res) :-
  getElem(L, NewE, Rest),
  NewE > Elem,
  buildValley(Rest, 1, [NewE, Elem|Col], Res).

solve(L, Res) :-
  getElem(L, E1, Rest1),
  getElem(Rest1, E2, Rest2),
  E2 < E1,
  buildValley(Rest2, 0, [E2, E1], Res).
  
  

% findall(R,  solve([5,3,4,2,7,11,1,8,6], R), Rez).
