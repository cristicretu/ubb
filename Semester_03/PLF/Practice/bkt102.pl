myperm([], []).
myperm(List, [X|Permutare]) :-
  append(Left, [X|Right], List), 
  append(Left, Right, Temp), % get temp, where temp is List / x
  myperm(Temp, Permutare).

mycomb(_, 0, []).
mycomb([], K, []) :- K > 0, !, fail.
mycomb([H|T], K, [H|Res]) :-
  K > 0,
  K1 is K - 1,
  mycomb(T, K1, Res).
mycomb([_|T], K, Res) :-
  K > 0,
  mycomb(T, K, Res).

test_combination(Res) :- findall(R, mycomb([1,2,3], 4, R), Res).

myaran(List, K, Res) :-
  mycomb(List, K, Rez),
  myperm(Rez, Res).

test_arrangements(Res) :- findall(R, myaran([1,2,3], 2, R), Res).

myproduct([], 1).
myproduct([H|T], R1) :-
    myproduct(T, Res),
    R1 is Res * H.

mysolution_helper(List, K, P, Aran) :-
  myaran(List, K, Aran),
  myproduct(Aran, Prod),
  Prod =:= P.

mysolution(List, K, P, R) :-
  findall(Aran, mysolution_helper(List, K, P, Aran), R).
