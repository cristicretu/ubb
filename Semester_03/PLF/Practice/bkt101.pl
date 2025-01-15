mysubset([], []).
mysubset([H|T], [H|Res]) :-
  mysubset(T, Res).
mysubset([_|T], Res):-
  mysubset(T, Res).

test_subset(Res) :- findall(R, mysubset([1,2,3], R), Res).

mycombination([], 0, []).
mycombination([], K, []) :- K > 0, !, fail.
mycombination([H|T], K, [H|Combi]) :-
    K > 0,
    K1 is K - 1,
    mycombination(T, K1, Combi).
mycombination([_|T], K, Combi) :-
    K > 0,
    mycombination(T, K, Combi).

test_combination(Res) :- findall(R, mycombination([1,2,3], 2, R), Res).

mypermutation([], []).
mypermutation(List, [X|Perm]) :-
  append(Left, [X|Right], List),
  append(Left, Right, Rest),
  mypermutation(Rest, Perm).

test_permutation(Res) :- findall(R, mypermutation([1,2,3], R), Res).

myarrangements(List, K, Res) :-
  mycombination(List, K, Rez),
  mypermutation(Rez, Res).

test_arrangements(Res) :- findall(R, myarrangements([1,2,3], 2, R), Res).