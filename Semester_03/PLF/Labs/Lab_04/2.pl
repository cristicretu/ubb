subset([], []).
subset([H|T], [H|R]):-
  subset(T, R).
subset([_|T], R):-
  subset(T, R).

collinear([[A, B], [M, N], [X, Y]]):-
    0 =:= A * (N - Y) + M * (Y - B) + X * (B - N).

process([], []).
process(L, K):-
	subset(L, K),
	collinear(K).

allSolutions(L, R) :-
    findall(RPartial, process(L, RPartial), R).