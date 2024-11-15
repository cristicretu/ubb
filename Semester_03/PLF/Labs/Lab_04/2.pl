% flow(i, i, i)
collinear([_]).
collinear([_,_]).
collinear([[A,B], [M,N], [X,Y] | Rest]) :-
    0 =:= A * (N - Y) + M * (Y - B) + X * (B - N),
    collinear([[M,N], [X,Y] | Rest]).

% flow(i ,i)
apartine(X, [X|_]).
apartine(X, [_|T]) :- apartine(X, T).

% flow(i, i)
subset([], []).
subset([H|T], [H|R]):- subset(T, R).
subset([_|T], R):- subset(T, R).

% flow (i, i)
isSol([], []).
isSol(L, K):- 
    subset(L, K),
    my_length(K, Len),
    Len >= 3, 
    collinear(K).

% flow (i, o)
colPoints(L, Solutions) :-
    collectSolutions(L, [], Solutions).

collectSolutions(L, Acc, Solutions) :-
    isSol(L, Solution),
    not(apartine(Solution, Acc)),
    !, 
    collectSolutions(L, [Solution|Acc], Solutions).
collectSolutions(_, Solutions, Solutions).

% flow(i, o)
my_length(List, Res) :-
  my_length(List, 0, Res).
my_length([], Acc, Acc).
my_length([_|T], Cnt, Res) :-
  Cnt1 is Cnt + 1,
  my_length(T,Cnt1, Res).