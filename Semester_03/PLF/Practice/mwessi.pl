g([], []).
g([_|T], S) :- !, g(T,S).
g([H|T], [H|S]) :- H mod 2 =:= 0, g(T,S).