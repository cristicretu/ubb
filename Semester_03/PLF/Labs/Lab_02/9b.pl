% define a predicate to determine the gcd of all numbers from a list

% flow -> (i, i, o)

gcd(X, 0, X).
gcd(X, Y, R) :- Y > 0, Z is X mod Y, gcd(Y, Z, R).


% flow -> (i, 0)
gcdList([], 0).
gcdList([X], X).
gcdList([H|T], R) :-
      gcdList(T, R1),
      gcd(H, R1, R).