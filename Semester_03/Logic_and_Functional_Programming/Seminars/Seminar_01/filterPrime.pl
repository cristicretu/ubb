#filter even numbers from a list

filterPrime([],[]).
filterPrime([H|T], [H|Rez1]):-
    isPrime(H, 2),
    filterPrime(T, Rez1).
filterPrime([H|T], Rez):-
    not(isPrime(H, 2)),
    filterPrime(T, Rez).


isPrime(X):- X>1, isPrime(X, 2).


isPrime(X, D):- D >= X.
isPrime(X, D):- X mod D =\= 0, D1 is D + 1, isPrime(X, D1).
