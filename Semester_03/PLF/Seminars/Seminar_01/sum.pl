# sum of all numbers smaller or equal to n
s(0, 0).
s(N, S) :-
    N > 0,
    N1 is N - 1,
    s(N1, S1),
    S is S1 + N.

