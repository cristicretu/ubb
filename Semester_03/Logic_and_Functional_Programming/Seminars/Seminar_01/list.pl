#filter even numbers from a list

filterEven([],[]).
filterEven([H|T], Rez):-
    H mod 2 =:= 0,
    filterEven(T, Rez).
filterEven([H|T], [H|Rez1]):-
    H mod 2 =\= 0,
    filterEven(T, Rez1).

# this is tail recursive, it is better than the previous one because it uses the same stack frame for all the recursive calls
