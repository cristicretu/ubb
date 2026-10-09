mountain([_], _).

mountain([H1, H2|T], 0) :- H1 < H2, mountain([H2|T], 0).
mountain([H1, H2|T], 0) :- H1 > H2, mountain([H2|T], 1).

mountain([H1, H2|T], 1) :- H1 > H2, mountain([H2|T], 1).

mountain([H1, H2|T]) :- H1 < H2, mountain([H2|T], 0).

remove_odd([], []).
remove_odd([H|T], [H|R]) :- 0 is H mod 2, !, remove_odd(T, R).
remove_odd([_|T], R) :- remove_odd(T, R).

main([], []):- !.
main([[H|T]|R], [Res|R]):- mountain([H|T]), remove_odd([H|T], Res), !.
main([H|T], [H|R]):- main(T, R).