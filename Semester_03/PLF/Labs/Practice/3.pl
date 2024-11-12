mergeLists([], [], []).
mergeLists([H|T], [], [H|T]) :- !.
mergeLists([], [H|T], [H|T]) :- !.

mergeLists([H1|T1], [H2|T2], [H1|Res]) :-
  H1 < H2,
  !,
  mergeLists(T1, [H2|T2], Res).

mergeLists([H1|T1], [H2|T2], [H2|Res]) :-
  H2 > H1,
  !,
  mergeLists([H1|T1], T2, Res).

mergeLists([H1|T1], [H2|T2], Res) :-
    mergeLists(T1, T2, Res).


main([], Curr, Curr).
main([[H|T]|Taillist], Curr, Rez) :-
    mergeLists([H|T], Curr, Res),
    !,
    main(Taillist, Res, Rez).
main([_|T], Curr, Res) :-
    main(T, Curr, Res).
