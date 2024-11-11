replace([], _, _, []).

replace([H|T], El1, El, [El|Res]) :-
    H =:= El1,
    !,
    replace(T, El1, El, Res).
replace([H|T], El1, El, [H|Res]) :-
    replace(T, El1, El, Res).

  
maxList([], -inf).
maxList([H|T], Max) :-
    maxList(T, TailMax),
    (H >= TailMax -> Max = H ; Max = TailMax), !.

secondMax([], _, -inf).
secondMax([H|T], Max, Res) :-
    secondMax(T, Max, TailRes),
    (H < Max, H >= TailRes -> Res = H ; Res = TailRes), !.

main([], []).
main([[H|T]|TailList], [Res|Rez]) :-
    maxList([H|T], Max),
    secondMax([H|T], Max, SecondMax),
    replace([H|T], SecondMax, Max, Res),
    main(TailList, Rez).
main([H|TailList], [H|Res]) :-
    main(TailList, Res).