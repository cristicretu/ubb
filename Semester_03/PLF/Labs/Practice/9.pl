ins(List, -infinity, List).

ins(List, El, Res) :-
  ins(List, 1, 1, El, Res).

ins([], _, _, _, []).
ins([H|T], Cur, Nxt, El, [H, El|Res]) :-
   Cur =:= Nxt,
   Cur1 is Cur + 1,
   Nxt1 is Nxt + Cur1,
   ins(T, Cur1, Nxt1, El, Res),
   !.
ins([H|T], Cur, Nxt, El, [H|Res]) :-
  Cur1 is Cur + 1,
  ins(T, Cur1, Nxt, El, Res).

% main(list, prev, res)
main(List, Res) :-
  main(List, -infinity, Res).

main([], _, []).
main([[H|T]|Taillist], Prev, [Res|Rez]) :-
  ins([H|T], Prev, Res),
  main(Taillist, Prev, Rez).
main([H|T], _, [H|Res]) :-
  Prev1 is H,
  main(T, Prev1, Res).


% [1,2,3,4], 69
% [1, 69, 2, 3, 69 ]