subs(_, N, S, []) :- S > 0, S mod N =:= 0.
subs([H|T], N, S, [H|Res]) :-
  S1 is S + H, 
  subs(T, N, S1, Res).
subs([_|T], N, S, Res) :-
  subs(T, N, S, Res).
