p(N, O, C, []) :- O =:= N div 2, C =:= N div 2.

p(N, O, C, ['('|R]) :- 
    O < N div 2, 
    O1 is O + 1, 
    p(N, O1, C, R).

p(N, O, C, [')'|R]) :- 
    C < O,
    C1 is C + 1, 
    p(N, O, C1, R).