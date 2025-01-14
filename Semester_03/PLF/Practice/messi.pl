p(1).
p(2).
q(1).
q(2).
r(1).
r(2).
s:- p(X),!, q(Y), r(Z), assert(result(X,Y,Z)), nl.

messi :- findall(_, s, _).