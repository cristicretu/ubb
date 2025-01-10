% comb (l1...ln, k) = { nil, k = 0 }
% comb (l1...ln, k) = { l1...ln, k = 1 }
% comb (l1...ln, k) = { l1 U comb (l2...ln, k-1), k > 1  }
% comb (l1...ln, k) = { comb (l2...ln, k), k > 1  }

comb(_, 0, []).
comb([H|T], K, [H|Res]) :- K > 0, K1 is K - 1, comb(T, K1, Res), isIncreasing([H|Res]).
comb([_|T], K, Res) :- K > 0, comb(T, K, Res).

isIncreasing([_]).
isIncreasing([X,Y|T]) :- X < Y, isIncreasing([Y|T]).



% comb (l1...ln, k) = { l1...ln, k = 0 }
% comb (l1...ln, k) = { comb (l1...ln, k-1, x U l1....ln) x < l1, (x, l) = getElem(l1...ln)  }
% comb (l1...ln, k) = { comb (l1...ln, k, l1...ln) x >= l1, (x, l) = getElem(l1...ln)  }

getElem([H|T], H, T).
getElem([H|T], Elem, [H|Res]) :- getElem(T, Elem, Res).

comb2(_, 0, Col, Col).
comb2(List, K, [H|Col], Res) :- 
    K > 0, 
    getElem(List, X, L1), 
    X < H, 
    K1 is K - 1, 
    comb2(L1, K1, [X,H|Col], Res).
comb2(List, K, [H|Col], Res) :- 
    K > 0, 
    getElem(List, _, L1), 
    comb2(L1, K, [H|Col], Res).

comb2(List, K, Res) :- K > 0, K1 is K - 1, getElem(List, E, L1), comb2(L1, K1, [E], Res).
