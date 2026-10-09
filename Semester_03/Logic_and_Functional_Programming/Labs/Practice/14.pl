% lon([H|T], Len, Curr, Ans)

lon(List, Res):-
    lon(List, [], 0, [], 0, Res).
lon([], _, _, Acc, _, Acc).
lon([H|T], AccCur, LenCur, Acc, LenAcc, Res) :-
    0 =:= H mod 2,
    append(AccCur, [H], NewAccCur),
    LenCur1 is LenCur + 1,
    (LenCur1 > LenAcc -> lon(T, NewAccCur, LenCur1, NewAccCur, LenCur1, Res), ! ; lon(T, NewAccCur, LenCur1, Acc, LenAcc, Res)).
lon([H|T], AccCur, LenCur, Acc, LenAcc, Res) :-
    1 =:= H mod 2,
    lon(T, [], 0, Acc, LenAcc, Res).


% lon(List, Res) :-
%   lon(List, 0, [], Res).

% lon([H1,H2|T], Len, Curr, Res):-
%   H1 =< H2,
%   0 is mod(H1, 2),
%   0 is mod(H2, 2),
%   Len1 is Len + 2,
%   lon(T, Len1, [H1, H2|Curr], Res).


% lon([H1,H2|T], Len, Curr, Res):-
%     (H1 > H2 ; 1 is mod(H1, 2) ; 1 is mod(H2, 2)),
%     lon([H2|T], 0, [], Res1),
%     (Len > 0 ->
%         reverse(Curr, RevCurr),
%         longer_seq(RevCurr, Res1, Len, Res)
%     ;
%         Res = Res1
%     ).


% lon([H], Len, Curr, Res):-
%     (Len > 0 ->
%         reverse(Curr, Res)
%     ;
%         (0 is mod(H, 2) ->
%             Res = [H]
%         ;
%             Res = []
%         )
%     ).

% lon([], Len, Curr, Res):-
%     (Len > 0 ->
%         reverse(Curr, Res)
%     ;
%         Res = []
%     ).

% longer_seq(Seq1, Seq2, Len1, Res):-
%     length(Seq2, Len2),
%     (Len1 >= Len2 ->
%         Res = Seq1
%     ;
%         Res = Seq2
%     ).
