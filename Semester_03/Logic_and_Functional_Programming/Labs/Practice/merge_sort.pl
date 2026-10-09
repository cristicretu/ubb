merge_sort([], []).
merge_sort([X], [X]).

merge_sort(List, Sorted) :-
  split_list(List, Left, Right),
  merge_sort(Left, SortedLeft),
  merge_sort(Right, SortedRight),
  merge(SortedLeft, SortedRight, Sorted).

split_list([], [], []).
split_list([X], [X], []).
split_list([X,Y|Tail], [X|Left], [Y|Right]) :-
  split_list(Tail, Left, Right).

merge([], List2, List2).
merge(List1, [], List1).
merge([H1|T1], [H2|T2], [H1|Merged]) :-
  H1 =< H2,
  !,
  merge(T1, [H2|T2], Merged).
merge([H1|T1], [H2|T2], [H2|Merged]) :-
  merge([H1|T1], T2, Merged).
