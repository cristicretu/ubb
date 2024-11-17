merge_sort([], []).
merge_sort([X], [X]).
merge_sort(List, SortedList) :-
  split_list(List, Left, Right),
  merge_sort(Left, SortedLeft),
  merge_sort(Right, SortedRight),
  merge_sorted(SortedLeft, SortedRight, SortedList).

split_list(List, Left, Right) :-
