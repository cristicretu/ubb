bubble_sort(List, Sorted) :-
    bubble_sort_step(List, List1),   
    !,                                
    (List1 = List                     % Check if any swaps were made
    -> Sorted = List                  % If no swaps, list is sorted
    ; bubble_sort(List1, Sorted)      % Otherwise, continue sorting
    ).

bubble_sort_step([], []).
bubble_sort_step([X], [X]).
bubble_sort_step([X,Y|Tail], [Y,X|Rest]) :-
    X > Y,
    !,
    bubble_sort_step([X|Tail], Rest).
bubble_sort_step([X,Y|Tail], [X|Rest]) :-
    bubble_sort_step([Y|Tail], Rest).