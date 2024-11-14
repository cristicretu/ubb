last_set(List, Set) :-
  last_set_helper(List, [], Set).

last_set_helper([], Acc, Acc).  
last_set_helper([H|T], Acc, Set) :-
  delete(Acc, H, Acc1),         
  append(Acc1, [H], Acc2),       
  last_set_helper(T, Acc2, Set). 
