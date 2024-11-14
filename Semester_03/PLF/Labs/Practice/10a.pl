% Main predicate to check for a valley aspect
valley(List) :-
  % Ensure the list has at least three elements
  List = [_, _, _ | _],
  % Start the helper with the initial flag set to 'decreasing'
  valley_helper(List, decreasing).

% Helper predicate with a flag indicating the current phase
valley_helper([X, Y | Rest], decreasing) :-
  X > Y,  % Continue decreasing
  !,      % Cut to prevent backtracking
  valley_helper([Y | Rest], decreasing).
valley_helper([X, Y | Rest], decreasing) :-
  X < Y,  % Transition from decreasing to increasing
  !,      % Cut to prevent backtracking
  valley_helper([Y | Rest], increasing).
valley_helper([X, Y | Rest], increasing) :-
  X < Y,  % Continue increasing
  !,      % Cut to prevent backtracking
  valley_helper([Y | Rest], increasing).
valley_helper([_], _).  % Single element left is acceptable

% Any other pattern fails the valley condition
valley_helper(_, _) :-
  fail.
