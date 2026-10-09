even_len([]).

even_len([_,_|T]):-
  even_len(T).