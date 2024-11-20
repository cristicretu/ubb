;; a) Write a function to return the n-th element of a list, or NIL if such an element does not exist.
;; b) Write a function to check whether an atom E is a member of a list which is not necessarily linear.
;; c) Write a function to determine the list of all sublists of a given list, on any level.
;; A sublist is either the list itself, or any element that is a list, at any level. Example:
;; (12(3 (45) (67)) 8(9 10)) => 5 sublists :
;; ((12(3(45) (67))8(910)) (3(45) (67)) (45) (67) (910))
;; d) Write a function to transform a linear list into a set.

(defun nth-elem (lista n)
  (if (= n 1)
    (car lista)
    (nth-elem (cdr lista) (- n 1))
  )
)

(print (nth-elem `(1 2 3 4) 0))
(print (nth-elem `(1 2 3 4) 3))