;; a) Write a function to return the n-th element of a list, or NIL if such an element does not exist.
;; b) Write a function to check whether an atom E is a membru of a list which is not necessarily linear.
;; c) Write a function to determine the list of all sublists of a given list, on any level.
;; A sublist is either the list itself, or any element that is a list, at any level. Example:
;; (1 2(3 (45) (67)) 8(9 10)) => 5 sublists :
;; ((1 2(3(4 5) (6 7)) 8 (9 10)) (3(4 5) (6 7)) (4 5) (6 7) (9 10))
;; d) Write a function to transform a linear list into a set.

; problem a
(defun n_th_element (lista n)
  (if lista
      (if (= n 1)
          (car lista)
          (n_th_element (cdr lista) (- n 1)))
    NIL))

; problem b
(defun membru (e lista)
  (if (null lista) 
      NIL
      (if (equal e (car lista))
          t
          (if (listp (car lista))
              (if (membru e (car lista))
                  t
                  (membru e (cdr lista)))
              (membru e (cdr lista))))))

; problem c
(defun sublists (lista)
  (if (listp lista)
      (append (list lista) (sublists-list lista))
      NIL))

(defun sublists-list (lista)
  (if (null lista)
      NIL
      (append (sublists (car lista))
              (sublists-list (cdr lista)))))


; problem d
(defun convert_to_set (lista)
  (if (null lista)
    NIL
    (if (member (car lista) (cdr lista))
      (convert_to_set (cdr lista))
      (cons (car lista) (convert_to_set (cdr lista))))))

(print (n_th_element '(1 2 3 4 5) 43))
(print (membru 2 '(1 2 3 4 5)))
(print (sublists '(1 2 (3 (4 5) (6 7)) 8 (9 10))))
(print (convert_to_set '(1 2 2 2 2 2 2 3 3 3 3 4 4 4 4 4 5 5 5 5 5  6 6 6 6)))