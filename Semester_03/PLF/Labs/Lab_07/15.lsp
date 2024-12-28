(defun reverse_list (l)
  (cond
    ((null l) nil)
    ((atom l) l)
    (T (my_reverse (mapcar #'reverse_list l)))
  )
)

(defun my_reverse (l)
  (reverse-helper l nil))

(defun reverse-helper (l acc)
  (cond
    ((null l) acc)
    (T (reverse-helper (cdr l) (cons (car l) acc)))
  )
)

(print (my_reverse '(1 2 3)))
(print (reverse_list '(1 2 3)))
(print (reverse_list '(1 2 (a (4 5 6)) 7)))


