(defun associate (l1 l2)
  (cond
    ((or (null l1) (null l2)) nil)
    (T (cons (cons (car l1) (car l2)) (associate (cdr l1) (cdr l2))))
  )
)

(defun split-list (l)
  (cond
    ((null l) (cons nil nil))
    (T (associate (car l) (cadr l)))
  )
)

(print (split-list '((A B C) (X Y Z))))

(defun dpth (lst acc)
  (cond
    ((null lst) acc)
    ((atom (car lst)) (dpth (cdr lst) acc))
    (T (max (dpth (cdr lst) acc) (dpth (car lst) (+ acc 1))))
  )
)

(print (dpth '(1 (2 (3 4) 5) 6) 1))
