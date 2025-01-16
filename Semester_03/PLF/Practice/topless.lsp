(defun elimin (l)
  (cond 
    ((and (realp l) (minusp l)) nil)
    ((atom l) (list l))
    (t (list (mapcan #'elimin l)))
  )
)

(defun elim (l) (car (elimin l)))

(print (elim'(a (1 b (-1 3 c)) 2 -3)))