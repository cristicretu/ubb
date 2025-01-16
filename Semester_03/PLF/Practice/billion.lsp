(defun haight (lst lvl)
  (cond
    ((null lst) nil)
    ((atom lst) lvl)
    (T (apply #'max (mapcar #'(
      lambda (y) (haight y (+ 1 lvl)))
      lst)))
  )
)

(defun sabtriHaight (lst el)
  (cond
    ((null lst) -1)
    ((equal (car lst) el) (haight lst -1))
    ((not (listp lst)) -1)
    (T (max (sabtriHaight (cadr lst) el)
            (sabtriHaight (caddr lst) el)))
  )
)

(print (sabtrihaight '(a (b (g)) (c (d (e)) (f))) 'e))