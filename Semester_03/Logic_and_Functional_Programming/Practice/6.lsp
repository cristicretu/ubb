(defun solusan (l lvl)
  (cond
    ((null l) nil)
    ((atom l) (cond
        ((numberp l) (* l lvl)) 
        (T l)
    ))
    (T (mapcar #'(lambda (x) (solusan x (+ 1 lvl))) l))
  )
)

(print (solusan '(1 (2) (3 (4 (5)))) 0))