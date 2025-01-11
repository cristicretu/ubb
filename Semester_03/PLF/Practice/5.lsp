(defun kLevel (lst lvl k)
  (cond
    ((null lst) nil) 
    ((atom lst) (cond
      ((= lvl k) 0)
      (T lst)
    ))
    (T (mapcar #'(lambda (x) (kLevel x (+ 1 lvl) k)) lst))
  )
)

(print (klevel '(a (1 (2 b)) (c (d))) 0 2))