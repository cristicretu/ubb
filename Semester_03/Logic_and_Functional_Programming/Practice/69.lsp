(defun myexists (lst el)
  (cond
    ((null lst) nil)
    ((atom lst) (equal lst el))
    (t (eval (cons 'or (mapcar #'(lambda (x) (myexists x el)) lst))))
  )
)

(print (myexists '(a (b (g)) (c (d (e)) (f))) 'h))