(defun repleis (lst lvl e e1)
  (cond
    ((null lst) nil) 
    ((atom lst) (cond
        ((and (equal lst e) (= (mod lvl 2) 1)) e1) 
        (T lst)
    ))
    (T (mapcar #'(lambda (x) (repleis x (+ 1 lvl) e e1)) lst))
  )
)

(print (repleis '(1 d (2 d (d))) 0 'd 'f))
