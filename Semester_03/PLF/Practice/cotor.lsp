(defun messi (lst lvl el)
  (cond
    ((null lst) nil)
    ((atom lst) (cond
      ((= (mod lvl 2) 0) nil)
      (T lst)))
    (T (mapcar #'(lambda (x) (messi x (+ 1 lvl) el)) lst))))

(print (messi '(a (b (g)) (c (d (e)) (f))) 0 'h))



(a (b) (c (d) (e)))




          a
      b       c
           d     e