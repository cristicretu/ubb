(defun drum (lst el)
  (cond
    ((null lst) nil)
    ((atom lst) (cond
      ((equal lst el) el)
      (T nil)))
    (T ((lambda (x) (cond
        ((equal x el) (list (car lst)))
        (x (cons (car lst) x))
        (T nil)))
      (mapcan #'(lambda (y) (drum y el)) lst)))
  )
)

(print (drum '(a (b (g)) (c (d (e)) (f))) 'e))