(defun invert-simple (lst)
  (cond
    ((null lst) nil)
    ((atom (car lst))
     (append (reverse (collect-atoms lst))
             (invert-simple (nthcdr (length (collect-atoms lst)) lst))))
    (T (cons (invert-simple (car lst))
            (invert-simple (cdr lst))))))

(defun collect-atoms (lst)
  (cond
    ((or (null lst) (not (atom (car lst)))) nil)
    (T (cons (car lst) (collect-atoms (cdr lst))))))

(print (invert-simple '(a b c (d (e f) g h i))))
