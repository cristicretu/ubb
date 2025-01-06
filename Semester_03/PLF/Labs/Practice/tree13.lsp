(defun maidefs (tree acc el)
  (cond
    ((null tree) nil) ; never reached el
    ((equal (car tree) el) (reverse (append (list el) acc))) ; found the path
    (T (or
      (maidefs (cadr tree) (cons (car tree) acc) el)
      (maidefs (caddr tree) (cons (car tree) acc) el)
    ))
  ) 
)

(print (maidefs '(a (b) (c (d) (e))) nil 'b))