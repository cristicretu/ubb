(defun priordar (tree)
  (cond
    ((null tree) nil)
    ((and (null (cadr tree)) (null (caddr tree))) (list (car tree))) ; leaf
    (T (append (append (priordar (cadr tree)) (priordar (caddr tree))) (list (car tree))))
  )
)

(print (priordar '(a (b) (c (d) (e)))))
(print (priordar '(a (b (c) (d)) (e (f) (g)))))
