(defun priordar (tree)
  (cond
    ((null tree) nil)
    (T (cons (car tree) (append (priordar (cadr tree)) (priordar (caddr tree)))))
  )
)

(print (priordar '(a (b) (c (d) (e)))))