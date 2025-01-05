(defun maiconvert (tree)
  (cond
    ((null tree) nil)
    ((and (null (cadr tree)) (null (caddr tree))) (cons (car tree) (list 0))) ; leaf
    ((null (caddr tree)) (append (car tree) (append (list 1) (maiconvert (cadr tree))))) ; 1 child 
    (T (append
        (cons (car tree) (list 2))
        (append
             (maiconvert (cadr tree))
             (maiconvert (caddr tree))
        )
       )
    ) ; 2 nodes
  )
)

(print (maiconvert '(a (b) (c (d) (e)))))