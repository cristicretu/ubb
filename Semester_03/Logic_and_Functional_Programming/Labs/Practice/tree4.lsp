(defun maiconvert (tree)
  (cond 
    ((null tree) nil)
    ((and (null (cadr tree)) (null (caddr tree))) (cons (car tree) (list 0)))                                       ; leaf node
    ((or (null (cadr tree)) (null (caddr tree)))
     (cons (cons (car tree) (list 1)) (if (cadr tree) (maiconvert (cadr tree)) (maiconvert (caddr tree)))))  ; have 1 node
    (T (append (cons (car tree) (list 2)) (append (maiconvert (cadr tree)) (maiconvert (caddr tree)))))          ; have 2 nodes
  )
)

(print (maiconvert '(a (b) (c (d) (e)))))