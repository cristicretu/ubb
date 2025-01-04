(defun mainorder (tree)
  (cond 
    ((null tree) nil)  
    ((and (null (cadr tree)) (null (caddr tree))) (list (car tree))) ; leaf
    ((or (null (cadr tree)) (null (caddr tree))) 
      (if (null (cadr tree))
          (mainorder (caddr tree))
          (mainorder (cadr tree)))) ; 1 child
    (T (append (mainorder (cadr tree)) (cons (car tree) (mainorder (caddr tree)))))
  )
)

(print (mainorder '(A (B) (C (D) (E)))))