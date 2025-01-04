(defun mainorder (tree acc)
  (cond
    ((and (null tree) (null acc)) nil)
    ((= (cadr tree) 0) (cons (car tree) (cond
      ((null acc) (mainorder (cddr tree) acc))
      ((= (cadr acc) 1) (mainorder (cons (car acc) (cons 0 (cddr tree))) (cddr acc))) 
      (T (cons (car acc) (mainorder (cddr tree) (cddr acc))))
    )))
    (T (mainorder (cddr tree) (cons (car tree) (cons (cadr tree) acc))))
  )
)

(print (mainorder '(a 2 b 0 c 2 d 0 e 0) nil))