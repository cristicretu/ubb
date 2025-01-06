(defun postordar (tree acc)
  (cond
    ((and (null tree) (null acc)) nil)
    ((null tree) (list (car acc)))
    ((= (cadr tree) 0)
        (cons (car tree)
              (cond 
                  ((= (cadr acc) 1) (append (list (car acc)) (postordar (cddr tree) (cddr acc))))
                  (T (postordar (append (cons (car acc) (list 1)) (cddr tree)) (cddr acc)))
              )    
        )  
    ) 
    (T (postordar (cddr tree) (append (append (list (car tree)) (list (cadr tree))) acc)))
  )
)

(print (postordar '(a 2 b 0 c 2 d 0 e 0) nil))
(print (postordar '(a 2 b 2 c 0 d 0 e 2 f 0 g 0) nil))