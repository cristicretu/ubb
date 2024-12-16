(defun getInOrder(tree aux)
  (cond
    ((and (null tree) (null aux))
     nil)
    ((= (cadr tree) 0)
     (cond 
       ((null aux)
        (cons (car tree) (getInOrder (cddr tree) aux)))
       ((= (cadr aux) 1)
        (cons (car tree) 
              (getInOrder (cons (car aux) (cons 0 (cddr tree)))
                         (cddr aux))))
       (T
        (cons (car tree)
              (cons (car aux) 
                    (getInOrder (cddr tree) (cddr aux)))))))
    (T
     (getInOrder (cddr tree) 
                 (cons (car tree) (cons (cadr tree) aux))))))

(defun tests()
  (assert (equal (getInOrder '(A 2 B 0 C 2 D 0 E 0) nil) '(B A D C E)))
  (format t "~%test 1 merge")
  (assert (equal (getInOrder '(A 2 B 1 C 0 D 2 E 0 F 0) nil) '(C B A E D F)))
  (format t "~%t2 merge!")
)

(tests)