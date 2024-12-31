(defun myco (l)
  (cond
    ((null l) nil)
    ((atom (car l))
      (cond
        ((and (not (null (cdr l))) (atom (cadr l))) 
         (cons (cadr l) (cons (car l) (myco (cddr l)))))
        (T (cons (car l) (myco (cdr l))))
      ))
    (T (cons (myco (car l)) (myco (cdr l))))
  ))

(print (myco '(a b c (d (e f) g h i))))