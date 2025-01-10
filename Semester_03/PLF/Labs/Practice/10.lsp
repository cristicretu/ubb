(defun parsar (lst operation nambar)
  (cond
    ((null lst) (car nambar))
    ((numberp (car lst)) ; operand
      (cond
        ((car nambar) (parsar (cdr lst) (cdr operation) 
                             (cons (funcall (car operation) (car lst) (car nambar)) 
                                   (cdr nambar))))
        (t (parsar (cdr lst) operation (cons (car lst) nambar)))
      ))
    (t (parsar (cdr lst) (cons (car lst) operation) nambar)) ; operator
  ))

(print (parsar '(+ 1 3) nil nil))
(print (parsar '(+ * 2 4  3) nil nil))
(print (parsar '(+ * 2 4 - 5 * 2 2) nil nil))