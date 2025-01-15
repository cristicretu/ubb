; (defun fct (f l) (cond
;     ((null l) nil) 
;     ((funcall f (car l)) (cons (funcall f (car l)) (fct f (cdr l))))
;     (T nil)
;   )
; )

(defun fct (f l)
  (cond
    ((null l) nil) 
    (T ((lambda (x) 
      (cond 
          (x (cons x (fct f (cdr l))))
          (T nil)
      )
    ) (funcall f (car l))))
  )
)


(print (fct #'(lambda (x) (= (mod x 2) 0)) '(2 2 4)))