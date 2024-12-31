(defun succc (lst acc cf)
  (cond
    ((null lst) 
      (cond
        ((null acc) nil)
        (T (append 
            (succc nil (cdr acc) (floor (+ (car acc) cf) 10))
            (list (mod (+ (car acc) cf) 10))
        ))
      )
    ) 
    (T (succc (cdr lst) (cons (car lst) acc) cf))
  )
)

(print (succc '(1 9 3 5 9 9) nil 1))