(defun sablist (lst fleg)
  (cond
    ((null lst) nil)
    ((atom lst) nil) 
    (T (append
    (cond
      ((null fleg) (list lst))
      (T nil)
    )
     (cond
        ((atom (car lst)) (sablist (cdr lst) t))
        (T (cons (sablist (car lst) nil) (sablist (cdr lst) t)))
    )))
  )
)

(print (sablist '(1 2 (3 (4 5) (6 7)) 8 (9 10)) nil))

(defun membar (lst e)
  (cond
    ((null lst) nil)
    ((equal (car lst) e) t)
    ((atom (car lst)) (membar (cdr lst) e))
    (T (cond
      ((null (membar (car lst) e)) (membar (cdr lst) e))  
      (T t) 
    ))
  )
)

(print (membar '(1 2 (3 (4 5) (6 7)) 8 (9 10)) 11))
