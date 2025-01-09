(defun mailen (lst)
  (cond 
    ((null lst) 0)
    (T (+ (mailen (cdr lst)) 1))
  )
)

(defun mein (lst) 
  (cond
    ((null lst) nil)
    ((atom (car lst)) (cons (car lst) (mein (cdr lst))))
    (T (cond
         ((and (every #'atom (car lst)) 
               (= (mod (mailen (car lst)) 2) 0))
          (mein (cdr lst))) 
         (T (cons (mein (car lst))
                 (mein (cdr lst))))
       ))
  )
)

(print (mailen '()))

(print (mein '((2 3 4) (6 (7 8) ((7 9) 8)) (6 8) 9)))
(print (mein '(6 (7 8) ((7 9) 8))))
