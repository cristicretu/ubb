(defun margi (a b)
  (cond
    ((and (null a) (null b)) nil)
    ((null a) b)
    ((null b) a)
    (T 
      (cond
        ((< (car a) (car b)) (cons (car a) (margi (cdr a) b)))
        (T (cons (car b) (margi a (cdr b))))
      )
    ) 
  )
)

(print (margi '(1 3 5 7 9) '(0 2 4 6 8)))