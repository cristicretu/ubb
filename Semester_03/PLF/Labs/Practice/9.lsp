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

(defun maisamhelpar (a b cf)
  (cond
    ((and (null a) (null b))
      (cond 
        ((> cf 0) (if (< cf 10) 
                      (list cf)
                      (append (list (mod cf 10))
                             (maisamhelpar nil nil (floor cf 10)))))
        (T nil)
      ))
    ((null a) 
      (cons (mod (+ (car b) cf) 10) 
            (maisamhelpar a (cdr b) (floor (+ (car b) cf) 10))))
    ((null b) 
      (cons (mod (+ (car a) cf) 10) 
            (maisamhelpar (cdr a) b (floor (+ (car a) cf) 10))))
    (T 
      (cons (mod (+ (car b) (car a) cf) 10) 
            (maisamhelpar (cdr a) (cdr b) (floor (+ (car b) (car a) cf) 10))))
  ))

(defun maisam (a b)
  (reverse (maisamhelpar (reverse a) (reverse b) 0))
)

(print (maisam '(1230) '(1230)))