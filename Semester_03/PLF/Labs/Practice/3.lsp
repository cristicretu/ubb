(defun messi (lst a cnt)
  (cond
    ((null lst) 
      (cond
        ((= cnt 2) (list a))
        (T nil)
      )
    )
    ((= cnt 2) (cons a (cons (car lst) (messi (cdr lst) a 1))))
    (T (cons (car lst) (messi (cdr lst) a (+ cnt 1))))
  )
)

(print (messi '(1 2 3 4 5 6 7 8 9 10) 99 0))  

(defun ronaldo (l acc)
  (cond
    ((null l) acc)
    ((atom (car l)) (ronaldo (cdr l) (cons (car l) acc)))
    (T (ronaldo (cdr l) (append (ronaldo (car l) nil) acc)))
  )
)

(print (ronaldo '(((a b) c) (d e)) nil))

(defun my-gcd (a b)
  (cond
    ((= b 0) a)
    (T (my-gcd b (mod a b)))
  )
)

(defun gcd-list (l)
  (cond
    ((null l) 0)
    (T (my-gcd (car l) (gcd-list (cdr l))))
  )
)

(print (gcd-list '(12 18 24)))

