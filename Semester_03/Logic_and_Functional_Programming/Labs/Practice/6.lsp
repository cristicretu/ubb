(defun get-last (l)
  (cond
    ((null l) nil)
    ((null (cdr l)) 
      (if (atom (car l))
          (car l)
          (get-last (car l))))
    (T (get-last (cdr l)))
  )
)

(defun replace-sublists (l)
  (cond
    ((null l) nil)
    ((atom (car l)) (cons (car l) (replace-sublists (cdr l))))
    (T (cons (get-last (car l)) (replace-sublists (cdr l))))
  )
)

(print (replace-sublists '(a (b c) (d (e (f))))))