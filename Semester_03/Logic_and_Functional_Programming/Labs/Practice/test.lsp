(defun margi (l1 l2)
  (cond
    ((and (null l1) (null l2)) nil)
    ((null l1) l2)
    ((null l2) l1)
    (T (cond
        ((< (car l1) (car l2)) (cons (car l1) (margi (cdr l1) l2)))
        ((> (car l1) (car l2)) (cons (car l2) (margi l1 (cdr l2))))
        (T (cons (car l1) (margi (cdr l1) (cdr l2))))
    ))
  )
)

(defun countocc (lst el)
  (cond
      ((null lst) 0)
      ((= (car lst) el) (+ (countocc (cdr lst) el) 1))
      (T (countocc (cdr lst) el))
  )
)

(defun setifai (lst acc)
  (cond
    ((null lst) (mairevars acc nil))
    ((= (countocc acc (car lst)) 0) (setifai (cdr lst) (cons (car lst) acc)))
    (T (setifai (cdr lst) acc))
  )
)

(defun mairevars (lst acc)
  (cond
      ((null lst) acc)
      (T (mairevars (cdr lst) (cons (car lst) acc)))
  )
)

(defun mein (l1 l2)
  (setifai (margi l1 l2) nil)
)

; (print (mein '(1 2 3 4 5) '(8 9)))
; (print (mein '(1 2 3 4 5) '(1 2 3 4 5)))
; (print (mein '(1 2) '(1 2 3 4 5)))
; (print (mein '(1 2) '(1 1 1 1 1 1)))
; (print (mein '(1 1 1 1 1 1) '(1 2)))
(print (mein '(1 3 5 7) '(2 4 6 8)))
(print (mein '(1 2 3 4 5) '(6 7 8 9 10)))
(print (mein '(1 3 5 7) '(2 4 6 8)))
(print (mein '(1 2 4 5 7) '(1 3 4 6 7)))

