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

(defun countocc (lst el)
  (cond
      ((null lst) 0)
      ((atom (car lst)) 
          (cond
            ((= (car lst) el) (+ (countocc (cdr lst) el) 1))
            (T (countocc (cdr lst) el))
          )
      )
      (T (+  (countocc (cdr lst) el) (countocc (car lst) el)))
  )
)

(defun setifai (lst acc)
  (cond
    ((null lst) (reverse acc))
    ((atom (car lst))
      (cond
        ((= (countocc acc (car lst)) 0) (setifai (cdr lst) (cons (car lst) acc)))
        (T (setifai (cdr lst) acc))
      )
    )
    (T (setifai (cdr lst) (setifai (car lst) acc)))
  )
)

(print (countocc '(1 (2 3 (3 4 3 (5 3 5 (3))))) 5))
(print (setifai '(1 (2 (1 3 (2 4) 3) 1) (1 4)) nil))