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
    ((null lst) (mairevars acc nil))
    ((atom (car lst))
      (cond
        ((= (countocc acc (car lst)) 0) (setifai (cdr lst) (cons (car lst) acc)))
        (T (setifai (cdr lst) acc))
      )
    )
    (T (setifai (cdr lst) (setifai (car lst) acc)))
  )
)

(defun mairevars (lst acc)
  (cond
      ((null lst) acc)
      ((atom (car lst)) (mairevars (cdr lst) (cons (car lst) acc)))
      (T (mairevars (cdr lst) (cons (mairevars (car lst) nil) acc)))
  )
)

(defun mailength (lst)
  (cond
    ((null lst) 0)
    (T (+ 1 (mailength (cdr lst))))
  )
)

(defun getfarzt (lst acc fleg)
  (cond
    ((null lst) (mairevars acc nil))
    ((atom (car lst))
      (cond
        ((and (null fleg) (= (mod (mailength lst) 2) 1)) (getfarzt (cdr lst) (cons (car lst) acc) t))  ; we are at the first element from the list, and lst is has odd length
        (T (getfarzt (cdr lst) acc t)) ; we're already past the first element, ignore
      )
    )
    (T (getfarzt (cdr lst) (append (getfarzt (car lst) nil nil) acc) fleg))
  )
)
(print (countocc '(1 (2 3 (3 4 3 (5 3 5 (3))))) 5))
(print (setifai '(1 (2 (1 3 (2 4) 3) 1) (1 4)) nil))
(print (mairevars '(1 (2) (3 (4) 5)) nil))
(print (mairevars '(1 (2)) nil))
(print (getfarzt '(1 2 (3 (4 5) (6 7)) 8 (9 10 11)) nil nil))
(print (getfarzt '(1 2 (4 5 6)) nil nil))