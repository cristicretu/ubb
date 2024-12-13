; flatten a list using a map function
; (1 (2 (3) 2) 1) -> (1 2 3 2 1)

(defun flatten (lst)
  (cond
    ((null lst) nil)
    ((atom lst) (list lst))
    (T (mapcan #'flatten lst))
  )
)

(print (flatten '(1 (2 (3) 2) 1)))

; triple all numeric atoms from a list

(defun triple (lst)
  (cond
    ((null lst) nil)
    ((atom lst)
      (cond
        ((numberp lst) (* lst 3))
        (T lst)
      )
    )
    (T (mapcar #'triple lst))
  )
)


(print (triple '(1 (a (3) 2) b)))


; count all the atoms from a list on the kth level. consider that the superficial level is level 0

(defun countLevel (lst k)
  (cond
    ((null lst) 0)
    ((and (atom lst) (= k 0)) 1)
    ((atom lst) 0)
    (T (apply #'+ (mapcar #'(lambda (x) (countLevel x (- k 1))) lst)))
  )
)

(print (countLevel '(1 (a (3) 2) b) 1))

; print all the nodes on a given level k from a n-ary tree (type 2)
; consider that the root is at level 0

(defun printTreeLevel (tree k)
  (cond 
    ((null tree) nil) 
    ((< k 0) nil)
    ((= k 0) (list (car tree)))
    (T (mapcan #'(lambda (x) (printTreeLevel x (- k 1))) (cdr tree)))
  )
)

(print (printTreeLevel '(a (b (e f)) (c) (d (g (h) (i) (j) (k)))) 2))
