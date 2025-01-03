(defun mergisuort (lst)
  (cond
    ((null lst) nil)
    ((null (cdr lst)) (list (car lst)))
    (t
      (margi (mergisuort (car (split_list lst))) 
            (mergisuort (cadr (split_list lst))))
    )
  )
)

(defun split_list (lst)
  (cond
    ((null lst) (list nil nil))
    ((null (cdr lst)) (list (list (car lst)) nil))
    (T 
      (let ((rest (split_list (cddr lst))))
        (list
          (cons (car lst) (car rest))
          (cons (cadr lst) (cadr rest))
        )
      )
      ;  (list
      ;   (cons (car lst) (car (split_list (cddr lst))))
      ;   (cons (cadr lst) (cadr (split_list (cddr lst))))
      ; )
    )
  )
)

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

(print (mergisuort '(7 5 4 3 9 0 1 2 6 8)))