(defun split_lists (lst)
  (cond
    ((null lst) (list nil nil))  
    ((null (cadr lst)) (list (list (car lst)) nil))
    (T
      (list
        (cons (car lst) (car (split_lists (cddr lst))))
        (cons (cadr lst) (cadr (split_lists (cddr lst))))
      )
    )
  )
)

(defun mergisuort (lst)
  (cond
    ((null lst) nil) 
    ((null (cdr lst)) (list (car lst)))
    (t
      (margi
        (mergisuort (car (split_lists lst)))
        (mergisuort (cadr (split_lists lst)))
      )
    )
  )
)

(defun margi (a b)
  (cond
    ((and (null a) (null b)) nil) 
    ((null a) b)
    ((null b) a)
    (T (cond
        ((< (car a) (car b)) (cons (car a) (margi (cdr a) b)))
        (T (cons (car b) (margi a (cdr b))))
    )) 
  )
)


(print (mergisuort '(7 5 4 3 9 0 1 2 6 8)))