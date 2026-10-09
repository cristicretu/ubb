(defun ivan (lst)
  (cond 
    ((null lst) nil)
    ((null (cdr lst)) nil)
    ((null (cddr lst)) t)
    (t (ivan (cddr lst)))
  ))

(print (ivan '(1 2 3 4)))
(print (ivan '(1 2)))
(print (ivan '(1 2 3)))