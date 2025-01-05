(defun leval (tree el lvl)
  (cond
    ((null tree) nil)
    ((equal (car tree) el) lvl)
    (T (or (leval (cadr tree) el (+ lvl 1)) 
           (leval (caddr tree) el (+ lvl 1))))
  )
)

(print (leval '(a (b) (c (d) (e))) 'd 0 ))