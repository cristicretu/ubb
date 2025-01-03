(defun leval (tree aux curr mx)
  (cond
    ((null tree) mx)
    ((= (cadr tree) 0) (leval (cddr tree) aux curr (max mx (+ curr 1))))
    (t
      (leval (cddr tree) (cons (car tree) aux) (+ curr 1) mx)
    )
  )
)

(print (leval '(a 2 b 0 c 2 d 0 e 1 f 0) nil 0 0))