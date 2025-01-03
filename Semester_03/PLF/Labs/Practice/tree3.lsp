(defun leval (tree curr mx)
  (cond
    ((null tree) mx)
    ((= (cadr tree) 0) (leval (cddr tree)  curr (max mx (+ curr 1))))
    (t
      (leval (cddr tree) (+ curr 1) mx)
    )
  )
)

(print (leval '(a 2 b 0 c 2 d 0 e 1 f 0) 0 0))