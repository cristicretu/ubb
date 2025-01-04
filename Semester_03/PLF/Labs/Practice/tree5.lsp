(defun maileval (tree acc el curr)
  (cond
    ((null tree) nil)
    ((equal (car tree) el) curr)
    ((= (cadr tree) 0) (cond
        ((null acc) (maileval (cddr tree) acc el curr))
        ((= (cadr acc) 1) (maileval (cddr tree) (cddr acc) el (- curr 1)))
        (T (maileval (cddr tree) (cddr acc) el curr))
    ))
    (t (maileval (cddr tree) (cons (car tree) (cons (cadr tree) acc)) el (+ curr 1)))
  )
)

(print (maileval '(a 2 b 1 f 0 c 2 d 0 e 1 z 0) nil 'z 0))