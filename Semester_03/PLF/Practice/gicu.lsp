(defun g(l)
  (list (car l) (car l))
)

(setq q 'g)
(setq p q)



(print (funcall #'g '(1 2 3)))

; (print (funcall #'+ 1 2 3))
; (print (apply #'+ '(1 2 3)))