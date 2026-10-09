;; First define G
(defun g (l) (list (car l) (car l)))

;; Test 1 - original version that should error
(setq q 'g)
(setq p 'q)
(format t "~%Testing first version:")
(format t "~%P evaluates to: ~A" (eval p))
(format t "~%Trying to FUNCALL:")
(print (funcall (eval q) '(a b c)))

; ;; Test 2 - corrected version with double eval
; (format t "~%~%Testing second version:")
; (format t "~%P evaluates to: ~A" (eval p))
; (format t "~%EVAL P twice evaluates to: ~A" (eval (eval p)))
; (format t "~%Trying with double EVAL:")
; (funcall (eval (eval p)) '(a b c))

; ;; Test 3 - alternative using function
; (format t "~%~%Testing function version:")
; (format t "~%Using function cell:")
; (funcall #'g '(a b c))