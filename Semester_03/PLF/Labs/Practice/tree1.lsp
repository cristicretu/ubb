(defun maidfs2 (lst acc el)
  (cond
    ((null lst) nil)
    ((equal (car lst) el) (reverse (cons el acc)))
    (t
      (or 
        (and (cadr lst) (maidfs2 (cadr lst) (cons (car lst) acc) el))
        (and (caddr lst) (maidfs2 (caddr lst) (cons (car lst) acc) el))
      ))    
  )
)

(print (maidfs2 '(A (B) (C (D) (E))) nil 'E))

(defun maidfs1 (lst acc el)
  (cond
    ((null lst) nil)
    ((equal (car lst) el) (reverse (cons el acc)))
    ((= (cadr lst) 0)
     (maidfs1 (cddr lst) acc el))
    (t
     (maidfs1 (cddr lst) (cons (car lst) acc) el))
  ))

(print (maidfs1 '(a 2 b 0 bc 2 d 0 e 0) nil 'E))



; (car) - nod
; cadr - left
; caddr - right