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
    ((equal (car lst) el) (reverse (cons el (removeDigits acc))))
    ((= (cadr lst) 0)
     (cond
      ((= (cadr acc) 1) (maidfs1 (cddr lst) (cddr acc) el)) ; stack top has 1 child, me :)
      (T (maidfs1 (cddr lst) (append (cons (car acc) (list 1)) (cddr acc)) el))) ; stack top has 2 children
     )
    (t
     (maidfs1 (cddr lst) (append (cons (car lst) (list (cadr lst))) acc) el))
  ))

(defun removeDigits (lst)
  (cond 
    ((null lst) nil)
    ((numberp (car lst)) (removeDigits (cdr lst)))
    (T (cons (car lst)  (removeDigits (cdr lst))))
  )
)

(print (maidfs1 '(a 2 b 1 f 0 c 2 d 0 e 0) nil 'E))
