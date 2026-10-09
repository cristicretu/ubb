(defun verific (lst acc)
  (cond
    ((null lst) (oddp acc))
    ((atom (car lst)) 
     (cond 
       ((numberp (car lst)) (verific (cdr lst) acc))
       (T (verific (cdr lst) (+ 1 acc)))))
    (T (verific (cdr lst) acc))
  ))

(defun rezolv (lst lvl)
  (cond
    ((atom lst) 0)
    (T (+ 
        (if (and (evenp lvl) (verific lst 0)) 1 0)
        (apply #'+ (mapcar #'(lambda (x) (rezolv x (+ lvl 1))) lst))))
  ))

(print (rezolv '(a (b 2) (1 c 4) (1 (6 f)) (((g) 4) 6)) 1))