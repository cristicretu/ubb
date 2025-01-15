(defun removeDivs (lst)
  (cond
    ((null lst) nil)
    ((atom lst) (cond
      ((and (numberp lst) (= (mod lst 3) 0)) nil) 
      (T lst)
    ))
    (T (mapcar #'removeDivs lst))
  )
)

(print (removeDivs '(1 (2 A (3 A)) (6))))