; Mathematical models coming soon

;Return the list of nodes on the k-th level of a tree of type (1).

(defun f (tree level aux)
  (cond
    ((null tree) nil)
    ((> (cadr tree) 0) (f (cddr tree) (- level 1) (cons (car tree) (cons (- (cadr tree) 1) aux))))
    ((= level 1) (cons (car tree)
      (cond
        ((null aux) (f (cddr tree) level aux))
        (T (f (cons (car aux) (cons (cadr aux) (cddr tree))) (+ level 1) (cddr aux)))
      ))
    )
    ((null aux) (f (cddr tree) level aux))
    (T (f (cons (car aux) (cons (cadr aux) (cddr tree))) (+ level 1) (cddr aux)))
 )
)

(print (f '(a 2 b 1 c 0 d 0) 1 nil))

; Define a function which merges, without keeping duplicated elements, 
; two linear lists that are sorted in ascending order.
; Each list contains unique elements.
(defun f1 (l1 l2)
  (cond
    ((null l1) l2)
    ((null l2) l1)
    ((< (car l1) (car l2)) (cons (car l1) (f1 (cdr l1) l2)))
    ((> (car l1) (car l2)) (cons (car l2) (f1 l1 (cdr l2))))
    (T (cons (car l1) (f1 (cdr l1) (cdr l2))))
  )
)

(print (f1 '(1 4 5 6 7 9) '(2 3 5 7 10)))

; Define a function to remove all the occurrences of an 
; element from a nonlinear list.

(defun f2 (l e)
  (cond
    ((null l) nil)
    ((and (atom (car l)) (equal (car l) e)) (f2 (cdr l) e))
    ((listp (car l)) (cons (f2 (car l) e) (f2 (cdr l) e)))
    (T (cons (car l) (f2 (cdr l) e)))
  )
)
(print (f2 '((1 2 3 4) 1 2 3 4 (5 6 (2 3 (2) 2) 7 2)) 2))

; Build a list with the positions of the minimum number from a linear list.
; The list may contain non-numerical atoms.
; Bonus requirement: Try to solve the problem using at most two functions (excluding any wrappers)

(defun getMin (l min)
  (cond
    ((null l) min)
    ((and (numberp (car l)) (or (null min) (> min (car l)))) (getMin (cdr l) (car l)))
    (T (getMin (cdr l) min))
  )
)

(defun getMinW (l)
  (getMin l nil)
)

(defun f3 (l min pos)
  (cond
    ((null l) nil)
    ((equal (car l) min) (cons pos (f3 (cdr l) min (+ 1 pos))))
    (T (f3 (cdr l) min (+ 1 pos)))
  )
)

(defun f3W (l)
  (f3 l (getMinW l) 1)
)

(print (f3W '(a 8 9 z 3 7 7 3 q)))

; Add an element to a linear list every nth position.

(defun f4 (l e n pos)
  (cond
    ((null l) nil)
    ((= (mod pos n) 0) (cons (car l) (cons e (f4 (cdr l) e n (+ 1 pos)))))
    (T (cons (car l) (f4 (cdr l) e n (+ 1 pos))))
  )
)











