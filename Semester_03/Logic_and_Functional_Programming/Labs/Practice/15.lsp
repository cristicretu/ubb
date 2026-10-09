(defun get-sublists (lst)
  (cond
    ((null lst) nil)
    ((listp (car lst))
     (append 
       (cons (car lst) (get-sublists (car lst)))
       (get-sublists (cdr lst))))
    (t (get-sublists (cdr lst)))))

(print (get-sublists '((1 2 3) ((4 5) 6))))

(defun member-p (elem lst)
  (cond
    ((null lst) nil)                          ; If list is empty, element not found
    ((equal (car lst) elem) t)                ; Found the element
    (t (member-p elem (cdr lst)))))           ; Keep searching

(defun set-equal (set1 set2)
  (cond
    ((and (null set1) (null set2)) t)         ; Both empty = equal
    ((or (null set1) (null set2)) nil)        ; If only one is empty = not equal
    ((not (member-p (car set1) set2)) nil)    ; Current element not in set2
    (t (set-equal (cdr set1)                  ; Remove current element from both sets
                  (remove (car set1) set2 :count 1)))))

;; Test cases
(print (set-equal '(1 2 3) '(3 1 2)))        ; Should return T
(print (set-equal '(1 2) '(1 2 3)))          ; Should return NIL
(print (set-equal '(1 2 3) '(1 2 4)))        ; Should return NIL