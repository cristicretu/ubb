; merge-lists two lists without keeping duplicates, two linear lists that are sorted

; merge-lists(a, b) = { a1, if a1 < b1
;               { b1, if a1 > b1
;               { merge-lists(a2, b), if a1 = b1

(defun merge-lists (a b)
  (cond ((null a) b)
        ((null b) a)
        ((< (car a) (car b)) (cons (car a) (merge-lists (cdr a) b)))
        ((> (car a) (car b)) (cons (car b) (merge-lists a (cdr b))))
        (t (cons (car a) (merge-lists (cdr a) (cdr b))))))

(print (merge-lists '(1 2 3 4) '(3 4 5 6)))

; remove all occurences of an element from a nonlinear list

(defun remove-element (element list)
  (cond ((null list) nil)
        ((listp (car list)) (cons (remove-element element (car list)) (remove-element element (cdr list))))
        ((equal (car list) element) (remove-element element (cdr list)))
        (t (cons (car list) (remove-element element (cdr list))))))

(print (remove-element 3 '(1 2 3 (4 3 5) 6 3)))

; build a list with the positions of the minumum number from a linear list

(defun min-positions (list currMin currPos positions)
  (cond ((null list) (reverse positions))  
        ((numberp (car list))
         (cond ((or (null currMin) (< (car list) currMin))
                (min-positions (cdr list) (car list) (+ currPos 1) (list currPos)))
               ((= (car list) currMin)
                (min-positions (cdr list) currMin (+ currPos 1) (cons currPos positions)))
               (t (min-positions (cdr list) currMin (+ currPos 1) positions))))
        (t (min-positions (cdr list) currMin (+ currPos 1) positions))))

; Test the function
(print (min-positions '( a 1 2 3 1 2 3 1 2 3 ) nil 0 nil))


; add an element to a liner list every nth position until the end of the list
(defun addN(l pos e n)
  (cond ((and (null l) (= pos n)) (cons e nil))
        ((null l) nil)
        ((= pos n) (cons e (cons (car l) (addN (cdr l) 1 e n))))
        (t (cons (car l) (addN (cdr l) (+ pos 1) e n)))))

(print (addN '(1 2 3 4 5 6 7 8 9 10) 0 0 1))


; get elements from level k from a tree

(defun kLevelElements(tree current level aux)
  (cond ((null tree) nil)
        ((= (cadr tree) 0)
          (cond 
            (
              (= level current) (cons (car tree) (kLevelElements (cons (car aux) (cons (cadr aux) (cddr tree)))))
            ) 
          )
          
  )
)


