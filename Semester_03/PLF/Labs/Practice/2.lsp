; Problem a)
(defun dot_product (a b)
  (cond
    ((or (null a) (null b)) 0) 
    (T (+ (* (car a) (car b)) (dot_product (cdr a) (cdr b))))
  )
)

(print(dot_product '(1 2 3) '(4 5 6)))

; Problem b)
(defun dpth (lst acc)
  (cond
    ((null lst) acc)
    ((atom (car lst)) (dpth (cdr lst) acc))
    (T (max (dpth (cdr lst) acc) (dpth (car lst) (+ acc 1))))
  )
)

(print (dpth '(1 (2 (3 4) 5) 6) 1))

; Problem c)
(defun merge_sort (lst)
  (cond
    ((null lst) nil)
    ((null (cdr lst)) lst)  ; single element case
    (T (let* ((split (split_list lst))
              (left (merge_sort (first split)))
              (right (merge_sort (second split))))
         (merge_lists left right)))))

(defun split_list (lst)
  (cond
    ((null lst) (list nil nil))
    ((null (cdr lst)) (list lst nil))
    (T (let* ((left (list (car lst)))
              (right (list (cadr lst)))
              (split (split_list (cddr lst))))
         (list (append left (first split))
               (append right (second split)))))))

(defun merge_lists (lst1 lst2)
  (cond
    ((null lst1) lst2)
    ((null lst2) lst1)
    ((<= (car lst1) (car lst2))
     (cons (car lst1) (merge_lists (cdr lst1) lst2)))
    (T (cons (car lst2) (merge_lists lst1 (cdr lst2))))))

(print (merge_sort '(6 5 4 3 2 1)))



(defun count_elements (a b)
  (cond
    ((null a) 0)
    ((= (car a) b) (+ 1 (count_elements (cdr a) b)))
    (T (count_elements (cdr a) b))
  )
)

(defun count_list (a b)
  (cond
    ((or (null a) (null b)) nil)
    ((> (count_elements a (car b)) 0) (cons (car b) (count_list a (cdr b))))
    (T (count_list a (cdr b)))
  )
)

(print (count_list '(1 2 3 4 5 6 7 8 9 10) '(5 10 11 12 13 14 15 16 17 18 19 20)))