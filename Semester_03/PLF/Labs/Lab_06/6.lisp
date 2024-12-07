(defun getInOrder(tree aux)
  (cond
    ((and (null tree) (null aux)) nil) ; tree si stack goale
    ((null tree) 
     (getInOrder (cons (car aux) 
                      (cons (- (cadr aux) 1) 
                           (cddr aux)))
                 (cddr aux))) ; adaugam inapoi cu -1, pentru a nu-l adauga la rezultat again cand intram pe urmatorul branch
    ((= (cadr tree) 0)
     (if aux 
         (cons (car tree)
              (cons (car aux) ; adaug top stack
                    (getInOrder (cddr tree) (cddr aux))))
         (cons (car tree) (getInOrder (cddr tree) aux)))) ; altfel continui
    (T ; adaug in stiva, scad count
     (getInOrder (cddr tree) 
                 (cons (car tree) (cons (cadr tree) aux))))))

(defun tests()
  (assert (equal (getInOrder '(A 2 B 0 C 2 D 0 E 0) nil) '(B A D C E)))
  (assert (equal (getInOrder '(A 2 B 1 C 0 D 2 E 0 F 0) nil) '(C B A E D F)))
)

(tests)