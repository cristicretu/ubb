(defun margi (a b)
  (cond
    ((and (null a) (null b)) nil)
    ((null a) b)
    ((null b) a)
    (T 
      (cond
        ((< (car a) (car b)) (cons (car a) (margi (cdr a) b)))
        (T (cons (car b) (margi a (cdr b))))
      )
    ) 
  )
)

(print (margi '(1 3 5 7 9) '(0 2 4 6 8)))

(defun maisamhelpar (a b cf)
  (cond
    ((and (null a) (null b))
      (cond 
        ((> cf 0) (if (< cf 10) 
                      (list cf)
                      (append (list (mod cf 10))
                             (maisamhelpar nil nil (floor cf 10)))))
        (T nil)
      ))
    ((null a) 
      (cons (mod (+ (car b) cf) 10) 
            (maisamhelpar a (cdr b) (floor (+ (car b) cf) 10))))
    ((null b) 
      (cons (mod (+ (car a) cf) 10) 
            (maisamhelpar (cdr a) b (floor (+ (car a) cf) 10))))
    (T 
      (cons (mod (+ (car b) (car a) cf) 10) 
            (maisamhelpar (cdr a) (cdr b) (floor (+ (car b) (car a) cf) 10))))
  ))

(defun maisam (a b)
  (reverse (maisamhelpar (reverse a) (reverse b) 0))
)

(print (maisam '(1230) '(1230)))

(defun compute (lst)
  (compute-helper lst nil))

(defun compute-helper (expr stack)
  (print (list 'expr expr 'stack stack))  ; Debug print
  (cond 
    ((null expr) 
     (if (and (>= (length stack) 3)
              (numberp (car stack))
              (numberp (caddr stack))
              (or (equal (cadr stack) '+) 
                  (equal (cadr stack) '*) 
                  (equal (cadr stack) '-) 
                  (equal (cadr stack) '/')))
         (let ((num2 (car stack))
               (op (cadr stack))
               (num1 (caddr stack)))
           (compute-helper 
             nil
             (cons (cond ((equal op '+) (+ num1 num2))
                        ((equal op '*) (* num1 num2))
                        ((equal op '-) (- num1 num2))
                        ((equal op '/) (/ num1 num2)))
                  (cdddr stack))))
         (car stack)))
    
    ((or (equal (car expr) '+) 
         (equal (car expr) '*) 
         (equal (car expr) '-) 
         (equal (car expr) '/))
     (compute-helper (cdr expr) (cons (car expr) stack)))
    
    ((and (numberp (car expr)) 
          (>= (length stack) 2)
          (numberp (car stack))
          (or (equal (cadr stack) '+) 
              (equal (cadr stack) '*) 
              (equal (cadr stack) '-) 
              (equal (cadr stack) '/)))
     (let ((num2 (car expr))
           (num1 (car stack))
           (op (cadr stack)))
       (print (list 'computing op num1 num2))  ; Debug print
       (compute-helper 
         (cdr expr)
         (cons (cond ((equal op '+) (+ num1 num2))
                     ((equal op '*) (* num1 num2))
                     ((equal op '-) (- num1 num2))
                     ((equal op '/) (/ num1 num2)))
               (cddr stack)))))
    
    (T (compute-helper (cdr expr) (cons (car expr) stack)))))

; Test cases
(print (compute '(+ 1 3)))          ; ==> 4
(print (compute '(+ * 2 4 3)))      ; ==> 11
(print (compute '(+ * 2 4 - 5 * 2 2))) ; ==> 9