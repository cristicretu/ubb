(defun parsar (lst operation nambar)
  (cond
    ((null lst) (car nambar))
    ((numberp (car lst)) ; operand
      (cond
        ((car nambar) (parsar (cdr lst) (cdr operation) 
                             (cons (funcall (car operation) (car lst) (car nambar)) 
                                   (cdr nambar))))
        (t (parsar (cdr lst) operation (cons (car lst) nambar)))
      ))
    (t (parsar (cdr lst) (cons (car lst) operation) nambar)) ; operator
  ))

(print (parsar '(+ 1 3) nil nil))
(print (parsar '(+ * 2 4  3) nil nil))
(print (parsar '(+ * 2 4 - 5 * 2 2) nil nil))


(defun parse-polish (tokens)
  (let ((token (car tokens)))
    (cond 
      ((null tokens) nil)
      ((member token '(+ - *))  ; if operator
       (let* ((rest1 (parse-polish (cdr tokens)))   ; parse left expr
              (rest2 (parse-polish (car rest1)))    ; parse right expr
              (remaining (cdr rest1)))              ; remaining tokens
         (cons (list token (cadr rest1) (cadr rest2)) remaining)))
      (t  ; if number
       (cons (cdr tokens)       ; remaining tokens
             (list token))))))  ; parsed number

;; Convert string to list of tokens
(defun tokenize (str)
  (read-from-string 
    (concatenate 'string "(" str ")")))

;; Main function to evaluate polish notation string
(defun eval-polish (str)
  (cadr (parse-polish (tokenize str))))


