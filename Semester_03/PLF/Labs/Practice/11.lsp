(defun maimunte (a dir)
  (cond
    ((null (cdr a)) t)                                   ; Base case - single element
    ((> (cadr a) (car a))                               ; Numbers increasing
     (if dir 
         nil                                             ; If we were decreasing, this isn't a mountain
         (maimunte (cdr a) nil)))                       ; Continue checking increasing part
    ((< (cadr a) (car a))                               ; Numbers decreasing
     (if dir 
         (maimunte (cdr a) t)                           ; Continue checking decreasing part
         (maimunte (cdr a) t)))                         ; Switch to decreasing phase
    (t nil)))                                           ; Equal numbers - not a mountain

(print (maimunte '(10 18 29 17 11 10) nil))             ; Changed to nil here