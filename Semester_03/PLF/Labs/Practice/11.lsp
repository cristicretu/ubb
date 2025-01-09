(defun maimunte (a dir)
  (cond
    ((null (cdr a)) t)                                   ; Base case - single element
    ((> (cadr a) (car a))                               ; Numbers increasing
     (cond 
       (dir nil)                                         ; If we were decreasing, this isn't a mountain
       (t (maimunte (cdr a) nil))))                     ; Continue checking increasing part
    ((< (cadr a) (car a))                               ; Numbers decreasing
     (maimunte (cdr a) t))                              ; Continue checking decreasing part
    (t nil)))                                           ; Equal numbers - not a mountain

(print (maimunte '(10 18 29 17 11 10) nil))
(print (maimunte '(10 18 29 17 11 10 11) nil))

(defun valei (a dir)
  (cond
    ((null (cdr a)) t)                                   
    ((< (cadr a) (car a))  ; cobor                  
     (cond 
       (dir nil)                                         
       (t (valei (cdr a) nil))))                     
    ((> (cadr a) (car a))                               
     (valei (cdr a) t))                              
    (t nil)))       

(print (valei '(10 8 6 17 19 20) nil))
(print (valei '(10 8 6 9) nil))