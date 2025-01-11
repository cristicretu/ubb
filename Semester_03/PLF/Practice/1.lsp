; (DEFUN F (L)
;   (COND
;     ((NULL L) NIL)
;     ((LISTP (CAR L)) (APPEND (F (CAR L)) (F (CDR L)) (CAR (F (CAR L)))))
;     (T (LIST(CAR L)))
;   )
; )

(DEFUN F (L)
  (COND
    ((NULL L) NIL)
    ((LISTP (CAR L)) 
      (LET ((FCAR (F (CAR L))))
        (APPEND FCAR (F (CDR L)) (CAR FCAR))))
    (T (LIST(CAR L)))
  )
)

(print (f '((1)  3 4 5 6 7)))