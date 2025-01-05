create function BigCards (
    @Sum int
) returns table
as
return
    SELECT C.number, C.cvv, SUM(T2.withdrawal) TS
    from Card C
    join Transactions T2 on C.id = T2.card_id
    group by C.number, C.cvv
    having SUM(T2.withdrawal) >= @Sum


select * from BigCards (100)
