create function TwoShoesWoman (@ShoeModel int) returns table
    as return
    select WS.woman_id
    from Womans W
    join WomanShoes WS on W.id = WS.woman_id
    join Shoe S on WS.shoe_id = S.id
    where S.model_id = @ShoeModel
    group by WS.woman_id
    having count(*) >= 2

select * from TwoShoesWoman(1)


