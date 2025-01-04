create function ShoesTShops
(
    @T int

) returns table
as return
    select S.id
    from Shoe S
    join ShoePresentationShops SPS on S.id = SPS.shoe_id
    group by S.id
    having count(distinct  SPS.shop_id) >= @T
