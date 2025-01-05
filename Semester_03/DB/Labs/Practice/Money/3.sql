create view CardNumbersATMS as
select C.number
from Card C
join Transactions T on C.id = T.card_id
group by C.number
having count(distinct T.atm_id) = (Select count(*) from ATM)

select * from CardNumbersATMS
