create function GetFreqRepairedCars
(
    @N int,
    @Month date
) returns table
as return

select C.id, C.owner, C.color
from Car C
join Fixx F2 on C.id = F2.car_id
join FixxDetails D on F2.id = D.fix_id
where datediff(month, D.repair_date, @Month) = 0
group by C.id, C.owner, C.color
having count(*) > @N


select * from GetFreqRepairedCars (1, '2020-05-01')
