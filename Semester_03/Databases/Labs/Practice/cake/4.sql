-- chefs that are specialized in more than P cake types

create function PChefs (
    @P int
) returns table
as return
select C.name
from Chef C
join ChefSpecializations S on C.id = S.chef_id
join Cake C2 on S.cake_id = C2.id
join CakeType CT on C2.cake_type = CT.id
group by C.name
having count(distinct CT.id) > @P


select *
from PChefs (3);


