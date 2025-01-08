create view BestChefs as
select C.name
from Chef C
join ChefSpecializations CS on C.id = CS.chef_id
group by C.name
having count(CS.cake_id) = (select count(*) from Cake)

select * from BestChefs

