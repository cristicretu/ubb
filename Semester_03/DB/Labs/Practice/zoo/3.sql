create view LeastVisitedZoos as
    select top (select count(*) from Zoos) z.id, z.name as zoo_id from
    Zoos z
    left join VisitorVisits VV on z.id = VV.zoo_id
    group by  z.id, z.name
    order by count(VV.zoo_id);

drop view LeastVisitedZoos

select * from LeastVisitedZoos
