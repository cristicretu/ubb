create function GetVisitors
(
    @N int
)
returns table
as
return (
    select v.id, v.name
    from Visitor v
    join VisitorVisits VV2 on v.id = VV2.visitor_id
    where
    (select count(*) from ZooAnimals where zoo_id = VV2.zoo_id) >= @N
    )
-- get no. of animals
select count(*) from ZooAnimals where zoo_id = 3

select * from GetVisitors(3)


