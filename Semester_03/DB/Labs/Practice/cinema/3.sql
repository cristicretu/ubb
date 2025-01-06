create view GetPopularActors as
select A.name from Actor A
join ProductionActors PA on A.id = PA.actor_id
join CinemaProductions CP on PA.production_id = CP.id
group by A.name
having count(distinct CP.name) = (select count(distinct name) from CinemaProductions)
