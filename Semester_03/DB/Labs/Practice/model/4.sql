create function getStations
(
    @R int
)
returns table
as
return (
    select S.name
    from Stations S
    join RouteStations RS2 on S.id = RS2.station_id
    group by S.id, S.name
    having count(distinct RS2.route_id) > @R
    )

SELECT * from getStations(1)
