
    CREATE VIEW AllStations AS
SELECT R.name
FROM Routes R
JOIN RouteStations RS ON R.id = RS.route_id
GROUP BY R.id, R.name
HAVING COUNT(DISTINCT RS.station_id) = (SELECT COUNT(*) FROM Stations);

select * from AllStations
