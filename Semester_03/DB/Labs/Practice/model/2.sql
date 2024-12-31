create procedure AddStation
    @RouteName varchar(255),
    @StationName varchar(255),
    @Arrival time,
    @Departure time
as
begin
    declare @RouteId int
    declare @StationId int

    select @RouteId = id from Routes where name = @RouteName
    select @StationId = id from Stations where name = @StationName

    if exists (select 1 from RouteStations where route_id = @RouteId and station_id = @StationId)
        update RouteStations
        set arrival_time = @Arrival, departure_time = @Departure
        where route_id = @RouteId and station_id = @StationId
    else
        insert into RouteStations (route_id, station_id, arrival_time, departure_time) values (@RouteId, @StationId, @Arrival, @Departure)
end


EXEC AddStation N'Morning Route', N'Airport Station', '08:30', '08:49';

select * from RouteStations
