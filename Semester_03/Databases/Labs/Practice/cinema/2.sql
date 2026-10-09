create procedure AddActor (
    @Name varchar(100),
    @Rewards int,
    @Entry datetime,
    @Production varchar(100)
) as
    begin
    declare @ActorId int
    declare @ProductionId int
    declare @Exists int

    select @ActorId = id from Actor where name = @Name and ranking = @Rewards

    if @ActorId is null begin
        print 'actor does not exist'
        return
    end

    select @ProductionId = id from CinemaProductions where name = @Production

    if @ProductionId is null begin
        print 'production does not exist'
        return
    end

    select @Exists = count(*) from ProductionActors where actor_id = @ActorId and production_id = @ProductionId

    if @Exists > 0 begin
        print 'actor is already at that production'
        return
    end

    begin
    insert into ProductionActors (production_id, actor_id, entry) values
    (@ProductionId, @ActorId, @Entry)
    end
end

exec AddActor 'Micu', 11, '2024-01-01','BIG BROTHERS'

select *
from ProductionActors;
