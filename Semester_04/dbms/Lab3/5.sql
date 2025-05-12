create function uf_ValidateCountry (@country VARCHAR(50))
returns INT as
begin
	declare @return INT = 0
	if @country in ('Italy', 'United Kingdom', 'Germany', 'France', 'Spain', 
                   'Netherlands', 'Austria', 'Australia', 'United States', 
                   'Mexico', 'Brazil', 'Canada', 'Japan', 'Monaco', 'Belgium')
	set @return = 1
return @return
end
go

create function uf_ValidateDriverNumber (@drivernumber int)
returns int as
begin
    declare @return int = 0
    if @drivernumber between 1 and 99
        set @return = 1
    return @return
end
go

create function uf_ValidateName (@name varchar(50))
returns int as
begin
    declare @return int = 0
    if len(@name) > 1 and left(@name, 1) = upper(left(@name, 1))
        set @return = 1
    return @return
end
go

create function uf_ValidatePosition (@position int)
returns int as
begin
    declare @return int = 0
    if @position between 1 and 20
        set @return = 1
    return @return
end
go

create function uf_ValidateRaceName (@racename varchar(100))
returns int as
begin
    declare @return int = 0
    if len(@racename) >= 3 and left(@racename, 1) = upper(left(@racename, 1))
        set @return = 1
    return @return
end
go

create function uf_ValidateRaceDate (@racedate date)
returns int as
begin
    declare @return int = 0
    if year(@racedate) >= year(getdate())
        set @return = 1
    return @return
end
go

---


create or alter procedure AddRaceDriverResult
	@raceName varchar(100),
	@circuit varchar(100),
	@country varchar(50),
	@raceDate date,
	@driverNumber int,
	@position int,
	@points int
as
begin
	begin try
		begin transaction;

		insert into LogTable (typeOperation, tableOperation, message)
		values ('begin', 'AddRaceDriverResult', 'start procedure');
		
		if dbo.uf_ValidateRaceName(@raceName) = 0 begin
			raiserror('invalid race name; need at least 3 chars and start with uppercase', 16, 1);
		end

		if dbo.uf_ValidateCountry(@country) = 0 begin
			raiserror('invalid country', 16, 1);
		end

		if dbo.uf_ValidateRaceDate(@raceDate) = 0 begin
			raiserror('invalid race date; must be >= 2025', 16, 1);
		end

		if dbo.uf_ValidateDriverNumber(@driverNumber) = 0 begin
			raiserror('invaliddriver number; 1 <= no <= 99', 16, 1);
		end

		if dbo.uf_ValidatePosition(@position) = 0 begin
			raiserror('invalid position; 1<= pos <= 20', 16, 1);
		end

		if @points < 0 begin
			raiserror('invalid points, must be a natural numb', 16, 1);
		end

		declare @driverId int;
		select @driverId = driverId from Drivers where driverNumber = @driverNumber;

		if @driverId is null begin
			raiserror('driver does not exist',16,1);
		end

		-- insert race and get its id
		insert into races (raceName, circuit, country, raceDate)
		values (@raceName, @circuit, @country, @raceDate);

		declare @raceId int = scope_identity();

		insert into LogTable(typeOperation, tableOperation, message)
		values ('insert', 'Race', 'Added race ' + @raceName);

		-- now put the dirver result
		insert into RaceResults(raceId, driverId, position, points)
		values (@raceId, @driverId, @position, @points);

		insert into LogTable(typeOperation, tableOperation, message)
		values ('insert', 'RaceResults', 'Added race result for driver ' + cast(@driverNumber as varchar));

		commit transaction;

		insert into LogTable(typeOperation, tableOperation, message)
		values ('commit', 'AddRaceDriverResult', 'commited transaction');
		

	end try

	--- errors
	begin catch
		insert into LogTable(typeOperation, tableOperation, message)
		values ('error', 'AddRaceDriverResult', error_message());

		if @@TRANCOUNT > 0
			rollback transaction;

		insert into LogTable(typeOperation, tableOperation, message)
		values ('rollback', 'AddRaceDriverResult', 'transaction rolled back');

		throw;
	end catch
end
go

-- tests


exec AddRaceDriverResult 
    @raceName = 'Monaco Grand Prix', 
    @circuit = 'Circuit de Monaco', 
    @country = 'Monaco', 
    @raceDate = '2025-05-25', 
    @driverNumber = 44, 
    @position = 1, 
    @points = 25.0;

-- fail
exec AddRaceDriverResult 
    @raceName = 'Singapore Grand Prix', 
    @circuit = 'Marina Bay Street Circuit', 
    @country = 'Romania', 
    @raceDate = '2025-09-20', 
    @driverNumber = 44, 
    @position = 2, 
    @points = 18.0;

select * from LogTable;
select * from RaceResults;
select * from Races;
select * from Drivers;

----


create or alter procedure AddRaceDriverResultRecoverable
    @raceName varchar(100),
    @circuit varchar(100),
    @country varchar(50),
    @raceDate date,
    @driverNumber int,
    @firstName varchar(50),
    @lastName varchar(50),
    @driverCountry varchar(50),
    @teamName varchar(100),
    @position int,
    @points int
as
begin
    
    declare @driverId int, @teamId int, @raceId int;
    declare @driverAdded int = 0, @teamAdded int = 0, @raceAdded int = 0;
    
    insert into LogTable (typeOperation, tableOperation, message)
    values ('begin', 'AddRaceDriverResultRecoverable', 'start procedure');
    
	-- team
    begin try
        select @teamId = teamId from Teams where teamName = @teamName;
        
        if @teamId is null
        begin
            if dbo.uf_ValidateCountry(@country) = 0
            begin
                insert into LogTable (typeOperation, tableOperation, message)
                values ('error', 'Teams', 'invalid country for team');
                
                raiserror('invalid country for team', 16, 1);
            end
            
            begin transaction;
            
            declare @constructorCode varchar(10);
            set @constructorCode = left(replace(@teamName, ' ', ''), 3);
            
			insert into Teams (teamName, country, foundedYear, constructorCode)
            values (@teamName, @country, year(getdate()), upper(@constructorCode));
            
            set @teamId = scope_identity();
            
            set @teamAdded = 1; -- am trecut aici
            
            commit transaction;
            
            insert into LogTable (typeOperation, tableOperation, message)
            values ('insert', 'Teams', 'added team: ' + @teamName);
        end
        else
        begin
            insert into LogTable (typeOperation, tableOperation, message)
            values ('info', 'Teams', 'using existing team: ' + @teamName);
        end
    end try
    begin catch
        insert into LogTable (typeOperation, tableOperation, message)
        values ('error', 'Teams', error_message());
        
        if @@trancount > 0
            rollback transaction;
            
    end catch
    
	-- driver
    begin try
        select @driverId = driverId from Drivers where driverNumber = @driverNumber;
        
        if @driverId is null
        begin
            if dbo.uf_ValidateDriverNumber(@driverNumber) = 0
            begin
                insert into LogTable (typeOperation, tableOperation, message)
                values ('error', 'Drivers', 'invalid driver number');
                
                raiserror('invalid driver number; 1 <= no <= 99', 16, 1);
            end
            
            if dbo.uf_ValidateName(@firstName) = 0
            begin
                insert into LogTable (typeOperation, tableOperation, message)
                values ('error', 'Drivers', 'invalid first name');
                
                raiserror('invalid first name; must start with uppercase', 16, 1);
            end
            
            if dbo.uf_ValidateName(@lastName) = 0
            begin
                insert into LogTable (typeOperation, tableOperation, message)
                values ('error', 'Drivers', 'invalid last name');
                
                raiserror('invalid last name; must start with uppercase', 16, 1);
            end
            
            if dbo.uf_ValidateCountry(@driverCountry) = 0
            begin
                insert into LogTable (typeOperation, tableOperation, message)
                values ('error', 'Drivers', 'invalid country for driver');
                
                raiserror('invalid country for driver', 16, 1);
            end
            
            begin transaction;
            
            select @driverId = isnull(max(driverId), 0) + 1 from Drivers;
            
            insert into Drivers (driverId, driverNumber, firstName, lastName, country, teamId)
            values (@driverId, @driverNumber, @firstName, @lastName, @driverCountry, @teamId);
            
            set @driverAdded = 1; -- aici
            
            commit transaction;
            
            insert into LogTable (typeOperation, tableOperation, message)
            values ('insert', 'Drivers', 'added driver: ' + @firstName + ' ' + @lastName);
        end
        else
        begin
            insert into LogTable (typeOperation, tableOperation, message)
            values ('info', 'Drivers', 'using existing driver: ' + cast(@driverNumber as varchar));
        end
    end try
    begin catch
        insert into LogTable (typeOperation, tableOperation, message)
        values ('error', 'Drivers', error_message());
        
        if @@trancount > 0
            rollback transaction;
            
    end catch
    


	-- race and result
    begin try
        if dbo.uf_ValidateRaceName(@raceName) = 0
        begin
            insert into LogTable (typeOperation, tableOperation, message)
            values ('error', 'Races', 'invalid race name');
            
            raiserror('invalid race name; need at least 3 chars and start with uppercase', 16, 1);
        end
        
        if dbo.uf_ValidateCountry(@country) = 0
        begin
            insert into LogTable (typeOperation, tableOperation, message)
            values ('error', 'Races', 'invalid country for race');
            
            raiserror('invalid country', 16, 1);
        end
        
        if dbo.uf_ValidateRaceDate(@raceDate) = 0
        begin
            insert into LogTable (typeOperation, tableOperation, message)
            values ('error', 'Races', 'invalid race date');
            
            raiserror('invalid race date; must be >= 2025', 16, 1);
        end
        
        if dbo.uf_ValidatePosition(@position) = 0
        begin
            insert into LogTable (typeOperation, tableOperation, message)
            values ('error', 'RaceResults', 'invalid position');
            
            raiserror('invalid position; 1<= pos <= 20', 16, 1);
        end
        
        if @points < 0
        begin
            insert into LogTable (typeOperation, tableOperation, message)
            values ('error', 'RaceResults', 'invalid points');
            
            raiserror('invalid points, must be a natural numb', 16, 1);
        end
        
        begin transaction;
        
        insert into Races (raceName, circuit, country, raceDate)
        values (@raceName, @circuit, @country, @raceDate);
        
        set @raceId = scope_identity();
        
        set @raceAdded = 1; -- trecut
        
        insert into LogTable (typeOperation, tableOperation, message)
        values ('insert', 'Races', 'added race: ' + @raceName);
        
        insert into RaceResults (raceId, driverId, position, points)
        values (@raceId, @driverId, @position, @points);
        
        insert into LogTable (typeOperation, tableOperation, message)
        values ('insert', 'RaceResults', 'added result for driver ' + cast(@driverNumber as varchar));
        
        commit transaction;
        
        insert into LogTable (typeOperation, tableOperation, message)
        values ('commit', 'RaceResults', 'race results added successfully');
    end try
    begin catch
        insert into LogTable (typeOperation, tableOperation, message)
        values ('error', 'Races/RaceResults', error_message());
        
        if @@trancount > 0
            rollback transaction;
    end catch
    
    select 
        case when @teamAdded = 1 then 'team added successfully' 
             when @teamId is not null then 'existing team used' 
             else 'failed to add or find team' 
        end as teamStatus,
        
        case when @driverAdded = 1 then 'driver added successfully' 
             when @driverId is not null then 'existing driver used' 
             else 'failed to add or find driver' 
        end as driverStatus,
        
        case when @raceAdded = 1 then 'race and results added successfully' 
             else 'failed to add race and results' 
        end as raceStatus;
end
go

-- tests

-- alles gut
exec AddRaceDriverResultRecoverable
    @raceName = 'Italian Grand Prix',
    @circuit = 'Monza',
    @country = 'Italy',
    @raceDate = '2025-09-07',
    @driverNumber = 16,
    @firstName = 'Charles',
    @lastName = 'Leclerc',
    @driverCountry = 'Monaco',
    @teamName = 'Ferrari',
    @position = 1,
    @points = 25.0;

-- team bun, driver fail, race bun
exec AddRaceDriverResultRecoverable
    @raceName = 'British Grand Prix',
    @circuit = 'Silverstone',
    @country = 'United Kingdom',
    @raceDate = '2025-07-06',
    @driverNumber = 4,
    @firstName = '', -- should start with uppercase
    @lastName = 'Norris',
    @driverCountry = 'United Kingdom',
    @teamName = 'McLaren',
    @position = 2,
    @points = 18.0;

-- team, driver bun, race rau
exec AddRaceDriverResultRecoverable
    @raceName = 'Australian Grand Prix',
    @circuit = 'Albert Park',
    @country = 'Australia',
    @raceDate = '2024-03-15',
    @driverNumber = 11,
    @firstName = 'Sergio',
    @lastName = 'Perez',
    @driverCountry = 'Mexico',
    @teamName = 'Red Bull Racing',
    @position = 3,
    @points = 15.0;

select * from LogTable;
select * from Teams;
select * from Drivers;
select * from Races;
select * from RaceResults;


