-- # Schema
--
-- -- Teams
-- -- Drivers
-- -- Races
-- -- Circuits
-- -- Results
-- -- Penalties
-- -- Sponsors
-- -- Engines
-- -- TeamSponsors
-- -- TeamEngines
--
-- ## Teams
--
-- _teamId_ int primary key
-- _teamName_ text not null
-- _country_ text not null
-- _foundedYear_ int not null
-- _constructorCode_ text not null unique
--
-- ## Drivers
--
-- _driverId_ int primary key
-- _driverNumber_ int not null unique
-- _firstName_ text not null
-- _lastName_ text not null
-- _country_ text not null
-- **teamId** int not null
--
-- ## Circuits
--
-- _circuitId_ int primary key
-- _name_ text not null unique
-- _location_ text not null
-- _country_ text not null
-- _length_ int not null
-- _capacity_ int not null
--
-- ## Races
--
-- _raceId_ int primary key
-- _name_ text not null
-- _date_ date not null
-- _round_ int not null
-- _season_ int not null
-- **circuitId** int not null

-- CREATE TABLE Races (
--     raceId INTEGER PRIMARY KEY,
--     name varchar(100) NOT NULL,
--     date DATE NOT NULL,
--     round INTEGER NOT NULL,
--     season INTEGER NOT NULL,
--     circuitId INTEGER NOT NULL,
--     FOREIGN KEY (circuitId) REFERENCES Circuits(circuitId)
-- );

-- ## Results
--
-- _resultId_ int primary key
-- _position_ int not null
-- _laps_ int not null
-- _time_ int not null
-- _points_ not null
-- _finishStatus_ text not null
-- **raceId** int not null
-- **driverId** int not null
-- **teamId** int not null

-- CREATE TABLE Results (
--     resultId INTEGER PRIMARY KEY,
--     position INTEGER NOT NULL,
--     laps INTEGER NOT NULL,
--     time INTEGER NOT NULL,
--     points INTEGER NOT NULL,
--     finishStatus varchar(50) NOT NULL,
--     raceId INTEGER NOT NULL,
--     driverId INTEGER NOT NULL,
--     teamId INTEGER NOT NULL,
--     FOREIGN KEY (raceId) REFERENCES Races(raceId),
--     FOREIGN KEY (driverId) REFERENCES Drivers(driverId),
--     FOREIGN KEY (teamId) REFERENCES Teams(teamId)
-- );

--
-- ## Sponsors
--
-- _sponsorId_ int primary key
-- _sponsorName_ text not null
--

-- CREATE TABLE Sponsors (
--     sponsorId INTEGER PRIMARY KEY,
--     sponsorName varchar(100) NOT NULL
-- );


-- ## Engines
--
-- _engineId_ int primary key
-- _engineName_ text not null
-- _manufacturer_ text not null
-- _horsepower_ int not null

-- CREATE TABLE Engines (
--     engineId INTEGER PRIMARY KEY,
--     engineName varchar(100) NOT NULL,
--     manufacturer varchar(100) NOT NULL,
--     horsepower INTEGER NOT NULL
-- );
--
-- ## TeamSponsors
--
-- _teamId_ int not null
-- _sponsorId_ int not null
-- _contractStart_ date not null
-- _contractEnd_ date not null
--

-- CREATE TABLE TeamSponsors (
--     teamId INTEGER NOT NULL,
--     sponsorId INTEGER NOT NULL,
--     contractStart DATE NOT NULL,
--     contractEnd DATE NOT NULL,
--     PRIMARY KEY (teamId, sponsorId),
--     FOREIGN KEY (teamId) REFERENCES Teams(teamId),
--     FOREIGN KEY (sponsorId) REFERENCES Sponsors(sponsorId)
-- )



-- ## TeamEngines
--
-- _teamId_ int not null
-- _engineId_ int not null

-- CREATE TABLE TeamEngines (
--     teamId INTEGER NOT NULL,
--     engineId INTEGER NOT NULL,
--     PRIMARY KEY (teamId, engineId),
--     FOREIGN KEY (teamId) REFERENCES Teams(teamId),
--     FOREIGN KEY (engineId) REFERENCES Engines(engineId)
-- );

--
-- ## Penalties
--
-- _penaltyId_ int primary key
-- _penaltyType_ text not null
-- _penaltyDescription_ text not null
-- _penaltyDroppedPositions_ int not null
-- **driverId** int not null
-- **raceId** int not null
--
-- CREATE TABLE Penalties (
--     penaltyId INTEGER PRIMARY KEY,
--     penaltyType varchar(100) NOT NULL,
--     penaltyDescription varchar(100) NOT NULL,
--     penaltyDroppedPositions INTEGER NOT NULL,
--     driverId INTEGER NOT NULL,
--     raceId INTEGER NOT NULL,
--     FOREIGN KEY (driverId) REFERENCES Drivers(driverId),
--     FOREIGN KEY (raceId) REFERENCES Races(raceId),
-- );

insert into Teams values (1, 'Mercedes', 'Germany', 2010, 'MER'),
                        (2, 'Ferrari', 'Italy', 1950, 'FER'),
                        (3, 'Red Bull Racing', 'Austria', 2005, 'RBR'),
                        (4, 'McLaren', 'United Kingdom', 1966, 'MCL'),
                        (5, 'Aston Martin', 'United Kingdom', 2021, 'AMR'),
                        (6, 'Alpine', 'France', 1977, 'ALP'),
                        (7, 'AlphaTauri', 'Italy', 2006, 'ATR'),
                        (8, 'Alfa Romeo Racing', 'Switzerland', 1993, 'ARR'),
                        (9, 'Haas', 'United States', 2016, 'HAS'),
                        (10, 'Williams', 'United Kingdom', 1977, 'WIL');

insert into Teams values (1, 'Kick', 'Italy', 2024, 'KCK');

select * from Teams;


insert into Drivers (driverId, driverNumber, firstName, lastName, country, teamId) values (1, 44, 'Lewis', 'Hamilton', 'United Kingdom', 1),
                                                                                (2, 16, 'Charles', 'Leclerc', 'Monaco', 2),
                                                                                (3, 33, 'Max', 'Verstappen', 'Netherlands', 3),
                                                                                (4, 77, 'Valtteri', 'Bottas', 'Finland', 4),
                                                                                (5, 5, 'Sebastian', 'Vettel', 'Germany', 5),
                                                                                (6, 3, 'Daniel', 'Ricciardo', 'Australia', 6),
                                                                                (7, 10, 'Pierre', 'Gasly', 'France', 7),
                                                                                (8, 99, 'Antonio', 'Giovinazzi', 'Italy', 8),
                                                                                (9, 47, 'Mick', 'Schumacher', 'Germany', 9),
                                                                                (10, 6, 'Nicholas', 'Latifi', 'Canada', 10);

select * from Drivers;


insert into Circuits (circuitId, circuitName, country, city, circuitLength) values (1, 'Silverstone Circuit', 'United Kingdom', 'Silverstone', 5891),
                                                                                (2, 'Albert Park', 'Melbourne', 'Australia', 5303),
                            (3, 'Bahrain International Circuit', 'Sakhir', 'Bahrain', 5412),
                            (4, 'Shanghai International Circuit', 'Shanghai', 'China', 5451),
                            (5, 'Circuit de Monaco', 'Monte Carlo', 'Monaco', 3337);

select * from Circuits;

insert into Races (raceId, name, date, round, season, circuitId) values (1, 'British Grand Prix', '2024-07-18', 10, 2024, 1),
                                                                        (2, 'Australian Grand Prix', '2024-11-21', 21, 2024, 2),
                                                                        (3, 'Bahrain Grand Prix', '2024-03-28', 1, 2024, 3),
                                                                        (4, 'Chinese Grand Prix', '2024-04-11', 3, 2024, 4),
                                                                        (5, 'Monaco Grand Prix', '2024-05-23', 5, 2024, 5);

select * from Races;

-- RACE 1
insert into Results (resultId, position, laps, time, points, finishStatus, raceId, driverId, teamId) values
(1, 1, 52, 5320, 25, 'Finished', 1, 1, 1),
(2, 2, 52, 5399, 18, 'Finished', 1, 2, 2),
(3, 3, 52, 5401, 15, 'Finished', 1, 3, 3),
(4, 4, 52, 5500, 12, 'Finished', 1, 4, 4),
(5, 5, 52, 5802, 10, 'Finished', 1, 5, 5),
(6, 6, 52, 6100, 8, 'Finished', 1, 6, 6),
(7, 7, 52, 6421, 6, 'Finished', 1, 7, 7),
(8, 8, 52, 6999, 4, 'Finished', 1, 8, 8),
(9, 9, 52, 7001, 2, 'Finished', 1, 9, 9),
(10, 10, 43, 7001, 1, 'DNF', 1, 10, 10);

-- Race 2
insert into Results (resultId, position, laps, time, points, finishStatus, raceId, driverId, teamId) values
(11, 1, 58, 6320, 25, 'Finished', 2, 2, 2),
(12, 2, 58, 7399, 18, 'Finished', 2, 1, 1),
(13, 3, 58, 8401, 15, 'Finished', 2, 4, 4),
(14, 4, 58, 9500, 12, 'Finished', 2, 9, 9),
(15, 5, 58, 9802, 10, 'Finished', 2, 8, 8),
(16, 6, 58, 10100, 8, 'Finished', 2, 3, 3),
(17, 7, 58, 10421, 6, 'Finished', 2, 5, 5),
(18, 8, 58, 10999, 4, 'Finished', 2, 6, 6),
(19, 9, 58, 11001, 2, 'Finished', 2, 7, 7),
(20, 10, 43, 11001, 1, 'DNF', 2, 10, 10);



select * from Results;

insert into Sponsors (sponsorId, sponsorName) values
  (1, 'Aramco'),
  (2, 'AWS'),
    (3, 'Pirelli'),
    (4, 'Rolex'),
    (5, 'DHL'),
    (6, 'Heineken'),
    (7, 'Emirates'),
    (8, 'DHL'),
    (9, 'Puma'),
    (10, 'UPS');

select * from Sponsors;

insert into TeamSponsors (teamId, sponsorId, contractStart, contractEnd) values
    (1, 1, '2024-01-01', '2028-12-31'),
    (1, 2, '2020-01-01', '2024-12-31'),
    (2, 3, '2018-01-01', '2024-12-31'),
    (2, 4, '2011-01-01', '2030-12-31'),
    (3, 5, '2024-01-01', '2028-12-31'),
    (3, 6, '2024-01-01', '2034-12-31'),
    (4, 7, '2024-01-01', '2025-12-31'),
    (4, 8, '2024-01-01', '2024-12-31'),
    (5, 9, '2024-01-01', '2040-12-31'),
    (5, 10, '2024-01-01', '2100-12-31');

select * from TeamSponsors;

insert into Engines (engineId, manufacturer, model, power) values
(1, 'Mercedes', 'M12', 1000),
(2, 'Ferrari', 'F12', 900),
(3, 'Renault', 'R12', 800),
(4, 'Honda', 'H12', 700),
(5, 'Mercedes', 'M12Pro', 1100),
(6, 'Ferrari', 'F13', 1215),
(7, 'Renault', 'R13h', 900),
(8, 'Honda', 'H13.11', 1300),
(9, 'Mercedes', 'M14', 1200),
(10, 'Ferrari', 'F14', 1500);

select * from Engines;

insert into TeamEngines (teamId, engineId) values
(1, 9),
(2, 10),
(3, 8),
(4, 8),
(5, 5),
(6, 3),
(7, 6),
(8, 2),
(9, 1),
(10, 7);

select * from TeamEngines;

insert into Penalties (penaltyId, penaltyType, penaltyDescription, penaltyDroppedPositions, driverId, raceId) values
(1, 'Drive Through', 'speeding in the pit lane', 2, 3, 1),
(2, 'Time Penalty', 'Unsafe release', 5, 2, 2),
(3, 'Grid Penalty', 'Gearbox change', 2, 3, 2),
(4, 'Drive Through', 'speeding in the pit lane', 5, 4, 2),
(5, 'Time Penalty', 'Unsafe release', 5, 5, 2),
(6, 'Grid Penalty', 'Gearbox change', 2, 6, 1),
(7, 'Drive Through', 'speeding in the pit lane', 5, 8, 2),
(8, 'Time Penalty', 'Unsafe release', 5, 8, 1),
(9, 'Grid Penalty', 'Gearbox change', 2, 9, 1),
(10, 'Drive Through', 'speeding in the pit lane', 2, 10, 1);

select * from Penalties;

select * from Results;

update Drivers set teamId = 2 where firstName = 'Lewis' and lastName = 'Hamilton';
update Teams set teamName = 'Scuderia Ferrari' where country = 'Italy' and foundedYear < 2008 and not constructorCode = 'ATR';
update Teams set foundedYear = foundedYear - 10 where foundedYear between 1950 and 1980 and not country = 'Switzerland';
update Circuits set circuitLength = circuitLength + 100 where country = 'Monaco';
update Results set time = time + 100 where position = 1 and raceId = 2 and not finishStatus = 'DNF';
update Engines set power = power * 1.5 where manufacturer = 'Renault' and power < 1000;

update Circuits set city = country, country = city where circuitId > 1 and circuitId < 5;


delete from Penalties where penaltyType = 'Drive Through'and penaltyDroppedPositions > 2;
delete from Engines where manufacturer = 'Honda' and power < 800;
delete from TeamSponsors where DATEDIFF(year, contractStart, contractEnd) > 10;

select driverId, firstName, lastName from Drivers where country = 'Germany'
union all
select driverId, firstName, lastName from Drivers where country = 'Italy'
order by lastName;

select circuitId, circuitName from Circuits where circuitLength > 5500
union
select circuitId, circuitName from Circuits where city = 'Sakhir' or city = 'Shanghai'
order by circuitName;

select driverId
from Results
where position <= 3 and raceId = 1
intersect
select driverId
from Results
where position <= 3 and raceId = 2;

select *
from Drivers d
where d.driverId in (
select driverId
from Results
where position <= 3 and raceId = 1
intersect
select driverId
from Results
where position <= 3 and raceId = 2
);


select teamId, teamName
from Teams
where foundedYear < 2000
and teamId in (
    select teamId
    from Teams
    where country in ('United Kingdom', 'Italy')
);


select *
from Drivers d
where d.teamId in (select teamId from Teams where country = 'United Kingdom')
except
select * from Drivers
where driverId in (
    select driverId
    from Results
    where position = 1
);

select * from Results r
where r.driverId not in (
    select driverId
    from Drivers
    where teamId in (
        select teamId
        from Teams
        where country = 'United Kingdom'
    )
) and r.raceId = 1;


select * from Teams;
select * from Results;
select * from Drivers;

select d.firstName, d.lastName, t.teamName, r.points from Drivers d
inner join Teams t on d.teamId = t.teamId
inner join Results r on d.driverId = r.driverId
where raceId = 1 and r.finishStatus not like 'DNF'
order by r.points desc;



select  t.teamId, t.teamName, s.sponsorName,  ts.contractStart, ts.contractEnd from Teams t
left join TeamSponsors ts on t.teamId = ts.teamId
left join Sponsors s on ts.sponsorId = s.sponsorId
where not DATEDIFF(month , ts.contractStart, ts.contractEnd) < 16;


select p.penaltyType, p.penaltyDescription, d.firstName, d.lastName, r.name AS raceName
from Penalties p
right join Drivers d ON p.driverId = d.driverId
right join Races r ON p.raceId = r.raceId
where p.penaltyType like 'Drive Through';

select d.firstName, d.lastName, e.power, e.engineId from Drivers d
full join Teams t on d.teamId = t.teamId
full join TeamEngines te on t.teamId = te.teamId
full join Engines e on te.engineId = e.engineId
where    e.power > 1000 AND d.firstName IS NOT NULL
order by e.power desc;

select * from TeamSponsors;
select sponsorId from Sponsors;

select * from Engines;

select d.driverId, d.firstName, d.lastName from Drivers d
where teamId in (
    select teamId from TeamSponsors
    where sponsorId in (
        select sponsorId from Sponsors where Sponsors.sponsorId in (
            select sponsorId from TeamSponsors where contractEnd > '2030-12-31'
            )
        )
    );

select raceId, name from Races
where raceId in (
    select raceId from Results
    where position > 3 and driverId in (
        select driverId from Drivers where teamId in (
            select teamId from TeamEngines where engineId in (
                select engineId from Engines where power > 1300
                )
            )
        )
    );


select teamName from Teams
where exists (
    select * from Drivers d
    where d.teamId = Teams.teamId
    and exists (
        select * from Results r
        where r.driverId = d.driverId
        and r.position <= 3
    )
);

select * from Circuits;

select circuitName from Circuits
where exists (
    select * from Races r
    where r.circuitId = Circuits.circuitId
    and exists (
        select * from Results rez
        where rez.raceId = r.raceId
        and exists (
            select * from Drivers d
            where d.driverId = rez.driverId
            and exists (
                select * from TeamSponsors ts
                where ts.teamId = d.teamId
                and ts.sponsorId = (
                    SELECT sponsorId from Sponsors where sponsorName = 'Pirelli'
                    )
            )
        )
    )
);

select * from Teams;

SELECT d.firstName, d.lastName, t.teamName, totalPoints
FROM (
    SELECT driverId, SUM(points) AS totalPoints
    FROM Results
    GROUP BY driverId
) AS driverPoints
JOIN Drivers d ON driverPoints.driverId = d.driverId
JOIN Teams t ON d.teamId = t.teamId
ORDER BY totalPoints DESC;

-- g. 1 the time for each driver in the most recent race
select d.firstName, d.lastName, lastRace.time
from (
    select r.driverId, r.position, r.time from Results r
    join Races race on r.raceId = race.raceId
    where race.date = (select MAX(date) from Races)
     ) as lastRace
join Drivers d on lastRace.driverId = d.driverId
order by lastRace.position;

-- g.2 teams and their engines (if they have more than1000hp)
select t.teamName, eng.model, eng.manufacturer, eng.power
from (
    select te.TeamId, e.manufacturer, e.power, e.model from TeamEngines te
    join Engines e on te.engineId = e.engineId
    where e.power > 1000
     ) as eng
join Teams t on t.teamId = eng.teamId
order by eng.power desc;

-- h. the teams that have more than 2 podiums
select t.teamName
from Results r
join Teams t on r.teamId = t.teamId
where position <= 3
group by t.teamName
having count(*) >= 2;

-- circuits that have the total race time greater than the average race time of other races
select c.circuitName
from Results r
join Races ra on r.raceId = ra.raceId
join Circuits c on ra.circuitId = c.circuitId
group by c.circuitName
having sum(r.time) > (
    select avg(total)
    from (
        select raceId, sum(time) as total
        from Results
        group by raceId
         ) as raceTime
);

-- teams that were at least in the podium and have more than 10 points in total
select t.teamName
from Results r
join Teams t on r.teamId = t.teamId
group by t.teamName
having sum(r.points) > 10 and min(r.position) <= 3;

-- drivers that have an average position lower than the average position of all drivers
select d.firstName, d.lastName, t.teamName, sum(r.points)
from Results r
join Drivers d on r.driverId = d.driverId
join Teams t on d.teamId = t.teamId
group by d.firstName, d.lastName, t.teamName
having avg(r.position) <= (
    select avg(position) from Results
    ) and sum(r.points) > 10
order by sum(r.points) desc;


-- i. drivers that had more points in any race than the average points of all races
select distinct d.firstName, d.lastName, r.points
from Drivers d
join Results r on d.driverId = r.driverId
where r.points > any(
    select avg(points)
    from Results
    group by raceId
    )
order by r.points desc;
-- rewrite with in
select distinct d.firstName, d.lastName, r.points
from Drivers d
join Results r on d.driverId = r.driverId
where r.points in(
    select points
    from Results
    where points > (
        select avg(points)
        from Results
        where r.raceId = raceId
        )
    )
order by r.points desc;


-- get circuits that have a length greater than any circuit in monaco
select distinct c.circuitName, c.circuitLength
from Circuits c
where c.circuitLength > any(
    select circuitLength
    from Circuits
    where country = 'Monaco'
    )
order by c.circuitLength desc;
-- rewrite with aggregate op
select distinct c.circuitName, c.circuitLength
from Circuits c
where c.circuitLength > (
    select max(circuitLength)
    from Circuits
    where country = 'Monaco'
    )
order by c.circuitLength desc;


-- all teams that have all their engines with more than 1300hp
select distinct t.teamName
from Teams t
where 1300 < all(
    select e.power
    from Engines e
    join TeamEngines te on e.engineId = te.engineId
    where te.teamId = t.teamId
    )
order by t.teamName;
-- rewrite with aggregate
select distinct t.teamName
from Teams t
where (select min(e.power) from Engines e
    join TeamEngines te on e.engineId = te.engineId
     where te.teamId = t.teamId
   ) > 1300


-- select teams that have their sponsorships ending after 2030
select distinct t.teamName
from Teams t
where '2030-12-31' < all(
    select ts.contractEnd
    from TeamSponsors ts
    where ts.teamId = t.teamId
    )
order by t.teamName;
-- rewrite using not in
select distinct t.teamName
from Teams t
where t.teamId not in(
    select ts.teamId
    from TeamSponsors ts
    where ts.contractEnd < '2030-12-31'
    )
order by t.teamName;
-------------------------

create procedure versionUp_2_modifyTimeColumn
as
begin
    alter table Results
    alter column time decimal(10, 2);
end


create procedure versionDown_2_modifyTimeColumn
as
begin
    alter table Results
    alter column time int;
end

exec versionUp_2_modifyTimeColumn;
exec versionDown_2_modifyTimeColumn;

create procedure versionUp_3_addMoneyColumn
as
begin
    alter table Sponsors
    add money decimal(10, 2);
end

create procedure versionDown_3_addMoneyColumn
as
begin
    alter table Sponsors
    drop column money;
end

exec versionUp_3_addMoneyColumn;
exec versionDown_3_addMoneyColumn;


create procedure versionUp_4_addDefaultCondition
as
begin
    alter table Results
    add constraint defaultFinishStatus default 'Finished' for finishStatus;
end

create procedure versionDown_4_addDefaultCondition
as
begin
    alter table Results
    drop constraint defaultFinishStatus;
end

select * from Results;

insert into Results values (22, 1, 52, 5320, 25, NULL , 3, 1, 1);

alter table Results
alter column finishStatus varchar(255) null;


exec versionUp_4_addDefaultCondition;
exec versionDown_4_addDefaultCondition;


create procedure versionUp_5_changePKSponsors
as
begin
    declare @pk_name nvarchar(128);

    select @pk_name = constraint_name
    from INFORMATION_SCHEMA.TABLE_CONSTRAINTS
    where TABLE_NAME = 'TeamSponsors'
    and CONSTRAINT_TYPE = 'PRIMARY KEY';

    -- drop the existing PK using dynamic SQL
    declare @sql nvarchar(max);
    set @sql = 'alter table TeamSponsors drop constraint ' + @pk_name;
    exec sp_executesql @sql;

    alter table TeamSponsors
    add constraint PK_TeamSponsorsDate primary key (teamId, sponsorId, contractStart);
end;

create procedure versionDown_5_changePKSponsors
as
begin
    alter table TeamSponsors
    drop constraint PK_TeamSponsorsDate;

    alter table TeamSponsors
    add constraint PK_TeamSponsors primary key (teamId, sponsorId);
end;

drop procedure versionUp_5_changePKSponsors;
drop procedure versionDown_5_changePKSponsors;


exec versionUp_5_changePKSponsors;
exec versionDown_5_changePKSponsors;


select * from INFORMATION_SCHEMA.TABLE_CONSTRAINTS
where TABLE_NAME = 'TeamSponsors';

create table DatabaseVersion (
    version int primary key,
    description varchar(255) not null
);

insert into DatabaseVersion values (1, 'firrst version');

create procedure versionUp_6_addCandidateKey
as
begin
    alter table Races
    add constraint CK_UniqueRaceInSeason unique (season, round);
end;

create procedure versionDown_6_addCandidateKey
as
begin
    alter table Races
    drop constraint CK_UniqueRaceInSeason;
end;

exec versionUp_6_addCandidateKey;
exec versionDown_6_addCandidateKey;

create procedure versionUp_7_addForeignKey
as
begin
    alter table Races
    add hostingTeamId int null;

    alter table Races
    add constraint FK_Races_HostingTeam
    foreign key (hostingTeamId) references Teams(teamId);
end;

create procedure versionDown_7_addForeignKey
as
begin
    -- remove foreign key constraint
    alter table Races
    drop constraint FK_Races_HostingTeam;

    -- remove the column
    alter table Races
    drop column hostingTeamId;
end;

exec versionUp_7_addForeignKey;
exec versionDown_7_addForeignKey;


create procedure versionUp_8_createTable
as
begin
    create table DriverStandings (
        standingId int primary key,
        driverId int not null,
        points int not null,
        position int not null,
        season int not null,
        foreign key (driverId) references Drivers(driverId)
    )
end;

create procedure versionDown_8_dropTable
as
begin
    drop table DriverStandings;
end;

exec versionUp_8_createTable;
exec versionDown_8_dropTable;

create procedure UpdateDatabaseVersion
    @version int
as
begin
    declare @currentVersion int;
    select @currentVersion = version from DatabaseVersion;

    while @currentVersion < @version
    begin
        set @currentVersion = @currentVersion + 1;

        if @currentVersion = 2 exec versionUp_2_modifyTimeColumn;
        if @currentVersion = 3 exec versionUp_3_addMoneyColumn;
        if @currentVersion = 4 exec versionUp_4_addDefaultCondition;
        if @currentVersion = 5 exec versionUp_5_changePKSponsors;
        if @currentVersion = 6 exec versionUp_6_addCandidateKey;
        if @currentVersion = 7 exec versionUp_7_addForeignKey;
        if @currentVersion = 8 exec versionUp_8_createTable;

        insert into DatabaseVersion (version, description)
        values (@currentVersion,
            case @currentVersion
                when 2 then 'modified time column to decimal'
                when 3 then 'added money column to sponsors'
                when 4 then 'added default finish status '
                when 5 then 'modified teamsponsors primary key'
                when 6 then 'added unique race in season constraint'
                when 7 then 'added hosting team to races'
                when 8 then 'creaeted driver standings table'
            end);
    end;

    while @currentVersion > @version
    begin
        if @currentVersion = 8 exec versionDown_8_dropTable;
        if @currentVersion = 7 exec versionDown_7_addForeignKey;
        if @currentVersion = 6 exec versionDown_6_addCandidateKey;
        if @currentVersion = 5 exec versionDown_5_changePKSponsors;
        if @currentVersion = 4 exec versionDown_4_addDefaultCondition;
        if @currentVersion = 3 exec versionDown_3_addMoneyColumn;
        if @currentVersion = 2 exec versionDown_2_modifyTimeColumn;

        delete from DatabaseVersion where version = @currentVersion;
        set @currentVersion = @currentVersion - 1;
    end;
end;



exec UpdateDatabaseVersion 8;


