-- Table Ta: Drivers table
create table drivers (
    driver_id int primary key,
    racing_number int unique,  
    name varchar(100)
);

-- Table Tb: Teams table
create table teams (
    team_id int primary key,
    team_number int, 
    budget_millions int
);

-- Table Tc: Race results table
create table race_results (
    result_id int primary key,
    driver_id int,
    team_id int,
    race_position varchar(255),
    foreign key (driver_id) references drivers(driver_id),
    foreign key (team_id) references teams(team_id)
);

-- Insert sample drivers
insert into drivers (driver_id, racing_number, name) values
(1, 44, 'lewis hamilton'),
(2, 1, 'max verstappen'),
(3, 16, 'charles leclerc'),
(4, 55, 'carlos sainz'),
(5, 63, 'george russell');

-- Create nonclustered index on driver name
create nonclustered index ix_drivers_name on drivers(name);

-- Insert sample teams
insert into teams (team_id, team_number, budget_millions) values
(1, 101, 145),
(2, 102, 150),
(3, 103, 140),
(4, 104, 135),
(5, 105, 130);

-- Create nonclustered index on team number for better performance
create nonclustered index ix_teams_number on teams(team_number);

-- Insert race results
insert into race_results (result_id, driver_id, team_id, race_position) values
(1, 1, 1, '1st place'),
(2, 2, 2, '2nd place'),
(3, 3, 3, '3rd place');

-- Create view joining drivers and results
create view race_details as
select 
    d.name as driver_name,
    t.budget_millions,
    r.race_position
from drivers d
inner join race_results r on d.driver_id = r.driver_id
inner join teams t on r.team_id = t.team_id;

-- Queries to demonstrate different index operations:

-- Clustered index scan
select * from drivers;

-- Clustered index seek
select * from drivers where driver_id = 3;

-- Nonclustered index scan
select name from drivers;

-- Nonclustered index seek
select racing_number from drivers where name = 'lewis hamilton';

-- Key lookup
select driver_id, racing_number, name from drivers where name = 'max verstappen';

-- Query to test index performance
select * from teams where team_number = 102; 