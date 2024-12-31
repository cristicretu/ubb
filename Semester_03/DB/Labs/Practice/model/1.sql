create table TrainTypes (
    id int identity(1,1) primary key,
    description varchar(255) not null
)

create table Stations (
   id int identity(1,1) primary key,
    name varchar (255) unique not null
)

create table Trains (
    id int identity(1,1) primary key,
    name varchar (255) not null,
    type_id int foreign key references TrainTypes(id)
)

create table Routes (
    id int identity(1, 1) primary key,
    name varchar (255) unique not null,
    train_id int foreign key references Trains(id)
)

create table RouteStations (
    route_id int foreign key references Routes(id),
    station_id int foreign key references Stations(id),
    arrival_time time,
    departure_time time,
    primary key (route_id, station_id)
)

