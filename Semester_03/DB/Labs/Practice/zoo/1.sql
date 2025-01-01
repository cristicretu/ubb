create table Zoos (
    id int identity(1,1) primary key,
    administrator varchar(255) not null,
    name varchar(255) not null
)

create table Animal (
    id int identity(1,1) primary key,
    name varchar(255) not null,
    dob date
)

create table Food (
    id int identity(1,1) primary key,
    name varchar(255) not null
)

create table Visitor (
    id int identity(1,1) primary key,
    age int not null,
    name varchar(255) not null
)

create table ZooAnimals (
    zoo_id int foreign key references Zoos(id),
    animal_id int foreign key references Animal(id),
    primary key (zoo_id, animal_id)
)

create table AnimalFoods (
    animal_id int foreign key references Animal(id),
    food_id int foreign key references Food(id),
    daily_quota int not null
    primary key (animal_id, food_id)
)

create table VisitorVisits (
    id int identity(1,1) primary key ,
    visitor_id int foreign key references Visitor(id),
    zoo_id int foreign key references Zoos(id),
    price decimal(10,2) not null,
    visit_date date not null
)


