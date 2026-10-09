create table Company (
    id int identity(1,1),
    name varchar(100) not null,
    constraint PK_C primary key(id)
)

create table Actor (
    id int identity(1,1),
    name varchar(100) not null,
    ranking int not null,
    constraint PK_A primary key(id)
)

create table StageDirector (
    id int identity(1,1),
    name varchar(100) not null,
    awards int not null,
    constraint PK_SD primary key(id)
)

create table Movie (
    id int identity(1,1),
    name varchar(100) not null,
    release_date date not null,
    company_id int,
    director_id int,
    constraint PK_M primary key(id),
    constraint FK_C foreign key(company_id) references Company(id),
    constraint FK_D foreign key(director_id) references StageDirector(id),
)

create table CinemaProductions (
    id int identity(1,1),
    name varchar(100) not null,
    movie_id int,
    constraint PK_CP primary key(id),
    constraint FK_M foreign key(movie_id) references Movie(id),
)

create table ProductionActors (
    production_id int,
    actor_id int,
    entry datetime not null,
    constraint PK primary key(production_id, actor_id),
    constraint FK_P foreign key (production_id) references CinemaProductions(id),
    constraint FK_AC foreign key (actor_id) references Actor(id)
)

