create table CarCategory (
    id int identity(1,1),
    description varchar(100) not null,
    constraint PK_CC primary key(id)
)

create table Car (
    id int identity(1,1),
    owner varchar(100) not null,
    yop int not null,
    color varchar(20) not null,
    category_id int not null,
    constraint FK_C foreign key (category_id) references CarCategory(id),
    constraint PK_CAR primary key (id)
)

create table Employee (
    id int identity(1,1),
    name varchar(100) not null,
    yob int not null,
    email varchar(30) not null,
    expertise varchar(100) not null,
    constraint PK_EMP primary key (id)
)

create table PartToFix (
    id int identity(1,1),
    description varchar(100) not null,
    duration time not null,
    constraint PK_PTF primary key(id)
)

create table Fixx (
    id int identity(1,1),
    car_id int not null,
    employee_id int not null,
    constraint FK_FC foreign key (car_id) references Car(id),
    constraint FK_EF foreign key (employee_id) references Employee(id),
    constraint PK_FX primary key (id)
)

create table FixxDetails (
    fix_id int not null,
    part_id int not null,
    repair_date date not null,
    constraint FK_FDF foreign key (fix_id) references Fixx(id),
    constraint FK_FPD foreign key (part_id) references PartToFix(id),
    constraint PK_FDX primary key (fix_id, part_id)
)
