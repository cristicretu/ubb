create table Chef (
    id int identity(1,1),
    name varchar(100) not null,
    gender varchar(10) not null,
    dob date not null,
    constraint PK_C primary key (id)
)

create table CakeType (
    id int identity(1,1),
    name varchar(100) not null,
    description varchar(255) not null,
    constraint PK_CT primary key (id)
)

create table Cake (
    id int identity(1,1),
    name varchar(100) not null,
    shape varchar(10) not null,
    weight int not null,
    price int not null,
    cake_type int,
    constraint FK_CK foreign key (cake_type) references CakeType(id),
    constraint PK_CA primary key (id)
)

create table ChefSpecializations (
    chef_id int,
    cake_id int,
    constraint FK_CID foreign key (chef_id) references Chef(id),
    constraint FK_CCKID foreign key (cake_id) references Cake(id),
    constraint PK_CS primary key (chef_id, cake_id)
)

create table Orders (
    id int identity(1,1),
    order_date date not null,
    constraint PK_O primary key (id)
)

create table OrderDetails (
    order_id int,
    cake_id int,
    quantity int not null,
    constraint FK_OD_O foreign key (order_id) references Orders(id),
    constraint FK_OD_C foreign key (cake_id) references Cake(id),
    constraint PK_OD primary key (order_id, cake_id)
)


