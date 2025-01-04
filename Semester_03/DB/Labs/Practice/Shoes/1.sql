create table PresentationShops (
    id int identity(1,1),
    name varchar(255),
    city varchar(255),
    constraint PK_PS primary key(id)
)

create table Womans (
    id int identity(1,1),
    name varchar(255),
    max_spend int,
    constraint PK_W primary key(id)
)

create table ShoeModel (
    id int identity(1,1),
    name varchar(255),
    season varchar(20),
    constraint PK_SM primary key(id)
)

create table Shoe (
    id int identity(1,1),
    price int,
    model_id int foreign key references ShoeModel(id),
    constraint PK_S primary key(id)
)


create table ShoePresentationShops (
    shoe_id int foreign key references Shoe(id),
    shop_id int foreign key references PresentationShops(id),
    constraint PK_SPS primary key (shoe_id, shop_id)
)

create table WomanShoes (
    shoe_id int foreign key references Shoe(id),
    woman_id int foreign key references Womans(id),
    constraint PK_WS primary key (shoe_id, woman_id)
)

----
