
create table TaxCompany (
    id int identity(1,1),
    name varchar(100) not null,
    no_clients int,
    no_srls int,
    constraint PK_TC primary key(id)
)

create table client (
    id int identity(1,1),
    money int not null,
    taxco_id int,
    constraint FK_CTC foreign key (taxco_id) references TaxCompany(id),
    constraint PK_C primary key(id)
)

create table Asset (
    id int identity(1,1),
    name varchar(100) not null,
    location varchar(100) not null,
    constraint PK_A primary key(id)
)

create table AssetDetail (
    client_id int not null,
    asset_id int not null,
    count int not null,
    constraint FK_AD foreign key (client_id) references client(id),
    constraint FK_ADA foreign key (asset_id) references Asset(id),
    constraint PK_AD primary key (asset_id, client_id)
)

create table ClientSRL (
    client_id int not null,
    srl_id int not null,
    constraint FK_CS foreign key (client_id) references client(id),
    constraint FK_SC foreign key (srl_id) references SRLs(id),
    constraint PK_CS primary key (client_id, srl_id)
)

create table SRLs (
    id int identity(1,1),
    name varchar(100) not null,
    activity varchar(100) not null,
    location varchar(100) not null,
    constraint PK_SRL primary key(id)
)
