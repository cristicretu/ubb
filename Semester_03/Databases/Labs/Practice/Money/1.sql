create table ATM (
    id int identity(1,1),
    address varchar(255) not null,
    constraint PK_AT primary key (id)
)

create table Customer (
    id int identity(1,1),
    name varchar(100) not null,
    dob date not null,
    constraint PK_C primary key(id)
)


create table BankAccount (
    id int identity(1,1),
    IBAN varchar(34) not null,
    balance int not null,
    holder_id int,
    constraint PK_BA primary key(id),
    constraint FK_BA_C foreign key (holder_id) references Customer(id)
)


create table Card (
    id int identity(1,1),
    number varchar(16) not null,
    cvv varchar(3)  not null,
    account_id int,
    constraint PK_CA primary key (id),
    constraint FK_CC foreign key (account_id) references BankAccount(id)
)



create table Transactions (
    id int identity(1,1),
    card_id int,
    atm_id int,
    withdrawal int not null,
    transaction_datetime datetime not null,
    constraint PK_TRA primary key (id),
    constraint FK_TC foreign key (card_id) references Card(id),
    constraint FK_TA foreign key (atm_id) references ATM(id),
)

