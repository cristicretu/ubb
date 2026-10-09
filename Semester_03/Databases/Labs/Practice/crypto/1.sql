
create table client (
    client_id int identity(1,1),
    name varchar(100) not null,
    constraint PK_C primary key (client_id )
)

create table Account (
    account_id int identity(1,1),
    client_id int not null,
    money int not null,
    constraint FK_CI foreign key (client_id) references client(client_id),
    constraint PK_A primary key (account_id)
)

create table Cryptos (
    id int identity(1,1),
    name varchar(50) not null,
    price int not null,
    constraint PK_CR primary key (id)
)

create table AccountCryptos (
    account_id int not null,
    crypto_id int not null,
    balance int not null default 0,
    constraint FK_AC foreign key(account_id) references Account(account_id),
    constraint FK_CFK foreign key(crypto_id) references Cryptos(id),
    constraint PK_AC primary key (account_id, crypto_id)
)

create table Transactions (
    transaction_id int identity(1,1),
    account_id int not null,
    crypto_id int not null,
    type varchar(4) not null,
    amount int not null,
    price int not null,
    constraint FK_ACCID foreign key(account_id) references Account(account_id),
    constraint FK_CCCID foreign key(crypto_id) references Cryptos(id),
    constraint PK_T primary key  (transaction_id)
)

create table Statz (
    id int identity(1,1),
    account_id int not null,
    buy_orders int not null default 0,
    sell_orders int not null default 0,
    total_ops as(buy_orders + sell_orders),
    money_left int,
    constraint FK_S foreign key(account_id) references Account(account_id),
    constraint PK_S primary key (id)
)
