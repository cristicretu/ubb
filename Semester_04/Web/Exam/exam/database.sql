-- SQLite compatible database schema

create table Users (
  id integer primary key autoincrement,
  username varchar(100)
);

create table Products (
  id integer primary key autoincrement,
  name varchar(100),
  price decimal(10,2) 
);


create table Orders (
  id integer primary key autoincrement,
  userId integer,
  totalPrice decimal (10,2),
  foreign key (userId) references User(id)
);

create table OrderItems (
  id integer primary key autoincrement,
  orderId integer,
  productId integer,
    foreign key (orderId) references Orders(id),
      foreign key (productId) references Product(id)
);

insert into Users (username) values ('admin'), ('user');

insert into Products (name, price) values ('BOOK-History', 10.5), ('TOY-messi', 4.9);