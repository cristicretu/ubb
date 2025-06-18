-- SQLite compatible database schema

create table User (
  id integer primary key autoincrement,
  username varchar(100)
);

create table Product (
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

create table Order (
  id integer primary key autoincrement,
  userId integer,
  totalPrice decimal(10,2),
  foreign key (userId) references User(id)
);

create table OrderItem (
  id integer primary key autoincrement,
  orderId integer,
  productId integer,
    foreign key (orderId) references Orders(id),
      foreign key (productId) references Product(id)
);

insert into User (username) values ('admin', 'user');

insert into Product (name, price) values ('BOOK-Math', 10.5), ('TOY-Car', 4.9);