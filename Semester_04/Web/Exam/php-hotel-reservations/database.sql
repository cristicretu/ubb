-- SQLite compatible database schema

create table User (
  id integer primary key autoincrement,
  username varchar(100),
  password integer
);

create table HotelRoom (
  id integer primary key autoincrement,
  roomNumber string,
  capacity integer,
  basePrice integer
);

create table Reservation (
  id integer primary key autoincrement,
  userId integer,
  roomId integer,
  checkInDate date,
  checkOutDate date,
  numberOfGuests integer,
  totalPrice integer,
  FOREIGN KEY (userId) REFERENCES User(id),
  FOREIGN KEY (roomId) REFERENCES HotelRoom(id)
);

insert into User (username, password) values ('cristi', 1), ('luca', 1), ('alex', 1);
insert into HotelRoom (roomNumber, capacity, basePrice) values ('101', 2, 100), ('102', 2, 100), ('103', 2, 100);

insert into Reservation (userId, roomId, checkInDate, checkOutDate, numberOfGuests, totalPrice) values (1, 1, '2025-06-17', '2025-06-18', 2, 200);
insert into Reservation (userId, roomId, checkInDate, checkOutDate, numberOfGuests, totalPrice) values (2, 2, '2025-06-17', '2025-06-25', 2, 200);
insert into Reservation (userId, roomId, checkInDate, checkOutDate, numberOfGuests, totalPrice) values (3, 2, '2025-06-27', '2025-06-29', 2, 200); 