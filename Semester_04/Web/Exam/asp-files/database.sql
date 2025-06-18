
create table Users (
  id integer primary key autoincrement,
  username varchar(100),
  password varchar(100)
);

create table Files (
  id integer primary key autoincrement,
  userId integer,
  filename varchar(100),
  filepath varchar(100),
  size integer,
  foreign key (userId) references Users(id)
);

insert into Users (username, password) values ('admin', 'admin');
insert into Users (username, password) values ('user', 'user');

insert into Files (userId, filename, filepath, size) values (1, 'file1.txt', 'files/file1.txt', 100);
insert into Files (userId, filename, filepath, size) values (2, 'file2.txt', 'files/file2.txt', 200);
