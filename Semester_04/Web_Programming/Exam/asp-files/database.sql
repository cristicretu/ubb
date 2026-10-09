
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
insert into Files (userId, filename, filepath, size) values (2, 'file3.txt', 'files/file3.txt', 300);
insert into Files (userId, filename, filepath, size) values (2, 'file4.txt', 'files/file4.txt', 400);
insert into Files (userId, filename, filepath, size) values (2, 'file5.txt', 'files/file5.txt', 500);
insert into Files (userId, filename, filepath, size) values (2, 'file6.txt', 'files/file6.txt', 600);
insert into Files (userId, filename, filepath, size) values (2, 'file7.txt', 'files/file7.txt', 700);
insert into Files (userId, filename, filepath, size) values (2, 'file8.txt', 'files/file8.txt', 800);
insert into Files (userId, filename, filepath, size) values (2, 'file9.txt', 'files/file9.txt', 900);
insert into Files (userId, filename, filepath, size) values (2, 'file10.txt', 'files/file10.txt', 1000);
