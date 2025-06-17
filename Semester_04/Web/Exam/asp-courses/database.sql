-- SQLite compatible database schema

create table Persons (
  id integer primary key autoincrement,
  name varchar(255),
  role varchar(255)
);

create table Courses (
  id integer primary key autoincrement,
  professorId integer,
  courseName varchar(255),
  participants varchar(255),
  grades varchar(255),
  foreign key (professorId) references Persons(id)
);

insert into Persons (name, role) values ('John Doe', 'Professor');
insert into Persons (name, role) values ('Jane Smith', 'Student');
insert into Persons (name, role) values ('Jim Beam', 'Student');

insert into Courses (professorId, courseName, participants, grades) values (1, 'Introduction to Computer Science', 'John Doe, Jane Smith, Jim Beam', 'A, B, C');
insert into Courses (professorId, courseName, participants, grades) values (1, 'Introduction to Computer Science', 'John Doe, Jane Smith, Jim Beam', 'A, B, C');