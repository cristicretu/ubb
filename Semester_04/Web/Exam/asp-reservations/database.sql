-- SQLite compatible database schema

DROP TABLE IF EXISTS Project;
DROP TABLE IF EXISTS SoftwareDeveloper;
DROP TABLE IF EXISTS cars;
DROP TABLE IF EXISTS categories;


create table Flights (
  id integer primary key autoincrement,
  date varchar(100),
  destinationCity varchar(100),
  availableSeats int
);

create table Hotels (
  id integer primary key autoincrement,
  hotelName varchar(100),
  date varchar(100),
  city varchar(100),
  availableRooms int
);

create table Reservations (
  id integer primary key autoincrement,
  person varchar(100),
  type int, -- 1: flight, 2: hotel
  idReservedResource int
);


CREATE TABLE SoftwareDeveloper (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  name varchar(100),
  age int default 0,
  skills varchar(100)
);

CREATE TABLE Project (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  ProjectManagerID INTEGER,
  name VARCHAR(100),
  description VARCHAR(100),
  members VARCHAR(100),
  FOREIGN KEY (ProjectManagerID) REFERENCES SoftwareDeveloper(id)
);

INSERT INTO SoftwareDeveloper (name, age, skills) VALUES ('John Doe', 30, 'Java, Python, SQL');
INSERT INTO SoftwareDeveloper (name, age, skills) VALUES ('Jane Smith', 25, 'JavaScript, React, Node.js');
INSERT INTO SoftwareDeveloper (name, age, skills) VALUES ('Mike Johnson', 35, 'C#, .NET, SQL');
INSERT INTO SoftwareDeveloper (name, age, skills) VALUES ('Emily Brown', 28, 'Python, SQL, Machine Learning');
INSERT INTO SoftwareDeveloper (name, age, skills) VALUES ('David Wilson', 40, 'Java, Spring, Hibernate');
INSERT INTO SoftwareDeveloper (name, age, skills) VALUES ('Sarah Davis', 27, 'JavaScript, React, Node.js');
INSERT INTO SoftwareDeveloper (name, age, skills) VALUES ('Robert Miller', 32, 'C#, .NET, SQL');

INSERT INTO Project (ProjectManagerID, name, description, members) VALUES (1, 'Project 1', 'Description 1', 'Members 1');
INSERT INTO Project (ProjectManagerID, name, description, members) VALUES (2, 'Project 2', 'Description 2', 'Members 2');
INSERT INTO Project (ProjectManagerID, name, description, members) VALUES (3, 'Project 3', 'Description 3', 'Members 3');
INSERT INTO Project (ProjectManagerID, name, description, members) VALUES (4, 'Project 4', 'Description 4', 'Members 4');
INSERT INTO Project (ProjectManagerID, name, description, members) VALUES (5, 'Project 5', 'Description 5', 'Members 5');
INSERT INTO Project (ProjectManagerID, name, description, members) VALUES (6, 'Project 6', 'Description 6', 'Members 6');
INSERT INTO Project (ProjectManagerID, name, description, members) VALUES (7, 'Project 7', 'Description 7', 'Members 7');

CREATE TABLE categories (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  name VARCHAR(100) NOT NULL,
  description TEXT
);

CREATE TABLE cars (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  model VARCHAR(100) NOT NULL,
  engine_power VARCHAR(50) NOT NULL,
  fuel_type VARCHAR(50) NOT NULL,
  price DECIMAL(10,2) NOT NULL,
  color VARCHAR(50) NOT NULL,
  year INT NOT NULL,
  history TEXT,
  category_id INT NOT NULL,
  created_at DATETIME NOT NULL,
  features TEXT,
  FOREIGN KEY (category_id) REFERENCES categories(id)
);

INSERT INTO categories (name, description) VALUES
('Sedan', 'Four-door passenger cars'),
('SUV', 'Sport Utility Vehicles');

INSERT INTO cars (model, engine_power, fuel_type, price, color, year, history, category_id, created_at) VALUES
('Toyota Camry', '203 hp', 'Gasoline', 15000.00, 'Silver', 2018, 'Single owner', 1, datetime('now')),
('Honda CR-V', '190 hp', 'Gasoline', 18500.00, 'Blue', 2019, 'Two owners', 2, datetime('now'));