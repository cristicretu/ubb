create database if not exists test;

create table SoftwareDeveloper (
  id int identity(1,1) primary key,
  name varchar(100),
  age int default 0,
  skills varchar(100)
)

CREATE TABLE Project (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  ProjectManagerID INTEGER,
  name VARCHAR(100),
  description VARCHAR(100),
  members VARCHAR(100),
  FOREIGN KEY (ProjectManagerID) REFERENCES SoftwareDeveloper(id)
);

insert into SoftwareDeveloper (name, age, skills) values ('John Doe', 30, 'Java, Python, SQL');
insert into SoftwareDeveloper (name, age, skills) values ('Jane Smith', 25, 'JavaScript, React, Node.js');
insert into SoftwareDeveloper (name, age, skills) values ('Mike Johnson', 35, 'C#, .NET, SQL');
insert into SoftwareDeveloper (name, age, skills) values ('Emily Brown', 28, 'Python, SQL, Machine Learning');
insert into SoftwareDeveloper (name, age, skills) values ('David Wilson', 40, 'Java, Spring, Hibernate');
insert into SoftwareDeveloper (name, age, skills) values ('Sarah Davis', 27, 'JavaScript, React, Node.js');
insert into SoftwareDeveloper (name, age, skills) values ('Robert Miller', 32, 'C#, .NET, SQL');

insert into Project (ProjectManagerID, name, description, members) values (1, 'Project 1', 'Description 1', 'Members 1');
insert into Project (ProjectManagerID, name, description, members) values (2, 'Project 2', 'Description 2', 'Members 2');
insert into Project (ProjectManagerID, name, description, members) values (3, 'Project 3', 'Description 3', 'Members 3');
insert into Project (ProjectManagerID, name, description, members) values (4, 'Project 4', 'Description 4', 'Members 4');
insert into Project (ProjectManagerID, name, description, members) values (5, 'Project 5', 'Description 5', 'Members 5');
insert into Project (ProjectManagerID, name, description, members) values (6, 'Project 6', 'Description 6', 'Members 6');
insert into Project (ProjectManagerID, name, description, members) values (7, 'Project 7', 'Description 7', 'Members 7');

CREATE TABLE IF NOT EXISTS categories (
  id INT NOT NULL AUTO_INCREMENT,
  name VARCHAR(100) NOT NULL,
  description TEXT,
  PRIMARY KEY (id)
);

CREATE TABLE IF NOT EXISTS cars (
  id INT NOT NULL AUTO_INCREMENT,
  model VARCHAR(100) NOT NULL,
  engine_power VARCHAR(50) NOT NULL,
  fuel_type VARCHAR(50) NOT NULL,
  price DECIMAL(10,2) NOT NULL,
  color VARCHAR(50) NOT NULL,
  year INT NOT NULL,
  history TEXT,
  category_id INT NOT NULL,
  created_at DATETIME NOT NULL,
  PRIMARY KEY (id),
  FOREIGN KEY (category_id) REFERENCES categories(id)
);

INSERT INTO categories (name, description) VALUES
('Sedan', 'Four-door passenger cars'),
('SUV', 'Sport Utility Vehicles');

INSERT INTO cars (model, engine_power, fuel_type, price, color, year, history, category_id, created_at) VALUES
('Toyota Camry', '203 hp', 'Gasoline', 15000.00, 'Silver', 2018, 'Single owner', 1, NOW()),
('Honda CR-V', '190 hp', 'Gasoline', 18500.00, 'Blue', 2019, 'Two owners', 2, NOW()); 

alter table cars add column features TEXT;