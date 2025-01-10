insert into CarCategory (description) values
('muscle'), ('sport'), ('off-road'), ('SUV'), ('sedan'), ('hatchback')

insert into Car (owner, yop, color, category_id) values
('Mihai', 2020, 'red', '1'), ('Mihai', 2010, 'blue', '3'),
('Costel', 1990, 'black', '2'), ('Gigel', 2000, 'blue', '6'),
('Cristi', 2009, 'black', '6'), ('Luca', 2024, 'blue', '5'),
('Victor', 2010, 'red', '4'), ('Vasile', 2000, 'blue', '2')

insert into Employee (name, yob, email, expertise) values
('Messi', 1990, 'messi@.com', 'engines'),
('Ronaldo', 1980, 'ronaldo@.com', 'chasis'),
('Neymar', 1970, 'neymar@.com', 'wheels'),
('Leclerc', 1970, 'leclerc@.com', 'engines'),
('MIRCEA', 1969, 'mircea@.com', 'pistons')

insert into PartToFix (description, duration) values
('change engine block', '4:12'),
('change replace oil', '0:10'),
('change wheels', '1:10'),
('replace injectors', '0:40'),
('align chasis', '8:40'),
('add one more piston lol', '10:40')

insert into Fixx (car_id, employee_id) values
(3, 1), (4, 2), (5, 1), (6, 2), (7, 3), (8, 4)

insert into FixxDetails (fix_id, part_id, repair_date) values
(1, 1, '2020-01-01'), (1, 2, '2020-01-03'), (1, 4, '2020-01-15'), (1, 5, '2020-01-24'),
(2, 5, '2020-02-24'),
(3, 1, '2020-03-01'), (3, 2, '2020-03-02'),
(4, 5, '2020-04-05'), (4, 3, '2020-05-20'),
(5, 3, '2020-05-25'),
(6, 6, '2020-05-31')


insert into FixxDetails (fix_id, part_id, repair_date) values
         (5, 4, '2020-05-27')


