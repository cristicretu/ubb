
insert into Chef (name, gender, dob) values
('Gicu', 'male', '1990-12-12'),
('Maria', 'female', '1980-12-12'),
('Gina', 'female', '1970-12-12'),
('Dumitrescu', 'male', '1960-12-12'),
('Floring', 'male', '1950-12-12'),
('Scarlatescu', 'male', '1940-12-12')

insert into CakeType (name, description) values
('birthday', 'for birthdays'),
('wedding', 'for weddings'),
('chocolate', 'with choco'),
('fruit', 'with fruits'),
('cheesecake', 'gut essen')

insert into Cake (name, shape, weight, price, cake_type) values
('barthdai', 'round', 400, 200, 1),
('barthdai improved', 'square', 1000, 500, 1),
('veding', 'round', 4110, 4000, 2),
('nutella', 'square', 400, 200, 3),
('pading', 'round', 800, 2000, 3),
('fresh', 'round', 40, 20, 4)

insert into ChefSpecializations (chef_id, cake_id) values
(1, 1), (1, 2), (1, 3), (1,4), (1,5),
(2, 1), (2, 3),
(3, 3), (3, 4),
(4, 1), (3, 2),
(5, 1), (5, 2), (5, 3), (5,4), (5,5)

insert into Orders (order_date) values ('2020-01-01'), ('2021-01-01'), ('2022-01-01'), ('2023-01-01')

insert into OrderDetails (order_id, cake_id, quantity) values
(1,  1, 10), (1, 2, 5),
(2, 2, 100),
(3, 3, 40), (3, 2, 20),
(4, 4, 10)


