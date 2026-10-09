
insert into TaxCompany (name, no_clients, no_srls) values
('Google', 100, 10),
('Amazon', 1000, 5000),
('Apple', 10000, 1),
('Microsoft', 10000, 1000),
('LIDL', 100000, 100000)

insert into client (money, taxco_id) values
(4000, 1), (100, 1), (40000, 1), (500, 1),
(200, 2), (300, 2),
(12345, 3),
(400, 4), (500, 4), (2000, 4),
(10000, 5), (10000, 5)

insert into Asset (name, location) values
('aur', 'Cluj'),
('platina', 'Bucuresti'),
('diamant', 'Bucuresti'),
('apartament', 'Cluj'),
('masina', 'Iasi'),
('colac', 'Cluj'),
('macbook', 'Iasi')

insert into AssetDetail (client_id, asset_id, count) values
(1, 2, 100), (1, 4, 10), (1,6, 10),
(2, 1, 10), (2, 6, 5),
(3, 4, 100),
(4, 1, 5),

(5, 2, 100), (5,3,10),
(6, 2, 10),

(7, 5, 100),

(8, 6, 100), (8, 7, 50),
(9, 6, 50), (9, 7, 100),

(10, 1, 100)

insert into SRLs (name, activity, location) values
('Robite', 'hr', 'Cluj'),
('Garmin', 'comert', 'Cluj'),
('Porsche', 'masini', 'Iasi'),
('BMW', 'masini', 'Iasi'),
('OLX', 'apartamente', 'Cluj'),
('MIN', 'pretioase', 'Bucuresti')

insert into ClientSRL (client_id, srl_id) values
(1, 1), (2, 1), (3, 1), (4,1),
(5, 6), (6, 6),
(7, 3), (7, 4),
(8, 2), (9, 4),
(10, 6)
