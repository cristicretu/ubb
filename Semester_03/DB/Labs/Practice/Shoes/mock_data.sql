INSERT INTO PresentationShops (name, city) VALUES
('Foot Locker', 'New York'),
('Zara', 'Los Angeles'),
('Nordstrom', 'Chicago');

INSERT INTO Womans (name, max_spend) VALUES
('Alice', 500),
('Bobbie', 300),
('Charlie', 700);

INSERT INTO ShoeModel (name, season) VALUES
('Summer Breeze', 'Summer'),
('Winter Warmers', 'Winter'),
('Spring Flings', 'Spring');

INSERT INTO Shoe (price, model_id) VALUES
(100, 1), -- Summer Breeze
(150, 1),
(200, 2), -- Winter Warmers
(250, 2),
(120, 3), -- Spring Flings
(180, 3);

INSERT INTO ShoePresentationShops (shoe_id, shop_id) VALUES
(1, 1), -- Shoe 1 in Foot Locker
(2, 1),
(3, 2), -- Shoe 3 in Zara
(4, 2),
(5, 3), -- Shoe 5 in Nordstrom
(6, 3);

INSERT INTO WomanShoes (shoe_id, woman_id) VALUES
(1, 1), -- Alice bought Shoe 1
(2, 1),
(3, 2), -- Bobbie bought Shoe 3
(4, 2),
(5, 3), -- Charlie bought Shoe 5
(6, 3);
