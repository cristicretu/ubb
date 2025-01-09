
insert into client (name) values
('alice'), ('bob'), ('charlie'), ('dave'), ('eve');

insert into Account (client_id, money) values
(1, 1000), (2, 2000), (3, 3000), (4, 4000), (5, 5000);

insert into Cryptos (name, price) values
('Bitcoin', 100000), ('Ethereum', 5000),  ('Dogecoin', 1)

insert into AccountCryptos (account_id, crypto_id, balance) values
(1, 1, 10), (1, 2, 20), (1, 3, 30),
(2, 1, 40),
 (3, 2, 80), (3, 3, 90),
(4, 1, 100),(4, 3, 120),
(5, 1, 130), (5, 2, 140), (5, 3, 150);

insert into Transactions (account_id, crypto_id, type, amount, price) values
(1, 1, 'buy', 10, 100000), (1, 2, 'buy', 20, 5000), (1, 3, 'buy', 30, 1),
(2, 1, 'buy', 40, 100000),
(3, 2, 'buy', 80, 5000), (3, 3, 'buy', 90, 1),
(4, 1, 'buy', 100, 100000), (4, 3, 'buy', 120, 1),
(5, 1, 'buy', 130, 100000), (5, 2, 'buy', 140, 5000), (5, 3, 'buy', 150, 1);

insert into Statz (account_id, buy_orders, sell_orders, money_left) values
(1, 10, 20, 1000), (2, 30, 40, 2000), (3, 50, 60, 3000), (4, 70, 80, 4000), (5, 90, 100, 5000);

