insert into Actor (name, ranking) values
('Robert', 4),
('Mihai', 24),
('Messi', 1),
('Ronaldo', 3),
('Gicu', 14),
('Micu', 11)

insert into Company (name) values
('Netflix'), ('Spotify'), ('YouTube'), ('Antena9'), ('AcasaTV')

insert into StageDirector (name, awards) values
('THE ROCK', 4),
('LECLERC', 42),
('TSUNAMI', 11),
('TRUMP', 7)

insert into Movie (name, release_date, company_id, director_id) values
('Barca King', '2024-12-11', 1, 3),  -- netflix <-> tsunami
('Zeus maximus', '2025-01-11', 1, 2),  -- netflix <->leclerc
('mundo deportivo', '2025-02-11', 1, 2),  -- netflix <->leclerc
('ankara messi', '2025-03-30', 1, 4),  -- netflix <->trump
('gool ronalo', '2024-03-23', 3, 1),  -- youtube <->the rock
('teapa cluj', '2022-10-14', 3, 2),  -- youtube <->leclerc
('THE DANIEL', '2024-04-01', 4, 2),  -- antena9 <->leclerc
('MAN ANT', '2025-08-01', 5, 3)  -- acasatv <->tsunami

insert into CinemaProductions (name, movie_id) values
('BIG BROTHERS', 1),
('BIG BROTHERS', 2),
('Mr Beast', 3),
('Mr Beast', 4),
('Rechinii', 5),
('Donbass', 6),
('Mr Beast', 7),
('Javilon', 8)

insert into ProductionActors (production_id, actor_id, entry) values
(1, 1, '2020-02-01 10:00:00'),
(1, 2, '2020-02-01 11:30:00'),
(1, 3, '2020-02-02 09:15:00'),
(2, 1, '2020-03-15 14:00:00'),
(2, 4, '2020-03-16 10:30:00'),
(3, 2, '2020-04-01 09:00:00'),
(3, 5, '2020-04-02 13:45:00'),
(4, 3, '2020-05-20 11:00:00'),
(4, 4, '2020-05-21 15:30:00'),
(5, 1, '2020-06-10 10:15:00'),
(5, 5, '2020-06-11 14:00:00'),
(6, 2, '2020-07-05 09:30:00'),
(6, 6, '2020-07-06 13:15:00'),
(7, 3, '2020-08-15 10:45:00'),
(7, 5, '2020-08-16 15:00:00'),
(8, 4, '2020-09-01 11:30:00'),
(8, 6, '2020-09-02 14:45:00')
