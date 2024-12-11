create table Ta (
    aid int primary key,
    a2 int unique,
    name varchar(100),
);

create table Tb (
    bid int primary key,
    b2 int,
    age int
);

create table Tc (
    cid int primary key,
    aid int,
    bid int,
    description varchar(255),
    foreign key (aid) references Ta(aid),
    foreign key (bid) references Tb(bid)
);

insert into Ta (aid, a2, name) values
(1, 100, 'messi'),
(2, 200, 'ronaldo'),
(3, 300, 'neymar'),
(4, 400, 'costel'),
(5, 500, 'gigel');

create nonclustered index ix_ta_name on Ta(name);

select * from Ta;

select * from Ta where aid = 3;

select name from Ta;

select a2 from Ta where name = 'messi';

select aid, a2, name from Ta where name = 'ronaldo';

----

insert into Tb (bid, b2, age) values
(1, 10, 25),
(2, 20, 30),
(3, 30, 35),
(4, 40, 40),
(5, 50, 45);


select * from Tb where b2 = 30;

create nonclustered index ix_tb_b2 on Tb(b2);

select * from Tb where b2 = 30;


---


insert into Tc (cid, aid, bid, description) values
(1, 1, 1, 'macbook'),
(2, 2, 2, 'bookmac'),
(3, 3, 3, 'honda');

create view person_details as
select
    t1.name,
    t2.age,
    t3.description
from Ta t1
inner join Tc t3 on t1.aid = t3.aid
inner join Tb t2 on t3.bid = t2.bid;

select * from person_details;





