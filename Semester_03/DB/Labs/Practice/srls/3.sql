create view ClientDetailz as
select c.id, c.money, SL.name, Sl.activity from
client c
join ClientSRL S on c.id = S.client_id
join SRLs SL on S.srl_id = SL.id

select * from ClientDetailz


