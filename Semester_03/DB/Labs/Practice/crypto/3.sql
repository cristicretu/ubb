create view ClientStatz as
select C.client_id, S.total_ops from
client C
join Account A2 on C.client_id = A2.client_id
join Statz S on A2.account_id = S.account_id

select *
from ClientStatz;
