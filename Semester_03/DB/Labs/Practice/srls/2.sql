create procedure ClientAssets (
    @ClientID int
) as begin
   select C.id as ID,
    (select sum(A.count)
     from AssetDetail A
     where A.client_id = @ClientID) as TotalAssets,
    (select count(*)
     from ClientSRL CS
     where CS.client_id = @ClientID) as TotalSRL
    from client C
    where C.id = @ClientID
end

exec ClientAssets 7

select C.id as ID, sum(AD.count) as TotalAssets from
client C
join AssetDetail AD on C.id = AD.client_id
where C.id = 1
group by C.id

select count(*) as TotalSRL
from client C
join ClientSRL CS on C.id = CS.client_id
where C.id =1
