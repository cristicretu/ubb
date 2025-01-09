create function SRLAssets () returns table
as return
select C.id, L.location, sum(D.count) as NoASSETS from
client C
join AssetDetail D on C.id = D.client_id
join ClientSRL CS2 on C.id = CS2.client_id
join SRLs L on CS2.srl_id = L.id
group by C.id, CS2.srl_id, L.location

select * from SRLAssets()
