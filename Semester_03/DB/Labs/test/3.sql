create function GetFixedParts (@CarID int) returns table
as return
select description as Part from
PartToFix
join FixxDetails FD on PartToFix.id = FD.part_id
join Fixx F on FD.fix_id = F.id
where F.car_id = @CarID

select  * from GetFixedParts (3)

---
