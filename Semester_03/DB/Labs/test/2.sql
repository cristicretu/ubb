create procedure AssignFixToCar
(
    @CarID int,
    @PartFix int,
    @Employee varchar(100)
) as begin
    declare @EmployeeID int
    declare @FixxId int

    select @EmployeeID = id from Employee where name= @Employee

    if @EmployeeID is null begin
        print 'doesnt exist'
        return
    end

    insert into Fixx (car_id, employee_id) values (@CarID, @EmployeeID)

    select @FixxId = count(*) from Fixx; -- get the id of the fix

    insert into FixxDetails (fix_id, part_id, repair_date) values (@FixxId, @PartFix, getdate())
end


exec AssignFixToCar 2, 2, 'Ronaldo'


select * from Fixx;
select * from FixxDetails
