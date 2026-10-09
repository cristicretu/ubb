create procedure InsertOrder
(
    @OrderId int,
    @CakeName varchar(100),
    @P int
) as
begin
    declare @CakeId int

    select @CakeId = id from Cake where name = @CakeName

    if @CakeId is null begin
        print 'cake does nto exist'
        return
    end

    if (select 1 from OrderDetails where cake_id = @CakeId and order_id = @OrderId) is null begin
        insert into OrderDetails (order_id, cake_id, quantity) values (@OrderId, @CakeId, @P)
        return
    end

    update OrderDetails set quantity = @P where order_id = @OrderId and cake_id = @CakeId
end

select * from OrderDetails

exec InsertOrder 4, 'fresh', 100
