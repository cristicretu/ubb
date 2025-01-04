create procedure AddShoeToShop
    @ShoeId varchar(255),
    @ShopName varchar(255),
    @ShoeQty int
as
    begin
declare
    @ShopId int;

    select @ShopId = id from PresentationShops where name = @ShopName

    if @ShopId is null
begin
    print 'shop not found'
    return
end
    if exists (select 1 from ShoePresentationShops where shoe_id = @ShoeId and shop_id = @ShopId)
        begin
            update ShoePresentationShops
            set quantity = quantity + @ShoeQty
            where shoe_id = @ShoeId and shop_id = @ShopId
        end
else
    begin
        insert into ShoePresentationShops (shoe_id, shop_id, quantity) values (@ShoeId, @ShopId, @ShoeQty)
    end
end

---



exec AddShoeToShop 1, 'Zara', 10

select * from ShoePresentationShops
