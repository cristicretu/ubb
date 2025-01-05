create procedure DeleteTransactions
(
    @CardNumber varchar(16),
    @CardCVV varchar(3)
)
as
declare
    @CardID int

    select @CardID = id from Card where cvv = @CardCVV and number = @CardNumber

    if @CardID is null
    begin
        raiserror ('Card does not exist', 16,1)
        return
    end


begin
    delete from Transactions where card_id = @CardID
end

select * from Transactions
