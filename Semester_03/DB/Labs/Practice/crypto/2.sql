create procedure GetClientDetails
(
    @ClientName varchar(100)
) as begin
   select distinct C2.name, (money - sum(T.amount * T.price)) as RemainingMoney from
    client C
    join Account A on C.client_id = A.client_id
    join Transactions T on A.account_id = T.account_id
    join Cryptos C2 on T.crypto_id = C2.id
    where C.name = @ClientName
    group by C2.name, money;
end

exec GetClientDetails 'alice'



 select distinct C2.name from
    client C
    join Account A on C.client_id = A.client_id
    join Transactions T on A.account_id = T.account_id
    join Cryptos C2 on T.crypto_id = C2.id
    where C.name = 'alice';

    select distinct C2.name, (money - sum(T.amount * T.price)) as RemainingMoney from
    client C
    join Account A on C.client_id = A.client_id
    join Transactions T on A.account_id = T.account_id
    join Cryptos C2 on T.crypto_id = C2.id
    where C.name = 'bob'
    group by C2.name, money;
