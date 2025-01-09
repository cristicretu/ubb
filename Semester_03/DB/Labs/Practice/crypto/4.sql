select C.client_id, count(*) as TransactionsMade
from client C
join Account A3 on C.client_id = A3.client_id
join AccountCryptos AC on A3.account_id = AC.account_id
join Cryptos C3 on AC.crypto_id = C3.id
join Transactions T2 on A3.account_id = T2.account_id
where C3.name = 'Bitcoin' and C.client_id not in (
    select distinct C.client_id
    from client C
    join Account A on C.client_id = A.client_id
    join AccountCryptos AC on A.account_id = AC.account_id
    join Cryptos C2 on AC.crypto_id = C2.id
    where C2.name != 'Bitcoin'
)
group by C.client_id
