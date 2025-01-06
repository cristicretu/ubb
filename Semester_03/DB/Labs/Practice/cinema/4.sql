create function GetNewMovies (
    @P int
) returns table
as
return
select M.name from Movie M
join CinemaProductions C on M.id = C.movie_id
where M.release_date > '2024-01-01'
group by M.name
having count(*) > @p

select * from GetNewMovies(0)
