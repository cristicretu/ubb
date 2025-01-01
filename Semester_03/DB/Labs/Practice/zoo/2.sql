create procedure DeleteFoodQuotas
    @AnimalName varchar(255)
as
begin
    delete from AnimalFoods where animal_id = (SELECT id from Animal where name = @AnimalName)
end
