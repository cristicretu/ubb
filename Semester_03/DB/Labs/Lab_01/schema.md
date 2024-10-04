# Schema

-- Teams
-- Drivers
-- Races
-- Circuits
-- Results

## Teams

*teamId* int primary key
*teamName* text not null
*country* text not null
*foundedYear* int not null
*constructorCode* text not null unique

## Drivers

*driverId* int primary key
*driverNumber* int not null unique
*firstName* text not null
*lastName* text not null
*country* text not null
**teamId** int not null

## Circuits

*circuitId* int primary key
*nam*e text not null unique
*location* text not null
*country* text not null
*length* int not null
*capacity* int not null

## Races

*raceId* int primary key
*name* text not null
*date* date not null
*round* int not null
*season* int not null
**circuitId** int not null

## Results

*resultId* int primary key
*position* int not null
*laps* int not null
*time* int not null
*points* not null
*finishStatus* text not null
**raceId** int not null
**driverId** int not null
**teamId** int not null
