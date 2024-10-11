# Schema

-- Teams
-- Drivers
-- Races
-- Circuits
-- Results
-- Penalties
-- Sponsors
-- Engines
-- TeamSponsors
-- TeamEngines

## Teams

_teamId_ int primary key
_teamName_ text not null
_country_ text not null
_foundedYear_ int not null
_constructorCode_ text not null unique

## Drivers

_driverId_ int primary key
_driverNumber_ int not null unique
_firstName_ text not null
_lastName_ text not null
_country_ text not null
**teamId** int not null

## Circuits

_circuitId_ int primary key
_name_ text not null unique
_location_ text not null
_country_ text not null
_length_ int not null
_capacity_ int not null

## Races

_raceId_ int primary key
_name_ text not null
_date_ date not null
_round_ int not null
_season_ int not null
**circuitId** int not null

## Results

_resultId_ int primary key
_position_ int not null
_laps_ int not null
_time_ int not null
_points_ not null
_finishStatus_ text not null
**raceId** int not null
**driverId** int not null
**teamId** int not null

## Sponsors

_sponsorId_ int primary key
_sponsorName_ text not null

## Engines

_engineId_ int primary key
_engineName_ text not null
_manufacturer_ text not null
_horsepower_ int not null

## TeamSponsors

_teamId_ int not null
_sponsorId_ int not null
_contractStart_ date not null
_contractEnd_ date not null

## TeamEngines

_teamId_ int not null
_engineId_ int not null

## Penalties

_penaltyId_ int primary key
_penaltyType_ text not null
_penaltyDescription_ text not null
_penaltyDroppedPositions_ int not null
**driverId** int not null
**raceId** int not null
