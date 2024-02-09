class Hammurabi:
    def __init__(self, year=1, starved=0, new_people=0, population=100, acres=1000, harvest=3, ate=200, land_price=20, grain_price=2800):
        self.year = year
        self.starved = starved
        self.new_people = new_people
        self.population = population
        self.acres = acres
        self.harvest = harvest
        self.ate = ate
        self.land_price = land_price
        self.grain_price = grain_price

    def __str__(self):
        return f"In Year: {self.year}, {self.starved} people starved.\n {self.new_people} people came to the city. \n City Population is {self.population} \n  City owns {self.acres} acres of land \n Harvest was {self.harvest} units per acre \n rats ate {self.ate} units. \n Land Price is {self.land_price} units \n\n Grain Price: {self.grain_price}"