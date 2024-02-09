import random
from domain import Hammurabi


class HammurabiService:
    def __init__(self):
        self.hammurabi = Hammurabi()
        self.game_over = False

    def buy_or_sell_acres(self, acres):
        # you cannot buy more land that you have grain for

        if acres < 0 and abs(acres) > self.hammurabi.acres:
            raise ValueError("You cannot sell more land than you have")
        
        if acres < 0:
            self.hammurabi.acres += acres
            self.hammurabi.grain_price += acres * self.hammurabi.land_price
            return

        if acres * self.hammurabi.land_price > self.hammurabi.grain_price:
            raise ValueError("You cannot buy more land than you have grain for")
        
        self.hammurabi.acres += acres
        self.hammurabi.grain_price -= acres * self.hammurabi.land_price

    def feed_people(self, grain):
        if grain > self.hammurabi.grain_price:
            raise ValueError("You cannot feed more people than you have grain for")
        
        self.hammurabi.grain_price -= grain
        ate = grain / 20
        
        self.hammurabi.starved = self.hammurabi.population - ate

        if self.hammurabi.starved == 0:
            self.hammurabi.new_people = random.randint(0,10)
            self.hammurabi.population += self.hammurabi.new_people
        else:
            self.hammurabi.new_people = 0
            self.hammurabi.population -= self.hammurabi.starved

        if self.hammurabi.population <= 0:
            self.game_over = True

    def plant_acres(self, acres):
        if acres > self.hammurabi.acres:
            raise ValueError("You cannot plant more acres than you have")
        
        self.hammurabi.grain_price -= acres

        self.hammurabi.harvest = random.randint(1, 6)
        self.hammurabi.grain_price += self.hammurabi.harvest * acres

        rats_infestation = random.randint(1, 10)
        if rats_infestation <= 2:
            self.hammurabi.ate = self.hammurabi.grain_price / 10
            self.hammurabi.grain_price -= self.hammurabi.ate

        self.hammurabi.year += 1

        if self.hammurabi.year == 5:
            self.game_over = True