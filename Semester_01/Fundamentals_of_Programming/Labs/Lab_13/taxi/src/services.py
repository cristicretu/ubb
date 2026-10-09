from domain import Taxi
import random


class TaxiService:
    def __init__(self, taxi_repo):
        self.taxi_repo = taxi_repo

    def manhattan_distance(self, x1, y1, x2, y2):
        return abs(x1 - x2) + abs(y1 - y2)
    
    def check_coords(self, x, y):
        return x >= 0 and x <= 100 and y >= 0 and y <= 100
    
    def add_taxi(self, id, x, y, fare):
        if not self.check_coords(x, y):
            raise ValueError("Coordinates out of bounds")
        taxi = Taxi(id, x, y, fare)
        self.taxi_repo.add(taxi)
        return taxi

    def get_taxis(self):
        return self.taxi_repo.get_all()
    
    def generate_random_taxis(self, length):
        for id in range(length):
            ok = False
            while not ok:
                x = random.randint(0, 100)
                y = random.randint(0, 100)
                ok = True
                for taxi in self.taxi_repo.get_all():
                    if self.manhattan_distance(taxi.x, taxi.y, x, y) < 5:
                        ok = False
                        break
                if ok:
                    self.add_taxi(id, x, y, 0)

    def sort_taxis_by_fare(self):
        return self.taxi_repo.sort_by_fare()

    def ride(self, startx, starty, endx, endy):
        if not self.check_coords(startx, starty) or not self.check_coords(endx, endy):
            raise ValueError("Coordinates out of bounds")
        self.get_taxis()
        closest_taxi = None
        for taxi in self.taxi_repo.get_all():
            # get the closest taxi
            if closest_taxi is None or self.manhattan_distance(taxi.x, taxi.y, startx, starty) < self.manhattan_distance(closest_taxi.x, closest_taxi.y, startx, starty):
                closest_taxi = taxi

        self.taxi_repo.modify_taxi(closest_taxi.id, endx, endy, closest_taxi.fare + self.manhattan_distance(startx, starty, endx, endy))

    def simulate(self):
        startx = random.randint(0, 45)
        starty = random.randint(0, 45)
        endx = random.randint(55, 100)
        endy = random.randint(55, 100)

        self.ride(startx, starty, endx, endy)
