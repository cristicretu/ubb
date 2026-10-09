class Taxi:
    def __init__(self, id, x, y, fare):
        self.id = id
        self.x = x
        self.y = y
        self.fare = fare

    def __str__(self):
        return f"Taxi #{self.id} is at ({self.x}, {self.y}) with fare {self.fare}"