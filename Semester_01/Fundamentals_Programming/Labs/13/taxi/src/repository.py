class TaxiRepository:
    def __init__(self):
        self._taxis = []

    def add(self, taxi):
        self._taxis.append(taxi)

    def get_all(self):
        return self._taxis
    
    def sort_by_fare(self):
        return sorted(self._taxis, key=lambda taxi: taxi.fare, reverse=True)

    def modify_taxi(self, id, x, y, fare):
        for taxi in self._taxis:
            if taxi.id == id:
                taxi.x = x
                taxi.y = y
                taxi.fare = fare
                return
        raise ValueError("Taxi not found")