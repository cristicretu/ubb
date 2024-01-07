from repository import BusRepository, RouteRepository


class BusService:
    def __init__(self):
        self.bus_repository = BusRepository()
        self.route_repository = RouteRepository() 

    def get_a_route_by_code(self, code):
        buses = self.bus_repository.get_buses()

        reti = []
        
        for bus in buses:
            if bus.code == code:
                reti.append(bus)

        return reti
    
    def compute_kilometers(self, id):
        bus = self.bus_repository.get_bus_by_id(id)

        if bus is None:
            raise ValueError("Invalid bus id")
        
        times = bus.times

        routes = self.route_repository.get_routes()

        for route in routes:
            if route.code == bus.code:
                kilometers = int(route.length) * int(times)
                return kilometers, bus

    def sort_bus_routes(self):
        buses = self.bus_repository.get_buses()
        routes = self.route_repository.get_routes()

        mileage = []

        for route in routes:
            kilometers = 0
            for bus in buses:
                if route.code == bus.code:
                    kilometers += int(route.length) * int(bus.times)
            mileage.append((route.code, kilometers))

        mileage.sort(key=lambda x: x[1], reverse=True)

        toret = []

        # return routes sorted by mileage, and for each route display the buses that travel on that route
        for route in mileage:
            toret.append((route[0], route[1], [bus for bus in buses if bus.code == route[0]]))

        return toret

        
        
