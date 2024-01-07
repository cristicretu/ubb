from domain import Bus, BusRoute


class BusRepository:
    def __init__(self):
        self.buses = []
        self.load_data()

    def load_data(self):
        try:
            with open("buses.txt") as f:
                for line in f:
                    if line == "":
                        continue

                    id, code, model, times = line.split(",")
                    bus = Bus(id, code, model, times)

                    self.buses.append(bus)
        except IOError:
            print("Error loading data")

    def get_buses(self):
        return self.buses

    def get_bus_by_id(self, id):
        for bus in self.buses:
            if bus.id == id:
                return bus
                
        return None


class RouteRepository:
    def __init__(self):
        self.routes = []
        self.load_data()

    def load_data(self):
        try:
            with open("bus_routes.txt") as f:
                for line in f:
                    if line == "":
                        continue

                    code, length = line.split(",")
                    route = BusRoute(code, length)
                    self.routes.append(route)
        except IOError:
            print("Error loading data")

    def get_routes(self):
        return self.routes