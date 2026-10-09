from service import BusService


class UI:
    def __init__(self):
        self.bus_service = BusService()
    
    def run(self):
        while True:
            print("1. display all buses by code")
            print("2. compute kilometers for a bus")
            print("3. display buses routes sorted by mileage") 
            
            option = input("Option: ")
            
            if option == "1":
                code = input("Code: ")
                reti = self.bus_service.get_a_route_by_code(code)

                print(reti)
                for bus in reti:
                    print(bus)
            elif option == "2":
                id = input("Id: ")
                try:
                    kilometers, bus = self.bus_service.compute_kilometers(id)
                    print("Bus {} has traveled {} kilometers".format(bus.code, kilometers))
                except ValueError as ve:
                    print(ve)
            elif option == "3":
                routes = self.bus_service.sort_bus_routes()
                for route in routes:
                    print("Route {}, {} kilometers".format(route[0], route[1]))
                    for bus in route[2]:
                        print(bus)
            elif option == "x":
                break
            else:
                print("Invalid option")