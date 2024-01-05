from repository import TaxiRepository
from services import TaxiService


class UI:
    def __init__(self) -> None:
        self._service = TaxiService(TaxiRepository())

    def _print_menu(self):
        print("1. Add a ride")
        print("2. Simulate a ride")
        print("0. Exit")

    def run(self):
        number_of_taxis = int(input("Number of taxis: "))

        self._service.generate_random_taxis(number_of_taxis)

        while True:
            self._print_menu()
            cmd = input(">> ")
            if cmd == "1":
                startx = int(input("Start x: "))
                starty = int(input("Start y: "))
                endx = int(input("End x: "))
                endy = int(input("End y: "))
                try:
                     self._service.ride(startx, starty, endx, endy)
                except ValueError as ve:
                    print(ve)
            elif cmd == "2":
                self._service.simulate()
            elif cmd == "0":
                break
            else:
                print("Invalid command")

            taxis = self._service.get_taxis()

            for taxi in taxis:
                print(taxi)