from service import ReservationService
import texttable


class UI:
    def __init__(self):
        self.service = ReservationService()

    def run(self):
        while True:
            print("1. Get rooms")
            print("2. Get reservations")
            print("3. Create a reservation")
            print("4. Exit")

            cmd = input("Command: ")

            self.service.generate_random_reservations(1000)

            if cmd == "1":
                start_date = input("Start date: ")
                end_date = input("End date: ")


                tables = self.service.get_reservations(start_date, end_date)

                for table in tables:
                    print(table.draw())