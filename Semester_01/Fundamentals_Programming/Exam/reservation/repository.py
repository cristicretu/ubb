from domain import Reservation, Room


class ReservationRepository:
    def __init__(self):
        self.reservations = []
        self.rooms = []
        self.load_reservations()
        self.load_rooms()

    def load_rooms(self, filename="rooms.txt"):
        try:
            with open(filename, 'r') as file:
                for line in file:
                    number, size = line.strip().split(',')
                    self.add_room(Room(number, size))
        except FileNotFoundError:
            print(f"File {filename} not found")
    
    def load_reservations(self, filename="reservations.txt"):
        try:
            with open(filename, 'r') as file:
                for line in file:
                    id, room_number, name, guests, start_date, end_date = line.strip().split(',')
                    self.add_reservation(Reservation(id, room_number, name, guests, start_date, end_date))
        except FileNotFoundError:
            print(f"File {filename} not found")

    def generate_random_reservations(self, count=1000):
        import random
        from datetime import datetime, timedelta

        first_names = [
            "Gigel",
            "Ion",
            "Marcel",
            "Vasile",
            "Mihai",
            "George",
            "Irina",
            "Maria",
            "Elena",
            "Ioana",
            "Ana",
            "Dana",
            "Andreea",
            "Alina",
            "Cristina",
            "Raluca",
            "Larisa",
            "Adriana",
            "Mihaela",
            "Gabriela",
            "Daniela",
            "Doina",
            "Eugenia",
            "Florina",
            "Geta",
        ]

        last_names = [
            "Popescu",
            "Ionescu",
            "Popa",
            "Marin",
            "Dumitru",
            "Stan",
            "Stoica",
            "Stanciu",
            "Florea",
            "Florescu",
            "Stefan",
            "Stefanescu",
            "Gheorghe",
            "Georgescu",
            "Mihai",
            "Mihaila",
            "Mihail",
            "Ionita",
            "Ion",
            "Ionescu",
            "Irina",
            "Irina",
        ]

        for i in range(count):
            room = random.choice(self.rooms)
            name = random.choice(first_names) + " " + random.choice(last_names)
            guests = random.randint(1, int(room.size))
           # Generate a random start date
            month = random.randint(1, 12)
            # Adjust day range for February considering it could be a leap year
            day = random.randint(1, 29) if month == 2 else random.randint(1, 28)
            start_date = datetime(2024, month, day)

            # Convert start_date to string format after all calculations are done
            start_date_str = start_date.strftime("%d-%m")

            # Generate a random timedelta
            days_to_add = random.randint(1, 10)
            end_date = start_date + timedelta(days=days_to_add)

            # Convert end_date to string format
            end_date_str = end_date.strftime("%d-%m")
            self.add_reservation(Reservation(i, room.number, name, guests, start_date_str, end_date_str))
            self.save_reservations("reservations.txt")

    def save_reservations(self, filename):
        with open(filename, 'w') as file:
            for reservation in self.reservations:
                file.write(f"{reservation.id},{reservation.room_number},{reservation.name},{reservation.guests},{reservation.start_date},{reservation.end_date}\n")

    def add_room(self, room):
        self.rooms.append(room)

    def add_reservation(self, reservation):
        self.reservations.append(reservation)

    def get_reservations(self):
        return self.reservations
    
    def get_rooms(self):
        return self.rooms

    def get_reservation_by_id(self, id):
        for reservation in self.reservations:
            if reservation.id == id:
                return reservation
        return None

    def remove_reservation(self, reservation):
        self.reservations.remove(reservation)