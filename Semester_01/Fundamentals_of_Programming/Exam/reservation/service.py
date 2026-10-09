from repository import ReservationRepository
import texttable


class ReservationService:
    def __init__(self):
        self.repo = ReservationRepository()

    @staticmethod
    def get_month_name(no):
        months = {
            1: "January",
            2: "February",
            3: "March",
            4: "April",
            5: "May",
            6: "June",
            7: "July",
            8: "August",
            9: "September",
            10: "October",
            11: "November",
            12: "December"
        }
        return months[no]

    def generate_random_reservations(self, count):
        self.repo.generate_random_reservations(count)
    
    def get_rooms(self):
        return self.repo.get_rooms()

    def get_reservations(self, start_date, end_date):
        reser = self.repo.get_reservations()

        print(reser)

        reser = [r for r in reser if r.start_date >= start_date and r.end_date <= end_date]

        reser = sorted(reser, key=lambda r: (r.start_date, r.name))

        months = []
        tables = []
        current_month = reser[0].start_date.split("-")[1]
        table = texttable.Texttable()

        for r in reser:
            if r.start_date.split("-")[1] != current_month:
                header = [self.get_month_name(int(current_month)), "Name", "Guests"]
                table.header(header)

                for r in months:
                    table.add_row([f"{r.start_date} - {r.end_date}",r.name, f"{r.guests} persons"])

                tables.append(table)
                current_month = r.start_date.split("-")[1]
                months = []
            else:
                months.append(r)

        # table.add_rows([["Room", "Name", "Guests", "Start Date", "End Date"], [r.room_number, r.name, r.guests, r.start_date, r.end_date]])


        return tables

    def get_possible_rooms(self, start_date, end_date):
        reser = self.repo.get_reservations()
        rooms = self.repo.get_rooms()

        reser = [r for r in reser if r.start_date <= start_date and r.end_date >= end_date]

        for r in reser:
            rooms = [room for room in rooms if room.number != r.room_number]

        return rooms
    
    def add_reservation(self, id, room_number, name, guests, start_date, end_date):
        reser = self.repo.get_reservations()

        for r in reser:
            if r.room_number == room_number and r.start_date <= start_date and r.end_date >= end_date:
                raise ValueError("Room is already booked")
            
            if r.id == id:
                raise ValueError("Id already exists")
            
        if start_date > end_date:
            raise ValueError("Start date must be before end date")
        
        if guests < 1:
            raise ValueError("At least 1 guest must be present")
        
        rooms = self.repo.get_rooms()

        for r in rooms:
            if r.number == room_number:
                if r.size < guests:
                    raise ValueError("Room is too small for the number of guests")
                break

        self.repo.add_reservation(id, room_number, name, guests, start_date, end_date)

    def delete_reservation(self, id):
        reser = self.repo.get_reservations()

        for r in reser:
            if r.id == id:
                reser.remove(r)
                self.repo.save_reservations()
                return

        raise ValueError("Reservation not found")
    
    def delete_reservations_from_start_to_end_dates(self, start_date, end_date):
        reser = self.repo.get_reservations()

        for r in reser:
            if r.start_date >= start_date and r.end_date <= end_date or r.start_date >= start_date and r.end_date >= end_date or r.start_date <= start_date and r.end_date <= end_date:
                reser.remove(r)

        self.repo.save_reservations()

    
    
    
