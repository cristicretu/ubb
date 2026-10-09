class Room:
    def __init__(self, number, size):
        self.number = number
        self.size = size

class Reservation:
    def __init__(self, id, room_number, name, guests, start_date, end_date):
        self.id = id
        self.room_number = room_number
        self.name = name
        self.guests = guests
        self.start_date = start_date
        self.end_date = end_date