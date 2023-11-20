flights = []


def add_flight(code: str, duration: int, departure: str, destination: str):
    if len(code) < 3 or len(departure) < 3 or len(destination) < 3:
        return "Invalid flight data"

    if duration < 20:
        return "Invalid flight duration"

    flights.append(
        {
            "code": code,
            "duration": duration,
            "departure": departure,
            "destination": destination,
        }
    )

    return flights


def get_flight(code: str):
    for flight in flights:
        if flight["code"] == code:
            return flight

    return "Flight not found"


def get_flights_by_destination(destination: str):
    return [flight for flight in flights if flight["destination"] == destination]


def modify_duration(code: str, duration: int):
    for flight in flights:
        if flight["code"] == code:
            flight["duration"] = duration
            return flights

    return "Flight not found"


def reroute_all_from_destination(destination: str, new_destination: str):
    """
    Reroutes all flights from a place to another

    :param destination: a string determining where the destination came from
    :param new_destination: where to reroute eerything.
    """
    for flight in flights:
        if flight["destination"] == destination:
            flight["destination"] = new_destination

    return flights


def sort_by_duration():
    flights = get_flights_by_destination("Cluj-Napoca")
    return sorted(flights, key=lambda flight: flight["duration"])


add_flight("WZ123", 120, "Cluj-Napoca", "Bucuresti")
add_flight("AA123", 120, "Cluj-Napoca", "Bucuresti")
add_flight("A443", 120, "Cluj-Napoca", "Bucuresti")

print(flights)

modify_duration("WZ123", 200)
modify_duration("AA123", 21)

print(flights)

reroute_all_from_destination("Bucuresti", "Timisoara")

print(flights)

print(sort_by_duration())

print(sort_by_duration("Bucuresti", "timisoara"))
