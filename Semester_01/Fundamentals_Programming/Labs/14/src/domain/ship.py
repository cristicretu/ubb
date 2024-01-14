from constants import HORIZONTAL_INDEX, ORIENTATIONS, VERTICAL_INDEX


class Ship:
    def __init__(self, name: str, size: int):
        self.name = name
        self.size = size
        self.hits = 0
        self.coordinates = []

    def place_ship_coordinates(self, starting_coordinates: tuple[int, int], ship_orientation: str):
        """
        Add the coordinates of the ship to the ship's coordinates list

        :param starting_coordinates: the starting coordinates of the ship
        :param ship_orientation: the orientation of the ship

        :return: None
        """
        x, y = starting_coordinates
        if ship_orientation == ORIENTATIONS[HORIZONTAL_INDEX]:
            self.coordinates = [(x, y + i) for i in range(self.size)] 
        elif ship_orientation == ORIENTATIONS[VERTICAL_INDEX]:
            self.coordinates = [(x + i, y) for i in range(self.size)]

    
    def is_ship_hit_in_coordinate(self, coordinate_to_check: tuple[int, int]):
        """
        Check if the ship is hit in the given coordinate

        :param coordonate: the coordinate to check
        """
        if coordinate_to_check in self.coordinates:
            self.hits += 1
            return True
        return False
    
    def is_ship_sunk(self):
        """
        Check if the ship is sunk (all the coordinates are hit)
        """
        return self.hits == self.size
    
    def __str__(self):
        return f"{self.name} ({self.size}) with {self.hits} hits, placed at {self.coordinates}"
    