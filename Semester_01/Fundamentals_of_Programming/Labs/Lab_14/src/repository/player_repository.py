from domain.ship import Ship


class PlayerRepository:
    def __init__(self):
        self.ships = []
        self.attacks_positions = {}

    def add_ship_on_human_board(self, ship: Ship, starting_coordinates: tuple[int, int], orientation: str):
        """
        Adds a ship to the player's repository

        :param ship: the ship to be added
        :param starting_coordinates: the starting coordinates of the ship
        :param orientation: the orientation of the ship

        :return: None
        """
        ship.place_ship_coordinates(starting_coordinates, orientation)
        self.ships.append(ship)

    def get_ships(self):
        """
        Returns the ships of the player and the attacks positions
        """
        return self.ships, self.attacks_positions

    def add_attack_on_player_board(self, coordinates_to_hit: tuple[int, int]) -> tuple[bool, bool]:
        """
        Adds an attack to the player's repository

        :param coordinates_to_hit: the coordinates of the attack
        :return: True if the attack was successful, False otherwise and True if the ship was sunk, False otherwise
        """
        if coordinates_to_hit in self.attacks_positions:
            return False, False

        for ship in self.ships:
            if ship.is_ship_hit_in_coordinate(coordinates_to_hit):
                self.attacks_positions[coordinates_to_hit] = True

                return True, ship.is_ship_sunk()
            
        self.attacks_positions[coordinates_to_hit] = False
        return False, False
    
    def are_all_ships_sunk(self):
        """
        Checks if all the ships are sunk

        :return: True if all the ships are sunk, False otherwise
        """
        return all(ship.is_ship_sunk() for ship in self.ships)