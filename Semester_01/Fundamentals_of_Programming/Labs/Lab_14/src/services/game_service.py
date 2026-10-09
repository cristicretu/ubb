from constants import BOARD_SIZE, COLUMN_COORDINATE_INDEX, HORIZONTAL_INDEX, LENGTH_OF_COORDINATE, LOWER_COORDINATE_BOUND, ORIENTATIONS, ROW_COORDINATE_INDEX, SHIPS_NAME_AND_SIZES, UPPER_COORDINATE_BOUND, VERTICAL_INDEX
from domain.ship import Ship
from repository.player_repository import PlayerRepository
import random
import numpy as np

class RepositoryError(Exception):
    pass

class GameService:
    def __init__(self, player_repository = PlayerRepository(), computer_repository = PlayerRepository()):
        self.player_repository = player_repository
        self.computer_repository = computer_repository
        self.ship_types_and_sizes = SHIPS_NAME_AND_SIZES
        self.computer_heat_map_of_probabilities = np.full((BOARD_SIZE, BOARD_SIZE), fill_value='.')
        

    def is_valid_placement(self, repository, ship_size: int, start_coord: tuple, orientation: str):
        """
        Check if the placement of a ship is valid

        :param repository: the repository to check
        :param ship_size: the size of the ship
        :param start_coord: the starting coordinates of the ship
        :param orientation: the orientation of the ship

        :return: True if the placement is valid, False otherwise
        """
        starting_x_coordonate, starting_y_coordonate = start_coord
        if orientation == ORIENTATIONS[HORIZONTAL_INDEX]:
            if starting_y_coordonate + ship_size > BOARD_SIZE:
                return False
        elif orientation == ORIENTATIONS[VERTICAL_INDEX]:
            if starting_x_coordonate + ship_size > BOARD_SIZE:
                return False
            
        proposed_coordinates_for_placement = [(starting_x_coordonate, starting_y_coordonate + ship_point_size) for ship_point_size in range(ship_size)] if orientation == ORIENTATIONS[HORIZONTAL_INDEX] else [(starting_x_coordonate + ship_point_size, starting_y_coordonate) for ship_point_size in range(ship_size)]
        for ship in repository.ships:
            for coordonate in proposed_coordinates_for_placement:
                if coordonate in ship.coordinates:
                    # If the coordinates are already occupied by another ship
                    # then the placement is invalid
                    return False
                
        # we skipped the for loop, so the placement is valid
        return True
    
    def init_game(self):
        """
        Initialize the game upon starting. Place the computer's ships randomly on the board.
        """
        for ship_name, ship_size in self.ship_types_and_sizes.items():
            needs_to_be_placed = True
            while needs_to_be_placed:
                horizontal_coordinate = random.randint(LOWER_COORDINATE_BOUND, UPPER_COORDINATE_BOUND)
                vertical_coordinate = random.randint(LOWER_COORDINATE_BOUND, UPPER_COORDINATE_BOUND)
                orientation = random.choice(ORIENTATIONS)
                if self.is_valid_placement(self.computer_repository, ship_size, (horizontal_coordinate, vertical_coordinate), orientation):
                    ship = Ship(ship_name, ship_size)
                    self.computer_repository.add_ship_on_human_board(ship, (horizontal_coordinate, vertical_coordinate), orientation)
                    needs_to_be_placed = False

    def convert_coordinates_from_string_to_tuple(self, board_coordinate_as_string: str):
        """
        Converts coordinates like 'A5' to (0, 5)
        """
        if len(board_coordinate_as_string) != LENGTH_OF_COORDINATE:
            raise RepositoryError('Invalid coordinate. The coordinate must have 2 characters.')

        try:
            column_coordinate = ord(board_coordinate_as_string[COLUMN_COORDINATE_INDEX]) - ord('A')
            row_coordinate = int(board_coordinate_as_string[ROW_COORDINATE_INDEX])

            if column_coordinate < LOWER_COORDINATE_BOUND or column_coordinate >= BOARD_SIZE or row_coordinate < LOWER_COORDINATE_BOUND or row_coordinate >= BOARD_SIZE:
                raise RepositoryError("Invalid coordinates. Must be within the board range.")
            
            return row_coordinate, column_coordinate
        except ValueError:
            raise RepositoryError("Invalid coordinate format. Use the format 'LetterNumber', e.g., 'A5'.")


    def place_ship_for_human_player(self, ship_name: str, starting_coordinates: tuple[int,int], orientation: str):
        """
        Places a ship for the player if the placement is valid

        :param ship_name: the name of the ship
        :param starting_coordinates: the starting coordinates of the ship
        :param orientation: the orientation of the ship

        :return: None
        """
        if orientation not in ORIENTATIONS:
            raise RepositoryError('Invalid orientation. The orientation must be h or v.')

        if not self.is_valid_placement(self.player_repository, self.ship_types_and_sizes[ship_name], starting_coordinates, orientation):
            raise RepositoryError('Invalid move. A ship cannot be placed here.')

        selected_ship = Ship(ship_name, self.ship_types_and_sizes[ship_name])
        self.player_repository.add_ship_on_human_board(selected_ship, starting_coordinates, orientation)

    def add_human_attack_against_computer(self, coordinates_to_hit: tuple[int, int]):
        """
        Adds an attack to the computer's repository
        """
        # check if the attack is valid
        return self.computer_repository.add_attack_on_player_board(coordinates_to_hit)

    def generate_probability_map(self):
        """
        Generates a probability map for the computer's next attack in order to maximize the chance of hitting a ship.
        """

        # initialize the probability map
        self.computer_heat_map_of_probabilities = np.zeros((BOARD_SIZE, BOARD_SIZE))
        for _, ship_size in self.ship_types_and_sizes.items():
            for row in range(BOARD_SIZE):
                for column in range(BOARD_SIZE):
                    # Check if the ship can fit horizontally or vertically
                    # if so, increase the probability of those squares
                    if self.can_ship_fit_within_the_board(ship_size, row, column, ORIENTATIONS[HORIZONTAL_INDEX]):
                        for ship_unit in range(ship_size):
                            self.computer_heat_map_of_probabilities[row][column + ship_unit] += 1
                    if self.can_ship_fit_within_the_board(ship_size, row, column, ORIENTATIONS[VERTICAL_INDEX]):
                        for ship_unit in range(ship_size):
                            self.computer_heat_map_of_probabilities[row + ship_unit][column] += 1

        # increase probability of neighboring squares if a ship was hit
        for (row, column), _ in np.ndenumerate(self.computer_heat_map_of_probabilities):
            if self.is_coordinate_position_hit(row, column):
                self.update_adjacent_probabilities(row, column)

        # previously attacked squares should not be attacked again
        for coordinate, was_attacked in self.player_repository.attacks_positions.items():
            if was_attacked:
                self.computer_heat_map_of_probabilities[coordinate] = 0

    def can_ship_fit_within_the_board(self, ship_size: int, row: int, col: int, orientation: str):
        """
        Checks if a ship can fit within the board, and was not previously attacked (and missed)
        """
        if orientation == ORIENTATIONS[HORIZONTAL_INDEX] and col + ship_size <= BOARD_SIZE:
            return not any(self.is_coordinate_position_missed(row, col + size) for size in range(ship_size))
        if orientation == ORIENTATIONS[VERTICAL_INDEX] and row + ship_size <= BOARD_SIZE:
            return not any(self.is_coordinate_position_missed(row + size, col) for size in range(ship_size))
        return False

    def is_coordinate_position_hit(self, row, column):
        """
        Checks if a player coordinate was hit (used for the probability map by the computer)
        """
        return self.player_repository.attacks_positions.get((row, column), False)

    def is_coordinate_position_missed(self, row, column):
        """
        Checks if a player coordinate was missed (used for the probability map by the computer)
        """
        return (row, column) in self.player_repository.attacks_positions and not self.player_repository.attacks_positions[(row, column)]

    def update_adjacent_probabilities(self, initial_row, initial_column):
        """
        Updates the probability of the adjacent squares after a ship was hit
        """

        # Go through the direction vectors in the 4 directions, and manually increase the probability of those squares
        for direction_vector_horizontal, direction_vector_vertical in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            new_row, new_col = initial_row + direction_vector_horizontal, initial_column + direction_vector_vertical
            if 0 <= new_row < BOARD_SIZE and 0 <= new_col < BOARD_SIZE:
                self.computer_heat_map_of_probabilities[new_row][new_col] *= 2

    def computer_attack(self):
        """
        Use the probability map to attack the human player, get the best square to attack, attack, then update the probability map.
        """

        # generate the probability map
        self.generate_probability_map()
        # get the maximum probability square
        target_coordinate_for_hit = np.unravel_index(self.computer_heat_map_of_probabilities.argmax(), self.computer_heat_map_of_probabilities.shape)

        # attack the human player, see if the ship was hit or sunk
        is_human_ship_hit, is_human_ship_sunk = self.player_repository.add_attack_on_player_board(target_coordinate_for_hit)

        # update the probability map
        if is_human_ship_hit:
            # prevent the computer from attacking the same square again
            self.computer_heat_map_of_probabilities[target_coordinate_for_hit] = 0
        return is_human_ship_hit, is_human_ship_sunk

    def check_if_game_over(self):
        """
        Check if the game is over: either the human player or the computer won by sinking all the ships
        """
        return self.player_repository.are_all_ships_sunk() or self.computer_repository.are_all_ships_sunk()

    def get_board_representation(self, is_human_player: bool, is_player_cheating=False):
        """
        Get the board representation for the given player (output is a matrix). If the player is human, the ships will be shown. 
        If the player is the computer, the ships will not be shown unless the player is cheating.

        :param is_human_player: True if the player is human, False otherwise
        :param is_player_cheating: True if the human player is cheating, False otherwise

        :return: the board representation
        """
        if is_human_player:
            ships, attacks = self.player_repository.get_ships()
        else:
            ships, attacks = self.computer_repository.get_ships()

        battleship_board = [["." for _ in range(BOARD_SIZE)] for _ in range(BOARD_SIZE)]

        if is_human_player or is_player_cheating:
            for ship in ships:
                for coordinate in ship.coordinates:
                    horizontal_coordinate, vertical_coordiate = coordinate
                    battleship_board[horizontal_coordinate][vertical_coordiate] = "S"

        for coordinate_attack in attacks:
            horizontal_coordinate, vertical_coordiate = coordinate_attack
            if attacks[coordinate_attack]:  # Hit
                battleship_board[horizontal_coordinate][vertical_coordiate] = "X"
            else:  # Miss
                battleship_board[horizontal_coordinate][vertical_coordiate] = "O"

        return battleship_board
    