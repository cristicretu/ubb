from constants import CHEAT_CODE, ORIENTATIONS
from services.game_service import GameService, RepositoryError
import termtables as tt


class UI:
    def __init__(self):
        self.game_service = GameService()
        self.is_human_player_cheating = False

    def print_board(board):
        labeled_board = [[str(number)] + row for number, row in enumerate(board)]
        board_header = [""] + [chr(ord('A') + letter) for letter in range(len(board[0]))]

        tt.print(labeled_board, header=board_header)

    def print_boards(self):
        print()
        print("Your board:")
        UI.print_board(self.game_service.get_board_representation(is_human_player=True))
        print()
        print()
        print("Computer board:")
        UI.print_board(self.game_service.get_board_representation(is_human_player=False, is_player_cheating=self.is_human_player_cheating))

    def place_inital_five_ships(self):
        print("Place your ships on the board.")
        print("You can place the following ships: ")


        for ship_name, ship_size in self.game_service.ship_types_and_sizes.items():
            print(f"{ship_name} ({ship_size})")

        for ship_name, ship_size in self.game_service.ship_types_and_sizes.items():
            needs_to_be_placed = True
            while needs_to_be_placed:
                try:
                    print()
                    print("Your board:")
                    UI.print_board(self.game_service.get_board_representation(is_human_player=True)) 
                    print()
                    print(f"Placing {ship_name} ({ship_size})")

                    is_ship_coordinate_good = False
                    while not is_ship_coordinate_good:
                        ship_starting_coordinates = input("Enter the start coordinate (e.g. A5): ").upper()
                        try:
                            ship_starting_coordinates = self.game_service.convert_coordinates_from_string_to_tuple(ship_starting_coordinates)
                            is_ship_coordinate_good = True
                        except RepositoryError as repository_error_message:
                            print(repository_error_message)


                    is_ship_orientation_good = False
                    while not is_ship_orientation_good:
                        ship_orientation = input("Enter the orientation (h/v): ")
                        if ship_orientation in ORIENTATIONS:
                            is_ship_orientation_good = True
                        else:
                            print("Invalid orientation. The orientation must be h or v.")

                        
                    self.game_service.place_ship_for_human_player(ship_name, ship_starting_coordinates, ship_orientation)
                    needs_to_be_placed = False
                except RepositoryError as repository_error_message:
                    print(repository_error_message) 

    def attack(self):
        is_attack_placed = False
        while not is_attack_placed:
            try:
                print()


                is_ship_coordinate_good = False
                while not is_ship_coordinate_good:
                        ship_coordinate_as_string = input("Enter the start coordinate (e.g. A5): ").upper()

                        if ship_coordinate_as_string == CHEAT_CODE:
                            self.is_human_player_cheating = True
                            continue
                        
                        try:
                            ship_coordinate_as_string = self.game_service.convert_coordinates_from_string_to_tuple(ship_coordinate_as_string)
                            is_ship_coordinate_good = True
                        except RepositoryError as repository_error_message:
                            print(repository_error_message)

                is_attack_a_hit, is_ship_sunk_after_attack = self.game_service.add_human_attack_against_computer(ship_coordinate_as_string)
                if is_attack_a_hit:
                    print("Hit!")
                    if is_ship_sunk_after_attack:
                        print("You sunk a ship!")
                else:
                    print("Miss!")

                is_attack_placed = True
            except RepositoryError as repository_error_message:
                print(repository_error_message)

    def run(self):
        # place human ships first
        self.place_inital_five_ships()
        # place computer ships
        self.game_service.init_game()

        game_is_running = True
        while game_is_running:
            self.print_boards()
            self.attack()
            if self.game_service.check_if_game_over():
                game_is_running = False

            self.game_service.computer_attack()
            if self.game_service.check_if_game_over():
                game_is_running = False

        self.print_boards()
        if self.game_service.player_repository.are_all_ships_sunk():
            print("You lost!")
        else:
            print("You won!")