import random
import unittest
from constants import LOWER_COORDINATE_BOUND, UPPER_COORDINATE_BOUND

from services.game_service import GameService


class TestShips(unittest.TestCase):
    def setUp(self):
        self.game_service = GameService()

    def test_if_computer_placed_ships(self):
        self.game_service.init_game()
        self.assertEqual(len(self.game_service.computer_repository.ships), 5)

    def test_place_ships_before_starting_the_game(self):
        for length, (ship_name, _) in enumerate(self.game_service.ship_types_and_sizes.items()):
                self.game_service.place_ship_for_human_player(ship_name, (length + 1, length + 4), 'h')

        self.assertEqual(len(self.game_service.player_repository.ships), 5)

    def test_computer_attacks(self):
        while self.game_service.player_repository.are_all_ships_sunk() is False:
            self.game_service.computer_attack()

        self.assertTrue(self.game_service.player_repository.are_all_ships_sunk())

    def test_attack_computer(self):
        for _ in range(100):
            while True or self.game_service.computer_repository.all_sunk():
                horizontal_coordinate = random.randint(LOWER_COORDINATE_BOUND, UPPER_COORDINATE_BOUND)
                vertical_coordinate = random.randint(LOWER_COORDINATE_BOUND, UPPER_COORDINATE_BOUND)

                if (horizontal_coordinate, vertical_coordinate) not in self.game_service.player_repository.attacks_positions:
                    break

                self.game_service.add_human_attack_against_computer((horizontal_coordinate, vertical_coordinate))


        self.assertTrue(self.game_service.computer_repository.are_all_ships_sunk())

            


