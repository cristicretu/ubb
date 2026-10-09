from unittest import TestCase
import random
from constants import ALIEN, EMPTY, HIT

from service import GameError, GameService

class TestFire(TestCase):
    def setUp(self) -> None:
        self.service = GameService()

    def test_fire_miss(self):
        # try to lose the game (let the computer win)
        # by only attacking empty squares

        while not self.service.game_over:
            # get some empty squares

            while True:
                x = random.randint(0, self.service._board.size - 1)
                y = random.randint(0, self.service._board.size - 1)

                if self.service._board.matrix[x][y] != EMPTY:
                    continue

                self.assertEqual(self.service.fire_at_coordinate(x,y), False)
                self.assertEqual(self.service._board.matrix[x][y], HIT)
                break

            self.service.teleport_remaining_ships()
        
        # we should have lost the game, since the computer would have gone closer to the earth by now
        self.assertEqual(self.service.game_over, True)
        self.assertEqual(self.service.winner, "The Aliens")

    def test_fire_aliens(self):
        # try to attack the first alien ship
        x, y = self.service._board.aliens[0]
        self.assertEqual(self.service._board.matrix[x][y], ALIEN)
        self.assertEqual(self.service.fire_at_coordinate(x, y), True)
        self.assertEqual(self.service._board.matrix[x][y], HIT)

        # the previous alien should have been hit
        x, y = self.service._board.aliens[0]
        self.assertEqual(self.service._board.matrix[x][y], ALIEN)
        self.assertEqual(self.service.fire_at_coordinate(x,y), True)
        self.assertEqual(self.service._board.matrix[x][y], HIT)


        # we removed the aliens, we should have won the game
        self.assertEqual(self.service.game_over, True)
        self.assertEqual(self.service.winner, "Human Player")