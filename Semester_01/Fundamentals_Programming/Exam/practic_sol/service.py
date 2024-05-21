from copy import deepcopy
from constants import ALIEN, ASTEROID, EARTH, EMPTY, HIT
from domain import Board
import random

class GameError(Exception):
    pass

class GameService:
    def __init__(self):
        self._board = Board()
        self.game_over = False
        self.place_random_asteroids()
        self.place_alien_ships()
        self.winner = None

    def place_symbol_on_board(self, x, y, symbol):
        self._board.matrix[x][y] = symbol

    def place_random_asteroids(self, amount = 8):
        counter = amount

        while counter > 0:
            while True:
                # try to get good coordinates (valid)
                x = random.randint(0, self._board.size - 1)
                y = random.randint(0, self._board.size - 1)

                if self._board.matrix[x][y] == EARTH or self._board.matrix[x][y] == ASTEROID:
                    continue

                # now check if it's overlapping any other asteroids
                dir_vector_horiz = [-1, -1, -1, 0, 1, 1, 1, 0]
                dir_vector_vert = [-1, 0, 1, 1, 1, 0, -1, -1]

                good = True

                for k in range(len(dir_vector_horiz)):
                    ni = x + dir_vector_horiz[k]
                    nj = y + dir_vector_vert[k]

                    if not (ni >= 0 and nj >=0 and ni < self._board.size and nj < self._board.size):
                        continue
                        # check for in bounds of the new coordinates

                    if self._board.matrix[ni][nj] == ASTEROID:
                        good = False

                if good == True:
                    self.place_symbol_on_board(x, y, ASTEROID)
                    break
                    
                continue

            counter -= 1  

    def display_board_table(self):
        return self._board  

    def place_alien_ships(self, amount = 2):
        for _ in range(amount):
            self.place_a_single_alien_ship()

    def place_a_single_alien_ship(self):
        choices = ["col", "row"]

        while True:
            # try to get good positions for alien ships
            # either last/first low or column
            last = random.choice(choices)

            if last == "col":
                x = random.randint(0, self._board.size - 1)
                y = self._board.size - 1
            else:
                y = random.randint(0, self._board.size - 1)
                x = self._board.size - 1

            # check if it does not overlap an asteroid
            # we already know that it should be in a good position (regarding the distance to the earth)
                
            if self._board.matrix[x][y] == ASTEROID or self._board.matrix[x][y] == ALIEN:
                continue

            # all good, let's place the alien
            self.place_symbol_on_board(x, y, ALIEN)
            self._board.aliens.append((x,y))
            break

    def toggle_cheating_game(self):
        self._board.cheating = not self._board.cheating 

    def check_if_game_over(self):
        """
        We first check if there are any aliens left on the board. If not => the game is over, and the user wins.

        Otherwise, we scan in the proximity of the Earth, up down left right and diagonally in order to see if there are any aliens nearby.
        If there are, the game is lost. Otherwise the game continues.
        """
        # first, let's check if there are any aliens left
        if len(self._board.aliens) == 0:
            self.game_over = True
            self.winner = "Human Player"
            return
        
        dir_vector_horiz = [-1, -1, -1, 0, 1, 1, 1, 0]
        dir_vector_vert = [-1, 0, 1, 1, 1, 0, -1, -1]

        x = self._board.size // 2
        y = x

        for k in range(len(dir_vector_horiz)):
            ni = x + dir_vector_horiz[k]
            nj = y + dir_vector_vert[k]

            if not (ni >= 0 and nj >=0 and ni < self._board.size and nj < self._board.size):
                continue
                # check for in bounds of the new coordinates

            if self._board.matrix[ni][nj] == ALIEN:
                self.game_over = True
                self.winner = "The Aliens"
                return

    def get_matrix_of_distances_from_earth(self):
        """
        Using Lee's algorithm, we start from the center of the matrix (the Earth) and go progressively through each square to get the distance from the earth.
        This is used in order to know where to teleport the ships at each step.
        """
        lee = [[0 for _ in range(self._board.size)] for _ in range(self._board.size)]

        queue = [] 
        board_size = self._board.size
        lee[board_size // 2][board_size // 2] = 1
        queue.append((board_size // 2, board_size // 2))

        dir_vector_horiz = [-1, -1, -1, 0, 1, 1, 1, 0]
        dir_vector_vert = [-1, 0, 1, 1, 1, 0, -1, -1] 

        while not len(queue) == 0:
            i,j = queue[0]
            queue.remove((i,j))

            for k in range(8):
                ni = i + dir_vector_horiz[k]
                nj = j + dir_vector_vert[k]

                if not (ni >= 0 and nj >=0 and ni < self._board.size and nj < self._board.size):
                    continue
                    # check for in bounds of the new coordinates

                if lee[ni][nj] == 0:
                    queue.append((ni, nj))
                    lee[ni][nj] = 1 + lee[i][j]

        return lee


    def teleport_remaining_ships(self):
        if self.game_over == True:
            return

        old_aliens = deepcopy(self._board.aliens)
        # make a deep copy in order to make sure we're not generating aliens continously
        # and that we work with the initial array of aliens (before any teleportations)

        for ship in old_aliens:
            x, y = ship[0], ship[1]

            choices = ["closer", "same"]
            result = random.choice(choices)

            # we need to get the distance from the earth to the aliens
            lee_matrix = self.get_matrix_of_distances_from_earth()

            nx = 0
            ny = 0
            current_distance = lee_matrix[x][y]
            # this distance corresponds to the two 50% cases:
            next_distance = current_distance if result == "closer" else current_distance - 1

            # we check to find the first good candidate for our chosen move
            while True:
                nx = random.randint(0, self._board.size - 1)
                ny = random.randint(0, self._board.size - 1)

                if self._board.matrix[nx][ny] != EMPTY or lee_matrix[nx][ny] != next_distance:
                    continue

                break

            for alien in self._board.aliens:
                if alien == (x, y):
                    self._board.aliens.remove(alien)
                    self._board.aliens.append((nx, ny)) 

            # clear the previous square
            self.place_symbol_on_board(x, y, EMPTY)
            # and mark the new one
            self.place_symbol_on_board(nx, ny, ALIEN)

            # if we're too close to the earth --> the game is over, and the player lost
            self.check_if_game_over()


    def get_coordinates_from_string(self, symbol):
        """
        We validate the coordinates that:
        - they are in bounds
        - they are in our corresponding format and nothing else: "B5" good, "ABCD4" - bad
        """
        if not isinstance(symbol, str):
            raise GameError("invalid coordinate")

        if len(symbol) != 2:
            raise GameError("Invalid coordinate")

        indexes = {}
        for i, x in enumerate("ABCDEFGHIJKLMNOPQRSTUVWXYZ"):
            indexes[x] = i

        try:
            x = indexes[symbol[0].upper()]
        except:
            raise GameError("Invalid Coorinate")

        try:
            y = int(symbol[1]) - 1
        except:
            raise GameError("Invalid coordinate")
        y = int(symbol[1]) - 1

        if not (x >= 0 and y >= 0 and x < self._board.size and y < self._board.size):
            raise GameError("Out of bounds")
        
        return y, x

    def fire_at_coordinate(self, x, y) -> bool:
        """
        This function attacks a given coordinate position eg 'G5'. 
        
        It checks if the position is within the bounds of the board, and if the fire is valid, i.e we don't have there neither an asteroid, earth, or that we already hit that position.

        If we hit an alien, we delete the alien from the game, and then check if the game was possibly won.

        :param coordinate: the string coordinate given by the user
        :return: a boolean if an alien ship was hit otherwise False -> either it was hit an alien ship, or it was a miss
        """
        if self._board.matrix[x][y] == ASTEROID or self._board.matrix[x][y] == HIT or self._board.matrix[x][y] == EARTH:
            raise GameError("You cannot attack this coordinate man!")
        
        
        if self._board.matrix[x][y] == ALIEN:
            for alien in self._board.aliens:
                if alien == (x,y):
                    self._board.aliens.remove(alien)

            self.check_if_game_over()

            self._board.matrix[x][y] = HIT
            return True
        
        self._board.matrix[x][y] = HIT
        return False

        
        
