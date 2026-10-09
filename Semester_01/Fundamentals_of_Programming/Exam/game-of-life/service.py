from copy import deepcopy
from domain import Board


class GameService:
    def __init__(self):
        self._board = Board()
        self.patterns = {
            'block': 0,
            'tub': 1,
            'blinker': 2,
            'beacon': 3,
            'spaceship': 4,
        }

    def display_board(self):
        return str(self._board)
    
    def save_board(self):
        with open('save.txt', 'w') as f:
            for row in self._board.board:
                f.write(' '.join(map(str, row)) + '\n')

    def load_board(self):
        with open('save.txt', 'r') as f:
            self._board.board = [[int(x) for x in line.split()] for line in f]

    def get_pattern(self, pattern):
        patternslist = []

        curr = []
        with open('patterns.txt', 'r') as f:
            for line in f:
                if line == "---\n":
                    patternslist.append(curr)
                    curr = []
                else:
                    for el in line.split():
                        curr.append(el)


        return patternslist[self.patterns[pattern]]
                

    def set_pattern(self, pattern, x, y):
        p = self.get_pattern(pattern)

        board2 = deepcopy(self._board.board)

        for i in range(len(p)):
            for j in range(len(p[0])):
                if 0 <= x + i < 8 and 0 <= y + j < 8:
                    board2[x + i][y + j] += int(p[i][j])
                    
                    if board2[x + i][y + j] == 2:
                        return False
                else:
                    return False
                
        self._board.board = board2

    def tick(self):
        board2 = [[0 for _ in range(8)] for _ in range(8)]

        dx = [-1, -1, -1, 0, 0, 1, 1, 1]
        dy = [-1, 0, 1, -1, 1, -1, 0, 1]

        for i in range(8):
            for j in range(8):
                neighbours = 0

                for k in range(8):
                    ni = i + dx[k]
                    nj = j + dy[k]

                    if 0 <= ni < 8 and 0 <= nj < 8 and self._board.board[ni][nj] == 1:
                        neighbours += 1

                if self._board.board[i][j] == 1:
                    # live cell
                    if neighbours < 2 or neighbours > 3:
                        board2[i][j] = 0
                    else:
                        board2[i][j] = 1
                else:
                    # dead cell
                    if neighbours == 3:
                        board2[i][j] = 1
                    else:
                        board2[i][j] = 0

        self._board.board = board2

    def tick_n_times(self, n=1):
        for _ in range(n):
            self.tick()