from domain import Board
import texttable


class GameService:
    def __init__(self):
        self.board = Board(7)
        self.game_over = False

    def move(self, times=1):
        self.board.snake.move_n_times(times)
        if self.board.check_collision():
            self.game_over = True
        if self.board.check_apple():
            self.board.snake.grow()

    def get_board(self):

        table = texttable.Texttable()
        table.set_cols_align(["c"] * self.board.size)
        table.set_cols_valign(["m"] * self.board.size)
        table.add_rows(self.board.get_board(), header=False)
        return table.draw()
    