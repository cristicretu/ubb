import texttable

class Board:
    def __init__(self, size=8):
        self.board = [[0 for _ in range(8)] for _ in range(8)]

    def __str__(self):
        tt = texttable.Texttable()

        for row in self.board:
            tt.add_row(row)

        return tt.draw()