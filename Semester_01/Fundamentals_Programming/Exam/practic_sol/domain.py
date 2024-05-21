import texttable

from constants import ALIEN, ASTEROID, EARTH, EMPTY, HIT

class Board:
    def __init__(self, size = 7):
        self.matrix = [[EMPTY if i != size // 2 or j != size // 2 else EARTH for i in range(size)] for j in range(size)]
        self.size = size
        self.cheating = False
        self.aliens = []


    def __str__(self):
        tt = texttable.Texttable()

        header = [" "] + [x for x in "ABCDEFG"]
        tt.header(header)

        for i in range(self.size):
            row = [str(i + 1)] + [self.get_symbol_for_constant(x) for x in self.matrix[i]]
            tt.add_row(row)

        return tt.draw()

    def get_symbol_for_constant(self, integer):
        if integer == EMPTY:
            return " "
        elif integer == EARTH:
            return "E"
        elif integer == ASTEROID:
            return "*"
        elif integer == HIT:
            return "-"
        elif integer == ALIEN:
            if self.cheating:
                return "X"
            return " "
