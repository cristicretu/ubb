class Board:
    def __init__(self, size):
        self.size = size
        self.board = [['.' for i in range(size)] for j in range(size)]
        self.apples = []
        self.snake = Snake(self.board, self.apples)
        self.game_over = False
        self.generate_10_apples()

    def get_board(self):
        # for texttable

        result = []

        for i in range(self.size):
            row = []
            for j in range(self.size):
                if (i, j) in self.snake.body:
                    row.append('X')
                elif (i, j) in self.apples:
                    row.append('A')
                else:
                    row.append('.')
            result.append(row)

        return result

    def generate_10_apples(self):
        for i in range(10):
            self.generate_apple()


    def check_collision(self):
        head = self.snake.body[0]
        if head[0] < 0 or head[0] >= self.size or head[1] < 0 or head[1] >= self.size:
            return True
        if head in self.snake.body[1:]:
            return True
        return False

    def generate_apple(self):
        from random import randint
        x = randint(0, self.size - 1)
        y = randint(0, self.size - 1)
        while (x, y) in self.snake.body or (x, y) in self.apples:
            x = randint(0, self.size - 1)
            y = randint(0, self.size - 1)
        self.apples.append((x, y))


    def check_apple(self):
        head = self.snake.body[0]
        if head in self.apples:
            self.apples.remove(head)
            return True
        return False

class Snake:
    def __init__(self, board, apples):
        self.board = board
        self.body = [(len(board)//2, len(board)//2)]
        self.direction = (1, 0)
        self.apples = apples

    def check_apple(self):
        head = self.body[0]
        if head in self.apples:
            return True
        return False

    def move(self):
        head = self.body[0]
        new_head = (head[0] + self.direction[0], head[1] + self.direction[1])
        self.body.insert(0, new_head)
       
        # if not self.check_apple():
        self.body.pop()

    def move_n_times(self, times):
        for i in range(times):
            self.move()

    def grow(self):
        tail = self.body[-1]
        self.body.append(tail)

    def change_direction(self, direction):
        if direction == "up":
            self.direction = (-1, 0)
        elif direction == "down":
            self.direction = (1, 0)
        elif direction == "left":
            self.direction = (0, -1)
        elif direction == "right":
            self.direction = (0, 1)

    