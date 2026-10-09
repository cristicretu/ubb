from service import GameService


class UI:
    def __init__(self):
        self.service = GameService()
        self.game_over = False

    def show_board(self):
        print(self.service.get_board())

    def run(self):
        while not self.game_over:
            self.show_board()
            
            command = input('Enter command: ')
           
            command = command.split(' ')

            if command[0] == 'move':
                if len(command) == 1:
                    self.service.move()
                else:
                    self.service.move(int(command[1]))

            if command[0] == 'exit':
                self.game_over = True
                break

            if command[0] == 'up':
                self.service.board.snake.change_direction('up')
            elif command[0] == 'down':
                self.service.board.snake.change_direction('down')
            elif command[0] == 'left':
                self.service.board.snake.change_direction('left')
            elif command[0] == 'right':
                self.service.board.snake.change_direction('right')

            self.game_over = self.service.game_over
        print('Game over!')