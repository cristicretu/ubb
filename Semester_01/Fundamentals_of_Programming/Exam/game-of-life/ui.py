from service import GameService


class UI:
    def __init__(self):
        self.service = GameService()

    def run(self):  
        while True:
            print(self.service.display_board())
            cmd = input('Enter command: ').strip().split()
            if cmd[0] == 'tick':
                if len(cmd) == 1:
                    self.service.tick()
                else:
                    self.service.tick_n_times(int(cmd[1]))
            elif cmd[0] == 'save':
                self.service.save_board()
            elif cmd[0] == 'load':
                self.service.load_board()
            elif cmd[0] == 'exit':
                break
            elif cmd[0] == "place":
                self.service.set_pattern(cmd[1], int(cmd[2].split(',')[0]), int(cmd[2].split(',')[1]))
            else:
                print('Invalid command')