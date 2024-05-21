from service import GameError, GameService


class UI:
    def __init__(self):
        self.service = GameService()
    
    def run(self):
        print("Welcome to the Game!\n")
        while not self.service.game_over:
            print(self.service.display_board_table())
            
            command = input(">>> Enter your command: ").strip()

            command = command.split(" ")

            if len(command) == 1:
                if command[0] == "cheat":
                    self.service.toggle_cheating_game()
                elif command[0] == "exit":
                    print("Exiting!")
                    break
                else:
                    print("Command unrecognized. Please try again!")
            elif len(command) == 2 and command[0] == "fire":
                    try:
                        self.service.get_coordinates_from_string(command[1])
                    except GameError:
                        print("Invalid Coordinate. Please try another coordinate.")
                        continue

                    
                    x,y = self.service.get_coordinates_from_string(command[1])
                    message = ""
                    try:
                        message = "You hit an alien" if self.service.fire_at_coordinate(x, y) else "You miss"
                    except GameError:
                        print("You cannot attack this coordinate. PLease try again!")
                        continue

                    print(message)

                    # move the alien ships
                    self.service.teleport_remaining_ships()

            else:
                print("Command unrecognized. Please try again!")
            

        if self.service.winner == "Human Player":
            print("Congrats! you won!")
        else:
            print("You lost, loser!")

            
