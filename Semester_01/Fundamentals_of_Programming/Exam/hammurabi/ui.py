from service import HammurabiService


class UI:
    def __init__(self):
        self.game_service=HammurabiService()

    def run(self):
        while not self.game_service.game_over:
            print(self.game_service.hammurabi)
            print()
            print()
            try:
                self.game_service.buy_or_sell_acres(int(input("How many acres will you buy or sell? ")))
                self.game_service.feed_people(int(input("How many bushels of grain will you feed to the people? ")))
                self.game_service.plant_acres(int(input("How many acres will you plant with seed? ")))
            except ValueError as ve:
                print(ve)
                continue
        print("Game Over")