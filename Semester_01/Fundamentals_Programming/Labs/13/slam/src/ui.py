from repository import PlayerRepository
from service import TenisService


class UI:
    def __init__(self):
        self.tenis_service = TenisService(PlayerRepository("input.txt"))

    def run(self):
        players = self.tenis_service.get_all_players()

        for player in players:
            print(player)

        qual = self.tenis_service.is_qualifying_round_needed()


        if not qual:
            print("No qualifying round needed")

        while True:
            round = self.tenis_service.get_round()

            if round is None:
                qual = False
                if self.tenis_service.players == 1:
                    print(f"Winner is {self.tenis_service.get_all_players()[0]}")
                    break
                else:
                    self.tenis_service.clear_tournaments()
                    self.tenis_service.generate_random_tournaments()
                    
            # update the round
            round = self.tenis_service.get_round()

            round_string = "Qualificaiton" if qual else "Last 8" if self.tenis_service.players > 4 else "Last 4" if self.tenis_service.players > 2 else "Final"


            print(f"{round_string}: Round between {self.tenis_service.get_player_by_id(round[0])} and {self.tenis_service.get_player_by_id(round[1])}")
            while True:
                choice = input("Who wins? ")
                if choice == "1" or choice == "2":
                    break
                else:
                    print("Invalid choice")

            self.tenis_service.play_round(choice, True)

            
        