from repository import TournamentRepository


class TenisService:
    def __init__(self, player_repo):
        self.player_repo = player_repo
        self.tournament_repo = TournamentRepository()
        self.players = len(self.player_repo.get_all_players())

    def get_all_players(self):
        return self.player_repo.get_all_players()

    def get_player_by_id(self, id):
        return self.player_repo.get_player_by_id(id)

    def is_qualifying_round_needed(self):
        players = self.player_repo.get_all_players()
        
        is_power_of_two = lambda x: x != 0 and ((x & (x - 1)) == 0)

        if is_power_of_two(len(players)):
            return False
        
        players = sorted(players, key=lambda x: x.strength, reverse=True)

        lenny = len(players)

        # until we have a power of two, add tournaments between the last players
        while not is_power_of_two(lenny):
            self.tournament_repo.add_tournament(players[-1].id, players[-2].id)
            players.pop()
            players.pop()
            lenny -= 1

        return True

    def get_all_tournaments(self):
        return self.tournament_repo.get_all_tournaments()

    def clear_tournaments(self):    
        self.tournament_repo.clear_tournaments()

    def generate_random_tournaments(self):
        # the number of players is self.players

        lenny = self.players

        import random

        ids = [x.id for x in self.get_all_players()]

        while lenny > 1:
            id1 = random.choice(ids)
            id2 = random.choice(ids)

            if id1 == id2 or self.tournament_repo.is_playing(id1) or self.tournament_repo.is_playing(id2):
                continue

            self.tournament_repo.add_tournament(id1, id2)
            lenny -= 2

    def get_round(self):
        tournaments = self.tournament_repo.get_all_tournaments()

        for tournament in tournaments:
            if not tournament[2]:
                return tournament
            
        return None

    def play_round(self, choice, qualifying):
        round = self.get_round()
            
        self.tournament_repo.play_tournament(round[0], round[1])
            
        if choice == "1":
            if qualifying:
                self.player_repo.remove_player(round[1])
            
            self.player_repo.increase_strength(round[0])
        
        elif choice == "2":
            if qualifying:
                self.player_repo.remove_player(round[0])
            
            self.player_repo.increase_strength(round[1])

        self.players -= 1
