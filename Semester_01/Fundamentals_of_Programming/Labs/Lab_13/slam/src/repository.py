from domain import Player


class TournamentRepository:
    def __init__(self):
        self.tournaments = []

    def clear_tournaments(self):
        self.tournaments.clear()

    def add_tournament(self, id1, id2):
        self.tournaments.append((id1, id2, False))

    def get_all_tournaments(self):
        return self.tournaments
    
    def play_tournament(self, id1, id2):
        for i in range(len(self.tournaments)):
            if self.tournaments[i][0] == id1 and self.tournaments[i][1] == id2:
                self.tournaments[i] = (id1, id2, True)
                return True
        return False

    def is_playing(self, id):
        for tournament in self.tournaments:
            if tournament[0] == id or tournament[1] == id:
                return True
        return False
    

class PlayerRepository:
    def __init__(self, filename):
        self.players = []
        self.filename = filename
        self.load_players()

    def load_players(self):
        try:
            with open(self.filename, "r") as f:
                for line in f:
                    line = line.strip()
                    if line == "":
                        continue
                    parts = line.split(",")
                    self.players.append(Player(int(parts[0]), parts[1], int(parts[2])))
        except IOError:
            print("Error reading file")

    def save_players(self):
        try:
            with open(self.filename, "w") as f:
                for player in self.players:
                    f.write(f"{player.id},{player.name},{player.strength}\n")
        except IOError:
            print("Error writing file")

    def get_all_players(self):
        return self.players
    
    def get_player_by_id(self, id):
        for player in self.players:
            if player.id == id:
                return player

        return None
    
    def remove_player(self, id):
        for i in range(len(self.players)):
            if self.players[i].id == id:
                self.players.pop(i)
                break 

        self.save_players()                

    def increase_strength(self, id):
        for i in range(len(self.players)):
            if self.players[i].id == id:
                self.players[i] = Player(self.players[i].id, self.players[i].name, self.players[i].strength + 1)

        self.save_players()
        