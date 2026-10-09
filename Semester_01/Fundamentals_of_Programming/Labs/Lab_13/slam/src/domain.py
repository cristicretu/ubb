class Player:
    def __init__(self, id, name, strength):
        self.id = id
        self.name = name
        self.strength = strength

    def __str__(self):
        return f"Player #{self.id}  -  {self.name} with strength {self.strength}"