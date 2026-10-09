class Client:
    def __init__(self, id: str, name: str):
        self.__id = id
        self.__name = name

    @property
    def id(self) -> str:
        return self.__id

    @property
    def name(self) -> str:
        return self.__name

    def __eq__(self, other):
        if not isinstance(other, Client):
            return False
        return self.id == other.id

    def __str__(self):
        return f"Client: {self.name} [id: {self.id}]"

    def __iter__(self):
        return iter([self.id, self.name])
