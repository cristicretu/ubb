class Client:
    """
    fields set in class constructor (id, name)
    getter properties for
        -> id : str
        -> name : str

    -> __eq__ overwritten
        - two clients are == if they have the same id
    """

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
        return self.__id == other.__id
