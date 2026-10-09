class Book:
    def __init__(self, id: str, title: str, author: str):
        self.__id = id
        self.__title = title
        self.__author = author

    @property
    def id(self) -> str:
        return self.__id

    @property
    def title(self) -> str:
        return self.__title

    @property
    def author(self) -> str:
        return self.__author

    def __eq__(self, other):
        if not isinstance(other, Book):
            return False
        return self.id == other.id

    def __str__(self):
        return f"Book: {self.title} by {self.author} --- [id: {self.id}]"

    def __iter__(self):
        return iter([self.id, self.title, self.author])
