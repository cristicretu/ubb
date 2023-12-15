import datetime


class Rental:
    def __init__(
        self,
        id: str,
        book_id: str,
        client_id: str,
        rented_date: datetime,
        returned_date: datetime,
    ):
        self.__id = id
        self.__book_id = book_id
        self.__client_id = client_id
        self.__rented_date = rented_date
        self.__returned_date = returned_date

    @property
    def id(self) -> str:
        return self.__id

    @property
    def book_id(self) -> str:
        return self.__book_id

    @property
    def client_id(self) -> str:
        return self.__client_id

    @property
    def rented_date(self) -> datetime:
        return self.__rented_date

    @property
    def returned_date(self) -> datetime:
        return self.__returned_date

    def __eq__(self, other):
        if not isinstance(other, Rental):
            return False
        return self.id == other.id

    def __str__(self):
        return (
            f"Rental ID {self.id} with Book ID: {self.book_id} owned by Client ID: {self.client_id} from {self.rented_date}"
            + (f" to {self.returned_date}" if self.returned_date else "")
        )
