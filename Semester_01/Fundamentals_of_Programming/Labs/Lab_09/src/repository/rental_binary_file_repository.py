import os
import pickle
from repository.rental_repository import RentalRepository


class RentalBinaryFileRepository(RentalRepository):
    def __init__(self, file_name):
        super().__init__()
        self.__file_name = file_name
        self.__load_data()

    def __load_data(self):
        if not os.path.exists(self.__file_name):
            return
        with open(self.__file_name, "rb") as file:
            while True:
                try:
                    rental = pickle.load(file)
                    self._rentals.append(rental)
                except EOFError:
                    break

    def __save_data(self):
        with open(self.__file_name, "wb") as file:
            for rental in self.get_all():
                pickle.dump(rental, file)

    def add(self, rental, books, clients, undo_redo=False):
        super().add(rental, books, clients, undo_redo)
        self.__save_data()

    def remove(self, rental_id):
        super().remove(rental_id)
        self.__save_data()

    def update(self, rental):
        super().update(rental)
        self.__save_data()

    def get_all(self):
        return super().get_all()

    def get_rental_by_id(self, rental_id):
        return super().get_rental_by_id(rental_id)
