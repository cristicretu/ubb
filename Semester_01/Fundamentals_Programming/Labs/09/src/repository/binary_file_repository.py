import os
import pickle
import re
from domain.book import Book
from domain.client import Client
from repository.repository import Repository


class BinaryFileRepository(Repository):
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
                    data = pickle.load(file)
                    self.add(data)
                except EOFError:
                    break

    def __save_data(self):
        with open(self.__file_name, "wb") as file:
            for data in self.get_all():
                pickle.dump(data, file)

    def add(self, element):
        super().add(element)
        self.__save_data()

    def remove(self, element_id):
        super().remove(element_id)
        self.__save_data()

    def update(self, element):
        super().update(element)
        self.__save_data()

    def get_all(self):
        return super().get_all()

    def get(self, element_id):
        return super().get(element_id)

    def search(self, field_to_search_for):
        return super().search(field_to_search_for)
