import os
from domain.book import Book
from domain.client import Client
from repository.repository import Repository


class TextFileRepository(Repository):
    def __init__(self, file_name):
        super().__init__()
        self.__file_name = file_name
        self.__load_data()

    def __load_data(self):
        if not os.path.exists(self.__file_name):
            return

        with open(self.__file_name, "r") as file:
            lines = file.read().splitlines()
            for line in lines:
                if line == "\n":
                    continue

                count_of_quotes = line.count(";")
                if count_of_quotes == 2:
                    line = line.replace(";", ",")
                    id, title, author = line.split(",")
                    book = Book(int(id), title, author)
                    self._data[int(id)] = book
                elif count_of_quotes == 1:
                    line = line.replace(";", ",")
                    id, name = line.split(",")
                    client = Client(int(id), name)
                    self._data[int(id)] = client

    def __save_data(self):
        with open(self.__file_name, "w") as file:
            for data in self._data.values():
                try:
                    if data.title is not None:
                        file.write(
                            str(data.id)
                            + ";"
                            + str(data.title)
                            + ";"
                            + str(data.author)
                            + "\n"
                        )
                except:
                    file.write(str(data.id) + ";" + str(data.name) + "\n")

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
