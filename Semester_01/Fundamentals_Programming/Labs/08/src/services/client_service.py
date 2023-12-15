import random
from domain.client import Client
from repository.repository import Repository
from repository.repository_exception import RepositoryException


class ClientService:
    def __init__(self):
        self.__client_repository = Repository()
        self.generate_random_clients()

    def generate_random_clients(self, number_of_clients: int = 20):
        names = [
            "George",
            "John",
            "Thomas",
            "James",
            "Andrew",
            "Martin",
        ]

        for index in range(number_of_clients):
            client_id = index
            name = random.choice(names) + " " + random.choice(names)
            self.add_client(client_id, name)

    def add_client(self, client_id, name, testing=False):
        client = Client(client_id, name)
        try:
            self.__client_repository.add(client)
        except RepositoryException as repository_exception:
            print(f"Client with id {client_id} already exists!")
            if testing:
                raise repository_exception

    def remove_client(self, client_id, testing=False):
        try:
            self.__client_repository.remove(client_id)
        except RepositoryException as repository_exception:
            print(f"Client with id {client_id} does not exist!")
            if testing:
                raise repository_exception

    def update_client(self, client_id, name, testing=False):
        client = Client(client_id, name)
        try:
            self.__client_repository.update(client)
        except RepositoryException as repository_exception:
            print(f"Client with id {client_id} does not exist!")
            if testing:
                raise repository_exception

    def get_all_clients(self):
        return self.__client_repository.get_all()

    def search_client(self, field_to_search_for):
        return self.__client_repository.search(field_to_search_for)
