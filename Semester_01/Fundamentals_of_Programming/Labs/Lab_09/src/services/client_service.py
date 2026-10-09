import random
from domain.client import Client
from repository.repository import Repository
from repository.repository_exception import RepositoryException
from services.undo_service import FunctionCall, Operation


class ClientService:
    def __init__(self, client_repository, rental_service, undo_service):
        self.__client_repository = client_repository
        self.__rental_service = rental_service
        self.__undo_service = undo_service

    def generate_random_clients(self, number_of_clients: int = 20):
        check_if_there_are_any_clients = len(self.__client_repository.get_all()) > 0
        if check_if_there_are_any_clients:
            return
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

            # record operation for undo / redo
            redo = FunctionCall(self.__client_repository.add, client)
            undo = FunctionCall(self.__client_repository.remove, client_id)

            self.__undo_service.record(Operation(undo, redo))
        except RepositoryException as repository_exception:
            print(f"Client with id {client_id} already exists!")
            if testing:
                raise repository_exception

    def remove_client(self, client_id, testing=False):
        try:
            redo_operations = [FunctionCall(self.__client_repository.remove, client_id)]
            undo_operations = [
                FunctionCall(
                    self.__client_repository.add,
                    self.__client_repository.get(client_id),
                )
            ]

            self.__client_repository.remove(client_id)

            rentals = self.__rental_service.get_all_rentals()
            for rental in rentals:
                if rental.client_id == client_id:
                    redo_operations.append(
                        FunctionCall(self.__rental_service.remove_rental, rental.id)
                    )
                    undo_operations.append(
                        FunctionCall(
                            self.__rental_service.add_rental,
                            rental.id,
                            rental.book_id,
                            rental.client_id,
                            rental.rented_date,
                            None,
                            None,
                            None,
                            True,
                        )
                    )

                    self.__rental_service.remove_rental(rental.id)

            concatenated_redo_operations = FunctionCall(
                self.__execute_operations, redo_operations
            )
            concatenated_undo_operations = FunctionCall(
                self.__execute_operations, undo_operations
            )

            self.__undo_service.record(
                Operation(concatenated_undo_operations, concatenated_redo_operations)
            )

        except RepositoryException as repository_exception:
            print(f"Client with id {client_id} does not exist!")
            if testing:
                raise repository_exception

    def update_client(self, client_id, name, testing=False):
        client = Client(client_id, name)
        try:
            redo = FunctionCall(self.__client_repository.update, client)
            undo = FunctionCall(
                self.__client_repository.update, self.__client_repository.get(client_id)
            )
            self.__undo_service.record(Operation(undo, redo))

            self.__client_repository.update(client)
        except RepositoryException as repository_exception:
            print(f"Client with id {client_id} does not exist!")
            if testing:
                raise repository_exception

    def get_all_clients(self):
        return self.__client_repository.get_all()

    def search_client(self, field_to_search_for):
        return self.__client_repository.search(field_to_search_for)

    def __execute_operations(self, operations):
        for operation in operations:
            operation.call()
