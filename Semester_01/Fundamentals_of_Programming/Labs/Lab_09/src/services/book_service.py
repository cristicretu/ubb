import random
from domain.book import Book
from repository.repository_exception import RepositoryException
from services.undo_service import FunctionCall, Operation


class BookService:
    def __init__(self, book_repository, rental_service, undo_service):
        self.__book_repository = book_repository
        self.__rental_service = rental_service
        self.__undo_service = undo_service

    def generate_random_books(self, number_of_books: int = 20):
        check_if_there_are_any_books = len(self.__book_repository.get_all()) > 0
        if check_if_there_are_any_books:
            return

        words = [
            "the",
            "not",
            "on",
            "who",
            "me",
            "your",
            "good",
            "day",
            "most",
            "us",
        ]

        names = [
            "George",
            "John",
            "Thomas",
            "James",
            "Andrew",
            "Martin",
        ]

        for index in range(number_of_books):
            book_id = index
            title = (
                random.choice(words).capitalize()
                + " "
                + random.choice(words).capitalize()
            )
            author = random.choice(names) + " " + random.choice(names)
            self.add_book(book_id, title, author)

    def add_book(self, book_id, title, author, testing=False):
        book = Book(book_id, title, author)
        try:
            self.__book_repository.add(book)

            # record operation for undo / redo
            redo = FunctionCall(self.__book_repository.add, book)
            undo = FunctionCall(self.__book_repository.remove, book_id)

            self.__undo_service.record(Operation(undo, redo))
        except RepositoryException as repository_exception:
            print(f"Book with id {book_id} already exists!")
            if testing:
                raise repository_exception

    def remove_book(self, book_id, testing=False):
        try:
            # record operation for undo / redo
            redo_operations = [FunctionCall(self.__book_repository.remove, book_id)]
            undo_operations = [
                FunctionCall(
                    self.__book_repository.add, self.__book_repository.get(book_id)
                )
            ]

            # delete all the rentals that have the book with the given id
            rentals = self.__rental_service.get_all_rentals()
            for rental in rentals:
                if rental.book_id == book_id:
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

            # delete the book
            self.__book_repository.remove(book_id)

        except RepositoryException as repository_exception:
            print(f"Book with id {book_id} does not exist!")
            if testing:
                raise repository_exception

    def update_book(self, book_id, title, author, testing=False):
        book = Book(book_id, title, author)
        try:
            # record operation for undo / redo
            redo = FunctionCall(self.__book_repository.update, book)
            undo = FunctionCall(
                self.__book_repository.update, self.__book_repository.get(book_id)
            )

            self.__undo_service.record(Operation(undo, redo))
            self.__book_repository.update(book)
        except RepositoryException as repository_exception:
            print(f"Book with id {book_id} does not exist!")
            if testing:
                raise repository_exception

    def get_all_books(self):
        return self.__book_repository.get_all()

    def search_book(self, field_to_search_for):
        return self.__book_repository.search(field_to_search_for)

    def __execute_operations(self, operations):
        for operation in operations:
            operation.call()
