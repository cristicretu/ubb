import datetime
import random
from domain.rental import Rental
from lib.helpers import random_date
from repository.rental_repository import RentalRepository
from repository.repository_exception import RepositoryException
from services.undo_service import FunctionCall, Operation


class RentalService:
    def __init__(self, rental_repository, undo_service):
        self.__rental_repository = rental_repository
        self.__undo_service = undo_service

    def generate_random_rentals(self, books, clients, number_of_rentals: int = 20):
        check_if_there_are_any_rentals = len(self.__rental_repository.get_all()) > 0
        if check_if_there_are_any_rentals:
            return

        if books is None or clients is None:
            raise ValueError("Book IDs and client IDs must be provided!")

        DATE_LOWER_BOUND = datetime.datetime(2020, 1, 1)
        DATE_MIDDLE_BOUND = datetime.datetime(2024, 1, 1)
        DATE_UPPER_BOUND = datetime.datetime(2028, 1, 1)

        added_rentals = 0

        # last treturn dates
        last_return_dates = {book.id: None for book in books}

        while added_rentals < number_of_rentals:
            book = random.choice(books)
            client = random.choice(clients)
            rental_id = f"{book.id}_{client.id}"

            is_book_already_rented = any(
                rental.book_id == book.id and rental.returned_date is None
                for rental in self.__rental_repository.get_all()
            )

            if is_book_already_rented:
                continue

            # ensure that the book is not already rented
            rented_date = last_return_dates[book.id]
            if rented_date is None:
                rented_date = random_date(DATE_LOWER_BOUND, DATE_MIDDLE_BOUND)
            else:
                rented_date = random_date(rented_date, DATE_UPPER_BOUND)

            to_pick = random.randint(0, 1)
            returned_date = (
                random_date(rented_date, DATE_UPPER_BOUND) if to_pick else None
            )

            # Update last returned date for this book
            last_return_dates[book.id] = returned_date or rented_date

            try:
                self.add_rental(
                    rental_id,
                    book.id,
                    client.id,
                    rented_date,
                    None,
                    None,
                    returned_date,
                )
                added_rentals += 1

            except RepositoryException:
                pass

    def add_rental(
        self,
        rental_id,
        book_id,
        client_id,
        rented_date,
        books,
        clients,
        returned_date=None,
        undo_redo=False,
    ):
        """
        Adds a rental to the repository.

        :param rental_id: The rental ID.
        :param book_id: The book ID.
        :param client_id: The client ID.
        :param rented_date: The rented date.
        :param books: The books list
        :param clients: The clients list

        """

        # Create a new Rental object
        rental = Rental(
            rental_id, book_id, client_id, rented_date, returned_date or None
        )

        # record operation for undo / redo
        redo = FunctionCall(self.__rental_repository.add, rental, books, clients, True)
        undo = FunctionCall(self.__rental_repository.remove, rental_id)

        self.__undo_service.record(Operation(undo, redo))
        # Add the rental using the repository's add method
        self.__rental_repository.add(rental, books, clients, undo_redo=undo_redo)

    def remove_rental(self, rental_id):
        # record operation for undo / redo
        redo = FunctionCall(self.__rental_repository.remove, rental_id)
        rental = self.__rental_repository.get_rental_by_id(rental_id)
        undo = FunctionCall(
            self.__rental_repository.add,
            rental,
            None,
            None,
            True,
        )

        self.__undo_service.record(Operation(undo, redo))

        self.__rental_repository.remove(rental_id)

    def update_rental(self, rental_id, returned_date):
        try:
            rental = self.get_rental_by_id(rental_id)
        except RepositoryException as repository_exception:
            print(repository_exception)

        rental = Rental(
            rental.id,
            rental.book_id,
            rental.client_id,
            rental.rented_date,
            returned_date,
        )

        # record operation for undo / redo
        redo = FunctionCall(self.__rental_repository.update, rental)
        undo = FunctionCall(
            self.__rental_repository.update,
            self.__rental_repository.get_rental_by_id(rental_id),
        )

        self.__undo_service.record(Operation(undo, redo))

        self.__rental_repository.update(rental)

    def get_all_rentals(self):
        return self.__rental_repository.get_all()

    def get_rental_by_id(self, rental_id):
        return self.__rental_repository.get_rental_by_id(rental_id)

    def sort_rentals_by_most_rented_book(self, books):
        """
        Sort descending by the number of times a book was rented.

        :param books: The books list.
        """

        rentals = self.get_all_rentals()

        book_count = {}
        for rental in rentals:
            book_id = rental.book_id
            book_count[book_id] = book_count.get(book_id, 0) + 1

        sorted_books = sorted(book_count.items(), key=lambda x: x[1], reverse=True)

        sorted_list_containing_book_title = []
        for book_id, count in sorted_books:
            book = next((b for b in books if b.id == book_id), None)
            if book:
                sorted_list_containing_book_title.append(
                    f"Book with ID {book_id}, title {book.title} and author {book.author} was rented {count} times."
                )

        return sorted_list_containing_book_title

    def sort_rentals_by_most_active_client(self, clients):
        """
        Sort descending by the number of times a client rented a book.

        :param clients: The clients list.
        """

        rentals = self.get_all_rentals()

        client_count = {}
        for rental in rentals:
            client_id = rental.client_id
            client_count[client_id] = client_count.get(client_id, 0) + 1

        sorted_clients = sorted(client_count.items(), key=lambda x: -x[1])

        sorted_list_containing_client_name = []
        for client_id, count in sorted_clients:
            client = next((c for c in clients if c.id == client_id), None)
            if client:
                sorted_list_containing_client_name.append(
                    f"Client with ID {client_id} and name {client.name} rented {count} books."
                )

        return sorted_list_containing_client_name

    def sort_rentals_by_most_rented_author(self, books):
        """
        Sort descending by the number of times an author was rented.

        :param books: The books list.
        """

        rentals = self.get_all_rentals()

        # how many times each author was rented
        author_count = {}
        for rental in rentals:
            book = next((book for book in books if book.id == rental.book_id), None)
            if book:
                author_count[book.author] = author_count.get(book.author, 0) + 1

        sorted_authors = sorted(author_count.items(), key=lambda x: -x[1])

        sorted_list_containing_author_name = [
            f"Author {author} was rented {count} times."
            for author, count in sorted_authors
        ]

        return sorted_list_containing_author_name
