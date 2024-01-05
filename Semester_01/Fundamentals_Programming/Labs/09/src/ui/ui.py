from lib.helpers import read_date, read_string, read_valid_integer
from repository.binary_file_repository import BinaryFileRepository
from repository.rental_binary_file_repository import RentalBinaryFileRepository
from repository.rental_repository import RentalRepository
from repository.rental_text_file_repository import RentalTextFileRepository
from repository.repository import Repository
from repository.repository_exception import RepositoryException
from repository.text_file_repository import TextFileRepository
from services.book_service import BookService
from services.client_service import ClientService
from services.rental_service import RentalService
from services.undo_service import UndoError, UndoService


class UI:
    def __init__(self):
        try:
            (
                self.__book_repository,
                self.__client_repository,
                self.__rental_repository,
            ) = self.__validate_settings_file("settings.properties")
        except RepositoryException as repository_exception:
            print(repository_exception)
            exit(1)

        self.__undo_service = UndoService()
        self.__rental_service = RentalService(
            self.__rental_repository, self.__undo_service
        )

        self.__book_service = BookService(
            self.__book_repository, self.__rental_service, self.__undo_service
        )
        self.__client_service = ClientService(
            self.__client_repository, self.__rental_service, self.__undo_service
        )
        self.__book_service.generate_random_books()
        self.__client_service.generate_random_clients()

        self.__rental_service.generate_random_rentals(
            self.__book_service.get_all_books(),
            self.__client_service.get_all_clients(),
        )

    def __print_menu(self):
        print("1. Book operations")
        print("2. Client operations")
        print("3. Rental operations")
        print("4. Search operations")
        print("5. Statistics")
        print("6. Undo last operation")
        print("7. Redo last operation")
        print("0. Exit")

    def __print_book_menu(self):
        print("1. Add book")
        print("2. Remove book")
        print("3. Update book")
        print("4. List all books")
        print("0. Back")

    def __print_client_menu(self):
        print("1. Add client")
        print("2. Remove client")
        print("3. Update client")
        print("4. List all clients")
        print("0. Back")

    def __print_rental_menu(self):
        print("1. Rent book")
        print("2. Return book")
        print("3. List all rentals")
        print("0. Back")

    def __print_search_menu(self):
        print("1. Search books")
        print("2. Search clients")
        print("0. Back")

    def __print_statistics_menu(self):
        print("1. Most rented books")
        print("2. Most active clients")
        print("3. Most rented authors")
        print("0. Back")

    def __validate_settings_file(self, file_name):
        """
        Determines if the given file_name is a valid settings file.
        """
        with open(file_name, "r") as settings_file:
            settings = settings_file.readlines()

        settings = [setting.strip().split("=")[1].strip() for setting in settings]

        if settings[0] == "inmemory":
            return Repository(), Repository(), RentalRepository()
        elif settings[0] == "binaryfiles":
            return (
                BinaryFileRepository(settings[1]),
                BinaryFileRepository(settings[2]),
                RentalBinaryFileRepository(settings[3]),
            )
        elif settings[0] == "textfiles":
            return (
                TextFileRepository(settings[1]),
                TextFileRepository(settings[2]),
                RentalTextFileRepository(settings[3]),
            )
        else:
            raise RepositoryException("Invalid repository type!")

    def _main_loop(self):
        BOOK_OPERATIONS = 1
        CLIENT_OPERATIONS = 2
        RENTAL_OPERATIONS = 3
        SEARCH_OPERATIONS = 4
        STATISTICS = 5
        UNDO_LAST_OPERATION = 6
        REDO_LAST_OPERATION = 7
        EXIT = 0

        while True:
            print()
            self.__print_menu()
            print()
            user_command = read_valid_integer("Enter command: ")

            if user_command == BOOK_OPERATIONS:
                print()
                self.__print_book_menu()
                print()

                ADD_BOOK = 1
                REMOVE_BOOK = 2
                UPDATE_BOOK = 3
                LIST_ALL_BOOKS = 4
                BACK = 0

                user_command = read_valid_integer("Enter command: ")

                if user_command == ADD_BOOK:
                    book_id = read_valid_integer("Enter book ID: ")
                    title = read_string("Enter title: ")
                    author = read_string("Enter author: ")

                    self.__book_service.add_book(book_id, title, author)

                elif user_command == REMOVE_BOOK:
                    book_id = read_valid_integer("Enter book ID: ")

                    self.__book_service.remove_book(book_id)

                elif user_command == UPDATE_BOOK:
                    book_id = read_valid_integer("Enter book ID: ")
                    title = read_string("Enter title: ")
                    author = read_string("Enter author: ")

                    self.__book_service.update_book(book_id, title, author)

                elif user_command == LIST_ALL_BOOKS:
                    books = self.__book_service.get_all_books()
                    if len(books) == 0:
                        print("\nNo books!\n")
                        continue

                    print("\nAll books:\n")
                    for book in books:
                        print(book)
                    print()

                elif user_command == BACK:
                    continue

            elif user_command == CLIENT_OPERATIONS:
                print()
                self.__print_client_menu()
                print()

                ADD_CLIENT = 1
                REMOVE_CLIENT = 2
                UPDATE_CLIENT = 3
                LIST_ALL_CLIENTS = 4
                BACK = 0

                user_command = read_valid_integer("Enter command: ")

                if user_command == ADD_CLIENT:
                    client_id = read_valid_integer("Enter client ID: ")
                    name = read_string("Enter name: ")

                    self.__client_service.add_client(client_id, name)

                elif user_command == REMOVE_CLIENT:
                    client_id = read_valid_integer("Enter client ID: ")

                    self.__client_service.remove_client(client_id)

                elif user_command == UPDATE_CLIENT:
                    client_id = read_valid_integer("Enter client ID: ")
                    name = read_string("Enter name: ")

                    self.__client_service.update_client(client_id, name)

                elif user_command == LIST_ALL_CLIENTS:
                    clients = self.__client_service.get_all_clients()
                    if len(clients) == 0:
                        print("\nNo clients!\n")
                        continue

                    print("\nAll clients:\n")
                    for client in clients:
                        print(client)
                    print()

                elif user_command == BACK:
                    continue

            elif user_command == RENTAL_OPERATIONS:
                print()
                self.__print_rental_menu()
                print()

                RENT_BOOK = 1
                RETURN_BOOK = 2
                LIST_ALL_RENTALS = 3
                BACK = 0

                user_command = read_valid_integer("Enter command: ")

                if user_command == RENT_BOOK:
                    rental_id = read_string("Enter rental ID: ")
                    book_id = read_valid_integer("Enter book ID: ")
                    client_id = read_valid_integer("Enter client ID: ")
                    rented_date = read_date("Enter rented date: ")
                    # returned_date = read_date("Enter returned date: ")

                    books = self.__book_service.get_all_books()
                    clients = self.__client_service.get_all_clients()

                    try:
                        self.__rental_service.add_rental(
                            rental_id,
                            book_id,
                            client_id,
                            rented_date,
                            books,
                            clients,
                            None,
                        )
                    except RepositoryException as undo_error:
                        print(undo_error)

                elif user_command == RETURN_BOOK:
                    rental_id = read_string("Enter rental ID: ")
                    returned_date = read_date("Enter returned date: ")

                    self.__rental_service.update_rental(rental_id, returned_date)

                elif user_command == LIST_ALL_RENTALS:
                    rentals = self.__rental_service.get_all_rentals()
                    if len(rentals) == 0:
                        print("\nNo rentals!\n")
                        continue

                    print("\nAll rentals:\n")
                    for rental in rentals:
                        print(rental)
                    print()

                elif user_command == BACK:
                    continue

            elif user_command == SEARCH_OPERATIONS:
                print()
                self.__print_search_menu()
                print()

                SEARCH_BOOKS = 1
                SEARCH_CLIENTS = 2
                BACK = 0

                user_command = read_valid_integer("Enter command: ")

                if user_command == SEARCH_BOOKS:
                    field_to_search_for = read_string(
                        "Enter field to search for in books: "
                    )
                    books = self.__book_service.search_book(field_to_search_for)
                    if len(books) == 0:
                        print("\nNo books!\n")
                        continue

                    print("\nAll books:\n")
                    for book in books:
                        print(book)
                    print()

                elif user_command == SEARCH_CLIENTS:
                    field_to_search_for = read_string(
                        "Enter field to search for in clients: "
                    )
                    clients = self.__client_service.search_client(field_to_search_for)
                    if len(clients) == 0:
                        print("\nNo clients!\n")
                        continue

                    print("\nAll clients:\n")
                    for client in clients:
                        print(client)
                    print()

                elif user_command == BACK:
                    continue

            elif user_command == STATISTICS:
                print()
                self.__print_statistics_menu()
                print()

                MOST_RENTED_BOOKS = 1
                MOST_ACTIVE_CLIENTS = 2
                MOST_RENTED_AUTHORS = 3
                BACK = 0

                user_command = read_valid_integer("Enter command: ")

                if user_command == MOST_RENTED_BOOKS:
                    books = self.__book_service.get_all_books()
                    most_rented_books = (
                        self.__rental_service.sort_rentals_by_most_rented_book(books)
                    )
                    if len(most_rented_books) == 0:
                        print("\nNo books!\n")
                        continue

                    print("\nMost rented books:\n")
                    for book in most_rented_books:
                        print(book)
                    print()

                elif user_command == MOST_ACTIVE_CLIENTS:
                    clients = self.__client_service.get_all_clients()

                    most_active_clients = (
                        self.__rental_service.sort_rentals_by_most_active_client(
                            clients
                        )
                    )
                    if len(most_active_clients) == 0:
                        print("\nNo clients!\n")
                        continue

                    print("\nMost active clients:\n")
                    for client in most_active_clients:
                        print(client)
                    print()

                elif user_command == MOST_RENTED_AUTHORS:
                    # get book data
                    books = self.__book_service.get_all_books()

                    most_rented_authors = (
                        self.__rental_service.sort_rentals_by_most_rented_author(books)
                    )
                    if len(most_rented_authors) == 0:
                        print("\nNo authors!\n")
                        continue

                    print("\nMost rented authors:\n")
                    for author in most_rented_authors:
                        print(author)
                    print()
                elif user_command == BACK:
                    continue

            elif user_command == UNDO_LAST_OPERATION:
                try:
                    self.__undo_service.undo()

                except UndoError as undo_error:
                    print(undo_error)

            elif user_command == REDO_LAST_OPERATION:
                try:
                    self.__undo_service.redo()
                except UndoError as undo_error:
                    print(undo_error)

            elif user_command == EXIT:
                break
            else:
                print("\nInvalid command!\n")
