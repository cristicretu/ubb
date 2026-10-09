import random
from domain.book import Book
from repository.repository import Repository
from repository.repository_exception import RepositoryException


class BookService:
    def __init__(self):
        self.__book_repository = Repository()
        self.generate_random_books()

    def generate_random_books(self, number_of_books: int = 20):
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
        except RepositoryException as repository_exception:
            print(f"Book with id {book_id} already exists!")
            if testing:
                raise repository_exception

    def remove_book(self, book_id, testing=False):
        try:
            self.__book_repository.remove(book_id)
        except RepositoryException as repository_exception:
            print(f"Book with id {book_id} does not exist!")
            if testing:
                raise repository_exception

    def update_book(self, book_id, title, author, testing=False):
        book = Book(book_id, title, author)
        try:
            self.__book_repository.update(book)
        except RepositoryException as repository_exception:
            print(f"Book with id {book_id} does not exist!")
            if testing:
                raise repository_exception

    def get_all_books(self):
        return self.__book_repository.get_all()

    def search_book(self, field_to_search_for):
        return self.__book_repository.search(field_to_search_for)
