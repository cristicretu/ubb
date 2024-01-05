import unittest
from domain.book import Book
from repository.rental_repository import RentalRepository
from repository.repository import Repository
from services.book_service import BookService
from repository.repository_exception import RepositoryException
from services.rental_service import RentalService
from services.undo_service import UndoService


class TestBookService(unittest.TestCase):
    def setUp(self):
        self.rental_repository = RentalRepository()
        self.book_repository = Repository()
        self.undo_service = UndoService()
        self.rental_service = RentalService(self.rental_repository, self.undo_service)

        self.book_service = BookService(
            self.book_repository, self.rental_service, self.undo_service
        )
        self.book_service.generate_random_books()

    def test_add_book_success(self):
        book_id = 10
        title = "Lionel Messi"
        author = "Cristiano Ronaldo"
        self.book_service.add_book(book_id, title, author)
        self.assertEqual(len(self.book_service.get_all_books()), 20)

    def test_remove_book_success(self):
        book_id = 10
        title = "Lionel Messi"
        author = "Cristiano Ronaldo"
        self.book_service.add_book(book_id, title, author)
        self.book_service.remove_book(book_id)
        self.assertEqual(len(self.book_service.get_all_books()), 19)

    def test_remove_book_failure(self):
        with self.assertRaises(RepositoryException):
            self.book_service.remove_book(99, testing=True)

    def test_update_book_success(self):
        book_id = 20
        title = "Lionel Messi"
        author = "Cristiano Ronaldo"
        self.book_service.add_book(book_id, title, author)
        new_title = "THE GOAT"
        self.book_service.update_book(book_id, new_title, author)
        updated_book = self.book_service.get_all_books()[book_id]
        self.assertEqual(updated_book.title, new_title)

    def test_update_book_failure(self):
        with self.assertRaises(RepositoryException):
            self.book_service.update_book(
                511, "Lionel Messi", "Cristiano Ronaldo", testing=True
            )

    def test_update_book_failure(self):
        book_id = 99
        title = "Jr"
        author = "Messi"
        with self.assertRaises(RepositoryException):
            self.book_service.update_book(book_id, title, author, testing=True)

    def test_get_all_books(self):
        self.assertEqual(len(self.book_service.get_all_books()), 20)
