import sqlite3
from domain.expense import Expense

from repository.memory_repository import MemoryRepository, RepositoryException


class SQLRepository(MemoryRepository):
    def __init__(self, database_path: str):
        super().__init__()
        self._database_path = database_path
        self._connect_and_initialize_database()

    def _connect_and_initialize_database(self):
        """
        Connects to the database and initializes the expenses table.
        """
        self.connection = sqlite3.connect(self._database_path)
        self.connection.execute(
            """
            CREATE TABLE IF NOT EXISTS expenses (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                day INTEGER,
                amount INTEGER,
                type TEXT
            )
          """
        )
        self.connection.commit()

    def _update_database(self):
        # Clear the table
        self.connection.execute("DELETE FROM expenses")
        # insert old values
        for expense in self._expense_data:
            query = "INSERT INTO expenses (day, amount, type) VALUES (?, ?, ?)"
            self.connection.execute(query, (expense.day, expense.amount, expense.type))
        self.connection.commit()

    def _load_expenses(self):
        """
        Loads the expenses from the database.
        """
        cursor = self.connection.execute("SELECT day, amount, type FROM expenses")
        for row in cursor:
            self._expense_data.append(Expense(*row))

    def add_expense(self, new_expense: Expense, store_undo_snapshot: bool = False):
        if new_expense in self._expense_data:
            raise RepositoryException("Expense already exists!")

        if store_undo_snapshot:
            self._save_undo_snapshot()

        query = "INSERT INTO expenses (day, amount, type) VALUES (?, ?, ?)"
        self.connection.execute(
            query, (new_expense.day, new_expense.amount, new_expense.type)
        )
        self.connection.commit()

        # account for the new generated id
        self._load_expenses()

    def filter_expenses_by_amount(self, amount: float):
        self._save_undo_snapshot()
        super().filter_expenses_by_amount(amount)
        # Update database to reflect the current state
        self._update_database()

    def undo_last_operation(self):
        super().undo_last_operation()
        # Update database to reflect the undone state
        self._update_database()
