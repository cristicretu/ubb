import os
import pickle
from repository.memory_repository import MemoryRepository


class BinaryFileRepository(MemoryRepository):
    def __init__(self, file_path: str):
        super().__init__()
        self._file_path = file_path
        self._load_expenses()

    def _load_expenses(self):
        """
        Loads the expenses from the file.
        """
        if not os.path.exists(self._file_path):
            return

        with open(self._file_path, "rb") as file:
            while True:
                try:
                    expense = pickle.load(file)
                    self._expense_data.append(expense)
                except EOFError:
                    break

    def _save_expenses(self):
        """
        Saves the expenses to the file.
        """
        with open(self._file_path, "wb") as file:
            for expense in self._expense_data:
                pickle.dump(expense, file)

    def add_expense(self, new_expense, store_undo_snapshot=False):
        """
        Adds a new expense to the repository.

        :param expense: The expense to be added.
        :raises RepositoryException: If the expense already exists.
        """
        super().add_expense(new_expense, store_undo_snapshot)
        self._save_expenses()

    def filter_expenses_by_amount(self, amount: int):
        """
        Filters if the expenses amount is greater than the given amount.

        :param amount: The amount to compare to.
        :return: A list of expenses wehre the amount is greater than the given amount.
        """
        super().filter_expenses_by_amount(amount)
        self._save_expenses()

    def undo_last_operation(self):
        """
        Undoes the last operation that changed the repository.
        """
        super().undo_last_operation()
        self._save_expenses()
