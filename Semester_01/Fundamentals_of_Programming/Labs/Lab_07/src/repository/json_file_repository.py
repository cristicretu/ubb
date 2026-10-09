from repository.memory_repository import MemoryRepository
import os
import json
from domain.expense import Expense


class JSONFileRepository(MemoryRepository):
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

        with open(self._file_path, "r") as file:
            # Check if the file is not empty
            if os.path.getsize(self._file_path) > 0:
                expenses_data = json.load(file)
                for expense_data in expenses_data:
                    self._expense_data.append(Expense.from_dict(expense_data))
            else:
                self._expense_data = []

    def _save_expenses(self):
        """
        Saves the expenses to the JSON file.
        """
        with open(self._file_path, "w") as file:
            # Convert the expenses to a list of dictionaries before saving
            expenses_data = [expense.to_dict() for expense in self._expense_data]
            json.dump(expenses_data, file, indent=4)

    def add_expense(self, new_expense: Expense, store_undo_snapshot: bool = False):
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
