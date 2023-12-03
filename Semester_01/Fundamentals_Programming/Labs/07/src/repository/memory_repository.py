from domain.expense import Expense


class RepositoryException(Exception):
    pass


class MemoryRepository:
    def __init__(self) -> None:
        self._expense_data = []
        self._undo_stack = []

    def _save_undo_snapshot(self):
        self._undo_stack.append(self._expense_data.copy())

    def add_expense(self, new_expense: Expense, store_undo_snapshot: bool = False):
        """
        Adds a new expense to the repository.

        :param expense: The expense to be added.
        :raises RepositoryException: If the expense already exists.
        """
        if new_expense in self._expense_data:
            raise RepositoryException("Expense already exists!")

        if store_undo_snapshot:
            self._save_undo_snapshot()
        self._expense_data.append(new_expense)

    def get_all_expenses(self):
        """
        Returns all the expenses in the repository.

        :return: A list of all the expenses in the repository.
        """
        return self._expense_data.copy()

    def filter_expenses_by_amount(self, amount: int):
        """
        Filters if the expenses amount is greater than the given amount.

        :param amount: The amount to compare to.
        :return: A list of expenses wehre the amount is greater than the given amount.
        """
        self._save_undo_snapshot()
        self._expense_data = [
            expense for expense in self._expense_data if expense.amount > amount
        ]

    def undo_last_operation(self):
        """
        Undoes the last operation that changed the repository.
        """
        if not self._undo_stack:
            raise RepositoryException("No more undos!")

        self._expense_data = self._undo_stack.pop()
