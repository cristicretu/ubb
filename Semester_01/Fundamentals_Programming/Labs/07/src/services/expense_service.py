from domain.expense import Expense


class ExpenseService:
    def __init__(self, expense_repository):
        self._expense_repository = expense_repository
        self.generate_random_expenses()

    def generate_random_expenses(self, number_of_expenses: int = 10):
        """
        Generates random expenses and adds them to the repository.

        :param number_of_expenses: The number of expenses to generate.
        :return: None
        """
        from random import randint, choice

        DAY_LOWER_BOUND = 1
        DAY_UPPER_BOUND = 30
        AMOUNT_LOWER_BOUND = 1
        AMOUNT_UPPER_BOUND = 100

        descriptions = ["food", "clothes", "rent", "utilities", "internet"]

        for _ in range(number_of_expenses):
            day = randint(DAY_LOWER_BOUND, DAY_UPPER_BOUND)
            amount = randint(AMOUNT_LOWER_BOUND, AMOUNT_UPPER_BOUND)
            type = choice(descriptions)

            self.add_expense(day, amount, type, store_undo_snapshot=False)

    def add_expense(
        self,
        expense_day,
        expense_amount,
        expense_type,
        store_undo_snapshot: bool = True,
    ):
        expense = Expense(expense_day, expense_amount, expense_type)

        self._expense_repository.add_expense(expense, store_undo_snapshot)

    def get_all_expenses(self):
        """
        Returns all the expenses in the repository.

        :return: A list of all the expenses in the repository.
        """
        return self._expense_repository.get_all_expenses()

    def filter_expenses_by_amount(self, amount: int):
        """
        Returns a list of expenses whose amount is greater than the g iven amount.

        :param amount: The amount to compare to.
        :return: A list of expenses whose amount is greater than the given amount.
        """
        return self._expense_repository.filter_expenses_by_amount(amount)

    def undo_last_operation(self):
        """
        Undoes the last operation that changed the repository.

        :return: None
        """
        self._expense_repository.undo_last_operation()
