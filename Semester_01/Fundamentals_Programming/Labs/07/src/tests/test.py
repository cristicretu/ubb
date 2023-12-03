from repository.memory_repository import RepositoryException
from services.expense_service import ExpenseService


class Test:
    def __init__(self, repository) -> None:
        self.expense_service = ExpenseService(repository)

    def test_add_expense(self):
        expense_data = self.expense_service.get_all_expenses()
        initial_expense_count = len(expense_data)

        self.expense_service.add_expense(1, 100, "food")
        assert len(self.expense_service.get_all_expenses()) == initial_expense_count + 1

        self.expense_service.add_expense(2, 200, "transport")
        assert len(self.expense_service.get_all_expenses()) == initial_expense_count + 2

        self.expense_service.add_expense(3, 300, "housekeeping")
        assert len(self.expense_service.get_all_expenses()) == initial_expense_count + 3

        try:
            self.expense_service.add_expense(3, 300, "housekeeping")
        except RepositoryException:
            pass
        assert len(self.expense_service.get_all_expenses()) == initial_expense_count + 3

    def test_filter_expenses_by_amount(self):
        self.expense_service.filter_expenses_by_amount(9999999)
        assert len(self.expense_service.get_all_expenses()) == 0

        self.expense_service.add_expense(10, 100, "food")
        self.expense_service.add_expense(20, 200, "transport")
        self.expense_service.add_expense(30, 300, "housekeeping")
        self.expense_service.add_expense(24, 400, "clothing")

        assert len(self.expense_service.get_all_expenses()) == 4

        self.expense_service.filter_expenses_by_amount(100)
        assert len(self.expense_service.get_all_expenses()) == 3

        self.expense_service.filter_expenses_by_amount(400)
        assert len(self.expense_service.get_all_expenses()) == 0

    def test_undo_last_operation(self):
        initial_expense_data = self.expense_service.get_all_expenses()
        self.expense_service.add_expense(1, 100, "food")
        self.expense_service.add_expense(2, 200, "transport")

        assert (
            len(self.expense_service.get_all_expenses())
            == len(initial_expense_data) + 2
        )

        self.expense_service.undo_last_operation()
        assert (
            len(self.expense_service.get_all_expenses())
            == len(initial_expense_data) + 1
        )

        self.expense_service.undo_last_operation()
        assert len(self.expense_service.get_all_expenses()) == len(initial_expense_data)

    def run_all_tests(self):
        self.test_add_expense()
        self.test_filter_expenses_by_amount()
        self.test_undo_last_operation()
