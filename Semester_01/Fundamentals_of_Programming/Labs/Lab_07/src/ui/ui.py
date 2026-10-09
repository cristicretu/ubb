from lib.helpers import (
    read_valid_amount,
    read_valid_day,
    read_valid_integer,
    read_valid_type,
)
from services.expense_service import ExpenseService


class UI:
    def __init__(self, expense_repository) -> None:
        self.__expense_service = ExpenseService(expense_repository)

    def __print_menu(self):
        print()
        print("1. Add expense")
        print("2. Show all expenses")
        print("3. Filter expenses by amount")
        print("4. Undo")
        print("0. Exit")
        print()

    def __add_expense(self):
        day = read_valid_day("Day: ")
        amount = read_valid_amount("Amount: ")
        type = read_valid_type("Type: ")

        self.__expense_service.add_expense(day, amount, type)

    def __show_all_expenses(self):
        if not self.__expense_service.get_all_expenses():
            print("No more expenses!")
            return

        for expense in self.__expense_service.get_all_expenses():
            print(expense)

    def __filter_expenses_by_amount(self):
        amount = read_valid_amount("Amount: ")

        self.__expense_service.filter_expenses_by_amount(amount)

    def __undo(self):
        self.__expense_service.undo_last_operation()

    def run(self):
        ADD_EXPENSE = 1
        SHOW_ALL_EXPENSES = 2
        FILTER_EXPENSES_BY_AMOUNT = 3
        UNDO = 4
        EXIT = 0

        commands = {
            ADD_EXPENSE: self.__add_expense,
            SHOW_ALL_EXPENSES: self.__show_all_expenses,
            FILTER_EXPENSES_BY_AMOUNT: self.__filter_expenses_by_amount,
            UNDO: self.__undo,
        }

        while True:
            self.__print_menu()
            command = read_valid_integer("Command: ")

            if command == EXIT:
                break

            if command not in commands:
                print("Invalid command!")
                continue

            try:
                commands[command]()
            except Exception as exception_error:
                print(exception_error)
