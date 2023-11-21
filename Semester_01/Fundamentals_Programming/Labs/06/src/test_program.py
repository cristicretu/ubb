import functions
import datetime


def test_add_and_insert_transaction():
    transactions = []
    transactions_history_stack = []
    assert len(transactions) == 0

    transactions = functions.add_transaction(
        transactions, 100, "in", "salary", transactions_history_stack
    )
    assert transactions == [
        {
            "day": datetime.datetime.now().day % 30,
            "amount": 100,
            "type": "in",
            "description": "salary",
        }
    ]

    transactions = functions.add_transaction(
        transactions, 200, "out", "rent", transactions_history_stack
    )
    assert transactions == [
        {
            "day": datetime.datetime.now().day % 30,
            "amount": 100,
            "type": "in",
            "description": "salary",
        },
        {
            "day": datetime.datetime.now().day % 30,
            "amount": 200,
            "type": "out",
            "description": "rent",
        },
    ]

    assert len(transactions) == 2

    transactions = functions.insert_transaction_for_a_day(
        transactions, 12, 300, "out", "food", transactions_history_stack
    )

    transactions = functions.insert_transaction_for_a_day(
        transactions, 20, 400, "out", "food", transactions_history_stack
    )

    assert transactions == [
        {
            "day": datetime.datetime.now().day % 30,
            "amount": 100,
            "type": "in",
            "description": "salary",
        },
        {
            "day": datetime.datetime.now().day % 30,
            "amount": 200,
            "type": "out",
            "description": "rent",
        },
        {
            "day": 12,
            "amount": 300,
            "type": "out",
            "description": "food",
        },
        {
            "day": 20,
            "amount": 400,
            "type": "out",
            "description": "food",
        },
    ]


def test_modify_transaction():
    transactions = []
    transactions_history_stack = []

    transactions = functions.insert_transaction_for_a_day(
        transactions, 12, 300, "out", "food", transactions_history_stack
    )

    transactions = functions.insert_transaction_for_a_day(
        transactions, 20, 400, "out", "food", transactions_history_stack
    )

    transactions = functions.remove_transaction_by_day(
        transactions, 12, transactions_history_stack
    )

    assert transactions == [
        {
            "day": 20,
            "amount": 400,
            "type": "out",
            "description": "food",
        },
    ]

    transactions = functions.remove_transactions_by_type(
        transactions, "out", transactions_history_stack
    )

    assert transactions == []

    transactions = functions.insert_transaction_for_a_day(
        transactions, 12, 300, "in", "food", transactions_history_stack
    )

    transactions = functions.insert_transaction_for_a_day(
        transactions, 15, 400, "out", "food", transactions_history_stack
    )

    transactions = functions.insert_transaction_for_a_day(
        transactions, 20, 400, "out", "food", transactions_history_stack
    )

    transactions = functions.remove_transactions_in_between_days(
        transactions, 12, 15, transactions_history_stack
    )

    assert transactions == [
        {
            "day": 20,
            "amount": 400,
            "type": "out",
            "description": "food",
        },
    ]

    transactions = functions.replace_transaction(
        transactions, 20, "out", "food", 1337, transactions_history_stack
    )

    assert transactions == [
        {
            "day": 20,
            "amount": 1337,
            "type": "out",
            "description": "food",
        },
    ]


def test_all_functions() -> None:
    test_add_and_insert_transaction()
    test_modify_transaction()
