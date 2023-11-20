import functions
import datetime

def test_add_transaction():
    transactions = []
    assert len(transactions) == 0

    transactions = functions.add_transaction(transactions, 100, "in", "salary")
    assert transactions == [
        {
            "day": datetime.datetime.now().day % 30,
            "amount": 100,
            "type": "in",
            "description": "salary"
        }
    ]

    transactions = functions.add_transaction(transactions, 200, "out", "rent")
    assert transactions == [
        {
            "day": datetime.datetime.now().day % 30,
            "amount": 100,
            "type": "in",
            "description": "salary"
        },
        {
            "day": datetime.datetime.now().day % 30,
            "amount": 200,
            "type": "out",
            "description": "rent"
        }
    ]

    assert len(transactions) == 2

def test_all_functions() -> None:
    test_add_transaction()
