#
# The program's functions are implemented here. There is no user interaction in this file, therefore no input/print statements. Functions here
# communicate via function parameters, the return statement and raising of exceptions.
#
import datetime
from copy import deepcopy


def push_transaction_history_stack(
    transaction_history_stack: list, transactions: list
) -> list:
    """
    Pushes the current transactions list to the transaction history stack.

    :param transaction_history_stack: The transaction history stack.
    :param transactions: The list of transactions.
    :return: The transaction history stack with the current transactions list pushed.
    """
    transaction_history_stack.append(deepcopy(transactions))

    return transaction_history_stack


def add_transaction(
    transactions: list,
    transaction_amount: int,
    transaction_type: str,
    transaction_description: str,
    transaction_history_stack: list,
) -> list:
    """
    Adds a transaction to the list of transactions.

    :param transactions: The list of transactions.
    :param day: The day of the transaction.
    :param amount: The amount of the transaction.
    :param transaction_type: The type of the transaction.
    :param description: The description of the transaction.
    :param transaction_history_stack: The transaction history stack.
    :return: The list of transactions with the new transaction added.
    """
    if transaction_amount <= 0:
        raise ValueError("Invalid amount. Must be positive.")

    if transaction_type not in ["in", "out"]:
        raise ValueError("Invalid transaction type. Must be either 'in' or 'out'.")

    if transaction_description == "":
        raise ValueError("Invalid description. Must not be empty.")

    transaction = {
        # Current day of the month % 30
        "day": datetime.datetime.now().day % 30,
        "amount": transaction_amount,
        "type": transaction_type,
        "description": transaction_description,
    }

    push_transaction_history_stack(transaction_history_stack, transactions)

    transactions.append(transaction)

    return transactions


def insert_transaction_for_a_day(
    transactions: list,
    transaction_day: int,
    transaction_amount: int,
    transaction_type: str,
    transaction_description: str,
    transaction_history_stack: list,
) -> list:
    """
    Updates the transaction for a given day.

    :param transactions: The list of transactions.
    :param day: The day of the transaction.
    :param amount: The amount of the transaction.
    :param transaction_type: The type of the transaction.
    :param description: The description of the transaction.
    :param transaction_history_stack: The transaction history stack.
    :return: The list of transactions with the new transaction added.
    """
    if transaction_day < 1 or transaction_day > 31:
        raise ValueError("Invalid day. Must be between 1 and 31.")

    if transaction_amount <= 0:
        raise ValueError("Invalid amount. Must be positive.")

    if transaction_type not in ["in", "out"]:
        raise ValueError("Invalid transaction type. Must be either 'in' or 'out'.")

    if transaction_description == "":
        raise ValueError("Invalid description. Must not be empty.")

    transaction = {
        "day": transaction_day,
        "amount": transaction_amount,
        "type": transaction_type,
        "description": transaction_description,
    }

    push_transaction_history_stack(transaction_history_stack, transactions)

    transactions.append(transaction)

    return transactions


def remove_transaction_by_day(
    transactions: list, transaction_day: int, transaction_history_stack: list
) -> list:
    """
    Removes all transactions for a given day.

    :param transactions: The list of transactions.
    :param day: The day of the transaction.
    :param transaction_history_stack: The transaction history stack.
    :return: The list of transactions with the transactions for the given day removed.
    """
    if transaction_day < 1 or transaction_day > 31:
        raise ValueError("Invalid day. Must be between 1 and 31.")

    push_transaction_history_stack(transaction_history_stack, transactions)

    # Remove all transactions for the given day
    transactions[:] = [
        transaction
        for transaction in transactions
        if transaction["day"] != transaction_day
    ]

    return transactions


def remove_transactions_in_between_days(
    transactions: list,
    transaction_start_day: int,
    transaction_end_day: int,
    transaction_history_stack: list,
) -> list:
    """
    Removes all transactions for a given day.

    :param transactions: The list of transactions.
    :param start_day: The start day of the transaction.
    :param end_day: The end day of the transaction.
    :param transaction_history_stack: The transaction history stack.
    :return: The list of transactions with the transactions for the given day removed.
    """
    if transaction_start_day > transaction_end_day:
        raise ValueError("Invalid days. Start day must be less than end day.")

    if transaction_start_day < 1 or transaction_start_day > 31:
        raise ValueError("Invalid day. Must be between 1 and 31.")
    if transaction_end_day < 1 or transaction_end_day > 31:
        raise ValueError("Invalid day. Must be between 1 and 31.")

    push_transaction_history_stack(transaction_history_stack, transactions)

    # Remove all transactions for the given day
    transactions[:] = [
        transaction
        for transaction in transactions
        if transaction["day"] < transaction_start_day
        or transaction["day"] > transaction_end_day
    ]

    return transactions


def remove_transactions_by_type(
    transactions: list, transaction_type: str, transaction_history_stack: list
) -> list:
    """
    Removes all transactions of a given type.

    :param transactions: The list of transactions.
    :param transaction_type: The type of the transaction.
    :param transaction_history_stack: The transaction history stack.
    :return: The list of transactions with the transactions of the given type removed.
    """
    if transaction_type not in ["in", "out"]:
        raise ValueError("Invalid transaction type. Must be either 'in' or 'out'.")

    push_transaction_history_stack(transaction_history_stack, transactions)

    # Remove all transactions of the given type
    transactions[:] = [
        transaction
        for transaction in transactions
        if transaction["type"] != transaction_type
    ]

    return transactions


def replace_transaction(
    transactions: list,
    transaction_day: int,
    transaction_type: str,
    transaction_description: str,
    transaction_value: str,
    transaction_history_stack: list,
):
    """
    `replace 12 in salary with 2000` - replace the amount for the `in` transaction having the *“salary”* description from day 12 with `2000 RON

    :param transaction: The list of transactions.
    :param day: The day of the transaction.
    :param type: The type of the transaction.
    :param description: The description of the transaction.
    :param value: The value of the transaction.
    :param transaction_history_stack: The transaction history stack.
    :return: The list of transactions with the transactions of the given type removed.
    """

    if transaction_day < 1 or transaction_day > 31:
        raise ValueError("Invalid day. Must be between 1 and 31.")

    if transaction_type not in ["in", "out"]:
        raise ValueError("Invalid transaction type. Must be either 'in' or 'out'.")

    if transaction_description == "":
        raise ValueError("Invalid description. Must not be empty.")

    transaction = {
        "day": transaction_day,
        "type": transaction_type,
        "description": transaction_description,
        "value": transaction_value,
    }

    push_transaction_history_stack(transaction_history_stack, transactions)

    for transaction in transactions:
        if (
            transaction["day"] == transaction_day
            and transaction["type"] == transaction_type
            and transaction["description"].lower() == transaction_description.lower()
        ):
            transaction["amount"] = transaction_value
            break
    else:
        raise ValueError("Transaction not found.")

    return transactions


def list_all_transactions(transactions: list) -> list:
    """
    Lists all transactions.

    :param transactions: The list of transactions.
    :return: The list of transactions.
    """
    return transactions


def list_all_transactions_by_type(transactions: list, transaction_type: str) -> list:
    """
    Lists all transactions of a given type.

    :param transactions: The list of transactions.
    :param transaction_type: The type of the transaction.
    :return: The list of transactions of the given type.
    """
    if transaction_type not in ["in", "out"]:
        raise ValueError("Invalid transaction type. Must be either 'in' or 'out'.")

    return [
        transaction
        for transaction in transactions
        if transaction["type"] == transaction_type
    ]


def list_all_transactions_by_amount(
    transactions: list, transaction_amount: int, transaction_operator: str
) -> list:
    """
    Lists all transactions of a given amount.

    :param transactions: The list of transactions.
    :param amount: The amount of the transaction.
    :return: The list of transactions of the given amount.
    """
    if transaction_amount <= 0:
        raise ValueError("Invalid amount. Must be positive.")

    if transaction_operator not in ["<", "=", ">"]:
        raise ValueError("Invalid operator. Must be either '<', '=' or '>'.")

    if transaction_operator == "<":
        return [
            transaction
            for transaction in transactions
            if transaction["amount"] < transaction_amount
        ]
    elif transaction_operator == "=":
        return [
            transaction
            for transaction in transactions
            if transaction["amount"] == transaction_amount
        ]
    elif transaction_operator == ">":
        return [
            transaction
            for transaction in transactions
            if transaction["amount"] > transaction_amount
        ]

    return []


def list_account_balance_by_day(transactions: list, transaction_day: int) -> int:
    """
    Lists the account balance for a given day.

    :param transactions: The list of transactions.
    :param day: The day of the transaction.
    :return: The account balance for the given day.
    """
    if transaction_day < 1 or transaction_day > 31:
        raise ValueError("Invalid day. Must be between 1 and 31.")

    return sum(
        transaction["amount"] if transaction["type"] == "in" else -transaction["amount"]
        for transaction in transactions
        if transaction["day"] <= transaction_day
    )


def filter_transactions_by_type(
    transactions: list, transaction_type: str, transaction_history_stack: list
) -> list:
    """
    Filters transactions by type.

    :param transactions: The list of transactions.
    :param transaction_type: The type of the transaction.
    :param transaction_history_stack: The transaction history stack.
    :return: The list of transactions of the given type.
    """
    if transaction_type not in ["in", "out"]:
        raise ValueError("Invalid transaction type. Must be either 'in' or 'out'.")

    push_transaction_history_stack(transaction_history_stack, transactions)

    for transaction_index in range(len(transactions) - 1, -1, -1):
        if transactions[transaction_index]["type"] != transaction_type:
            transactions.pop(transaction_index)

    return transactions


def filter_transactions_by_smaller_amount(
    transactions: list,
    transaction_type: str,
    transaction_amount: int,
    transaction_history_stack: list,
) -> list:
    """
    Filters transactions by amount.

    :param transactions: The list of transactions.
    :param transaction_type: The type of the transaction.
    :param amount: The amount of the transaction.
    :param transaction_history_stack: The transaction history stack.
    :return: The list of transactions of the given amount.
    """
    if transaction_type not in ["in", "out"]:
        raise ValueError("Invalid transaction type. Must be either 'in' or 'out'.")

    if transaction_amount <= 0:
        raise ValueError("Invalid amount. Must be positive.")

    push_transaction_history_stack(transaction_history_stack, transactions)

    transactions[:] = [
        transaction
        for transaction in transactions
        if transaction["type"] == transaction_type
        and transaction["amount"] < transaction_amount
    ]

    return transactions


def undo_last_transaction(transactions: list, transaction_history_stack: list) -> list:
    """
    Undo the last transaction.

    :param transactions: The list of transactions.
    :param transaction_history_stack: The transaction history stack.
    :return: The list of transactions with the last transaction undone.
    """
    if len(transaction_history_stack) == 0:
        raise ValueError("No more transactions to undo.")

    last_state = transaction_history_stack.pop()
    transactions.clear()
    transactions.extend(last_state)

    return transactions
