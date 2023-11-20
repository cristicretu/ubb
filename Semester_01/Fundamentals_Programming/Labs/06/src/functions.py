#
# The program's functions are implemented here. There is no user interaction in this file, therefore no input/print statements. Functions here
# communicate via function parameters, the return statement and raising of exceptions.
#
import datetime

def add_transaction(transactions: list, amount: int, transaction_type: str, description: str) -> list:
    """
    Adds a transaction to the list of transactions.

    :param transactions: The list of transactions.
    :param day: The day of the transaction.
    :param amount: The amount of the transaction.
    :param transaction_type: The type of the transaction.
    :param description: The description of the transaction.
    :return: The list of transactions with the new transaction added.
    """
    if amount <= 0:
        raise ValueError("Invalid amount. Must be positive.")

    if transaction_type not in ["in", "out"]:
        raise ValueError("Invalid transaction type. Must be either 'in' or 'out'.")

    if description == "":
        raise ValueError("Invalid description. Must not be empty.")

    transaction = {
        # Current day of the month % 30
        "day": datetime.datetime.now().day % 30,
        "amount": amount,
        "type": transaction_type,
        "description": description
    }

    transactions.append(transaction)

    return transactions

def insert_transaction_for_a_day(transactions: list, day: int, amount: int, transaction_type: str, description: str) -> list:
    """
    Updates the transaction for a given day.

    :param transactions: The list of transactions.
    :param day: The day of the transaction.
    :param amount: The amount of the transaction.
    :param transaction_type: The type of the transaction.
    :param description: The description of the transaction.
    :return: The list of transactions with the new transaction added.
    """
    if day < 1 or day > 31:
        raise ValueError("Invalid day. Must be between 1 and 31.")

    if amount <= 0:
        raise ValueError("Invalid amount. Must be positive.")

    if transaction_type not in ["in", "out"]:
        raise ValueError("Invalid transaction type. Must be either 'in' or 'out'.")

    if description == "":
        raise ValueError("Invalid description. Must not be empty.")

    transaction = {
        "day": day,
        "amount": amount,
        "type": transaction_type,
        "description": description
    }

    return transactions

def remove_transaction_by_day(transactions: list, day: int) -> list:
    """
    Removes all transactions for a given day.

    :param transactions: The list of transactions.
    :param day: The day of the transaction.
    :return: The list of transactions with the transactions for the given day removed.
    """
    if day < 1 or day > 31:
        raise ValueError("Invalid day. Must be between 1 and 31.")

    # Remove all transactions for the given day
    transactions[:] = [transaction for transaction in transactions if transaction["day"] != day]

    return transactions

def remove_transactions_in_between_days(transactions: list, start_day: int, end_day: int) -> list:
    """
    Removes all transactions for a given day.

    :param transactions: The list of transactions.
    :param start_day: The start day of the transaction.
    :param end_day: The end day of the transaction.
    :return: The list of transactions with the transactions for the given day removed.
    """
    if start_day > end_day:
        raise ValueError("Invalid days. Start day must be less than end day.")

    if start_day < 1 or start_day > 31:
        raise ValueError("Invalid day. Must be between 1 and 31.")
    if end_day < 1 or end_day > 31:
        raise ValueError("Invalid day. Must be between 1 and 31.")

    # Remove all transactions for the given day
    transactions[:] = [transaction for transaction in transactions if transaction["day"] < start_day or transaction["day"] > end_day]

    return transactions

def remove_transactions_by_type(transactions: list, transaction_type: str) -> list:
    """
    Removes all transactions of a given type.

    :param transactions: The list of transactions.
    :param transaction_type: The type of the transaction.
    :return: The list of transactions with the transactions of the given type removed.
    """
    if transaction_type not in ["in", "out"]:
        raise ValueError("Invalid transaction type. Must be either 'in' or 'out'.")

    # Remove all transactions of the given type
    transactions[:] = [transaction for transaction in transactions if transaction["type"] != transaction_type]

    return transactions

def replace_transaction(transactions: list, day: int, type: str, description: str, value: str):
    """
    `replace 12 in salary with 2000` – replace the amount for the `in` transaction having the *“salary”* description from day 12 with `2000 RON

    :param transaction: The list of transactions.
    :param day: The day of the transaction.
    :param type: The type of the transaction.
    :param description: The description of the transaction.
    :param value: The value of the transaction.
    :return: The list of transactions with the transactions of the given type removed.
    """

    if day < 1 or day > 31:
        raise ValueError("Invalid day. Must be between 1 and 31.")

    if type not in ["in", "out"]:
        raise ValueError("Invalid transaction type. Must be either 'in' or 'out'.")

    if description == "":
        raise ValueError("Invalid description. Must not be empty.")


    transaction = {
        "day": day,
        "type": type,
        "description": description,
        "value": value
    }

    for i in range(len(transactions)):
        if transactions[i]["day"] == day and transactions[i]["type"] == type and transactions[i]["description"] == description:
            transactions[i]["value"] = value

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

    return [transaction for transaction in transactions if transaction["type"] == transaction_type]

def list_all_transactions_by_amount(transactions: list, amount: int, operator: str) -> list:
    """
    Lists all transactions of a given amount.

    :param transactions: The list of transactions.
    :param amount: The amount of the transaction.
    :return: The list of transactions of the given amount.
    """
    if amount <= 0:
        raise ValueError("Invalid amount. Must be positive.")

    if operator not in ["<", "=", ">"]:
        raise ValueError("Invalid operator. Must be either '<', '=' or '>'.")

    if operator == "<":
        return [transaction for transaction in transactions if transaction["amount"] < amount]
    elif operator == "=":
        return [transaction for transaction in transactions if transaction["amount"] == amount]
    elif operator == ">":
        return [transaction for transaction in transactions if transaction["amount"] > amount]

    return []

def list_account_balance_by_day(transactions: list, day: int) -> int:
    """
    Lists the account balance for a given day.

    :param transactions: The list of transactions.
    :param day: The day of the transaction.
    :return: The account balance for the given day.
    """
    if day < 1 or day > 31:
        raise ValueError("Invalid day. Must be between 1 and 31.")

    return sum(transaction["amount"] if transaction["type"] == "in" else -transaction["amount"] for transaction in transactions if transaction["day"] <= day)

def filter_transactions_by_type(transactions: list, transaction_type: str) -> list:
    """
    Filters transactions by type.

    :param transactions: The list of transactions.
    :param transaction_type: The type of the transaction.
    :return: The list of transactions of the given type.
    """
    if transaction_type not in ["in", "out"]:
        raise ValueError("Invalid transaction type. Must be either 'in' or 'out'.")

    return [transaction for transaction in transactions if transaction["type"] == transaction_type]

def filter_transactions_by_smaller_amount(transactions: list, amount: int) -> list:
    """
    Filters transactions by amount.

    :param transactions: The list of transactions.
    :param amount: The amount of the transaction.
    :return: The list of transactions of the given amount.
    """
    if amount <= 0:
        raise ValueError("Invalid amount. Must be positive.")

    return [transaction for transaction in transactions if transaction["amount"] < amount]

def undo_last_transaction(transactions: list):
    """
    Undo the last transaction.

    :param transactions: The list of transactions.
    :param undo_transactions: The list of undo transactions.
    :return: The list of transactions with the last transaction undone.
    """
    if len(transactions) == 0:
        raise ValueError("No transactions to undo.")

    return transactions[:-1]
