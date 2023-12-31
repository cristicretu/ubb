#
# This is the program's UI module. The user interface and all interaction with the user (print and input statements) are found here
#
import re
import functions
import random
import termtables as tt


def print_help() -> None:
    """
    This menu will be displayed when the user types 'help'.
    """
    print("The following commands are available:\n")
    print("add <value> <type> <description>")
    print("insert <day> <value> <type> <description>")
    print(
        "Add to the list of expenses. <value> is the expense's value, <type> is the expense's type and <description> is the expense's description.\n"
    )
    print("remove <day>")
    print("remove <start day> to <end day>")
    print("remove <type>")
    print("replace <day> <type> <description> with <new value>")
    print(
        "Modifies the list of expenses. <day> is the day of the expense to be removed, <start day> and <end day> are the start and end days of the interval of expenses to be removed, <type> is the type of the expense to be removed, <description> is the description of the expense to be removed, <new value> is the new value of the expense to be replaced.\n"
    )
    print("list")
    print("list <type>")
    print("list [ < | = | > ] <value>")
    print("list balance <day>")
    print(
        "Display transactions having different proprieties. <type> is the type of the expenses to be displayed, <value> is the value of the expenses to be displayed, <day> is the day of the balance to be displayed.\n"
    )
    print("filter <type>")
    print("filter <type> <value>")
    print(
        "Filter the list of expenses. <type> is the type of the expenses to be filtered, <value> is the value of the expenses to be filtered.\n"
    )
    print("undo")
    print("Undo the last operation.\n")


def print_hint_message() -> None:
    """
    This menu will be displayed after each command is executed.
    """
    print(
        "\nType a command. Type 'help' for a list of commands. Type 'exit' to exit the program.\n"
    )


class CommandValidator:
    ADD_TRANSACTION_PATTERN = r"^add\s+(\d+)\s+(in|out)\s+(.+)$"
    INSERT_TRANSACTION_PATTERN = (
        r"^insert\s+([1-9]|[1-2][0-9]|30)\s+(\d+)\s+(in|out)\s+(.+)$"
    )
    REMOVE_DAY_PATTERN = r"^remove\s+([1-9]|[1-2][0-9]|30)$"
    REMOVE_RANGE_PATTERN = (
        r"^remove\s+([1-9]|[1-2][0-9]|30)\s+to\s+([1-9]|[1-2][0-9]|30)$"
    )
    REMOVE_TYPE_PATTERN = r"^remove\s+(in|out)$"
    REPLACE_PATTERN = (
        r"^replace\s+([1-9]|[1-2][0-9]|30)\s+(in|out)\s+(.+)\s+with\s+(\d+)$"
    )
    LIST_PATTERN = r"^list$"
    LIST_TYPE_PATTERN = r"^list\s+(in|out)$"
    LIST_VALUE_PATTERN = r"^list\s+([<>]|[=])\s+(\d+)$"
    LIST_BALANCE_PATTERN = r"^list\s+balance\s+([1-9]|[1-2][0-9]|30)$"
    FILTER_PATTERN = r"^filter\s+(in|out)$"
    FILTER_VALUE_PATTERN = r"^filter\s+(in|out)\s+(\d+)$"
    UNDO_PATTERN = r"^undo$"
    HELP_PATTERN = r"^help$"
    EXIT_PATTERN = r"^exit$"

    patterns = [
        (ADD_TRANSACTION_PATTERN, "add"),
        (INSERT_TRANSACTION_PATTERN, "insert"),
        (REMOVE_DAY_PATTERN, "remove"),
        (REMOVE_RANGE_PATTERN, "remove"),
        (REMOVE_TYPE_PATTERN, "remove"),
        (REPLACE_PATTERN, "replace"),
        (LIST_PATTERN, "list"),
        (LIST_TYPE_PATTERN, "list"),
        (LIST_VALUE_PATTERN, "list"),
        (LIST_BALANCE_PATTERN, "list"),
        (FILTER_PATTERN, "filter"),
        (FILTER_VALUE_PATTERN, "filter"),
        (UNDO_PATTERN, "undo"),
        (HELP_PATTERN, "help"),
        (EXIT_PATTERN, "exit"),
    ]

    @staticmethod
    def validate_command(potential_command: str) -> bool:
        """
        Checks if a command is valid.
        The following commands are valid:
            - add <value> <type> <description>
            - insert <day> <value> <type> <description>
            - remove <day>
            - remove <start day> to <end day>
            - remove <type>
            - replace <day> <type> <description> with <new value>
            - list
            - list <type>
            - list [ < | = | > ] <value>
            - list balance <day>
            - filter <type>
            - filter <type> <value>
            - undo
            - exit

        :param command: The command to be checked.
        :return: True if the command is valid, False otherwise.
        """

        for pattern, _ in CommandValidator.patterns:
            if re.match(pattern, potential_command):
                return True

        return False

    @staticmethod
    def parse_command(unparsed_command: str):
        """
        Parses the command into its components using regular expressions.

        :param command: The command to be parsed.
        :return: A tuple containing the action (e.g., 'add', 'insert', etc.) and its parameters.
        """

        for pattern, action in CommandValidator.patterns:
            match = re.match(pattern, unparsed_command)
            if match:
                return action, match.groups()

        return None, None


def check_if_valid_day_using_regular_expressions(potential_day: str) -> bool:
    """
    Check if the given string day is a valid day. For simplicity, it should be a positive integer

    :param potential_day: The string day to be checked.
    :return: True if the day is valid, False otherwise.
    """

    # Either 0, or 1-9, or 10-29, or 30
    return bool(re.match(r"^([1-9]|[1-2][0-9]|30)$", potential_day))


def check_if_valid_integer_value_using_regular_expressions(
    potential_number: str,
) -> bool:
    """
    Checks if the given value is a valid integer. It should also be positive.

    :param potential_number: The value to be checked.
    :return: True if the value is valid, False otherwise.
    """

    # Either 0, or a positive integer
    return bool(re.match(r"^(0|[1-9][0-9]*)$", potential_number))


def check_if_valid_type_using_regular_expressions(potential_type: str) -> bool:
    """
    Checks if the given type is valid. It should be a string of letters.
    Type should either be `in` or `out`.

    :param potential_type: The type to be checked.
    :return: True if the type is valid, False otherwise.
    """

    # Either in or out
    return bool(re.match(r"^(in|out)$", potential_type))


def read_command_from_user() -> str:
    """
    Reads a command from the user.
    """
    command_validator = CommandValidator()
    while True:
        command = input("\n>>> ")
        if command_validator.validate_command(command) is False:
            print("\nInvalid command. Type 'help' for a list of commands.\n")
            continue
        else:
            return command


def print_transactions_table(transactions_list, value, operation):
    transactions = functions.list_all_transactions_by_amount(
        transactions_list, value, operation
    )
    if len(transactions) < 1:
        print("No transactions match the criteria.")
    else:
        header_of_table = ["Day", "Amount", "Type", "Description"]
        data_to_display = [
            [t["day"], t["amount"], t["type"], t["description"]] for t in transactions
        ]
        tt.print(data_to_display, header=header_of_table, padding=(0, 1, 0, 1))


def perform_bank_operations_from_given_command(
    command: str,
    command_validator: CommandValidator,
    transactions_list: list,
    transaction_history_stack: list,
) -> int:
    """
    This function performs the bank operations based on the given command.
    This is useful both for the user interface and for the testing part.

    :param command: The command to be performed.
    :param command_validator: The command validator.
    :param transactions_list: The list of transactions.

    :return: An exit code indicating the status of the program.
    0 means the program should end (after success)
    1 means the program should end (after failure)
    2 means the program should continue
    """

    END_PROGRAM_SUCCESS = 0
    END_PROGRAM_FAILURE = 1
    CONTINUE_PROGRAM = 2

    try:
        # Parse the command
        parsed_command = command_validator.parse_command(command)
        if parsed_command is None:
            raise ValueError("Command could not be parsed.")

        command_action, command_params = parsed_command

        # only python >= 3.10
        match command_action:
            # A - Add transaction
            case "add":
                if command_params and len(command_params) >= 3:
                    amount = int(command_params[0])
                    transaction_type = command_params[1]
                    description = command_params[2]
                    functions.add_transaction(
                        transactions_list,
                        amount,
                        transaction_type,
                        description,
                        transaction_history_stack,
                    )
                    print(
                        f"Added transaction: {amount} {transaction_type} '{description}'"
                    )
                else:
                    raise ValueError("Insufficient parameters for 'add' command.")

                return CONTINUE_PROGRAM

            case "insert":
                if command_params and len(command_params) >= 4:
                    day = int(command_params[0])
                    amount = int(command_params[1])
                    transaction_type = command_params[2]
                    description = command_params[3]
                    functions.insert_transaction_for_a_day(
                        transactions_list,
                        day,
                        amount,
                        transaction_type,
                        description,
                        transaction_history_stack,
                    )
                    print(
                        f"Inserted transaction: {day} {amount} {transaction_type} '{description}'"
                    )
                else:
                    raise ValueError("Insufficient parameters for 'insert' command.")

                return CONTINUE_PROGRAM

            # B - Modify transaction

            case "remove":
                if command_params is None or len(command_params) == 0:
                    raise ValueError("Insufficient parameters for 'remove' command.")

                if len(command_params) == 1:
                    if check_if_valid_day_using_regular_expressions(command_params[0]):
                        day = int(command_params[0])
                        functions.remove_transaction_by_day(
                            transactions_list, day, transaction_history_stack
                        )
                        print(f"Removed all transactions for day {day}")
                    elif check_if_valid_type_using_regular_expressions(
                        command_params[0]
                    ):
                        transaction_type = command_params[0]
                        functions.remove_transactions_by_type(
                            transactions_list,
                            transaction_type,
                            transaction_history_stack,
                        )
                        print(f"Removed all transactions of type {transaction_type}")
                elif len(command_params) == 2:
                    start_day = int(command_params[0])
                    end_day = int(command_params[1])
                    functions.remove_transactions_in_between_days(
                        transactions_list, start_day, end_day, transaction_history_stack
                    )
                    print(
                        f"Removed all transactions between days {start_day} and {end_day}"
                    )
                else:
                    raise ValueError("Invalid parameters for 'remove' command.")

                return CONTINUE_PROGRAM

            case "replace":
                if command_params is None or len(command_params) == 0:
                    raise ValueError("Insufficient parameters for 'replace' command.")

                day = int(command_params[0])
                transaction_type = command_params[1]
                description = command_params[2]
                new_amount = int(command_params[3])
                functions.replace_transaction(
                    transactions_list,
                    day,
                    transaction_type,
                    description,
                    new_amount,
                    transaction_history_stack,
                )
                print(
                    f"Replaced transaction: {day} {transaction_type} '{description}' with {new_amount}"
                )

                return CONTINUE_PROGRAM

            # C - List transactions

            case "list":
                if command_params is None or len(command_params) == 0:
                    if transactions_list:
                        header_of_table = ["Day", "Amount", "Type", "Description"]
                        data_to_display = [
                            [t["day"], t["amount"], t["type"], t["description"]]
                            for t in functions.list_all_transactions(transactions_list)
                        ]

                        tt.print(
                            data_to_display,
                            header=header_of_table,
                            padding=(0, 1, 0, 1),
                        )
                    else:
                        print("There are no transactions to display.")
                elif len(command_params) == 1:
                    if transactions_list:
                        if check_if_valid_type_using_regular_expressions(
                            command_params[0]
                        ):
                            transaction_type = command_params[0]
                            header_of_table = ["Day", "Amount", "Description"]
                            data_to_display = [
                                [t["day"], t["amount"], t["description"]]
                                for t in functions.list_all_transactions_by_type(
                                    transactions_list, transaction_type
                                )
                            ]

                            tt.print(
                                data_to_display,
                                header=header_of_table,
                                padding=(0, 1, 0, 1),
                            )
                        if check_if_valid_day_using_regular_expressions(
                            command_params[0]
                        ):
                            day = int(command_params[0])

                            print(
                                f"The sum of the amounts until day {day} is {functions.list_account_balance_by_day(transactions_list, day)}"
                            )
                    else:
                        print("There are no transactions to display.")
                elif len(command_params) == 2:
                    if check_if_valid_integer_value_using_regular_expressions(
                        command_params[1]
                    ):
                        value = int(command_params[1])
                        if command_params[0] in ("<", "=", ">"):
                            print_transactions_table(
                                transactions_list, value, command_params[0]
                            )
                        else:
                            raise ValueError("Invalid parameters for 'list' command.")
                    elif check_if_valid_day_using_regular_expressions(
                        command_params[1]
                    ):
                        if command_params[0] == "balance":
                            day = int(command_params[1])
                            print(
                                f"The balance for day {day} is {functions.get_balance_for_a_day(transactions_list, day)}"
                            )
                        else:
                            raise ValueError("Invalid parameters for 'list' command.")

                return CONTINUE_PROGRAM

            # D - Filter transactions

            case "filter":
                if command_params is None or len(command_params) == 0:
                    raise ValueError("Insufficient parameters for 'filter' command.")

                transaction_type = command_params[0]
                if len(command_params) == 1:
                    transactions_list = functions.filter_transactions_by_type(
                        transactions_list, transaction_type, transaction_history_stack
                    )
                elif len(command_params) == 2:
                    transactions_list = functions.filter_transactions_by_smaller_amount(
                        transactions_list,
                        transaction_type,
                        int(command_params[1]),
                        transaction_history_stack,
                    )
                else:
                    raise ValueError("Invalid parameters for 'filter' command.")

                return CONTINUE_PROGRAM

            # E - Undo

            case "undo":
                transactions_list = functions.undo_last_transaction(
                    transactions_list, transaction_history_stack
                )
                print("Last transaction was undone.")

                return CONTINUE_PROGRAM

            case "help":
                print_help()
                return CONTINUE_PROGRAM

            case "exit":
                print("Exiting...")
                return END_PROGRAM_SUCCESS

            case _:
                raise ValueError(f"Unknown command action: {command_action}")

    except ValueError as value_error:
        print(f"Error: {value_error}")
    except IndexError as index_error:
        print(f"Error: {index_error}")
    except Exception as exception_error:
        print(f"An unexpected error occurred: {exception_error}")

    return END_PROGRAM_FAILURE


def create_10_random_transactions(
    transactions_list: list, transaction_history_stack: list
) -> None:
    """
    Creates 10 random transactions and adds them to the transactions list.

    :param transactions_list: The list of transactions.
    :param transaction_history_stack: The stack of transactions.
    """

    for _ in range(10):
        day = random.randint(1, 30)
        amount = random.randint(1, 1000)
        transaction_type = random.choice(["in", "out"])
        description = random.choice(
            ["salary", "rent", "food", "clothes", "car", "gas", "games"]
        )
        functions.insert_transaction_for_a_day(
            transactions_list,
            day,
            amount,
            transaction_type,
            description,
            transaction_history_stack,
        )


def print_ui() -> None:
    print("Welcome to the Bank Account!")

    command_validator = CommandValidator()
    bank_account_transactions = []
    transaction_history_stack = []

    create_10_random_transactions(bank_account_transactions, transaction_history_stack)

    END_PROGRAM_SUCCESS = 0
    END_PROGRAM_FAILURE = 1

    while True:
        print_hint_message()
        command_from_user = read_command_from_user()

        exit_code = perform_bank_operations_from_given_command(
            command_from_user,
            command_validator,
            bank_account_transactions,
            transaction_history_stack,
        )

        if exit_code == END_PROGRAM_SUCCESS or exit_code == END_PROGRAM_FAILURE:
            break
