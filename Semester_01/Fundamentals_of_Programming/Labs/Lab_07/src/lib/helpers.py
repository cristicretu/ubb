def read_valid_day(prompt: str) -> int:
    """
    Reads a valid day from the keyboard.

    :param prompt: The prompt to be displayed to the user.
    :return: The day read from the keyboard.
    """
    while True:
        try:
            day = int(input(prompt))
            if day < 1 or day > 31:
                raise ValueError
            return day
        except ValueError:
            print("Invalid day! Please try again.")


def read_valid_integer(prompt: str) -> int:
    """
    Reads a valid integer from the keyboard.

    :param prompt: The prompt to be displayed to the user.
    :return: The integer read from the keyboard.
    """
    while True:
        try:
            integer = int(input(prompt))
            return integer
        except ValueError:
            print("Invalid integer! Please try again.")


def read_valid_amount(prompt: str) -> int:
    """
    Reads a valid amount from the keyboard.

    :param prompt: The prompt to be displayed to the user.
    :return: The amount read from the keyboard.
    """
    while True:
        try:
            amount = int(input(prompt))
            if amount < 0:
                raise ValueError
            return amount
        except ValueError:
            print("Invalid amount! Please try again.")


def read_valid_type(prompt: str) -> str:
    """
    Reads a valid type from the keyboard.

    :param prompt: The prompt to be displayed to the user.
    :return: The type read from the keyboard.
    """
    while True:
        type = input(prompt)
        if type == "":
            print("Invalid type! Please try again.")
        else:
            return type
