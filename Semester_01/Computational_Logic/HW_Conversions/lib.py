# @author: Crețu Cristian, Group 913

"""
Lib contains all helper functions for the program, including
`add_in_base_p`, `subtract_in_base_p`, `multiply_in_base_p`, `divide_in_base_p`,
for operations in base `p`, and
`convert_number_with_substitution_method`, `convert_a_number_with_successive_divisions`,
`convert_a_number_using_10_as_intermediary_base`, and `rapid_conversion` for conversions between bases.
"""


def check_if_valid_base(base: int) -> bool:
    """
    Checks if a given base is valid.
    A base is valid if it is an integer within the set {2,3,4,5,6,7,8,9,10,16}.

    :param base: The base to be checked.
    :return: True if the base is valid, False otherwise.
    """
    return base in {2, 3, 4, 5, 6, 7, 8, 9, 10, 16}


def get_digit_mappings(base: int) -> (dict, dict):
    """
    Generates two dictionaries for digit-to-value and value-to-digit mappings based on the given base.

    :param base: The base for which the mappings are to be created.
    :return: A tuple containing two dictionaries, one for digit-to-value and another for value-to-digit mappings.

    The digit-to-value dictionary maps a digit to its decimal equivalent.
    The value-to-digit dictionary maps a decimal value to its digit equivalent.
    """
    if check_if_valid_base(base) is False:
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    hex_digits = "0123456789ABCDEF"
    value_to_digit = {index: hex_digits[index] for index in range(base)}
    digit_to_value = {ch: index for index, ch in value_to_digit.items()}

    return digit_to_value, value_to_digit


def add_in_base_p(number1: str, number2: str, base: int) -> str:
    """
    Adds two numbers represented as strings in a given base p.

    :param number1: The first number as a string.
    :param number2: The second number as a string.
    :param base: The base in which the numbers are represented, and the result should be.
                 Must be an integer within the set {2,3,4,5,6,7,8,9,10,16}.
    :return: The result of the addition as a string in the same base.

    The function performs the addition by converting each digit to its decimal equivalent,
    adding them with carry, and then converting back to the base `p` representation.

    Pseudocode:

    1. Make both numbers the same length by adding leading zeros to the shorter one.
    2. Start iterations from the rightmost digit.
    3. Convert the current digit to decimal.
    4. Add the digits and account for carry.
    5. Save the result.

    if check_if_valid_base(base) is False:
        stop the program and raise an error

    result <- ""
    carry <- 0

    for i <- max_length - 1 to 0:
        digit1 <- convert number1[i] to decimal
        digit2 <- convert number2[i] to decimal

        total <- digit1 + digit2 + carry
        carry <- total // base
        result_digit <- total % base

        result <- convert result_digit to base p + result

    if carry > 0:
        result <- convert carry to base p + result

    return result
    """

    if check_if_valid_base(base) is False:
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    # Make both numbers the same length
    max_length = max(len(number1), len(number2))
    number1 = number1.zfill(max_length)
    number2 = number2.zfill(max_length)

    result = ""
    carry = 0

    # digit mapping
    map_value_to_digit, map_value_to_char = get_digit_mappings(base)

    # Start iterations from the rightmost digit
    for i in range(max_length - 1, -1, -1):
        # Convert the digit to decimal
        digit1 = map_value_to_digit[number1[i]]
        digit2 = map_value_to_digit[number2[i]]

        # Iteration  `i`
        total = digit1 + digit2 + carry
        carry = total // base
        result_digit = total % base

        # Save the result
        result = map_value_to_char[result_digit] + result

    # Add carry from the latest iteration
    if carry > 0:
        result = map_value_to_char[carry] + result

    return result


def subtract_in_base_p(number1: str, number2: str, base: int) -> str:
    """
    Subtracts two numbers represented as strings in a given base p.

    :param number1: The first number as a string, representing the minuend.
    :param number2: The second number as a string, representing the subtrahend.
    :param base: The base in which the numbers are represented, and the result should be.
                 Must be an integer within the set {2,3,4,5,6,7,8,9,10,16}.
    :return: The result of the subtraction as a string in the same base.

    The function performs the subtraction by converting each digit to its decimal equivalent,
    subtracting them with borrow, and then converting back to the base `p` representation.

    Pseudocode:

    1. Make both numbers the same length by adding leading zeros to the shorter one.
    2. Check if the result will be negative.
    3. Start iterations from the rightmost digit.
    4. Convert the current digit to decimal.
    5. Subtract the digits and account for borrow.
    6. Save the result.

    if check_if_valid_base(base) is False:
        stop the program and raise an error

    result <- ""
    borrow <- 0

    for i <- max_length - 1 to 0:
        digit1 <- convert number1[i] to decimal
        digit2 <- convert number2[i] to decimal

        total <- digit1 - digit2 - borrow
        if total < 0:
            total += base
            borrow <- 1
        else:
            borrow <- 0

        result_digit <- total % base

        result <- convert result_digit to base p + result

    return result
    """

    if check_if_valid_base(base) is False:
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    map_value_to_digit, map_value_to_char = get_digit_mappings(base)

    # Make both numbers the same length
    max_length = max(len(number1), len(number2))
    number1 = number1.zfill(max_length)
    number2 = number2.zfill(max_length)

    # Check if the result will be negative
    if number1 < number2:
        number1, number2 = number2, number1
        negative_result = True
    else:
        negative_result = False

    result = ""
    borrow = 0

    # Start iterations from the rightmost digit
    for i in range(max_length - 1, -1, -1):
        # Convert the digit to decimal
        digit1 = map_value_to_digit[number1[i]]
        digit2 = map_value_to_digit[number2[i]]

        # Subtract the digits and account for borrow
        total = digit1 - digit2 - borrow
        if total < 0:
            total += base
            borrow = 1
        else:
            borrow = 0

        result_digit = total % base

        # Save the result
        result = map_value_to_char[result_digit] + result

    # Remove leading zeros
    result = result.lstrip("0")

    # If result is empty, it means the numbers were equal
    if not result:
        result = "0"

    # Add minus sign if the result is negative
    if negative_result:
        result = "-" + result

    return result


def multiply_in_base_p(number1: str, number2: str, base: int) -> str:
    """
    Multiplies a number by a single-digit number, which may be negative, both represented as strings in a given base p.

    :param number1: The first number as a string, representing the multiplicand.
    :param number2: The second number as a string, representing the single-digit multiplier, which may be negative.
    :param base: The base in which the numbers are represented, and the result should be.
                 Must be an integer within the set {2,3,4,5,6,7,8,9,10,16}.
    :return: The result of the multiplication as a string in the same base.

    The function performs the multiplication by converting each digit to its decimal equivalent,
    multiplying them with carry, and then converting back to the base `p` representation.

    Pseudocode:

    1. Check if the multiplier is negative and adjust accordingly.
    2. Convert the single-digit multiplier to its decimal equivalent.
    3. Start iterations from the rightmost digit.
    4. Convert the current digit to decimal.
    5. Multiply the current digit by the multiplier and add the carry.
    6. Save the result.

    if check_if_valid_base(base) is False:
        stop the program and raise an error

    if number2 is negative:
        number2 <- number2 without the minus sign
        negative_multiplier <- True
    else:
        negative_multiplier <- False

    result <- ""
    carry <- 0

    for i <- max_length - 1 to 0:
        digit1 <- convert number1[i] to decimal

        total <- digit1 * multiplier + carry
        carry <- total // base
        result_digit <- total % base

        result <- convert result_digit to base p + result

    if carry > 0:
        result <- convert carry to base p + result

    if negative_multiplier:
        result <- "-" + result

    return result
    """

    # Check if number2 is a single digit
    if len(number2) != 1 and number2[0] != "-":
        print(number2)
        raise ValueError("number2 must be a single digit.")

    if check_if_valid_base(base) is False:
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    # digit mapping
    map_value_to_digit, map_value_to_char = get_digit_mappings(base)

    # Check for negative multiplier and adjust accordingly
    negative_multiplier = number2.startswith("-")
    if negative_multiplier:
        number2 = number2[1:]

    # Convert the single-digit multiplier to its decimal equivalent
    multiplier = map_value_to_digit[number2]

    result = ""
    carry = 0

    # Multiply each digit of the number1 by the multiplier starting from the rightmost digit
    for i in range(len(number1) - 1, -1, -1):
        # Convert the current digit of number1 to decimal
        current_digit = map_value_to_digit[number1[i]]

        # Multiply the current digit by the multiplier and add the carry
        total = current_digit * multiplier + carry
        carry = total // base
        result_digit = total % base

        # Save the result
        result = map_value_to_char[result_digit] + result

    # Add carry from the latest iteration
    if carry > 0:
        result = map_value_to_char[carry] + result

    # If the original multiplier was negative, add the negative sign to the result
    if negative_multiplier:
        result = "-" + result

    return result


def divide_in_base_p(number1: str, number2: str, base: int) -> (str, str):
    """
    Divides a number by a single-digit number, both represented as strings in a given base p, and also returns the remainder.

    :param number1: The first number as a string, representing the dividend.
    :param number2: The second number as a string, representing the single-digit divisor.
    :param base: The base in which the numbers are represented, and the result should be.
                 Must be an integer within the set {2,3,4,5,6,7,8,9,10,16}.
    :return: A tuple containing the result of the division as a string and the remainder as a string in the same base.

    The function performs the division by converting each digit to its decimal equivalent,
    dividing them with remainder, and then converting back to the base `p` representation.

    Pseudocode:

    1. Check if the divisor is negative and adjust accordingly.
    2. Convert the single-digit divisor to its decimal equivalent.
    3. Start iterations from the leftmost digit.
    4. Bring down the next digit.
    5. Convert the current digit to decimal.
    6. Divide the current digit by the divisor and save the remainder.

    if check_if_valid_base(base) is False:
        stop the program and raise an error

    if number2 is negative:
        number2 <- number2 without the minus sign
        negative_divisor <- True
    else:
        negative_divisor <- False

    result <- ""
    remainder <- 0

    for i <- 0 to max_length - 1:
        current <- remainder * base + convert number1[i] to decimal
        result_digit <- current // divisor
        remainder <- current % divisor

        result <- convert result_digit to base p + result

    result <- remove leading zeros from result

    if negative_divisor:
        result <- "-" + result

    remainder_str <- convert remainder to base p

    return result, remainder_str
    """

    # Check if number2 is a single digit
    if len(number2) != 1 and number2[0] != "-":
        raise ValueError("number2 must be a single digit.")

    if check_if_valid_base(base) is False:
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    map_value_to_digit, map_value_to_char = get_digit_mappings(base)

    # Check for negative divisor and adjust accordingly
    negative_divisor = number2.startswith("-")
    if negative_divisor:
        number2 = number2[1:]

    # Convert the single-digit divisor to its decimal equivalent
    divisor = map_value_to_digit[number2]

    if divisor == 0:
        raise ValueError("Division by zero is not allowed.")

    result = ""
    remainder = 0

    # Divide each digit of the number1 by the divisor starting from the leftmost digit
    for ch in number1:
        # Bring down the next digit
        current = remainder * base + map_value_to_digit[ch]
        result_digit = current // divisor
        remainder = current % divisor

        result += map_value_to_char[result_digit]

    # Remove leading zeros from the result
    result = result.lstrip("0") or "0"

    # If the original divisor was negative, add the negative sign to the result
    if negative_divisor:
        result = "-" + result

    # Convert the remainder back to a string in the original base
    remainder_str = map_value_to_char[remainder] if remainder else "0"

    return result, remainder_str


def convert_number_with_substitution_method(number: str, b: int, h: int) -> str:
    """
    Converts a number from base b to base h using substitution method.

    :param number: The number to be converted as a string.
    :param b: The base of the number.
    :param h: The base to convert to.

    :return: The converted number as a string.

    The function performs the conversion by converting each digit to its base h equivalent.
    Then performs all calculations in the destination (h) base.

    Pseudocode:

    1. Check if the bases are valid.
    2. Convert the number to its representation in base h.
    3. Start iterations from the rightmost digit.
    4. Multiply the current digit by b^i in base h.
    5. Add the term to the result.

    if check_if_valid_base(b) is False or check_if_valid_base(h) is False:
        stop the program and raise an error

    result <- "0"
    multiplier <- "1"  # This will be b^i in base h, starting with i=0

    for i <- 0 to max_length - 1:
        digit <- convert number[i] to decimal
        term <- multiply_in_base_p(multiplier, digit, h)
        result <- add_in_base_p(result, term, h)

        multiplier <- multiply_in_base_p(multiplier, b, h)

    result <- remove leading zeros from result

    if result is empty:
        result <- "0"

    return result
    """

    # Check the validity of the bases
    if not check_if_valid_base(b) or not check_if_valid_base(h):
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    # digit mapping
    map_value_to_digit, map_value_to_char = get_digit_mappings(b)

    # Convert the base b to its representation in base h
    base_b_in_base_h = str(b)

    result = "0"  # Start with zero in the destination base
    multiplier = "1"  # This will be b^i in base h, starting with i=0

    integer_part = number.split(".")[0]

    # Iterate over each digit in the source number from least significant to most
    for digit in reversed(integer_part):
        digit_value = map_value_to_digit[digit.upper()]
        digit_in_base_h = map_value_to_char[digit_value]

        term = multiply_in_base_p(multiplier, digit_in_base_h, h)
        result = add_in_base_p(result, term, h)
        multiplier = multiply_in_base_p(multiplier, base_b_in_base_h, h)

    # Remove leading zeros from the result
    result = result.lstrip("0")

    # If the result is empty, it means the number was zero
    if not result:
        result = "0"

    return result


def convert_a_number_with_successive_divisions(number: str, b: int, h: int) -> str:
    """
    Converts a number from base b to base h using successive divisions.

    :param number: The number to be converted as a string.
    :param b: The base of the number.
    :param h: The base to convert to.

    :return: The converted number as a string.
    """

    # Check the validity of the bases
    if not check_if_valid_base(b) or not check_if_valid_base(h):
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    map_value_to_digit, map_value_to_char = get_digit_mappings(b)

    result = ""

    while number != "0":
        # Divide the number by the destination base h
        number, remainder = divide_in_base_p(number, str(h), b)

        # Convert the remainder to its representation in base h
        digit_in_base_h = map_value_to_char[map_value_to_digit[remainder]]

        # Add the digit to the result
        result = digit_in_base_h + result

    return result


def convert_a_number_using_10_as_intermediary_base(number: str, b: int, h: int) -> str:
    """
    Converts a number from base b to base h using 10 as an intermediary base.

    :param number: The number to be converted as a string.
    :param b: The base of the number.
    :param h: The base to convert to.
    """

    if not check_if_valid_base(b) or not check_if_valid_base(h):
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    intermediary_result = ""
    result = ""

    if h == 16:
        intermediary_result = convert_number_with_substitution_method(number, b, 10)
        result = convert_number_with_substitution_method(intermediary_result, 10, 16)
    elif b == 16:
        intermediary_result = convert_a_number_with_successive_divisions(number, 16, 10)
        result = convert_a_number_with_successive_divisions(intermediary_result, 10, h)
    else:
        intermediary_result = convert_number_with_substitution_method(number, b, 10)
        result = convert_a_number_with_successive_divisions(intermediary_result, 10, h)

    return intermediary_result, result


import math


def rapid_conversion(number, b=None, h=None) -> str:
    """
    Converts a number from base b to base h using 10 as an intermediary base.

    :param number: The number to be converted as a string.

    :param b: the source base if converting to binary (destination base is assumed to be binary).
    :param h: the destination base if converting from binary (source base is assumed to be binary).

    :return: The converted number as a string.
    """

    rapid_tables = {
        2: [
            "0",
            "1",
            "10",
            "11",
            "100",
            "101",
            "110",
            "111",
            "1000",
            "1001",
            "1010",
            "1011",
            "1100",
            "1101",
            "1110",
            "1111",
        ],
        4: [
            "0",
            "1",
            "2",
            "3",
            "10",
            "11",
            "12",
            "13",
            "20",
            "21",
            "22",
            "23",
            "30",
            "31",
            "32",
            "33",
        ],
        8: [
            "0",
            "1",
            "2",
            "3",
            "4",
            "5",
            "6",
            "7",
            "10",
            "11",
            "12",
            "13",
            "14",
            "15",
            "16",
            "17",
        ],
        16: [
            "0",
            "1",
            "2",
            "3",
            "4",
            "5",
            "6",
            "7",
            "8",
            "9",
            "A",
            "B",
            "C",
            "D",
            "E",
            "F",
        ],
    }

    def from_binary(a, base):
        # Calculate the number of binary digits needed for a single base 'b' digit
        k = int(math.log(base, 2))
        result = ""
        while len(a) > 0:
            # Extract k bits from the end of the binary number
            group = a[-k:] if len(a) >= k else "0" * (k - len(a)) + a
            a = a[:-k] if len(a) >= k else ""
            # Find the equivalent in the destination base
            result = rapid_tables[base][int(group, 2)] + result
        return result

    def to_binary(a, base):
        result = ""
        for digit in a:
            # Find the binary equivalent of the digit
            group = bin(rapid_tables[base].index(digit))[2:]
            # Add zeros manually to the group if it's not long enough
            while len(group) < k:
                group = "0" + group
            result += group
        # Trim leading zeros manually
        while len(result) > 1 and result[0] == "0":
            result = result[1:]
        return result

    # Determine which conversion to perform based on the arguments provided
    if h is not None:
        return from_binary(number, h)
    if b is not None:
        k = int(
            math.log(b, 2)
        )  # Calculate the number of binary digits needed for a single base 'b' digit
        return to_binary(number, b)

    raise ValueError("Either destination base (h) or source base (b) must be provided.")


def read_integer_from_keyboard(prompt: str) -> None:
    """
    Reads an integer from the keyboard.

    :param prompt: The prompt to be displayed to the user.

    :return: The integer read from the keyboard.

    If the input is not a valid integer, the function will display an error message and ask for input again.
    """
    while True:
        try:
            return int(input(prompt).strip())
        except ValueError:
            print("Invalid number. Please try again.")


def read_string_from_keyboard(prompt: str) -> None:
    """
    Reads a string from the keyboard.

    :param prompt: The prompt to be displayed to the user.

    :return: The string read from the keyboard.

    If the input is not a valid string, the function will display an error message and ask for input again.
    """
    while True:
        try:
            # remove whitespace
            return str(input(prompt)).strip()
        except ValueError:
            print("Invalid number. Please try again.")


def test_all_functions() -> None:
    """
    Tests all functions in the program.
    """

    def test_add_in_base_p() -> None:
        """
        Tests the add_in_base_p function.
        """
        print("Testing add_in_base_p function...")

        # Adding numbers where carry-overs span multiple digits
        assert add_in_base_p("999", "1", 10) == "1000"
        assert add_in_base_p("FFFF", "1", 16) == "10000"

        # Adding numbers of different lengths
        assert add_in_base_p("10101", "1101", 2) == "100010"
        assert add_in_base_p("123", "4567", 10) == "4690"

        # Adding numbers that result in a change of length
        assert add_in_base_p("7FFF", "1", 16) == "8000"
        assert add_in_base_p("99999", "1", 10) == "100000"

        # Adding with non-trivial carry-over scenarios
        assert add_in_base_p("2B", "59", 16) == "84"
        assert add_in_base_p("139", "864", 10) == "1003"

        print("All tests passed.\n")

    def test_subtract_in_base_p() -> None:
        """
        Tests the subtract_in_base_p function.
        """
        print("Testing subtract_in_base_p function...")

        # Test cases with larger numbers
        assert subtract_in_base_p("1010", "110", 2) == "100"
        assert subtract_in_base_p("1001", "101", 2) == "100"
        assert subtract_in_base_p("100", "100", 10) == "0"
        assert subtract_in_base_p("345", "123", 10) == "222"
        assert subtract_in_base_p("1A2B", "F3", 16) == "1938"

        # Test cases with carry over scenarios
        assert subtract_in_base_p("1000", "1", 2) == "111"
        assert subtract_in_base_p("1000", "1", 10) == "999"
        assert subtract_in_base_p("1000", "1", 16) == "FFF"

        # Test cases that result in negative numbers
        assert subtract_in_base_p("10", "11", 2) == "-1"
        assert subtract_in_base_p("100", "101", 10) == "-1"
        assert subtract_in_base_p("10", "11", 16) == "-1"

        # Test cases with leading zeros
        assert subtract_in_base_p("00100", "00001", 2) == "11"
        assert subtract_in_base_p("00100", "00001", 10) == "99"
        assert subtract_in_base_p("00100", "00001", 16) == "FF"

        # Test cases with subtraction that yields zeros
        assert subtract_in_base_p("1000", "1000", 2) == "0"
        assert subtract_in_base_p("7777", "7777", 10) == "0"
        assert subtract_in_base_p("FACE", "FACE", 16) == "0"

        # Complex test cases involving non-trivial subtraction
        assert subtract_in_base_p("101010", "110110", 2) == "-1100"
        assert subtract_in_base_p("987654", "123456", 10) == "864198"
        assert subtract_in_base_p("BEEF", "CAFE", 16) == "-C0F"

        print("All tests passed.\n")

    def test_multiply_in_base_p() -> None:
        """
        Additional complex test cases for the multiply_in_base_p function.
        """
        print("Testing more complex cases for multiply_in_base_p function...")

        # Base 4 examples
        assert multiply_in_base_p("23", "2", 4) == "112"
        assert multiply_in_base_p("31", "2", 4) == "122"

        # Base 5 examples
        assert multiply_in_base_p("24", "3", 5) == "132"
        assert multiply_in_base_p("34", "3", 5) == "212"

        # Base 8 examples
        assert multiply_in_base_p("7", "7", 8) == "61"
        assert multiply_in_base_p("16", "7", 8) == "142"

        # Base 16 examples
        assert multiply_in_base_p("A", "B", 16) == "6E"
        assert multiply_in_base_p("F", "F", 16) == "E1"

        print("All complex tests passed for multiply_in_base_p function.\n")

    def test_divide_in_base_p() -> None:
        """
        Tests the divide_in_base_p function.
        """
        print("Testing divide_in_base_p function...")

        assert divide_in_base_p("0", "1", 2) == ("0", "0")
        assert divide_in_base_p("0", "1", 10) == ("0", "0")
        assert divide_in_base_p("0", "1", 16) == ("0", "0")
        assert divide_in_base_p("1", "1", 2) == ("1", "0")
        assert divide_in_base_p("1", "1", 10) == ("1", "0")
        assert divide_in_base_p("1", "1", 16) == ("1", "0")
        assert divide_in_base_p("10", "1", 2) == ("10", "0")
        assert divide_in_base_p("10", "1", 10) == ("10", "0")
        assert divide_in_base_p("10", "1", 16) == ("10", "0")
        assert divide_in_base_p("11", "1", 2) == ("11", "0")
        assert divide_in_base_p("11", "1", 10) == ("11", "0")
        assert divide_in_base_p("11", "1", 16) == ("11", "0")
        assert divide_in_base_p("672", "4", 9) == ("162", "3")

        print("All tests passed.\n")

    def test_convert_substitution() -> None:
        """
        Tests the convert_number_with_substitution_method function.
        """
        print("Testing convert_number_with_substitution_method function...")

        assert convert_number_with_substitution_method("0", 2, 10) == "0"
        assert convert_number_with_substitution_method("0", 2, 16) == "0"
        assert convert_number_with_substitution_method("1", 2, 10) == "1"
        assert convert_number_with_substitution_method("1", 2, 16) == "1"
        assert convert_number_with_substitution_method("10", 2, 10) == "2"
        assert convert_number_with_substitution_method("10", 2, 16) == "2"
        assert convert_number_with_substitution_method("11", 2, 10) == "3"
        assert convert_number_with_substitution_method("11", 2, 16) == "3"
        assert convert_number_with_substitution_method("100", 2, 2) == "100"
        assert convert_number_with_substitution_method("100", 2, 10) == "4"
        assert convert_number_with_substitution_method("100", 2, 16) == "4"
        assert convert_number_with_substitution_method("101", 2, 2) == "101"
        assert convert_number_with_substitution_method("101", 2, 10) == "5"

        print("All tests passed.\n")

    def test_convert_successive_divisions() -> None:
        """
        Tests the convert_a_number_with_successive_divisions function.
        """
        print("Testing convert_a_number_with_successive_divisions function...")

        assert convert_a_number_with_successive_divisions("0", 2, 2) == "0"
        assert convert_a_number_with_successive_divisions("0", 2, 10) == "0"
        assert convert_a_number_with_successive_divisions("0", 2, 16) == "0"
        assert convert_a_number_with_successive_divisions("1", 2, 2) == "1"
        assert convert_a_number_with_successive_divisions("1", 2, 10) == "1"
        assert convert_a_number_with_successive_divisions("1", 2, 16) == "1"
        assert convert_a_number_with_successive_divisions("10", 2, 2) == "10"
        assert convert_a_number_with_successive_divisions("10", 2, 10) == "2"
        assert convert_a_number_with_successive_divisions("10", 2, 16) == "2"
        assert convert_a_number_with_successive_divisions("11", 2, 2) == "11"
        assert convert_a_number_with_successive_divisions("11", 2, 10) == "3"
        assert convert_a_number_with_successive_divisions("11", 2, 16) == "3"
        assert convert_a_number_with_successive_divisions("100", 2, 2) == "100"
        assert convert_a_number_with_successive_divisions("100", 2, 10) == "4"
        assert convert_a_number_with_successive_divisions("100", 2, 16) == "4"
        assert convert_a_number_with_successive_divisions("101", 2, 2) == "101"
        assert convert_a_number_with_successive_divisions("101", 2, 10) == "5"

        print("All tests passed.\n")

    def test_convert_10_as_intermediary_base() -> None:
        """
        Tests the convert_a_number_using_10_as_intermediary_base function.
        """
        print("Testing convert_a_number_using_10_as_intermediary_base function...")

        assert convert_a_number_using_10_as_intermediary_base("0", 2, 2) == ("0", "0")
        assert convert_a_number_using_10_as_intermediary_base("0", 2, 10) == ("0", "0")
        assert convert_a_number_using_10_as_intermediary_base("0", 2, 16) == ("0", "0")
        assert convert_a_number_using_10_as_intermediary_base("1", 2, 2) == ("1", "1")
        assert convert_a_number_using_10_as_intermediary_base("1", 2, 10) == ("1", "1")
        assert convert_a_number_using_10_as_intermediary_base("1", 2, 16) == ("1", "1")
        assert convert_a_number_using_10_as_intermediary_base("10", 2, 2) == (
            "10",
            "10",
        )
        assert convert_a_number_using_10_as_intermediary_base("10", 2, 10) == (
            "10",
            "2",
        )
        assert convert_a_number_using_10_as_intermediary_base("10", 2, 16) == (
            "10",
            "2",
        )
        assert convert_a_number_using_10_as_intermediary_base("11", 2, 2) == (
            "11",
            "11",
        )
        assert convert_a_number_using_10_as_intermediary_base("11", 2, 10) == (
            "11",
            "3",
        )

        print("All tests passed.\n")

    def test_rapid_conversion() -> None:
        """
        Tests the rapid_conversion function.
        """
        print("Testing rapid_conversion function...")

        assert rapid_conversion("0", b=2) == "0"
        assert rapid_conversion("0", h=2) == "0"
        assert rapid_conversion("1", b=2) == "1"
        assert rapid_conversion("1", h=2) == "1"
        assert rapid_conversion("10", b=2) == "10"
        assert rapid_conversion("10", h=2) == "10"
        assert rapid_conversion("11", b=2) == "11"
        assert rapid_conversion("11", h=2) == "11"
        assert rapid_conversion("100", b=2) == "100"
        assert rapid_conversion("100", h=2) == "100"
        assert rapid_conversion("101", b=2) == "101"
        assert rapid_conversion("101", h=2) == "101"

        print("All tests passed.\n")

    test_add_in_base_p()
    test_subtract_in_base_p()
    test_multiply_in_base_p()
    test_divide_in_base_p()
    test_convert_substitution()
    test_convert_successive_divisions()
    test_convert_10_as_intermediary_base()
    test_rapid_conversion()
