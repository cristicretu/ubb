# @author: Crețu Cristian


def check_if_valid_base(base: int) -> bool:
    """
    Checks if a given base is valid.
    :param base: The base to be checked.
    :return: True if the base is valid, False otherwise.
    """
    return base in {2, 3, 4, 5, 6, 7, 8, 9, 10, 16}


def add_in_base_p(number1: str, number2: str, base: int) -> str:
    """
    Adds two numbers represented as strings in a given base p.

    :param number1: The first number as a string.
    :param number2: The second number as a string.
    :param base: The base in which the numbers are represented, and the result should be.
                 Must be an integer within the set {2,3,4,5,6,7,8,9,10,16}.
    :return: The result of the addition as a string in the same base.

    The function performs the addition by converting each digit to its decimal equivalent,
    adding them with carry, and then converting back to the base 'p' representation.
    """

    if check_if_valid_base(base) is False:
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    # digit mapping
    hex_digits = "0123456789ABCDEF"
    map_value_to_digit = {}
    map_value_to_char = {}

    for index in range(base):
        ch = hex_digits[index]
        map_value_to_digit[ch] = index
        map_value_to_char[index] = ch

    # Make both numbers the same length
    max_length = max(len(number1), len(number2))
    number1 = number1.zfill(max_length)
    number2 = number2.zfill(max_length)

    result = ""
    carry = 0

    # Start iterations from the rightmost digit
    for i in range(max_length - 1, -1, -1):
        # Convert the digit to decimal
        digit1 = map_value_to_digit[number1[i]]
        digit2 = map_value_to_digit[number2[i]]

        # Iteration  'i'
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
    subtracting them with borrow, and then converting back to the base 'p' representation.
    """

    if check_if_valid_base(base) is False:
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    # digit mapping
    hex_digits = "0123456789ABCDEF"
    map_value_to_digit = {}
    map_value_to_char = {}

    for index in range(base):
        ch = hex_digits[index]
        map_value_to_digit[ch] = index
        map_value_to_char[index] = ch

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
    """

    # Check if number2 is a single digit
    if len(number2) != 1 and number2[0] != "-":
        raise ValueError("number2 must be a single digit.")

    if check_if_valid_base(base) is False:
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    # digit mapping
    hex_digits = "0123456789ABCDEF"
    map_value_to_digit = {}
    map_value_to_char = {}

    for index in range(base):
        ch = hex_digits[index]
        map_value_to_digit[ch] = index
        map_value_to_char[index] = ch

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
    """

    # Check if number2 is a single digit
    if len(number2) != 1 and number2[0] != "-":
        raise ValueError("number2 must be a single digit.")

    if check_if_valid_base(base) is False:
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    # digit mapping
    hex_digits = "0123456789ABCDEF"
    map_value_to_digit = {}
    map_value_to_char = {}

    for index in range(base):
        ch = hex_digits[index]
        map_value_to_digit[ch] = index
        map_value_to_char[index] = ch

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

        # Save the result
        result += map_value_to_char[result_digit]

    # Remove leading zeros from the result
    result = result.lstrip("0")

    # If the result is empty, it means the dividend was smaller than the divisor
    if not result:
        result = "0"

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
    """

    # Check the validity of the bases
    if not check_if_valid_base(b) or not check_if_valid_base(h):
        raise ValueError("Base must be one of the following: {2,3,4,5,6,7,8,9,10,16}")

    # digit mapping for destination base h
    hex_digits = "0123456789ABCDEF"
    map_value_to_digit = {ch: index for index, ch in enumerate(hex_digits[:h])}
    map_value_to_char = {index: ch for index, ch in enumerate(hex_digits[:h])}

    # Convert the base b to its representation in base h
    base_b_in_base_h = ""
    temp_b = b
    while temp_b > 0:
        base_b_in_base_h = map_value_to_char[temp_b % h] + base_b_in_base_h
        temp_b //= h

    result = "0"  # Start with zero in the destination base
    multiplier = "1"  # This will be b^i in base h, starting with i=0
    fractional_part = (
        False  # Flag to indicate if we are in the fractional part of the number
    )

    # Iterate over each digit in the source number from least significant to most
    for digit in reversed(number):
        if digit == ".":
            fractional_part = True
            multiplier = "1"  # Reset the multiplier for the fractional part
            continue
        # Convert the digit to its integer representation
        if digit.upper() not in map_value_to_digit:
            raise ValueError("Invalid digit '{}' for base {}".format(digit, b))
        digit_value = map_value_to_digit[digit.upper()]

        # Convert the digit to its representation in base h
        digit_in_base_h = map_value_to_char[digit_value]

        # Multiply the digit by b^i in base h and add to the result
        term = multiply_in_base_p(digit_in_base_h, multiplier, h)
        result = add_in_base_p(result, term, h)

        # Update the multiplier for the next digit by multiplying by b in base h
        if fractional_part:
            multiplier = multiply_in_base_p(multiplier, int(b), h)
        else:
            multiplier = multiply_in_base_p(multiplier, base_b_in_base_h, h)

    # Remove leading zeros from the result
    result = result.lstrip("0")

    # If the result is empty, it means the number was zero
    if not result:
        result = "0"

    return result


print(convert_number_with_substitution_method("10.10", 2, 4))
