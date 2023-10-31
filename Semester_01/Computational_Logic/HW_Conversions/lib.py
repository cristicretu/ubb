# @author: Crețu Cristian

def is_valid_base(b: int, h: int) -> bool:
    """
    Checks if the bases are valid. A base is valid if it is between 2 and 16.

    :param b: base of the number
    :param h: base to convert to

    :return: True if the bases are valid, False otherwise
    """
    return b >= 2 and h >= 2 and b <= 16 and h <= 16

def char_to_digit(c: str) -> int:
    return int(c, 16) if 'A' <= c <= 'F' else int(c)

def digit_to_char(d: int) -> str:
    return hex(d)[2:].upper()

def multiply_in_base(a: str, b: str, base: int) -> str:
    result = ''
    carry = 0
    max_len = max(len(a), len(b))
    for digit_a, digit_b in zip(a.zfill(max_len), b.zfill(max_len)):
        mult = char_to_digit(digit_a) * char_to_digit(digit_b) + carry
        carry, result_digit = divmod(mult, base)
        result = digit_to_char(result_digit) + result
    while carry:
        carry, result_digit = divmod(carry, base)
        result = digit_to_char(result_digit) + result
    return result

def add_in_base(a: str, b: str, base: int) -> str:
    result = ''
    carry = 0
    max_len = max(len(a), len(b))
    for digit_a, digit_b in zip(a.zfill(max_len), b.zfill(max_len)):
        summ = char_to_digit(digit_a) + char_to_digit(digit_b) + carry
        carry, result_digit = divmod(summ, base)
        result = digit_to_char(result_digit) + result
    if carry:
        result = digit_to_char(carry) + result
    return result

def add_2_numbers_in_base_p(num1, num2, base):
    carry = 0
    result = []
    len_diff = len(num1) - len(num2)

    # Making both numbers of the same length
    if len_diff > 0:
        num2 = '0' * len_diff + num2
    else:
        num1 = '0' * (-len_diff) + num1

    for ch1, ch2 in zip(reversed(num1), reversed(num2)):
        total = char_to_digit(ch1) + char_to_digit(ch2) + carry
        result.append(digit_to_char(total % base))
        carry = total // base

    while carry:
        result.append(digit_to_char(carry % base))
        carry //= base

    return ''.join(reversed(result))

def multiply_a_number_to_a_digit_in_base_p(num, digit, base):
    carry = 0
    result = []
    digit_value = char_to_digit(digit)

    for ch in reversed(num):
        prod = char_to_digit(ch) * digit_value + carry
        result.append(digit_to_char(prod % base))
        carry = prod // base

    while carry:
        result.append(digit_to_char(carry % base))
        carry //= base

    return ''.join(reversed(result))

def conversion_with_substitution_method(n: str, b: int, h: int):
    """
    Converts number 'n' from base b to base h using substitution method.
    :param n: number to be converted
    :param b: base of the number
    :param h: base to convert to
    :return: converted number
    """
    if not is_valid_base(b, h):
            return "Invalid bases"

    list_of_digits = '0123456789ABCDEF'
    integral_part, _, fractional_part = n.partition(".")

    # Convert base b to base h as a string
    b_in_base_h = base_conversion(str(b), 10, h)

    result = '0'
    multiplier = '1'

    # Convert the integral part
    for digit in reversed(integral_part):
        term = multiply_in_base(digit_to_char(digit), multiplier, h)
        result = add_in_base(result, term, h)
        multiplier = multiply_in_base(multiplier, b_in_base_h, h)

    # Convert the fractional part (if present)
    if fractional_part:
        result += '.'
        multiplier = b_in_base_h
        for digit in fractional_part:
            term = multiply_in_base(digit_to_char(digit), multiplier, h)
            result = add_in_base(result, term, h)
            multiplier = multiply_in_base(multiplier, b_in_base_h, h)

    return result

print(conversion_with_substitution_method("A54", 11, 16))
print(conversion_with_substitution_method("1001", 2, 10))
