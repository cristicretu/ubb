# @author: Crețu Cristian, Group 913
from lib import *


def print_conversion():
    """
    Prints the conversion menu

    1. Conversion using substitution method; b < h
    2. Conversion using successive divisions method; b > h
    3. Conversion using 10 as an intermediary base; b != 10, h != 10
    """
    print("\n1. Conversion using substitution method; b < h")
    print("2. Conversion using successive divisions method; b > h")
    print("3. Conversion using 10 as an intermediary base; b != 10, h != 10\n")


def print_operations():
    """
    Prints the operations menu

    1. Addition of two numbers in base p
    2. Subtraction of two numbers in base p
    3. Multiplication of a number and a single digit in base p
    """
    print("\n1. Addition of two numbers in base p")
    print("2. Subtraction of two numbers in base p")
    print("3. Multiplication of a number and a single digit in base p")
    print("4. Division of a number and a single digit in base p\n")


def print_menu():
    """
    Prints the main menu

    1. Conversion from base b to base h
    2. Operation with two numbers
    0. Exit
    """
    print("\n1. Conversion from base b to base h")
    print("2. Operation with two numbers")
    print("0. Exit\n")


def main():
    """
    Main function of the program

    Prints the main menu and reads the option from the keyboard
    """
    print("\nAuthor: Cretu Cristian, Group 913\n")

    while True:
        print_menu()

        option = read_integer_from_keyboard(">Option: ")

        if option == 1:
            print_conversion()

            conversion_method = read_integer_from_keyboard(">Conversion method: ")

            if conversion_method not in [1, 2, 3]:
                print("\nInvalid conversion method!\n")
                continue

            number = read_string_from_keyboard(">Number: ")
            base = read_integer_from_keyboard(">Source Base: ")
            target_base = read_integer_from_keyboard(">Target Base: ")

            if conversion_method == 1:
                if not check_if_valid_base(base) or not check_if_valid_base(
                    target_base
                ):
                    print(
                        "\nInvalid base!\nBase must be 2, 3, 4, 5, 6, 7, 8, 9, 10, or 16\n"
                    )
                    continue

                result = convert_number_with_substitution_method(
                    str(number), base, target_base
                )
                print(f"\nResult: {number}({base}) = {result}({target_base})\n")

            elif conversion_method == 2:
                if not check_if_valid_base(base) or not check_if_valid_base(
                    target_base
                ):
                    print(
                        "\nInvalid base!\nBase must be 2, 3, 4, 5, 6, 7, 8, 9, 10, or 16\n"
                    )
                    continue

                result = convert_a_number_with_successive_divisions(
                    str(number), base, target_base
                )
                print(f"\nResult: {number}({base}) = {result}({target_base})\n")

            elif conversion_method == 3:
                if not check_if_valid_base(base) or not check_if_valid_base(
                    target_base
                ):
                    print(
                        "\nInvalid base!\nBase must be 2, 3, 4, 5, 6, 7, 8, 9, 10, or 16\n"
                    )
                    continue

                base10, result = convert_a_number_using_10_as_intermediary_base(
                    str(number), base, target_base
                )
                print(f"\nResult: {number}({base}) = {result}({target_base})")
                print(f"Base 10 Result: {number}({base}) = {base10}(10)\n")

            else:
                print("\nInvalid conversion method!\n")
                continue

        elif option == 2:
            print_operations()

            operation = read_integer_from_keyboard(">Operation: ")
            number1 = read_string_from_keyboard(">Number 1: ")
            number2 = read_string_from_keyboard(">Number 2: ")
            base = read_integer_from_keyboard(">Base: ")

            if operation == 1:
                if not check_if_valid_base(base):
                    print(
                        "\nInvalid base!\nBase must be 2, 3, 4, 5, 6, 7, 8, 9, 10, or 16\n"
                    )
                    continue

                result = add_in_base_p(number1, number2, base)
                print(
                    f"\nResult: {number1}({base}) + {number2}({base}) = {result}({base})\n"
                )

            elif operation == 2:
                if not check_if_valid_base(base):
                    print(
                        "\nInvalid base!\nBase must be 2, 3, 4, 5, 6, 7, 8, 9, 10, or 16\n"
                    )
                    continue

                result = subtract_in_base_p(number1, number2, base)
                print(
                    f"\nResult: {number1}({base}) - {number2}({base}) = {result}({base})\n"
                )

            elif operation == 3:
                if not check_if_valid_base(base):
                    print(
                        "\nInvalid base!\nBase must be 2, 3, 4, 5, 6, 7, 8, 9, 10, or 16\n"
                    )
                    continue

                result = multiply_in_base_p(number1, number2, base)
                print(
                    f"\nResult: {number1}({base}) * {number2}({base}) = {result}({base})\n"
                )

            elif operation == 4:
                if not check_if_valid_base(base):
                    print(
                        "\nInvalid base!\nBase must be 2, 3, 4, 5, 6, 7, 8, 9, 10, or 16\n"
                    )
                    continue

                result, remainder = divide_in_base_p(number1, number2, base)
                print(
                    f"\nResult: {number1}({base}) / {number2}({base}) = {result}({base}), remainder: {remainder}({base})\n"
                )

        elif option == 0:
            print("\nExiting!\n")
            break

        else:
            print("\nInvalid option!\n")
            continue
