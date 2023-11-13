"""
A5. 1 and 11

@Author: Cretu Cristian, 913

"""
import sys
from random import randint
from typing import List, Tuple


#
# Write below this comment
# Functions to deal with complex numbers -- list representation
# -> There should be no print or input statements in this section
# -> Each function should do one thing only
# -> Functions communicate using input parameters and their return values
#
def convert_list_complex_number_into_string(complex_number: list) -> str:
    """
    Converts a complex number from list representation into string representation.

    :param complex_number: the complex number to be converted
    :returns a string representing the complex number
    """
    real_part = complex_number[0]
    imaginary_part = complex_number[1]
    return (
        f"{real_part} + {imaginary_part}i"
        if imaginary_part >= 0
        else f"{real_part} - {-imaginary_part}i"
    )


def generate_random_complex_number_as_list() -> list:
    """
    Generates a single complex number as a list representation.

    :return: a list representing a complex number
    """
    LOWER_BOUND = -100
    UPPER_BOUND = 100

    real_part = randint(LOWER_BOUND, UPPER_BOUND)
    imaginary_part = randint(LOWER_BOUND, UPPER_BOUND)

    return [real_part, imaginary_part]


def generate_random_complex_numbers_as_list(length_of_list: int = 10) -> list:
    """
    Generates random complex numbers. The representation will be a list of lists.

    :param length_of_list: the length of the list of complex numbers
    :return: a list of random complex numbers
    """
    return [generate_random_complex_number_as_list() for _ in range(length_of_list)]


#
# Write below this comment
# Functions to deal with complex numbers -- dict representation
# -> There should be no print or input statements in this section
# -> Each function should do one thing only
# -> Functions communicate using input parameters and their return values
#
def convert_dict_complex_number_into_string(complex_number: dict) -> str:
    """
    Converts a complex number from dict representation into string representation.

    :param complex_number: the complex number to be converted
    """
    real_part = complex_number["real_part"]
    imaginary_part = complex_number["imaginary_part"]
    return (
        f"{real_part} + {imaginary_part}i"
        if imaginary_part >= 0
        else f"{real_part} - {-imaginary_part}i"
    )


def generate_random_complex_number_as_dict() -> dict:
    """
    Generates a single complex number as a dict representation.

    :return: a dict representing a complex number
    """
    LOWER_BOUND = -100
    UPPER_BOUND = 100

    real_part = randint(LOWER_BOUND, UPPER_BOUND)
    imaginary_part = randint(LOWER_BOUND, UPPER_BOUND)

    return {"real_part": real_part, "imaginary_part": imaginary_part}


def generate_random_complex_numbers_as_dict(length_of_list: int = 10) -> list:
    """
    Generates random complex numbers. The representation will be a list of dicts.

    :param length_of_list: the length of the list of complex numbers
    :return: a list of random complex numbers
    """
    return [generate_random_complex_number_as_dict() for _ in range(length_of_list)]


#
# Write below this comment
# Functions that deal with subarray/subsequence properties
# -> There should be no print or input statements in this section
# -> Each function should do one thing only
# -> Functions communicate using input parameters and their return values
#
def get_value_of_complex_number_as_tuple(complex_number, representation: str) -> tuple:
    """
    Returns the value of a complex number, depending on the representation.

    :param complex_number: the complex number to be converted
    :param representation: the representation of the complex numbers (either list or dict)

    :return: a tuple containing the real part and the imaginary part of the complex number
    """
    return (
        tuple(complex_number)
        if representation == "list"
        else tuple(complex_number.values())
    )


def get_real_part_of_complex_number(complex_number, representation: str) -> int:
    """
    Returns the real part of a complex number, depending on the representation.

    :param complex_number: the complex number to be converted
    :param representation: the representation of the complex numbers (either list or dict)

    :return: the real part of the complex number
    """
    return (
        complex_number[0] if representation == "list" else complex_number["real_part"]
    )


def compute_length_and_elements_of_longest_subarray_of_distinct_complex_numbers(
    array_of_complex_numbers: list, representation: str
) -> Tuple[int, list]:
    """
    Computes the length and the elements of the longest subarray of distinct numbers.

    :param array_of_complex_numbers: list of complex numbers
    :param representation: the representation of the complex numbers (either list or dict)

    :return: a tuple containing the length and the elements of the longest subarray of distinct numbers

    https://www.geeksforgeeks.org/window-sliding-technique/
    """
    if representation not in {"list", "dict"}:
        return -1, []

    seen_complex_number = set()
    start_index = 0
    maximum_length = 0
    maximum_subarray = []

    for end_index, complex_number in enumerate(array_of_complex_numbers):
        while (
            get_value_of_complex_number_as_tuple(complex_number, representation)
            in seen_complex_number
        ):
            seen_complex_number.remove(
                get_value_of_complex_number_as_tuple(
                    array_of_complex_numbers[start_index], representation
                )
            )
            start_index += 1
        seen_complex_number.add(
            get_value_of_complex_number_as_tuple(complex_number, representation)
        )
        if end_index - start_index + 1 > maximum_length:
            maximum_length = end_index - start_index + 1
            maximum_subarray = [
                convert_list_complex_number_into_string(num)
                if representation == "list"
                else convert_dict_complex_number_into_string(num)
                for num in array_of_complex_numbers[start_index : end_index + 1]
            ]

    return maximum_length, maximum_subarray


def compute_length_and_elements_of_maximum_subarray_sum_of_real_parts(
    array_of_complex_numbers: list, representation: str
) -> Tuple[int, list]:
    """
    Computes the length and the elements of the maximum subarray sum of real parts.

    :param array_of_complex_numbers: list of complex numbers
    :param representation: the representation of the complex numbers (either list or dict)

    :return: a tuple containing the length and the elements of the maximum subarray sum of real parts
    """
    if representation not in {"list", "dict"}:
        return -1, []

    maximum_sum, current_sum = 0, 0
    start_sum_index, end_sum_index, temporary_start_index = 0, 0, 0
    maximum_subarray = []

    for current_index, complex_number in enumerate(array_of_complex_numbers):
        current_sum += get_real_part_of_complex_number(complex_number, representation)

        if current_sum > maximum_sum:
            maximum_sum = current_sum
            start_sum_index, end_sum_index = temporary_start_index, current_index
        if current_sum < 0:
            current_sum = 0
            temporary_start_index = current_index + 1

    maximum_subarray = (
        array_of_complex_numbers[start_sum_index : end_sum_index + 1]
        if maximum_sum > 0
        else []
    )

    maximum_subarray = (
        [
            convert_list_complex_number_into_string(num)
            if representation == "list"
            else convert_dict_complex_number_into_string(num)
            for num in array_of_complex_numbers[start_sum_index : end_sum_index + 1]
        ]
        if maximum_sum > 0
        else []
    )

    return len(maximum_subarray), maximum_subarray


#
# Write below this comment
# UI section
# Write all functions that have input or print statements here
# Ideally, this section should not contain any calculations relevant to program functionalities
#


def read_complex_number():
    """
    Reads two integers from the keyboard. First one represents the real part.
    Second one represents the imaginary part.

    :return: A tuple containing the real integer part and the imaginary integer part.
    """
    real_part = read_integer_from_keyboard(">Real part (integer): ")
    imaginary_part = read_integer_from_keyboard(">Imaginary part (integer): ")

    return (real_part, imaginary_part)


def read_integer_from_keyboard(input_prompt: str) -> int:
    """
    Reads an integer from the keyboard.

    :param prompt: prompt to display

    :return: integer read from the keyboard by the user
    """
    while True:
        try:
            return int(input(input_prompt).strip())
        except ValueError:
            print("Invalid input. Please try again using an integer.")


def user_option_2_display_entire_list_of_numbers(
    array_of_complex_numbers: list, representation: str
) -> None:
    """
    Displays the entire list of complex numbers, independantly of the representation.

    :param array_of_complex_numbers: list of complex numbers
    :param representation: the representation of the complex numbers (either list or dict)
    """
    if representation == "list":
        for complex_number in array_of_complex_numbers:
            print(convert_list_complex_number_into_string(complex_number))
    elif representation == "dict":
        for complex_number in array_of_complex_numbers:
            print(convert_dict_complex_number_into_string(complex_number))


def user_option_1_read_complex_numbers(
    array_of_complex_numbers: list, representation: str
) -> None:
    """
    Reads a length and then reads that many complex numbers from the keyboard.

    :param array_of_complex_numbers: list of complex numbers, where the numbers will be appended
    :param representation: the representation of the complex numbers (either list or dict)

    :return: the list of complex numbers
    """
    length_of_complex_numbers = read_integer_from_keyboard(
        ">Enter the ammount of numbers to read: "
    )

    for _ in range(length_of_complex_numbers):
        real_part, imaginary_part = read_complex_number()

        if representation == "list":
            array_of_complex_numbers.append([real_part, imaginary_part])
        elif representation == "dict":
            array_of_complex_numbers.append(
                {"real_part": real_part, "imaginary_part": imaginary_part}
            )


def print_menu() -> None:
    print("\n---------------\n")
    print("1. Read a list of complex numbers.")
    print("2. Display the entire list of numbers.")
    print("3. Display the following propreties:")
    print("   3.1 Length and elements of a longest subarray of distinct numbers.")
    print(
        "   3.2 The length and elements of a maximum subarray sum, considering the real part."
    )
    print("4. Exit the application.")
    print("\n---------------\n")


def get_command_line_argument() -> str:
    """
    Returns the first command line argument as a string.
    If it's either -l or --list, then the list representation will be used.
    If it's either -d or --dict, then the dict representation will be used.

    :return: the first command line argument as a string
    """
    if len(sys.argv) == 1:
        raise ValueError("Usage: python3 program.py [-l | --list | -d | --dict]")

    if sys.argv[1] in ["-l", "--list"]:
        return "list"
    elif sys.argv[1] in ["-d", "--dict"]:
        return "dict"

    raise ValueError("Invalid command line argument. Please try again.")


def get_representation_and_generate_lists() -> Tuple[List, str]:
    """
    Tries to get the representation from the command line arguments.
    If it fails, the program will exit.
    Otherwise, it will generate a list of 10 random complex numbers.

    :return: a list of 10 random complex numbers and the representation
    """
    array_of_complex_numbers = []
    representation = ""

    try:
        representation = get_command_line_argument()

        if representation == "list":
            array_of_complex_numbers = generate_random_complex_numbers_as_list()
        elif representation == "dict":
            array_of_complex_numbers = generate_random_complex_numbers_as_dict()

    except ValueError as exception_error:
        print(exception_error)
        return [], ""

    return array_of_complex_numbers, representation


def main_program() -> None:
    array_of_complex_numbers, representation = get_representation_and_generate_lists()

    if array_of_complex_numbers == []:
        return

    print(f"Running the program with {representation} representation.\n")

    READ_NUMBERS = 1
    DISPLAY_ENTIRE_LIST = 2
    DISPLAY_PROPERTIES = 3
    CLOSE_PROGRAM = 4

    while True:
        print_menu()

        user_option = read_integer_from_keyboard(">Please select an option: ")

        if user_option == READ_NUMBERS:
            user_option_1_read_complex_numbers(array_of_complex_numbers, representation)

        elif user_option == DISPLAY_ENTIRE_LIST:
            user_option_2_display_entire_list_of_numbers(
                array_of_complex_numbers, representation
            )

        elif user_option == DISPLAY_PROPERTIES:
            (
                length_of_longest_subarray_of_distinct_complex_numbers,
                longest_subarray_of_distinct_complex_numbers,
            ) = compute_length_and_elements_of_longest_subarray_of_distinct_complex_numbers(
                array_of_complex_numbers, representation
            )

            (
                length_of_maximum_subarray_sum_of_real_parts,
                maximum_subarray_sum_of_real_parts,
            ) = compute_length_and_elements_of_maximum_subarray_sum_of_real_parts(
                array_of_complex_numbers, representation
            )

            print("\n3.1")

            print(
                f"Length of the longest subarray of distinct complex numbers: {length_of_longest_subarray_of_distinct_complex_numbers}"
            )
            print(
                f"Elements of the longest subarray of distinct complex numbers: {longest_subarray_of_distinct_complex_numbers}"
            )

            print("\n")

            print("\n3.2")

            print(
                f"Length of the maximum subarray sum of real parts: {length_of_maximum_subarray_sum_of_real_parts}"
            )

            print(
                f"Elements of the maximum subarray sum of real parts: {maximum_subarray_sum_of_real_parts}"
            )

        elif user_option == CLOSE_PROGRAM:
            print("Exiting the aplication...")
            return
        else:
            print("! Invalid option. Please try again.")


if __name__ == "__main__":
    main_program()
