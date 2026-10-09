"""
Helper functions for the project.

@author: Cretu Cristian, 913
"""
from random import randint

def format_expression_string(a: int, b: int, c: int, d: int) -> str:
    """
    Formats the expression string A[m] - A[n] + A[p] - A[q] to handle negative numbers.
    If a number is negative, it will be displayed as - (-x) instead of - -x.

    :param a: Value of A[m]
    :param b: Value of A[n]
    :param c: Value of A[p]
    :param d: Value of A[q]

    :return: Formatted expression string
    """
    b_str = f"- (-{abs(b)})" if b < 0 else f"- {b}"
    d_str = f"- (-{abs(d)})" if d < 0 else f"- {d}"

    return f"{a} {b_str} + {c} {d_str}"

def generate_random_list_of_integers(length_of_list: int) -> list:
    """
    Generates a list of random integers.

    :param length_of_list: length of the list

    :return: list of random integers
    """
    return [randint(-100, 100) for _ in range(length_of_list)]


def generate_random_points_list(number_of_points: int) -> list:
    """
    Generates a list of random points on a 2D plane.

    :param number_of_points: number of points to generate

    :return: list of points
    """
    return [(randint(-100, 100), randint(-100, 100)) for _ in range(number_of_points)]

def generate_semi_random_points_list(number_of_points: int) -> list:
    """
    Generates a list of semi-random points on a 2D plane.
    This ensures they are at least three colinear points.

    :param number_of_points: number of points to generate

    :return: list of points
    """
    x_coordinate = randint(-100, 100)
    y_coordinate = randint(-100, 100)

    return [(x_coordinate, y_coordinate), (2 * x_coordinate, 2 * y_coordinate), (3 * x_coordinate,3 * y_coordinate)] + \
           [(randint(-100, 100), randint(-100, 100)) for _ in range(number_of_points - 3)]

def read_list_of_points_from_keyboard(length_of_the_list: int) -> list:
    """
    Given a list length, read that many 2D points from the keyboard.

    :param length_of_the_list: length of the list

    :return: list of points
    """
    return [(read_integer_from_keyboard("x-coordinate: "), read_integer_from_keyboard("y-coordinate: ")) for _ in range(length_of_the_list)]


def read_integer_from_keyboard(prompt: str) -> int:
    """
    Reads an integer from the keyboard.

    :param prompt: prompt to display

    :return: integer read from the keyboard by the user
    """
    while True:
        try:
            return int(input(prompt))
        except ValueError:
            print("Invalid input. Please try again using an integer.")
