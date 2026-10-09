"""
A4 - Backtracking and Dynamic Programming

6. Given an array of integers A, maximize the value of the
   expression A[m] - A[n] + A[p] - A[q], where m, n, p, q are
   array indices with m > n > p > q. For A = [30, 5, 15, 18, 30, 40],
   the maximum value is 32, obtained as 40 - 18 + 15 - 5. Display
   both the maximum value as well as the expression used to calculate
   it.

@author: Cretu Cristian, 913
"""

from helpers import generate_random_list_of_integers, format_expression_string


def maximize_expression_naive_solution(list_of_integers: list) -> tuple:
    """
    Given a list of integers, return the maximum value of the expression
    A[m] - A[n] + A[p] - A[q], where m, n, p, q are array indices with
    m > n > p > q. Uses a naive solution with four nested loops.

    :param list_of_integers: list of integers

    :return: tuple containing the maximum value and the expression used to calculate it
    """
    if len(list_of_integers) < 4:
        raise ValueError("The list must contain at least four elements.")

    maximum_expression_value = float("-inf")
    maximum_expression_string = ""

    for m in range(len(list_of_integers)):
        for n in range(m):
            for p in range(n):
                for q in range(p):
                    current_value = list_of_integers[m] - list_of_integers[n] + list_of_integers[p] - list_of_integers[q]
                    if current_value > maximum_expression_value:
                        maximum_expression_value = current_value
                        maximum_expression_string = format_expression_string(list_of_integers[m], list_of_integers[n], list_of_integers[p], list_of_integers[q])

    return maximum_expression_value, maximum_expression_string

def maximize_expression_dynamic_programming_solution(list_of_integers: list, print_partial_data_structure: bool = True) -> tuple:
    """
    Given a list of integers, return the maximum value of the expression
    A[m] - A[n] + A[p] - A[q], where m, n, p, q are array indices with
    m > n > p > q. Uses a dynamic programming solution.

    Generate two arrays, one containing the minimum value to the left of each index
    and one containing the maximum value to the right of each index.

    :param list_of_integers: list of integers

    :return: tuple containing the maximum value and the expression used to calculate it
    """
    list_length = len(list_of_integers)

    if list_length < 4:
        raise ValueError("The list must contain at least four elements.")

    # Arays to keep track of the minimum value to the left and maximum value to the right of each index
    min_value_to_the_left_of_index = [0] * list_length
    max_value_to_the_right_of_index = [0] * list_length

    min_idx = 0  # Index of the minimum value
    min_value_to_the_left_of_index[0] = 0

    for i in range(1, list_length):
        if list_of_integers[i] < list_of_integers[min_idx]:
            min_idx = i
        min_value_to_the_left_of_index[i] = min_idx

    max_idx = list_length - 1  # Index of the maximum value
    max_value_to_the_right_of_index[-1] = list_length - 1

    for i in reversed(range(list_length - 1)):
        if list_of_integers[i] > list_of_integers[max_idx]:
            max_idx = i
        max_value_to_the_right_of_index[i] = max_idx

    # Initialize variables to keep track of the maximum expression value and the corresponding expression
    maximum_expression_value = list_of_integers[3] - list_of_integers[2] + list_of_integers[1] - list_of_integers[0]
    maximum_expression_string = format_expression_string(list_of_integers[3], list_of_integers[2], list_of_integers[1], list_of_integers[0])

    # Loop through the array to find the maximum expression value
    for k in range(list_length - 1):
        for p in range(k + 1, list_length):
            if min_value_to_the_left_of_index[k] != k and max_value_to_the_right_of_index[p] != p and min_value_to_the_left_of_index[k] != max_value_to_the_right_of_index[p] and min_value_to_the_left_of_index[k] != p:
                expression_value = list_of_integers[k] - list_of_integers[min_value_to_the_left_of_index[k]] + list_of_integers[max_value_to_the_right_of_index[p]] - list_of_integers[p]

                if expression_value > maximum_expression_value:
                    maximum_expression_value = expression_value
                    maximum_expression_string = format_expression_string(list_of_integers[k], list_of_integers[min_value_to_the_left_of_index[k]], list_of_integers[max_value_to_the_right_of_index[p]], list_of_integers[p])

    if print_partial_data_structure:
        # Print the max_value_to_the_right_of_index array
        print("max_value_to_the_right_of_index:")
        for i in range(list_length):
            print(f"{max_value_to_the_right_of_index[i]}", end=" ")

        # Print the min_value_to_the_left_of_index array
        print("\nmin_value_to_the_left_of_index:")
        for i in range(list_length):
            print(f"{min_value_to_the_left_of_index[i]}", end=" ")

        print()

    return maximum_expression_value, maximum_expression_string

def test_program_using_example_list() -> None:
    example_list = [30, 5, 15, 18, 30, 40]

    print(f"Example list: {example_list}")

    result1 = maximize_expression_naive_solution(example_list)
    result2 = maximize_expression_dynamic_programming_solution(example_list)

    print(f"Naive solution: {result1[0]} = {result1[1]}")
    print(f"Dynamic programming solution: {result2[0]} = {result2[1]}")


def main_program() -> None:
    length_of_list = int(input("Enter the length of the list: "))

    if length_of_list < 4:
        print("The list must contain at least four elements. Continuing with the example list.")

        test_program_using_example_list()
        return

    list_of_integers = generate_random_list_of_integers(length_of_list)

    result1 = maximize_expression_naive_solution(list_of_integers)
    result2 = maximize_expression_dynamic_programming_solution(list_of_integers)

    print(f"List of integers: {list_of_integers}")
    print("\n----------------------------------------\n")

    print(f"Naive solution: {result1[0]} = {result1[1]}")
    print(f"Dynamic programming solution: {result2[0]} = {result2[1]}")


if __name__ == "__main__":
    main_program()
