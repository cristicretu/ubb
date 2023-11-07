"""
A4 - Backtracking and Dynamic Programming

8. Consider n points in a plane, given by their coordinates.
   Determine all subsets with at least three elements formed
   by collinear points. If the problem has no solution, give
   a message.

@author: Cretu Cristian, 913
"""
from helpers import generate_random_points_list, generate_semi_random_points_list, read_list_of_points_from_keyboard, read_integer_from_keyboard
from itertools import combinations


def test_colinearity_for_three_points(point0: tuple, point1: tuple, point2: tuple) -> bool:
    """
    Given three tuples representing points in a plane, return True if
    the points are colinear, False otherwise. Using the cross product
    of two vectors to determine colinearity.

    :param p0: tuple representing the first point
    :param p1: tuple representing the second point
    :param p2: tuple representing the third point
    """
    if len(point0) != 2 or len(point1) != 2 or len(point2) != 2:
        raise ValueError("Points must be given as tuples of length 2")

    x1, y1 = point1[0] - point0[0], point1[1] - point0[1]
    x2, y2 = point2[0] - point0[0], point2[1] - point0[1]
    # if it's smaller than (1/10)^12
    return abs(x1 * y2 - x2 * y1) < 1e-12

def backtrack_recursive_collinear_subsets(list_of_points: list, starting_index: int, subset: list, result_list: list) -> None:
    """
    Given a list of points, return all subsets of at least three points
    that are colinear. Uses recursive backtracking to generate all subsets.

    :param list_of_points: list of points
    :param starting_index: index to start from
    :param subset: current subset
    :param result_list: list of all subsets

    :return: None
    """
    if len(subset) >= 3:
        collinear = True

        # Go through all combinations of three points
        for combination in combinations(subset, 3):
            point1, point2, point3 = combination
            if not test_colinearity_for_three_points(point1, point2, point3):
                collinear = False
                break

        if collinear:
            result_list.append(subset[:])

    for i in range(starting_index, len(list_of_points)):
        # Mark the current element as part of the subset
        subset.append(list_of_points[i])
        # Recursively call the function with the next index
        backtrack_recursive_collinear_subsets(list_of_points, i + 1, subset, result_list)
        # Unmark the current element as part of the subset
        subset.pop()


def find_collinear_subsets_recursively(list_of_points: list) -> list:
    """
    Calls a recursive backtracking function to find all subsets of at least
    three points that are colinear.

    :param list_of_points: list of points

    :return: list of subsets
    """
    if len(list_of_points) < 3:
        raise ValueError("List of points must have at least three elements")

    result_of_collinear_subsets = []
    backtrack_recursive_collinear_subsets(list_of_points, 0, [], result_of_collinear_subsets)
    return result_of_collinear_subsets

def find_collinear_subsets_iteratively(list_of_points: list) -> list:
    """
    Finds all subsets of at least three points that are colinear.
    Uses an iterative approach.

    :param list_of_points: list of points

    :return: list of subsets
    """
    length_of_list_of_points = len(list_of_points)

    if length_of_list_of_points < 3:
        raise ValueError("List of points must have at least three elements")

    # Create an empty list to store the subsets
    result_of_collinear_subsets = []

    # Loop through all sizes of subsets starting from 3
    for size in range(3, length_of_list_of_points + 1):

        # Generate combinations of [size] using a mask
        for taken_bitmask in range(1 << length_of_list_of_points):

            # How many bits of 1 are in the mask
            count_of_set_bits = bin(taken_bitmask).count('1')

            # IF the number of set bits is not equal to the size of the subset, continue
            if count_of_set_bits != size:
                continue

            # Create an empty subset
            subset = []

            # Populate the subset based on the bitmask
            for j in range(length_of_list_of_points):
                # If the j-th bit is set, add the j-th element to the subset
                # Test bit j of bitmask
                if (taken_bitmask & (1 << j)) > 0:
                    subset.append(list_of_points[j])

            collinear = True
            for combination in combinations(subset, 3):
                point1, point2, point3 = combination
                if not test_colinearity_for_three_points(point1, point2, point3):
                    collinear = False
                    break

            if collinear:
                result_of_collinear_subsets.append(subset[:])

    return result_of_collinear_subsets

def find_collinear_subsets_iteratively_using_stack(list_of_points: list) -> list:
    """
    Finds all subsets of at least three points that are colinear.
    Uses an iterative approach and a stack.

    :param list_of_points: list of points

    :return: list of subsets
    """
    length_of_list_of_points = len(list_of_points)

    if length_of_list_of_points < 3:
        raise ValueError("List of points must have at least three elements")

    stack = []
    result_of_collinear_subsets = []

    # Push the initial state onto the stack
    stack.append((0, []))

    while stack:
        starting_index, subset = stack.pop()

        if len(subset) >= 3:
            collinear = True
            for combination in combinations(subset, 3):
                point1, point2, point3 = combination
                if not test_colinearity_for_three_points(point1, point2, point3):
                    collinear = False
                    break

            if collinear:
                result_of_collinear_subsets.append(subset[:])

        for i in range(starting_index, len(list_of_points)):
            # Mark the current element as part of the subset
            new_subset = subset + [list_of_points[i]]
            # Push the new state onto the stack for future processing
            stack.append((i + 1, new_subset))

    return result_of_collinear_subsets


def main_program() -> None:
    # number_of_points_on_2D_plane = read_integer_from_keyboard("Number of points on the 2D plane: ")
    # list_of_points_on_2D_plane = generate_random_points(number_of_points_on_2D_plane)
    # list_of_points_on_2D_plane = generate_semi_random_points_list(number_of_points_on_2D_plane)
    # list_of_points_on_2D_plane = read_list_of_points_from_keyboard(number_of_points_on_2D_plane)
    list_of_points_on_2D_plane = [(0, 0), (1, 1), (2, 2), (3, 3), (3, 4), (4, 5)]
    subsets_of_collinear_points_from_2D_plane_recursive = find_collinear_subsets_recursively(list_of_points_on_2D_plane)
    subsets_of_collinear_points_from_2D_plane_iterative = find_collinear_subsets_iteratively(list_of_points_on_2D_plane)


    print(f"Recursive one:")
    if subsets_of_collinear_points_from_2D_plane_recursive:
        for subset in subsets_of_collinear_points_from_2D_plane_recursive:
            print(subset)
    else:
        print("No solution")
    print("\n")
    print(f"Iterative one:")
    if subsets_of_collinear_points_from_2D_plane_iterative:
        for subset in subsets_of_collinear_points_from_2D_plane_iterative:
            print(subset)
    else:
        print("No solution")




if __name__ == "__main__":
    main_program()
