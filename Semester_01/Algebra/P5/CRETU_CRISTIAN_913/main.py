"""
P5. No of unique RREF matrices for Z2^m x Z2^n over Z2.
@Author: Cretu Cristian, 913
Usage: python3 main.py 01_input.txt
"""

"""
reference: https://math.stackexchange.com/questions/3482267/counting-echelon-form-matrices
"""

from itertools import product
import sys

import numpy as np


def compute_reduced_row_echelon_form(matrix: np.ndarray) -> np.ndarray:
    """
    # taken from P4
    Compute the Reduced Row Echelon Form of a binary matrix over Z2.

    :param matrix: A numpy array representing the matrix
    :return: The RREF of the matrix
    """
    echelon_form = np.copy(matrix)
    rows, cols = echelon_form.shape

    pivot_index = 0
    for col_index in range(cols):
        nonzero_row = None
        for row_index in range(pivot_index, rows):
            if echelon_form[row_index, col_index] == 1:
                nonzero_row = row_index
                break

        if nonzero_row is None:
            continue

        echelon_form[[pivot_index, nonzero_row]] = echelon_form[
            [nonzero_row, pivot_index]
        ]

        for other_row in range(rows):
            if other_row != pivot_index and echelon_form[other_row, col_index] == 1:
                echelon_form[other_row] ^= echelon_form[pivot_index]

        pivot_index += 1

        if pivot_index == rows:
            break

    return echelon_form


def generate_rref_matrices(m, n):
    """
    Generate all possible RREF matrices of size m x n over Z2.

    :param m: The number of rows
    :param n: The number of columns

    :return: A set of all possible RREF matrices
    """
    # take all possible binary matrices of size m x n
    # = 2^(m * n)
    binary_matrices = list(product([0, 1], repeat=m * n))
    rref_matrices = set()

    for binary_matrix in binary_matrices:
        # convert the binary matrix to a numpy array
        matrix = np.array(binary_matrix).reshape(m, n)

        rref = compute_reduced_row_echelon_form(matrix)

        # since np arrays are not hashable, we need to convert them to tuples of tuples
        # in order to add them to a set (we only need unique ones)
        rref_matrices.add(tuple(map(tuple, rref)))

    return rref_matrices


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 main.py input_file.txt")
        sys.exit(1)

    input_file = sys.argv[1]

    try:
        with open(input_file, "r") as file:
            m = int(file.readline().strip())
            n = int(file.readline().strip())
    except FileNotFoundError:
        print(f"Input file '{input_file}' not found.")
        sys.exit(1)
    except ValueError:
        print(
            f"Invalid input in '{input_file}'. Please provide a single integer on the first line."
        )
        sys.exit(1)

    output_file = f"{input_file.split('_')[0]}_output.txt"

    unique_rref_matrices = generate_rref_matrices(n, m)
    count = len(unique_rref_matrices)

    with open(output_file, "w") as file:
        file.write(
            f"1. the number of matrices A from M{n},{m}(Z2) in reduced echelon form is: {count}\n"
        )

        if n <= 6:
            file.write(
                f"2. the matrices A from M{n},{m}(Z2) in reduced echelon form are:\n"
            )

            # convert the tuples back to numpy arrays
            rref_matrix_list = [np.array(matrix) for matrix in unique_rref_matrices]

            for matrix in rref_matrix_list:
                file.write(f"{matrix}\n")

    print(f"Results written to {output_file}")
