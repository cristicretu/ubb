"""
P3. No of bases for Z2^n over Z2 and the vectors that form them
@Author: Cretu Cristian, 913
Usage: python3 main.py 01_input.txt
"""
import sys
import numpy as np
from typing import List, Tuple
from itertools import combinations, permutations


def compute_number_of_bases(n: int) -> int:
    """
    Compute the number of bases for Z2^n over Z2.

    We start with the non-zero vector u1 => I can have 2^n - 1 possibilities (-1 for the 0 vector)
    u2 = everything besides u1, and 0 vector => I can have 2^n - 2
    u3 = -||-, and also 0, u1, u2, u1 + u2 ...

    :param n: The dimension of the vector space

    :return: The number of bases for Z2^n over Z2
    """
    return np.prod([(2**n) - (2**i) for i in range(n)])

# Function to compute the determinant of a matrix modulo 2
# https://numpy.org/doc/stable/reference/generated/numpy.linalg.det.html
def binary_det(matrix: np.ndarray) -> int:
    """
    Compute the determinant of a matrix modulo 2.
    Uses the numpy.linalg.det function.

    :param matrix: The matrix whose determinant we want to compute

    :return: The determinant of the matrix modulo 2
    """
    return np.linalg.det(matrix) % 2


# n = 3 =>
# [[0, 0, 1], [0, 1, 0], [0, 1, 1], [1, 0, 0], [1, 0, 1], [1, 1, 0], [1, 1, 1]]
def generate_binary_vectors(n: int) -> List[List[int]]:
    """
    Generate all binary vectors of length n.
    Starts from 1 and goes up to 2^n - 1. Then converts each number to its binary representation.

    :param n: The length of the binary vectors

    :return: A list of all binary vectors of length n
    """
    return [list(map(int, np.binary_repr(i, width=n))) for i in range(1, 2**n)]


def find_bases(vectors: List[List[int]], n: int, file) -> None:
    """
    This function generates all possible permutations of 'n' vectors.
    For each permutation, it creates a matrix and checks if its determinant is equal to 1.
    If the determinant is 1, the permutation is considered a basis.

    :param vectors: A list of all binary vectors of length n
    :param n: The length of the binary vectors
    :param file: The output file to write the bases to

    :return: None
    """
    for permutation in permutations(vectors, n):
        # Create a matrix from the current permutation
        mat = np.array(permutation, dtype=int)

        # Check if the determinant of the matrix is equal to 1
        if binary_det(mat) == 1:
            # Write the basis as a tuple of tuples to the output file
            file.write(str(tuple(map(tuple, mat))) + '\n')


if __name__ == "__main__":
    if len(sys.argv) != 2:
            print("Usage: python3 main.py input_file.txt")
            sys.exit(1)

    input_file = sys.argv[1]

    try:
        with open(input_file, "r") as file:
            n = int(file.readline().strip())
    except FileNotFoundError:
        print(f"Input file '{input_file}' not found.")
        sys.exit(1)
    except ValueError:
        print(f"Invalid input in '{input_file}'. Please provide a single integer on the first line.")
        sys.exit(1)

    output_file = f"{input_file.split('_')[0]}_output.txt"

    with open(output_file, "w") as file:
        file.write(f"1. the number of bases for Z2^n over Z2 is {compute_number_of_bases(n)}\n")

        if n <= 4:
            file.write("2. the vectors of each such basis are:\n")

            vectors: List[List[int]] = generate_binary_vectors(n)

            find_bases(vectors, n, file)

    print(f"Results written to {output_file}")
