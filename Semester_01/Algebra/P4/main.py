"""
P4. No of subspaces for Z2^n over Z2 and the basis for each subspace.
@Author: Cretu Cristian, 913
Usage: python3 main.py 01_input.txt
"""
import sys
import numpy as np
from typing import List, Tuple
from itertools import combinations


def gauss_binomial(dimension: int, subspace_dim: int, field_size=2) -> int:
    """
    Calculate the number of k-dimensional subspaces of Z_q^n
    https://en.wikipedia.org/wiki/Gaussian_binomial_coefficient#Central_q-binomial_identity

    :param n: The dimension of the vector space
    :param k: The dimension of the subspace
    :param q: The size of the field
    """
    numerator = 1
    denominator = 1
    for i in range(subspace_dim):
        numerator *= field_size ** (dimension - i) - 1
        denominator *= field_size ** (i + 1) - 1
    return numerator // denominator


def generate_binary_vectors(n: int) -> List[np.ndarray]:
    """
    Generate all the vectors in Z_2^n

    :param n: The dimension of the vector space
    :return: A list of numpy arrays representing the vectors
    """
    return [
        np.array(list(np.binary_repr(i, width=n)), dtype=int) for i in range(1, 2**n)
    ]


def is_basis(vectors: Tuple[np.ndarray, ...]) -> bool:
    """
    Check if the given vectors form a basis for the vector space

    :param vectors: A tuple of numpy arrays representing the vectors
    :return: True if the vectors form a basis, False otherwise
    """
    mat = np.stack(vectors)
    return np.linalg.matrix_rank(mat, tol=None, hermitian=False) == mat.shape[0]


def compute_reduced_row_echelon_form(matrix: np.ndarray) -> np.ndarray:
    """
    Compute the Reduced Row Echelon Form of a binary matrix over Z2.

    :param matrix: A numpy array representing the matrix
    :return: The RREF of the matrix
    """
    echleon_form = np.copy(matrix)
    rows, cols = echleon_form.shape

    pivot_index = 0
    for col_index in range(cols):
        nonzero_row = None
        for row_index in range(pivot_index, rows):
            if echleon_form[row_index, col_index] == 1:
                nonzero_row = row_index
                break

        if nonzero_row is None:
            continue

        echleon_form[[pivot_index, nonzero_row]] = echleon_form[
            [nonzero_row, pivot_index]
        ]

        for other_row in range(rows):
            if other_row != pivot_index and echleon_form[other_row, col_index] == 1:
                echleon_form[other_row] ^= echleon_form[pivot_index]

        pivot_index += 1

        if pivot_index == rows:
            break

    return echleon_form


def find_subspaces_with_unique_basis(
    vectors: Tuple[np.array, ...], k: int
) -> List[Tuple[np.array, ...]]:
    """
    Find all the unique k-dimensional subspaces of the given vectors using the RREF method.
    https://en.wikipedia.org/wiki/Row_echelon_form#Reduced_row_echelon_form

    :param vectors: A list of numpy arrays representing the vectors
    :param k: The dimension of the subspace
    :return: A list of tuples representing the subspaces
    """
    unique_subspaces = set()
    subspaces = []

    # Generate all the possible combinations of k vectors
    for potential_basis in combinations(vectors, k):
        if is_basis(potential_basis):
            matrix_basis = np.vstack(potential_basis)
            rref_mat = compute_reduced_row_echelon_form(matrix_basis)

            # Create a tuple representation of the RREF matrix
            canonical_form = tuple(map(tuple, rref_mat))

            # check if we've already encountered this canonical form
            if canonical_form not in unique_subspaces:
                unique_subspaces.add(canonical_form)
                subspaces.append(potential_basis)

    return subspaces


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 main.py input_file.txt")
        sys.exit(1)

    input_file = sys.argv[1]

    try:
        with open(input_file, "r") as file:
            k = int(file.readline().strip())
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

    with open(output_file, "w") as file:
        file.write(
            f"1. the number 2-dimensional subspaces of the vector space Z2^{n} over Z2 is {gauss_binomial(n, k)}\n"
        )

        if n <= 6:
            file.write("2. a basis for each subspace is:\n")

            vectors = generate_binary_vectors(n)
            unique_subspaces_rr = find_subspaces_with_unique_basis(vectors, k)

            for subspace in unique_subspaces_rr:
                subspace_as_tuples = tuple(map(tuple, subspace))

                file.write(f"{subspace_as_tuples}\n")

    print(f"Results written to {output_file}")
