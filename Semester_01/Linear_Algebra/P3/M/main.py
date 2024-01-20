"""
Input: non-zero integer n
Output:
        1. the number of bases of the vector space Z_2^n (2^n) over Z_2
        2. the vectors for each such basis for n <= 4
"""
import numpy as np
from itertools import combinations, permutations


def nr_of_bases(n):
    """
    This function calculates the number of bases of the vector space Z_2^n over Z_2
    with the following formula: 2^n - 2^i, where i = 0, 1, ..., n-1
    """
    result = 1
    for i in range(n):
        result *= 2**n - 2**i
    return result


def generate_bases(n):
    """
    This function generates all bases for Z_2^n over Z_2 for n <= 4
    by ensuring that each vector in a basis has a '1' in a unique position.
    """
    if n > 4:
        raise ValueError("n must be less than or equal to 4")

    # Generate all non-zero vectors
    all_vectors = [vec for vec in range(1, 2**n)]

    # Generate all permutations of the non-zero vectors
    bases = list(permutations(all_vectors, n))

    # We only need linearly independent bases
    valid_bases = []
    for basis in bases:
        matrix = [[(vec >> i) & 1 for i in range(n)] for vec in basis]
        # We check only in Z2
        if np.linalg.det(matrix) % 2:
            valid_bases.append(basis)

    # Convert integer representations back to binary vectors
    binary_bases = []
    for basis in valid_bases:
        binary_basis = []
        for vec in basis:
            binary_vec = [int(digit) for digit in bin(vec)[2:].zfill(n)]
            binary_basis.append(binary_vec)
        binary_bases.append(binary_basis)

    return binary_bases


input_file = open("input.txt", "r")
output_file = open("output.txt", "w")
n = int(input_file.read())
input_file.close()
output_file.write(
    f"1. the number of bases of the vector space Z_2^{n} over Z_2 is {nr_of_bases(n)} \n"
)
if n <= 4:
    output_file.write(f"2. the vectors for each such basis for n = {n} are: \n")

    bases = generate_bases(n)

    for basis in bases:
        output_file.write(f"{basis}\n")

output_file.close()
