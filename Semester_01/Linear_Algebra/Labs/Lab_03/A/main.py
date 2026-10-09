import numpy as np
from itertools import permutations


# use backtracking to generate all possible vectors
def generate_vectors(n) -> list:
    def backtrack(curr_permutation: list):
        if len(curr_permutation) == n:
            result.append(curr_permutation.copy())
            return

        # Try adding 0 and 1 to the current permutation
        for element in (0, 1):
            curr_permutation.append(element)
            backtrack(curr_permutation)
            curr_permutation.pop()

    result = []
    backtrack([])
    return result


def is_linearly_independent(vectors, n):
    result = []
    for subset in permutations(vectors, n):
        matrix = np.array(subset)
        if int(round(np.linalg.det(matrix))) % 2 != 0:
            result.append(subset)
    return result, len(result)


def formula_for_n_greater_than_4(n):
    number_of_bases = 1
    for i in range(n):
        number_of_bases *= 2**n - 2**i
    return number_of_bases


def main():
    with open("input.txt", "r") as filein:
        n = int(filein.readline())
    with open("output.txt", "w") as fileout:
        if n <= 4:
            vectors = generate_vectors(n)
            result, number_of_bases = is_linearly_independent(vectors, n)
            fileout.write(
                f"Number of bases of the vector space Z^n over Z_2 is {number_of_bases}.\n"
            )
            for i, basis in enumerate(result, start=1):
                fileout.write(f"Basis {i}: {basis}\n")
        else:
            number_of_bases = formula_for_n_greater_than_4(n)
            fileout.write(
                f"Number of bases of the vector space Z^n over Z_2 is {number_of_bases}.\n"
            )


main()
