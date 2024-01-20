"""
P1. Given a set A = {a1, a2, ..., an}, write a program to generate all permutations on A and their corresponding
    equivalence relations.

@Author: Cretu Cristian, 913

Usage: python3 main.py 01_input.txt
"""

import sys
# import permutations from itertools to generate all permutations on a set
from itertools import permutations
# import typing to enable type hints
from typing import List, Set, Tuple

# global dictionary to store unique solutions
solutions = {}

def add_to_set(v: List[int]) -> None:
    """
    Given a permutation list v, this function partitions it into sets
    where each set contains contiguous increasing elements from v.
    These partitions are then added to the global dictionary 'solutions' if unique.
    """
    solution = []
    s = {v[0]}  # start a new set with the first element from the list
    for i in range(1, len(v)):
        if v[i] < v[i - 1]:
            # if the current element is smaller than the last one, add the set to the solution
            solution.append(s)
            # start a new set
            s = set()
        # add the current element to the set
        s.add(v[i])

    # account for the last set
    solution.append(s)

    # sort each set and the solution itself to enable easy comparison
    solution = sorted([sorted(list(x)) for x in solution])

    # convert the solution to a string to use as a key in the dictionary
    key = str(solution)
    # add the solution to the dictionary if it is unique
    if key not in solutions:
        solutions[key] = solution

def create_pairs(s: Set[int]) -> Set[Tuple[int, int]]:
    """
    Given a set s, this function creates all possible pairs (x, y) where x and y are elements of s.
    """
    # use a set comprehension to create all possible pairs
    return {(x, y) for x in s for y in s}

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

    # generate all permutations for the numbers from 1 to n
    for p in permutations(range(1, n + 1)):
        add_to_set(list(p))

    with open(output_file, "w") as file:
            file.write(f"1. the number of permutations on a set A = {{{', '.join([f'a{i}' for i in range(1, n+1)])}}} is {len(solutions)}\n")
            file.write("2. the partitions on the set A, and their corresponding equivalence relations, are:\n")

            # iterate through each unique solution to write it and its equivalence relations to the output file
            for key in solutions:
                sol = solutions[key]
                # convert each set to a string to use in the output file
                sets_str = ", ".join([f"({', '.join([f'a[{elem}]' for elem in s])})" for s in sol])

                # create all possible pairs for each set
                pairs = create_pairs(set.union(*[set(s) for s in sol]))
                # convert each pair to a string to use in the output file
                pairs_str = ", ".join([f"(a[{pair[0]}], a[{pair[1]}])" for pair in pairs])

                # write the solution and its equivalence relations to the output file
                file.write(f"{{{sets_str}}} -> {{{pairs_str}}}\n")

    print(f"Results written to {output_file}")
