"""
    Problem Solving Methods -- Divide & Conquer and Backtracking
"""

"""
1. Find the smallest number in a list (chip & conquer, divide in halves, recursive vs non-recursive). Return None for
    an empty list
    a. Chip & conquer, recursive
    b. Divide in halves, non-recursive
    c. Divide in halves, recursive
"""

"""
2. Exponential search
    a. Generate a pseudo-random array of increasing elements
    b. Implement exponential search
    c. Implement binary search
    d. Driver & test functions
"""

"""
3. Calculate the r-th root of a given number x with a given precision p
"""

def maximum_sum_subarray(data: list) -> int:
    """
    Returns the maximum sum of a subarray from the initial array

    :data: the initial array
    :return: the maximum sum of a subarray from the initial array
    """
    maximumSum = 0
    currentSum = 0

    for i in range(0, len(data)):
        if currentSum < 0:
            currentSum = 0

        currentSum += data[i]

        currentSum = max(currentSum + data[i])

        if currentSum > maximumSum:
            maximumSum = currentSum

    return maximumSum

print(maximum_sum_subarray([-2, -5, 6, -2, -3, 1, 5, -6]))

def divide_and_conquer_maximum_sum_subarray(data: list, left: int, right: int) -> int:
    if len(data) == 1:
        return data[0]

    m =  int(left + (right - left) // 2)

    max_left = divide_and_conquer_maximum_sum_subarray(data, left, m)
    max_right = divide_and_conquer_maximum_sum_subarray(data, m + 1, right)

    max_left_sum = 0
    current_left_sum = 0
    for i in range(m - 1, -1, -1):
        current_left_sum += data[i]
        if current_left_sum > max_left_sum:
            max_left_sum = current_left_sum

    max_right_sum = 0
    current_right_sum = 0
    for i in range(m, len(data)):
        current_right_sum += data[i]
        if current_right_sum > max_right_sum:
            max_right_sum = current_right_sum

    return max(max_left, max_right, max_left_sum + max_right_sum)


print(divide_and_conquer_maximum_sum_subarray([-2, -5, 6, -2, -3, 1, 5, -6], 0, 8))




"""
4. Calculate the maximum subarray sum (subarray = elements having continuous indices)
    a. Naive implementation
    b. Divide & conquer implementation

    e.g.
    for data = [-2, -5, 6, -2, -3, 1, 5, -6], maximum subarray sum is 7.
"""

# print(naive_maximum_subarray_sum([-2, -5, 6, -2, -3, 1, 7, -6]))

"""
    Backtracking
"""

"""
5. Recursive implementation for permutations
"""


def consistent(x):
    """
    Determines whether the current partial array can lead to a solution
    """
    return len(set(x)) == len(x)


def solution(x, n):
    """
    Determines whether we have a solution
    """
    return len(x) == n


def solution_found(x):
    """
    What to do when a solution is found
    """
    print("Solution: ", x)


def bkt_rec(x, n):
    """
    Backtracking algorithm for permutations problem, recursive implementation
    """
    x.append(0)
    for i in range(0, n):
        x[len(x) - 1] = i
        if consistent(x):
            if solution(x, n):
                solution_found(x)
            else:
                bkt_rec(x[:], n)


# bkt_rec([], 4)

"""
6. Change the code for generating the permutation above to work for the n-Queen problem
"""

"""
A Latin square is an n × n square filled with n different symbols, each occurring exactly once in each row and exactly
once in each column

7. Generate all the N x N Latin squares for a given number N.

8. Generate all reduced N x N Latin squares for a given number N. In a reduced Latin square, the elements of the first
row and column are sorted.
"""

