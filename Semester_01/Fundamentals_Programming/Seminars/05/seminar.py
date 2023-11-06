"""
    Greedy + Dynamic programming
"""

"""
1. principle of optimality
2. overlapping subproblems
3. memoization
"""

cache = {0: 0, 1: 1}

def fib(n: int) -> int:
    if n in cache:
        return cache[n]
    cache[n] = fib(n - 1) + fib(n - 2)
    return cache[n]

"""
1. Calculate the maximum subarray sum (subarray = elements having
continuous indices)

    e.g.
    for data = [-2, -5, 6, -2, -3, 1, 5, -6], maximum subarray sum is 7.
"""

"""
2. Knapsack problem. Given the weights and values of N items, put them in a knapsack having capacity W so that you
   maximize the value of the stored items. Items can be broken up
"""


"""
3. 0-1 Knapsack problem. Given the weights and values of N items, put them in a knapsack having capacity W so that you
   maximize the value of the stored items. Items cannot be broken up (0-1 property)
"""

"""
4. Count in how many ways we can provide change to a given sum of money (N), when provided infinite
   supply of given coin denominations.

   e.g. Let's say N = 10, and we have coins of values (1, 5, 10); we can give change in 4 ways (10, 5 + 5, 5 + 1 + ...
   and 1 + ... + 1)
"""

def count_change(N: int, coins: list) -> int:
    if N == 0:
        return 1
    if N < 0:
        return 0
    if len(coins) == 0:
        return 0
    return count_change(N - coins[0], coins) + count_change(N, coins[1:])


print(count_change(10, [1, 5, 10]))


"""
5. Gold mine problem (a.k.a checkerboard problem)
   https://www.geeksforgeeks.org/gold-mine-problem
"""

def valid(i, j, n) -> bool:
    return i >= 0 and j >= 0 and i < n and j < n

def gold_mine_problem(matrix) -> int:
    rows = len(matrix)
    cols = len(matrix[0])
    dp = [[0 for _ in range(2)] for _ in range(rows)]

    for i in range(rows - 1, -1, -1):
        for j in range(cols):

            if valid(i, j - 1, rows) is True:
                max_right = dp[i][(j+1)%2]
            else:
                max_right = 0

            if valid(i - 1, j - 1, rows) is True:
                max_right_up = dp[i - 1][(j+1)%2]
            else:
                max_right_up = 0

            if valid(i + 1, j - 1, rows) is True:
                max_right_down = dp[i + 1][(j+1)%2]
            else:
                max_right_down = 0

            dp[i][j] = matrix[i][j] + max(max_right, max_right_up, max_right_down)

    res = dp[0][0]
    for i in range(rows):
        res = max(res, dp[i][0])

    for i in range(rows):
        print(dp[i])

    return res




# dp[i][j] = max(dp[])
    pass

# matrix = [
#     [1, 3, 3],
#     [2, 1, 4],
#     [0, 6, 4]
# ]
#
matrix = [
    [10, 33, 13, 15],
    [22, 21, 4, 1],
    [5, 0, 2, 3],
    [0, 6, 14, 2]
]

# print(gold_mine_problem(matrix))

