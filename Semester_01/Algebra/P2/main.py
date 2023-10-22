from itertools import product

def is_associative(operation, A):
    for a, b, c in product(A, repeat=3):
        if operation[(operation[(a, b)], c)] != operation[(a, operation[(b, c)])]:
            return False
    return True

def count_associative_operations(n):
    A = list(range(n))
    total_operations = 0
    associative_operations = 0

    for operation_table in product(A, repeat=n*n):
        total_operations += 1
        operation = {(A[i], A[j]): operation_table[i * n + j] for i in range(n) for j in range(n)}

        if is_associative(operation, A):
            associative_operations += 1

    return associative_operations

# Count associative operations for n <= 6
# results = {}
# for n in range(1, 7):
#     results[n] = count_associative_operations(n)

# results
print(count_associative_operations(4))
