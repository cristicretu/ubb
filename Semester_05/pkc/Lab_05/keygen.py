# macelice
import random, json
from itertools import combinations

def mat(rows, cols, rand=False): return [[random.randint(0,1) if rand else 0 for _ in range(cols)] for _ in range(rows)]
def eye(n): return [[1 if i==j else 0 for j in range(n)] for i in range(n)]
def transpose(m): return [[m[i][j] for i in range(len(m))] for j in range(len(m[0]))]
def hstack(a, b): return [a[i] + b[i] for i in range(len(a))]
def mul(a, b): return [[sum(a[i][k]*b[k][j] for k in range(len(b)))%2 for j in range(len(b[0]))] for i in range(len(a))]

def inv(m):
    n = len(m)
    aug = [m[i][:] + eye(n)[i] for i in range(n)]
    for c in range(n):
        p = next((r for r in range(c, n) if aug[r][c]), None)
        if p is None: return None
        aug[c], aug[p] = aug[p], aug[c]
        for r in range(n):
            if r != c and aug[r][c]: aug[r] = [(aug[r][j] + aug[c][j]) % 2 for j in range(2*n)]
    return [row[n:] for row in aug]

def weight(v): return sum(v)

def min_dist(G):
    # d-1 errors
    k, n = len(G), len(G[0])
    best = n + 1
    for r in range(1, k+1):
        for rows in combinations(range(k), r):
            c = [0]*n
            for row in rows: c = [(c[j] + G[row][j]) % 2 for j in range(n)]
            w = weight(c)
            if 0 < w < best: best = w
    return best

def gen_good_code(n, k, t):
    while True:
        P = mat(k, n-k, rand=True)
        G = hstack(eye(k), P)
        if min_dist(G) >= 2*t+1:
            H = hstack(transpose(P), eye(n-k))
            return G, H

# codeword, msg
n, k, t = 20, 8, 2
print(f"generating code with d >= {2*t+1}...")
G, H = gen_good_code(n, k, t)
print(f"found code with d = {min_dist(G)}")

while True:
    S = mat(k, k, rand=True)
    S_inv = inv(S)
    if S_inv: break

perm = list(range(n))
random.shuffle(perm)
P_mat = [eye(n)[i] for i in perm]
P_inv = [[1 if perm[j]==i else 0 for j in range(n)] for i in range(n)]
G_pub = mul(mul(S, G), P_mat)

json.dump({'G_pub': G_pub, 'n': n, 'k': k, 't': t}, open('public.json', 'w'))
json.dump({'S_inv': S_inv, 'G': G, 'P_inv': P_inv, 'H': H, 'k': k, 't': t}, open('private.json', 'w'))

print(f"keys generated (n={n}, k={k}, t={t})")
