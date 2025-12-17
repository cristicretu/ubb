import numpy as np
from numpy.random import default_rng

rng = default_rng()

def gf2_inv(m):
    n = m.shape[0]
    aug = np.hstack([m.copy(), np.eye(n, dtype=np.int8)])
    for c in range(n):
        p = next((r for r in range(c, n) if aug[r, c]), None)
        if p is None: return None
        aug[[c, p]] = aug[[p, c]]
        for r in range(n):
            if r != c and aug[r, c]: aug[r] = (aug[r] + aug[c]) % 2
    return aug[:, n:].astype(np.int8)

def gf2_mul(a, b): return np.mod(a @ b, 2).astype(np.int8)

def min_dist(G):
    k, n = G.shape
    from itertools import combinations
    min_w = n + 1
    for r in range(1, k+1):
        for rows in combinations(range(k), r):
            c = np.zeros(n, dtype=np.int8)
            for row in rows: c = (c + G[row]) % 2
            w = np.sum(c)
            if 0 < w < min_w: min_w = w
    return min_w

def gen_good_code(n, k, t):
    need_dist = 2*t + 1
    while True:
        P = rng.integers(0, 2, (k, n-k), dtype=np.int8)
        G = np.hstack([np.eye(k, dtype=np.int8), P])
        if min_dist(G) >= need_dist:
            H = np.hstack([P.T, np.eye(n-k, dtype=np.int8)])
            return G, H

n, k, t = 20, 8, 2
print(f"generating code with d >= {2*t+1}...")
G, H = gen_good_code(n, k, t)
print(f"found code with d = {min_dist(G)}")

while True:
    S = rng.integers(0, 2, (k, k), dtype=np.int8)
    S_inv = gf2_inv(S)
    if S_inv is not None: break

perm = rng.permutation(n)
P_mat = np.eye(n, dtype=np.int8)[perm]
P_inv = np.eye(n, dtype=np.int8)[np.argsort(perm)]
G_pub = gf2_mul(gf2_mul(S, G), P_mat)

np.savez('public.npz', G_pub=G_pub, n=n, k=k, t=t)
np.savez('private.npz', S_inv=S_inv, G=G, P_inv=P_inv, H=H, k=k, t=t)

print(f"keys generated (n={n}, k={k}, t={t})")
