import numpy as np
from numpy.random import default_rng
import sys

ALPHA = ' ABCDEFGHIJKLMNOPQRSTUVWXYZ'
rng = default_rng()

def gf2_mul(a, b): return np.mod(a @ b, 2).astype(np.int8)

def txt2bits(txt):
    txt = txt.upper()
    assert all(c in ALPHA for c in txt), f"invalid chars, use: {ALPHA}"
    return np.array([int(b) for c in txt for b in f'{ALPHA.index(c):05b}'], dtype=np.int8)

pub = np.load('public.npz')
G_pub, t = pub['G_pub'], int(pub['t'])
k, n = G_pub.shape

plaintext = sys.argv[1] if len(sys.argv) > 1 else input("plaintext: ")
bits = txt2bits(plaintext)

ct = []
for i in range(0, len(bits), k):
    m = np.pad(bits[i:i+k], (0, max(0, k-len(bits[i:i+k]))))
    c = gf2_mul(m.reshape(1, -1), G_pub).flatten()
    e = np.zeros(n, dtype=np.int8)
    e[rng.choice(n, t, replace=False)] = 1
    ct.append((c + e) % 2)

ct = np.array(ct, dtype=np.int8)
np.save('ciphertext.npy', ct)
print(f"encrypted {len(plaintext)} chars -> ciphertext.npy")
