import numpy as np
from itertools import combinations

ALPHA = ' ABCDEFGHIJKLMNOPQRSTUVWXYZ'

def gf2_mul(a, b): return np.mod(a @ b, 2).astype(np.int8)

def bits2txt(bits):
    out = []
    for i in range(0, len(bits)//5*5, 5):
        idx = int(''.join(map(str, bits[i:i+5])), 2)
        if idx < len(ALPHA): out.append(ALPHA[idx])
    return ''.join(out).rstrip()

def decode(r, H, t):
    n = H.shape[1]
    syn = gf2_mul(H, r.reshape(-1, 1)).flatten()
    if not syn.any(): return r
    for w in range(1, t+1):
        for pos in combinations(range(n), w):
            e = np.zeros(n, dtype=np.int8)
            e[list(pos)] = 1
            if np.array_equal(syn, gf2_mul(H, e.reshape(-1, 1)).flatten()):
                return (r + e) % 2
    assert False, "decode failed"

priv = np.load('private.npz')
S_inv, G, P_inv, H = priv['S_inv'], priv['G'], priv['P_inv'], priv['H']
k, t = int(priv['k']), int(priv['t'])

ct = np.load('ciphertext.npy')
assert ct.ndim == 2 and ct.shape[1] == P_inv.shape[0] and np.isin(ct, [0,1]).all(), "bad ciphertext"

bits = []
for blk in ct:
    c_p = gf2_mul(blk.reshape(1, -1), P_inv).flatten()
    dec = decode(c_p, H, t)
    m = dec[:k]
    m = gf2_mul(m.reshape(1, -1), S_inv).flatten()
    bits.extend(m)

print(bits2txt(np.array(bits)))
