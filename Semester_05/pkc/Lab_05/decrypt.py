import json
from itertools import combinations

ALPHA = ' ABCDEFGHIJKLMNOPQRSTUVWXYZ'

def mul(a, b): return [[sum(a[i][k]*b[k][j] for k in range(len(b)))%2 for j in range(len(b[0]))] for i in range(len(a))]
def transpose(m): return [[m[i][j] for i in range(len(m))] for j in range(len(m[0]))]

def bits2txt(bits):
    out = []
    for i in range(0, len(bits)//5*5, 5):
        idx = int(''.join(map(str, bits[i:i+5])), 2)
        if idx < len(ALPHA): out.append(ALPHA[idx])
    return ''.join(out).rstrip()

def syndrome(r, H): return mul(H, transpose([r]))[0] if isinstance(H[0], list) else [sum(H[i]*r[i] for i in range(len(r)))%2]
def syn(r, H): return [sum(H[i][j]*r[j] for j in range(len(r)))%2 for i in range(len(H))]

def decode(r, H, t):
    n = len(r)
    s = syn(r, H)
    if not any(s): return r
    for w in range(1, t+1):
        for pos in combinations(range(n), w):
            e = [1 if i in pos else 0 for i in range(n)]
            if syn(e, H) == s:
                return [(r[i] + e[i]) % 2 for i in range(n)]
    assert False, "decode failed"

priv = json.load(open('private.json'))
S_inv, G, P_inv, H, k, t = priv['S_inv'], priv['G'], priv['P_inv'], priv['H'], priv['k'], priv['t']

ct = json.load(open('ciphertext.json'))
assert all(len(blk) == len(P_inv) and all(b in [0,1] for b in blk) for blk in ct), "bad ciphertext"

bits = []
for blk in ct:
    c_p = mul([blk], P_inv)[0]
    dec = decode(c_p, H, t)
    m = dec[:k]
    m = mul([m], S_inv)[0]
    bits.extend(m)

print(bits2txt(bits))
