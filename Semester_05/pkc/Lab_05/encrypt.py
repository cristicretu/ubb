import random, json, sys
from itertools import combinations

ALPHA = ' ABCDEFGHIJKLMNOPQRSTUVWXYZ'

def mul(a, b): return [[sum(a[i][k]*b[k][j] for k in range(len(b)))%2 for j in range(len(b[0]))] for i in range(len(a))]

def txt2bits(txt):
    txt = txt.upper()
    assert all(c in ALPHA for c in txt), f"bad char, use: {ALPHA}"
    return [int(b) for c in txt for b in f'{ALPHA.index(c):05b}']

pub = json.load(open('public.json'))
G_pub, t = pub['G_pub'], pub['t']
k, n = len(G_pub), len(G_pub[0])

plaintext = sys.argv[1] if len(sys.argv) > 1 else input("plaintext: ")
bits = txt2bits(plaintext)

ct = []
for i in range(0, len(bits), k):
    m = bits[i:i+k] + [0]*(k - len(bits[i:i+k]))
    c = mul([m], G_pub)[0]
    err_pos = random.sample(range(n), t)
    for p in err_pos: c[p] ^= 1
    ct.append(c)

json.dump(ct, open('ciphertext.json', 'w'))
print(f"encrypted {len(plaintext)} chars -> ciphertext.json")
