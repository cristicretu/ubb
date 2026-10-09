# Public Key Cryptography

Semester 5 · Year 3 · Python

Number theory for cryptography (GCD, modular inverses, primality, factoring) and the classic public-key systems built on it. Labs and assignments are small standalone Python scripts, most of them interactive.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | Three GCD algorithms (recursive, extended Euclid, subtraction Euclid) with a timing comparison table |
| [Labs/Lab_02](Labs/Lab_02) | Hill cipher with a 2x2 key matrix: encrypt and decrypt |
| [Labs/Lab_03](Labs/Lab_03) | Fermat factorization (generalized, with multiplier k) |
| [Labs/Lab_04](Labs/Lab_04) | ElGamal over a 27-letter alphabet: `main.py` encrypts, `decrypt.py` generates keys and decrypts `ciphertext.txt` |
| [Labs/Lab_05](Labs/Lab_05) | McEliece cryptosystem: `keygen.py`, `encrypt.py`, `decrypt.py` (syndrome decoding) |
| [Assignments/Assignment_01](Assignments/Assignment_01) | Miller-Rabin primality test with step-by-step output |
| [Assignments/Assignment_02](Assignments/Assignment_02) | Factoring: Fermat's method (`fermat.py`) and Pollard's rho with selectable polynomials (`pollard.py`) |
| [Assignments/Assignment_03](Assignments/Assignment_03) | RSA with a 27-letter alphabet and block encoding (k-letter plaintext blocks, l-letter ciphertext blocks) |

## How to run

```sh
pip install numpy prettytable tabulate
python3 Assignments/Assignment_01/miller_rabin.py
```

Lab_05 reads and writes its JSON files in the current folder, so run it from there:

```sh
cd Labs/Lab_05
python3 keygen.py
python3 encrypt.py "HELLO WORLD"
python3 decrypt.py
```

## Notes

- The alphabet used in most scripts is space + A-Z (27 symbols).
- `private.json` / `public.json` in Lab_05 are toy keys generated for the lab, not real secrets. The `.npz` files are not used by the current scripts.
