#!/usr/bin/env python3

import random
import ast

ALPHABET = " ABCDEFGHIJKLMNOPQRSTUVWXYZ"


def extended_gcd(a, b):
    if a == 0:
        return b, 0, 1
    gcd_val, x1, y1 = extended_gcd(b % a, a)
    x = y1 - (b // a) * x1
    y = x1
    return gcd_val, x, y


def mod_inverse(a, m):
    gcd_val, x, _ = extended_gcd(a, m)
    if gcd_val != 1:
        raise ValueError("Modular inverse does not exist")
    return x % m


def is_prime(n):
    if n < 2:
        return False
    if n == 2 or n == 3:
        return True
    if n % 2 == 0:
        return False
    
    r, d = 0, n - 1
    while d % 2 == 0:
        r += 1
        d //= 2
    
    for a in [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37]:
        if a >= n:
            continue
        x = pow(a, d, n)
        if x == 1 or x == n - 1:
            continue
        for _ in range(r - 1):
            x = pow(x, 2, n)
            if x == n - 1:
                break
        else:
            return False
    return True

def generate_prime(bits=256):
    while True:
        p = random.getrandbits(bits)
        p |= (1 << bits - 1) | 1
        if is_prime(p):
            return p


def generate_keys(bits=256):
    p = generate_prime(bits)
    g = random.randint(2, p - 2)
    x = random.randint(2, p - 2)  # private key
    y = pow(g, x, p)  # public key component
    
    return p, g, y, x  # public_key, private_key

def decrypt(ciphertext_blocks, p, x):
    message = ""
    for c1, c2 in ciphertext_blocks:
        c1_x = pow(c1, x, p)
        c1_x_inv = mod_inverse(c1_x, p)
        m = (c2 * c1_x_inv) % p
        if m < len(ALPHABET):
            message += ALPHABET[m]
        else:
            message += f"[{m}]"
    return message


def main():
    # Ask for the keys that were used during encryption
    p = int(input("p (prime): "))
    x = int(input("x (private key): "))
    
    with open("ciphertext.txt", "r") as f:
        ciphertext_blocks = ast.literal_eval(f.read())
    
    decrypted_text = decrypt(ciphertext_blocks, p, x)
    print(f"\nDecrypted message: '{decrypted_text}'")


if __name__ == "__main__":
    main()