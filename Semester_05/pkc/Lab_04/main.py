import random

ALPHABET = " ABCDEFGHIJKLMNOPQRSTUVWXYZ"

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


def encrypt(plaintext, public_key):
    p, g, y = public_key
    # y = g^x mod p
    plaintext = plaintext.upper()
    
    for char in plaintext:
        if char not in ALPHABET:
            raise ValueError(f"Invalid character: {char}")
    
    ciphertext = []
    for char in plaintext:
        m = ALPHABET.index(char)
        k = random.randint(2, p - 2)
        c1 = pow(g, k, p)
        c2 = (m * pow(y, k, p)) % p 
        ciphertext.append((c1, c2))
    
    return ciphertext


if __name__ == "__main__":
    p = int(input("p = "))
    g = int(input("g = "))
    y = int(input("y = "))
    
    public_key = (p, g, y)
    
    plaintext = input("Plaintext: ")
    ciphertext = encrypt(plaintext, public_key)
    
    print(f"Ciphertext: {ciphertext}")
