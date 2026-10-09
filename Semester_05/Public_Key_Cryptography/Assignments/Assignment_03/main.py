#!/usr/bin/env python3
"""
RSA Encryption/Decryption Implementation
Based on the provided slides with 27-letter alphabet
"""

import math


def gcd(a, b):
    """Calculate the Greatest Common Divisor using Euclidean algorithm"""
    while b:
        a, b = b, a % b
    return a


def extended_gcd(a, b):
    """
    Extended Euclidean Algorithm
    Returns (gcd, x, y) such that a*x + b*y = gcd(a, b)
    """
    if a == 0:
        return b, 0, 1
    gcd_val, x1, y1 = extended_gcd(b % a, a)
    x = y1 - (b // a) * x1
    y = x1
    return gcd_val, x, y


def mod_inverse(e, phi):
    """
    Calculate the modular multiplicative inverse of e modulo phi
    Returns d such that (e * d) % phi = 1
    """
    gcd_val, x, _ = extended_gcd(e, phi)
    if gcd_val != 1:
        raise ValueError("Modular inverse does not exist")
    return x % phi


def is_prime(n):
    """Check if a number is prime"""
    if n < 2:
        return False
    if n == 2:
        return True
    if n % 2 == 0:
        return False
    for i in range(3, int(math.sqrt(n)) + 1, 2):
        if n % i == 0:
            return False
    return True


def calculate_phi(n, p, q):
    """
    Calculate Euler's totient function φ(n)
    
    Case 1: n even → p=2, q=n/2, φ(n) = n/2 - 1
    Case 2: n odd → φ(n) = (p-1)(q-1)
    """
    if n % 2 == 0:
        # Case 1: n even
        return n // 2 - 1
    else:
        # Case 2: n odd
        return (p - 1) * (q - 1)


def find_smallest_valid_e(phi):
    """
    Find the smallest valid odd prime e such that gcd(e, φ(n)) = 1
    Start from 3 (smallest odd prime) and check each odd prime
    """
    e = 3
    while True:
        if is_prime(e) and gcd(e, phi) == 1:
            return e
        e += 2  # Check only odd numbers


def text_to_number(text, block_size):
    """
    Convert text to numbers using 27-letter alphabet
    '_' (space) = 0, 'A' = 1, 'B' = 2, ..., 'Z' = 26
    
    Blocks are formed using base-27 representation:
    For block "ab": value = a * 27 + b
    """
    # Normalize text: uppercase and replace spaces with '_'
    text = text.upper().replace(' ', '_')
    
    blocks = []
    for i in range(0, len(text), block_size):
        block = text[i:i + block_size]
        number = 0
        for char in block:
            if char == '_':
                digit = 0
            elif 'A' <= char <= 'Z':
                digit = ord(char) - ord('A') + 1
            else:
                raise ValueError(f"Invalid character: {char}")
            number = number * 27 + digit
        blocks.append(number)
    
    return blocks


def number_to_text(numbers, block_size):
    """
    Convert numbers back to text using 27-letter alphabet
    """
    alphabet = '_ABCDEFGHIJKLMNOPQRSTUVWXYZ'
    text = ""
    
    for number in numbers:
        block = ""
        temp = number
        for _ in range(block_size):
            digit = temp % 27
            block = alphabet[digit] + block
            temp //= 27
        text += block
    
    return text


def calculate_block_size(n):
    """
    Calculate the maximum block size k such that 27^k < n
    This ensures that encrypted blocks don't exceed the modulus
    """
    k = 1
    while 27 ** (k + 1) < n:
        k += 1
    return k


def verify_block_size(k, n):
    """
    Verify that the given block size k is valid for modulus n
    Must satisfy: 27^k < n
    """
    if 27 ** k >= n:
        raise ValueError(f"Block size k={k} is too large for n={n}. Must satisfy 27^k < n.")
    return True


def encrypt(message, n, e, block_size):
    """
    Encrypt a message using RSA
    Returns list of encrypted numbers
    """
    # Pad message to be multiple of block_size
    while len(message) % block_size != 0:
        message += '_'
    
    # Convert to numbers
    message_blocks = text_to_number(message, block_size)
    
    # Encrypt each block: ciphertext = message^e (mod n)
    encrypted_blocks = []
    for block in message_blocks:
        encrypted = pow(block, e, n)
        encrypted_blocks.append(encrypted)
    
    return encrypted_blocks


def decrypt(ciphertext_blocks, n, d, block_size):
    """
    Decrypt ciphertext using RSA
    Returns decrypted message
    """
    # Decrypt each block: message = ciphertext^d (mod n)
    decrypted_blocks = []
    for block in ciphertext_blocks:
        decrypted = pow(block, d, n)
        decrypted_blocks.append(decrypted)
    
    # Convert back to text
    message = number_to_text(decrypted_blocks, block_size)
    
    return message


def generate_keys(n, p, q):
    """
    Generate RSA public and private keys
    
    Returns:
        (e, d, phi, block_size) where:
        - e is the public exponent
        - d is the private exponent
        - phi is φ(n)
        - block_size is the maximum block size
    """
    # Verify that n = p * q
    if p * q != n:
        raise ValueError(f"Invalid primes: p * q = {p * q} ≠ {n}")
    
    # Calculate φ(n) based on whether n is even or odd
    phi = calculate_phi(n, p, q)
    
    # Find smallest valid odd prime e
    e = find_smallest_valid_e(phi)
    
    # Calculate private key d using Extended Euclidean Algorithm
    d = mod_inverse(e, phi)
    
    # Calculate block size
    block_size = calculate_block_size(n)
    
    return e, d, phi, block_size


def format_ciphertext_blocks(encrypted_blocks, l):
    """
    Format encrypted blocks with specified length l
    Pads numbers with leading zeros to length l
    """
    formatted = []
    for block in encrypted_blocks:
        # Convert to string and pad with zeros
        block_str = str(block).zfill(l)
        formatted.append(block_str)
    return formatted


def main():
    """
    Main function demonstrating RSA encryption/decryption
    """
    print("=" * 70)
    print("RSA Encryption/Decryption Implementation")
    print("=" * 70)
    
    # Choose mode
    print("\nSelect mode:")
    print("1. Encrypt")
    print("2. Decrypt")
    mode_choice = input("Enter choice (1 or 2): ").strip()
    
    if mode_choice not in ['1', '2']:
        print("Invalid choice! Please enter 1 or 2.")
        return
    
    is_encrypt = (mode_choice == '1')
    mode_name = "Encryption" if is_encrypt else "Decryption"
    
    print(f"\n{mode_name} Mode Selected")
    
    # Get input from user
    print("\nEnter the RSA parameters:")
    p = int(input("Enter p (first prime): "))
    q = int(input("Enter q (second prime): "))
    
    # Compute n automatically
    n = p * q
    print(f"\nComputed n = p × q = {p} × {q} = {n}")
    
    # Get block sizes
    k = int(input("Enter k (plaintext block size): "))
    l = int(input("Enter l (ciphertext block length): "))
    
    print("\n" + "-" * 70)
    print("STEP 1: Key Generation")
    print("-" * 70)
    
    # Verify that p and q are valid
    if not is_prime(p):
        print(f"Error: p = {p} is not prime!")
        return
    if not is_prime(q):
        print(f"Error: q = {q} is not prime!")
        return
    
    # Verify block size is valid
    try:
        verify_block_size(k, n)
    except ValueError as e:
        print(f"Error: {e}")
        return
    
    # Calculate φ(n)
    phi = calculate_phi(n, p, q)
    
    print(f"\nPrime factors: p = {p}, q = {q}")
    print(f"Modulus: n = {n}")
    
    # Display phi calculation based on even/odd case
    if n % 2 == 0:
        print(f"φ(n) = n/2 - 1 = {n//2} - 1 = {phi} (Case 1: n even)")
    else:
        print(f"φ(n) = (p-1)(q-1) = {p-1} × {q-1} = {phi} (Case 2: n odd)")
    
    # Generate keys - e is always computed as smallest valid odd prime
    try:
        # Compute e automatically (same for both encryption and decryption)
        e = find_smallest_valid_e(phi)
        print(f"Public exponent (e): {e} (smallest valid odd prime)")
        
        # Compute d from e
        d = mod_inverse(e, phi)
        print(f"Private exponent (d): {d} (computed from e)")
        
        print(f"Plaintext block size (k): {k}")
        print(f"Ciphertext block length (l): {l}")
        print(f"\nPublic Key: ({n}, {e})")
        print(f"Private Key: ({n}, {d})")
        
        # Verify 27^k < n
        print(f"\nVerification: 27^{k} = {27**k} < {n} ✓")
        
    except ValueError as e:
        print(f"Error: {e}")
        return
    
    if is_encrypt:
        # ===== ENCRYPTION MODE =====
        # Get message to encrypt
        print("\n" + "-" * 70)
        print("STEP 2: Text to Numbers Conversion")
        print("-" * 70)
        
        message = input("\nEnter plaintext to encrypt: ")
        
        # Show alphabet mapping
        print(f"\nAlphabet: '_'=0, 'A'=1, 'B'=2, ..., 'Z'=26")
        print(f"Original message: '{message}'")
        print(f"Normalized: '{message.upper().replace(' ', '_')}'")
        
        # Pad message if needed
        padded_message = message
        while len(padded_message) % k != 0:
            padded_message += '_'
        
        if len(padded_message) != len(message):
            print(f"Padded message: '{padded_message.upper().replace(' ', '_')}' (padded to multiple of {k})")
        
        # Convert to numbers
        message_blocks = text_to_number(padded_message, k)
        print(f"\nPlaintext blocks (as numbers): {message_blocks}")
        
        # Show detailed conversion for first block
        if message_blocks:
            first_block_text = padded_message[:k].upper().replace(' ', '_')
            print(f"\nFirst block '{first_block_text}' conversion:")
            result = 0
            for i, char in enumerate(first_block_text):
                if char == '_':
                    digit = 0
                else:
                    digit = ord(char) - ord('A') + 1
                result = result * 27 + digit
                print(f"  Step {i+1}: {result // 27 if i > 0 else 0} × 27 + {digit} = {result}")
        
        # Encrypt
        print("\n" + "-" * 70)
        print("STEP 3: Encryption")
        print("-" * 70)
        
        encrypted = encrypt(message, n, e, k)
        
        print(f"\nFormula: Ciphertext = (Plaintext)^e (mod n)")
        print(f"\nEncrypted blocks:")
        for i, (plain, cipher) in enumerate(zip(message_blocks, encrypted)):
            print(f"  Block {i+1}: {plain}^{e} ≡ {cipher} (mod {n})")
        
        # Format with length l
        formatted_cipher = format_ciphertext_blocks(encrypted, l)
        print(f"\nFormatted ciphertext (length {l}): {' '.join(formatted_cipher)}")
        
        # Convert ciphertext to text representation using block length l
        ciphertext_as_text = number_to_text(encrypted, l)
        print(f"\nCiphertext as text: '{ciphertext_as_text}'")
        print(f"\nCiphertext representation:")
        print(f"  Numerical: {encrypted}")
        print(f"  Text form: '{ciphertext_as_text}'")
        
    else:
        # ===== DECRYPTION MODE =====
        # Get ciphertext to decrypt
        print("\n" + "-" * 70)
        print("STEP 2: Ciphertext Input")
        print("-" * 70)
        
        ciphertext_input = input("\nEnter ciphertext to decrypt: ")
        
        # Show alphabet mapping
        print(f"\nAlphabet: '_'=0, 'A'=1, 'B'=2, ..., 'Z'=26")
        print(f"Ciphertext: '{ciphertext_input}'")
        print(f"Normalized: '{ciphertext_input.upper().replace(' ', '_')}'")
        
        # Pad ciphertext if needed to multiple of l
        padded_ciphertext = ciphertext_input
        while len(padded_ciphertext) % l != 0:
            padded_ciphertext += '_'
        
        if len(padded_ciphertext) != len(ciphertext_input):
            print(f"Padded ciphertext: '{padded_ciphertext.upper().replace(' ', '_')}' (padded to multiple of {l})")
        
        # Convert ciphertext to numbers using block length l
        ciphertext_blocks = text_to_number(padded_ciphertext, l)
        print(f"\nCiphertext blocks (as numbers): {ciphertext_blocks}")
        
        # Show detailed conversion for first block
        if ciphertext_blocks:
            first_block_text = padded_ciphertext[:l].upper().replace(' ', '_')
            print(f"\nFirst block '{first_block_text}' conversion:")
            result = 0
            for i, char in enumerate(first_block_text):
                if char == '_':
                    digit = 0
                else:
                    digit = ord(char) - ord('A') + 1
                result = result * 27 + digit
                print(f"  Step {i+1}: {result // 27 if i > 0 else 0} × 27 + {digit} = {result}")
        
        # Decrypt
        print("\n" + "-" * 70)
        print("STEP 3: Decryption")
        print("-" * 70)
        
        decrypted = decrypt(ciphertext_blocks, n, d, k)
        
        print(f"\nFormula: Plaintext = (Ciphertext)^d (mod n)")
        print(f"\nDecrypted blocks:")
        decrypted_blocks = []
        for i, cipher in enumerate(ciphertext_blocks):
            plain = pow(cipher, d, n)
            decrypted_blocks.append(plain)
            print(f"  Block {i+1}: {cipher}^{d} ≡ {plain} (mod {n})")
        
        print(f"\nPlaintext representation:")
        print(f"  Numerical: {decrypted_blocks}")
        print(f"  Text form: '{decrypted.strip('_')}'")
        
        print(f"\n✓ Decryption complete!")
    
    print("\n" + "=" * 70)


if __name__ == "__main__":
    main()
