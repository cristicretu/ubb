import numpy as np
def gcd(a, b):
    if a == 0:
      return b, 0, 1
    g, y, x = gcd(b % a, a)
    return g, x - (b // a) * y, y

def mod_inverse(a, m):
    g, x, _ = gcd(a, m)
    if g != 1:
      return None
    return x % m

def get_matrix_inverse(cheie, n):
    k = np.array(cheie)
    det = int(round(np.linalg.det(k)))
    
    determin = mod_inverse(det, n)
    if determin is None:
        raise ValueError(f"teapa")
    
    mat_adjuncta = np.array([
        [k[1, 1], -k[0, 1]],
        [-k[1, 0], k[0, 0]]
    ])
    
    inversa = (determin * mat_adjuncta) % n
    # (det * x) % n = 1
    
    return inversa

def preproces(text, alphabet):
    text = text.upper()
    filtered_chars = []
    for char in text:
        if char in alphabet:
            filtered_chars.append(char)
    text = ''.join(filtered_chars)
    if len(text) % 2 != 0:
        text += ' '
    return text

def hillcipher(text, cheie, alp, m='encrypt'):
    n = len(alp)
    char_to_num = {char: i for i, char in enumerate(alp)}
    num_to_char = {i: char for i, char in enumerate(alp)}
    if m == 'encrypt':
        matrix = np.array(cheie)
    elif m == 'decrypt':
        try:
            matrix = get_matrix_inverse(cheie, n)
        except ValueError as e:
            return f"teapa"
    PT = preproces(text, alp)
    result = []
    for i in range(0, len(PT), 2):
        block = PT[i:i+2]
        nparr = np.array([char_to_num[char] for char in block])
        multiplicat = np.dot(nparr, matrix) % n
        result.extend([num_to_char[num] for num in multiplicat])
    return "".join(result)


if __name__ == "__main__":
    alf = "ABCDEFGHIJKLMNOPQRSTUVWXYZ "
    cheie = [[3, 7], 
           [1, 8]]
    # det = 9 gcd(17,27) = 1

    ptext = "NU CREIC"
    print(f"P: {ptext}")
    
    ctext = hillcipher(ptext, cheie, alf, m='encrypt')
    print(f"C: {ctext}")
    
    
    decrypted_text = hillcipher(ctext, cheie, alf, m='decrypt')
    print(f"D: {decrypted_text}")

    kkkkey = [[2, 4], 
                   [1, 2]] 
    t1 = hillcipher(ptext, kkkkey, alf, m='encrypt')
    print(f"C: {t1}")
    
    t2 = hillcipher(ctext, kkkkey, alf, m='decrypt')
    print(f"D: {t2}")
