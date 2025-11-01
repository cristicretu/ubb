def decompose(n):
    """Decompose n-1 into 2^s * t where t is odd"""
    n_minus_1 = n - 1
    s = 0
    t = n_minus_1
    
    while t % 2 == 0:
        t //= 2
        s += 1
    
    return s, t


def power_mod(base, exp, mod):
    """Compute (base^exp) % mod efficiently"""
    result = 1
    base = base % mod
    
    while exp > 0:
        if exp % 2 == 1:
            result = (result * base) % mod
        exp = exp >> 1
        base = (base * base) % mod
    
    return result


def miller_rabin_detailed(n, bases=None):
    """
    Miller-Rabin primality test with detailed output
    """
    if bases is None:
        bases = [2, 3, 5, 7, 11, 13, 17]
    
    print(f"Use the Miller-Rabin test to decide whether the number n = {n} is prime or not.")
    print()
    
    # Decomposition
    s, t = decompose(n)
    t_binary = bin(t)[2:]  # Remove '0b' prefix
    
    print("Solution.")
    print()
    print("Decomposition:")
    print(f"s = {s}")
    print(f"t = {t}")
    print(f"t in binary = {t_binary}")
    print()
    
    is_prime = True
    witness_found = False
    
    for k, a in enumerate(bases, 1):
        # Skip bases that are >= n (not valid for testing)
        if a >= n:
            continue
            
        print(f"Iteration k = {k} for a = {a} (results mod n):")
        print()
        
        # If we already found a witness, compute first value then show x's
        if witness_found:
            # Still compute at least the first value
            first_val = power_mod(a, t, n)
            
            # Show a^(2^i) for small i first
            for i in range(0, min(10, 5)):
                val = power_mod(a, 2**i, n)
                print(f"{a}^(2^{i}) = {val}")
            print()
            
            print(f"{a}^t = {first_val}")
            for i in range(1, 5):  # Show a few iterations
                # Compute one more value, then x's
                if i == 1:
                    val = power_mod(first_val, 2, n)
                    print(f"{a}^(2t) = {val}")
                else:
                    print(f"{a}^(2^{i}t) = x")
            print()
            continue
        
        # First compute a^t mod n
        first_val = power_mod(a, t, n)
        
        # Compute the full sequence
        values = []
        current = first_val
        values.append(current)
        
        found_n_minus_1 = (first_val == n - 1)
        
        for i in range(1, s + 1):
            current = power_mod(current, 2, n)
            values.append(current)
            if current == n - 1:
                found_n_minus_1 = True
                break
        
        # Display the values based on format in the image
        # Show a^(2^i) for small i first (compute up to i=7, then x's)
        for i in range(0, 8):
            val = power_mod(a, 2**i, n)
            print(f"{a}^(2^{i}) = {val}")
        # Show 2 more as x
        for i in range(8, 10):
            print(f"{a}^(2^{i}) = x")
        print()
        
        # Show a^(2^i * t) mod n
        print(f"{a}^t = {values[0]}")
        
        for i in range(1, len(values)):
            if i == 1:
                print(f"{a}^(2t) = {values[i]}")
            else:
                print(f"{a}^(2^{i}t) = {values[i]}")
        
        # Show remaining positions as x (show up to 4-5 total positions)
        max_positions = max(5, s + 1)  # Show at least 5 positions total
        for i in range(len(values), max_positions):
            if i == 1:
                print(f"{a}^(2t) = x")
            else:
                print(f"{a}^(2^{i}t) = x")
        
        print()
        
        # Check if this is a witness (a^t != 1 and we never reached n-1)
        if first_val != 1 and not found_n_minus_1:
            is_prime = False
            witness_found = True
    
    print("Conclusion:")
    result = "no" if not is_prime else "yes"
    print(f"n is prime (yes/no) = {result}")
    print()
    
    return is_prime


def main():
    print("Miller-Rabin Primality Test")
    print("=" * 50)
    print()
    
    try:
        n = int(input("Enter a number to test: "))
        
        if n < 2:
            print("Please enter a number >= 2")
            return
        
        if n == 2:
            print("2 is prime")
            return
        
        if n % 2 == 0:
            print(f"{n} is even and greater than 2, so it's composite")
            return
        
        print()
        
        # Run Miller-Rabin with bases 2, 3, 5
        miller_rabin_detailed(n, [2, 3, 5])
        
    except ValueError:
        print("Invalid input. Please enter an integer.")
    except KeyboardInterrupt:
        print("\nExiting...")


if __name__ == "__main__":
    main()

