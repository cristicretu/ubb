import math
from tabulate import tabulate


def is_perfect_square(n):
    """
    Check if a number is a perfect square.
    
    Parameters:
    -----------
    n : int
        The number to check
        
    Returns:
    --------
    tuple (bool, int)
        (True, sqrt) if n is a perfect square, (False, 0) otherwise
    """
    if n < 0:
        return False, 0
    
    sqrt_n = int(math.isqrt(n))
    if sqrt_n * sqrt_n == n:
        return True, sqrt_n
    return False, 0


def fermat_factorization(n, B=None):
    """
    Fermat's factorization algorithm.
    
    Parameters:
    -----------
    n : int
        An odd composite number (which is not a perfect square)
    B : int, optional
        A suitable bound for the number of iterations. 
        If None, defaults to n (effectively unlimited for practical purposes)
        
    Returns:
    --------
    tuple (int, int)
        A pair of non-trivial factors (a, b) such that n = a × b
        
    Raises:
    -------
    ValueError
        If n is not odd or if n is a perfect square
    RuntimeError
        If the algorithm fails to find factors within the bound B
    """
    # Check that n is odd
    if n % 2 == 0:
        raise ValueError(f"n must be odd, but got n = {n}")
    
    # Check that n is not a perfect square
    is_square, sqrt_n = is_perfect_square(n)
    if is_square:
        raise ValueError(f"n must not be a perfect square, but n = {sqrt_n}²")
    
    # Set default bound if not provided
    if B is None:
        B = n  # Effectively unlimited for practical purposes
    
    # Let t₀ = ⌈√n⌉
    t0 = math.floor(math.sqrt(n))
    
    print(f"\nFermat's Algorithm:")
    print(f"t₀ = ⌈√{n}⌉ = {t0}")
    print(f"Bound B = {B}")
    print(f"\nIterations:\n")
    
    # Store table rows for pretty printing
    table_data = []
    
    # For t = t₀, t₀ + 1, ..., t₀ + B do
    for i in range(1, B + 1):
        t = t0 + i
        t_squared = t * t
        diff = t_squared - n
        
        # Check if t² - n is a perfect square
        is_square, s = is_perfect_square(diff)
        
        if is_square:
            # Add row to table showing success
            table_data.append([
                f"t = {t}",
                f"t² = {t_squared}",
                f"t² - n = {diff}",
                f"s = {s} ✓"
            ])
            
            # Print the table
            print(tabulate(table_data, 
                          headers=["t", "t²", "t² - n", "Is square?"],
                          tablefmt="simple_grid"))
            
            # Calculate factors: n = (t - s)(t + s)
            a = t - s
            b = t + s
            
            print(f"\n✓ Found that t² - n = s² where s = {s}")
            print(f"✓ Therefore: n = (t - s)(t + s) = ({t} - {s})({t} + {s}) = {a} × {b}")
            
            return a, b
        else:
            # Add row to table showing it's not a square
            table_data.append([
                f"t = {t}",
                f"t² = {t_squared}",
                f"t² - n = {diff}",
                "Not a square"
            ])
    
    # If we've exhausted the bound without finding factors
    print(tabulate(table_data, 
                  headers=["t", "t²", "t² - n", "Is square?"],
                  tablefmt="simple_grid"))
    raise RuntimeError(f"Algorithm failed: no factors found within bound B = {B}. Try increasing B.")


def main():
    """
    Interactive usage of Fermat's factorization algorithm
    """
    print("=" * 60)
    print("Fermat's Factorization Algorithm")
    print("=" * 60)
    print()
    
    # Get input n from user
    while True:
        try:
            n = int(input("Enter an odd composite number n (not a perfect square): "))
            if n % 2 == 0:
                print("Error: n must be odd. Please try again.")
                continue
            if n <= 1:
                print("Error: n must be greater than 1. Please try again.")
                continue
            
            # Check if n is a perfect square
            is_square, sqrt_n = is_perfect_square(n)
            if is_square:
                print(f"Error: n is a perfect square ({sqrt_n}²). Please try again.")
                continue
            
            break
        except ValueError:
            print("Error: Please enter a valid integer.")
    
    print()
    
    # Get bound B from user (optional)
    while True:
        bound_input = input(f"Enter bound B (press Enter for default = {n}): ").strip()
        
        if bound_input == "":
            B = None
            print(f"Using default bound B = {n}")
            break
        
        try:
            B = int(bound_input)
            if B <= 0:
                print("Error: B must be positive. Please try again.")
                continue
            print(f"Using bound B = {B}")
            break
        except ValueError:
            print("Error: Please enter a valid integer.")
    
    print()
    print("-" * 60)
    
    # Run the algorithm
    try:
        a, b = fermat_factorization(n, B)
        
        # Verify the factorization
        if a * b == n:
            print("\n✓ Factorization verified!")
        else:
            print("\n✗ Verification failed!")
            
    except ValueError as e:
        print(f"✗ Error: {e}")
    except RuntimeError as e:
        print(f"✗ Algorithm failed: {e}")
    
    print()
    print("=" * 60)


if __name__ == "__main__":
    main()

