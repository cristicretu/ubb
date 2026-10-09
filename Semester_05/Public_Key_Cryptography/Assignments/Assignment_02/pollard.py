import math
from tabulate import tabulate


# Predefined polynomial functions for Pollard's ρ algorithm
def f1(x):
    """f(x) = x^2 + 1"""
    return x**2 + 1


def f2(x):
    """f(x) = x^2 + 3"""
    return x**2 + 3


def f3(x):
    """f(x) = x^2 - 1"""
    return x**2 - 1


def f4(x):
    """f(x) = x^2 + 2"""
    return x**2 + 2


def f5(x):
    """f(x) = x^2 + x + 1"""
    return x**2 + x + 1


# Dictionary mapping function names to functions
POLYNOMIAL_FUNCTIONS = {
    '1': (f1, "f(x) = x² + 1"),
    '2': (f2, "f(x) = x² + 3"),
    '3': (f3, "f(x) = x² - 1"),
    '4': (f4, "f(x) = x² + 2"),
    '5': (f5, "f(x) = x² + x + 1"),
}


def pollard_rho(n, f=None):
    """
    Pollard's ρ algorithm for integer factorization.
    
    Parameters:
    -----------
    n : int
        An odd composite number to factor
    f : callable, optional
        A suitable random polynomial map. Default is f(x) = x^2 + 1
        
    Returns:
    --------
    int
        A non-trivial factor d of n (where 1 < d < n)
        
    Raises:
    -------
    ValueError
        If n is not odd
    """
    # Check that n is odd
    if n % 2 == 0:
        raise ValueError(f"n must be odd, but got n = {n}")
    
    # Default polynomial map: f(x) = x^2 + 1
    if f is None:
        f = lambda x: x**2 + 1
    
    # Initialize x0 = 2
    x_curr = 2
    
    # Store all x values for comparison
    x_values = [x_curr]  # x_0 = 2
    
    # Store table rows for pretty printing
    table_data = []
    
    print(f"\nIterations (results mod {n}):\n")
    
    i = 0
    while True:
        # Compute x_{i+1} and x_{i+2}
        x_odd = f(x_curr) % n
        x_values.append(x_odd)
        i += 1
        
        x_even = f(x_odd) % n
        x_values.append(x_even)
        i += 1
        
        # For Floyd's algorithm, we compare x_j with x_{2j}
        # When we're at position i (even), we check gcd(|x_i - x_{i/2}|, n)
        j = i // 2
        
        # Compute d = gcd(|x_i - x_j|, n) where i = 2j
        d = math.gcd(abs(x_even - x_values[j]), n)
        
        # Add row to table
        table_data.append([f"x_{i-1} = {x_odd}", f"x_{i} = {x_even}", f"(|x_{i} - x_{j}|, n) = {d}"])
        
        # If 1 < d < n, then STOP and d is a non-trivial factor of n
        if 1 < d < n:
            # Print the table
            print(tabulate(table_data, tablefmt="simple_grid"))
            print(f"\n✓ Found non-trivial factor: d = {d}")
            return d
        
        # If d == n, the algorithm failed (rare case)
        if d == n:
            print(tabulate(table_data, tablefmt="simple_grid"))
            raise RuntimeError(f"Algorithm failed: gcd = n. Try a different polynomial or starting value.")
        
        # Update current x for next iteration
        x_curr = x_even
        
        # Optional: Add a maximum iteration limit to prevent infinite loops
        if i > 10**6:
            print(tabulate(table_data, tablefmt="simple_grid"))
            raise RuntimeError(f"Algorithm did not converge after {i} iterations")


def main():
    """
    Interactive usage of Pollard's ρ algorithm
    """
    print("=" * 60)
    print("Pollard's ρ Algorithm - Integer Factorization")
    print("=" * 60)
    print()
    
    # Get input n from user
    while True:
        try:
            n = int(input("Enter an odd composite number n: "))
            if n % 2 == 0:
                print("Error: n must be odd. Please try again.")
                continue
            if n <= 1:
                print("Error: n must be greater than 1. Please try again.")
                continue
            break
        except ValueError:
            print("Error: Please enter a valid integer.")
    
    print()
    
    # Display available polynomial functions
    print("Available polynomial functions:")
    for key, (func, description) in POLYNOMIAL_FUNCTIONS.items():
        print(f"  {key}. {description}")
    print()
    
    # Get function choice from user
    while True:
        choice = input("Choose a polynomial function (1-5, or press Enter for default): ").strip()
        
        if choice == "":
            # Use default function
            selected_func = None
            print("Using default: f(x) = x² + 1")
            break
        elif choice in POLYNOMIAL_FUNCTIONS:
            selected_func, description = POLYNOMIAL_FUNCTIONS[choice]
            print(f"Using: {description}")
            break
        else:
            print(f"Error: Invalid choice. Please enter a number between 1 and 5.")
    
    print()
    print("-" * 60)
    
    # Run the algorithm
    try:
        factor = pollard_rho(n, selected_func)
        other_factor = n // factor
        
        print(f"\nThe other factor is: {other_factor}")
        print(f"\nVerification: {n} = {factor} × {other_factor}")
        
        # Verify the factorization
        if factor * other_factor == n:
            print("✓ Factorization verified!")
        else:
            print("✗ Verification failed!")
            
    except ValueError as e:
        print(f"✗ Error: {e}")
    except RuntimeError as e:
        print(f"✗ Algorithm failed: {e}")
        print("Tip: Try using a different polynomial function.")
    
    print()
    print("=" * 60)


if __name__ == "__main__":
    main()



