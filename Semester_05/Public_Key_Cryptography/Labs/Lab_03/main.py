import math

upper_limit = 1000000

def is_perfect_square(n):
    if n < 0:
        return -1
    rad = math.isqrt(n)
    return rad if rad * rad == n else -1

def ferm(n):
    if n <= 1:
        return None
    if n % 2 == 0:
        return (2, n // 2)

    k = 1
    while True:
        kn = k * n
        x = math.isqrt(kn) + 1
        lim_sup = x + upper_limit 
        
        while x < lim_sup:
            y_squared = x * x - kn
            y = is_perfect_square(y_squared)
            
            if y != -1:
                factor = math.gcd(x - y, n)
                if 1 < factor < n:
                    return (factor, n // factor)
                    
                factor = math.gcd(x + y, n)
                if 1 < factor < n:
                    return (factor, n // factor)
            x += 1
            
        k += 1

if __name__ == '__main__':
    num = 2132423
    result = ferm(num)
    if result:
        f1, f2 = result
        print(f"{num} = {f1} * {f2}")
    else:
        print(f"nu are")
