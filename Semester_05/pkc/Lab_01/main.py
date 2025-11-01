import time
from prettytable import PrettyTable 


def rec_gcd(a, b):
    if b == 0:
        return a
    return rec_gcd(b, a % b)

# https://www.pbinfo.ro/articole/18942/algoritmul-lui-euclid-extins-invers-modular
def ext_gcd(a, b):
    r1, r2 = a, b
    s1, s2 = 1, 0 
    t1, t2 = 0, 1

    while r2 != 0:
        q = r1 // r2
        r1 = r2
        r2 = r1 - q * r2
        s1 = s2
        s2 = s1 - q * s2
        t1 = t2
        t2 = t1 - q * t2

    return r1, s1, t1

def euclid(a, b):
  if b == 0:
    return a
  if a  == 0: 
    return b

  if a == b:
    return a
  
  if a > b:
    return euclid(a - b, b)
  else:
    return euclid(a, b - a)

def measure_time(func, a, b):
    start = time.perf_counter()
    func(a, b)
    end = time.perf_counter()
    return (end - start) * 1_000_000 

def run_analysis():
    test_cases = [
        (0, 15),
        (100, 150),
        (1000, 1500),
        (10000, 15000),
        (100000, 150000),
        (987654321, 123456789),
        (2**20, 2**19),
        (48, 18),
        (1071, 462),
        (17711, 10946),
        (1000000007, 1000000009),
        (123456789012345, 987654321098765),
        (2**200 + 1, 2**100),
    ]
    
    table = PrettyTable(["a", "b", "recursive", "extended", "euclid"]) 
    
    for a, b in test_cases:
        try:
            rec_time = measure_time(rec_gcd, a, b)
        except RecursionError:
            rec_time = 'RecursionError'
        
        try:
            extended_time = measure_time(ext_gcd, a, b)
        except RecursionError:
            extended_time = 'RecursionError'
        
        try:
            euclid_time = measure_time(euclid, a, b)
        except RecursionError:
            euclid_time = 'RecursionError'
        
        table.add_row([str(a)[:30], str(b)[:30], str(rec_time)[:20], str(extended_time)[:20], str(euclid_time)[:20]])
    
    print(table)
    
   

run_analysis()