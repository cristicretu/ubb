Avra is a language built specifically for mathematical computations, especially number theory. Think of it as a tool that lets students, teachers, and anyone interested in math write programs to solve mathematical problems—without dealing with all the complexity of traditional programming languages. You get simple I/O, operators made for math, and the control structures you need.

We've kept the language simple on purpose. You have `read` and `say` for getting input and showing output, `let` for variables, `fn` for creating functions (with `return` to send values back), `if`/`else` for making decisions, and `for` loops for repetition. Different than other langs, we have the built-in math operators: `^^` calculates the greatest common divisor, `%%` finds the least common multiple, and `@@` computes power sums.

Avra is great for teaching math concepts in a classroom, quickly prototyping mathematical algorithms, or even embedding into larger systems as a lightweight scripting engine. It's designed such that people without programming experience can pick it up easily, while still giving you enough power to do real computational work. 

### statements

### variable decl (let)
You need variables to store computed values. 

```
program
    let a = 15;
    let b = 25;
    let sum = a + b;
    say sum;
end
```

### read (read)
Read is for getting input from the user.

```
program
    read num;
    let doubled = num * 2;
    say doubled;
end
```

### output (say)
Say is for printing results to the user.

```
program
    let x = 12;
    let y = 8;
    say x ^^ y;
end
```

This prints out the GCD of 12 and 8, which is 4. 

### if/else
If/else is for checking conditions. 

```
program
    read n;
    if (n % 2 == 0) {
        say "even";
    } else {
        say "odd";
    }
end
```

Or checking if a number is prime:

```
program
    if (n > 1) {
        say "could be prime";
    } else {
        say "not prime";
    }
end
```

### fn/return 
Functions let you reuse logic.

```
program
    fn gcd(a, b) {
        return a ^^ b;
    }

    let result = gcd(48, 18);
    say result;
end
```

### gcd operator (^^)
Finding the greatest common divisor is fundamental in number theory. 

```
program
    let a = 48;
    let b = 18;
    say a ^^ b;
end
```

### lcm operator (%%)
The least common multiple shows up when you're working with fractions or periodic patterns. 

```
program
    let x = 12;
    let y = 15;
    say x %% y;
end
```

### power sum operator (@@)
The `@@` operator computes power sums (a raised to b).

```
program
    let base = 2;
    let exp = 10;
    say base @@ exp;  # Computes 2^10 = 1024
end
```

### factorial operator (!)

```
program
    let n = 5;
    say n!;  # Computes 5! = 120
end
```


```
program
    fn combination(n, r) {
        let numerator = n!;
        let denominator = r! * (n - r)!;
        return numerator / denominator;
    }
    
    say combination(5, 2);  # Computes 5C2 = 10
end
```


### vectors and tensors
Creating vectors:

```
program
    let v = [1, 2, 3, 4, 5];
    let empty = [];
    let features = [0.5, 0.3, 0.8, 0.1];
    say v;
end
```

### floor division operator (//)


```
program
    read n;
    read d;
    if (n // d * d == n) {
        say "divides evenly";
    } else {
        say "has remainder";
    }
end
```

### sigmoid activation operator (~)
```
program
    let x = 0;
    say x~;  # Computes sigmoid(0) = 0.5
    
    let large = 10;
    say large~;  # Computes sigmoid(10) ≈ 0.9999
    
    let negative = -5;
    say negative~;  # Computes sigmoid(-5) ≈ 0.0067
end
```


```

### relu activation operator (|>)

```
program
    let positive = 5;
    say positive|>;  # Computes max(0, 5) = 5
    
    let negative = -3;
    say negative|>;  # Computes max(0, -3) = 0
end
```

Using ReLU in a neural network layer:

```
program
    fn neuron(x, weight, bias) {
        let z = x * weight + bias;
        return z|>;  # Apply ReLU activation
    }
    
    read input;
    let activation = neuron(input, 2, -1);
    say activation;
end
```



### dot product  (<.>)

```
program
    let v1 = [1, 2, 3];
    let v2 = [4, 5, 6];
    let similarity = v1 <.> v2;
    say similarity;  # Computes 1*4 + 2*5 + 3*6 = 32
end
```





```
program
    let numbers = [10, 20, 30, 40];
    say numbers[0];  # Prints 10
    say numbers[2];  # Prints 30
    
    let last_idx = 3;
    say numbers[last_idx];  # Prints 40
end
```


```
program
    let a = [1, 2, 3];
    let b = [4, 5, 6];
    
    # Dot product
    let dot = a <.> b;
    say "Dot product:", dot;
    
    let sum = [0, 0, 0];
    for (i = 0; i < 3; i = i + 1) {
        let sum[i] = a[i] + b[i];
    }
    say "Element-wise sum:", sum;
end
```




```
program
    let matrix = [[1, 2, 3], [4, 5, 6]];
    
    say "Row 0:", matrix[0];
    say "Element [1][2]:", matrix[1][2];  # Prints 6
    
    # Matrix-vector multiplication
    fn matvec(mat, vec) {
        let row1 = mat[0] <.> vec;
        let row2 = mat[1] <.> vec;
        return [row1, row2];
    }
    
    let v = [1, 1, 1];
    let result = matvec(matrix, v);
    say result;  # Prints [6, 15]
end
```

### for loop

```
program
    read n;
    let fact = 1;
    for (i = 1; i <= n; i = i + 1) {
        let fact = fact * i;
    }
    say fact;
end
```

or

```
program
    read nl;
    say nl!;
end
```


### ex: simplyfing fractions

```
program
    fn simplify(num, den) {
        let divisor = num ^^ den;
        let newNum = num / divisor;
        let newDen = den / divisor;
        return newNum;
    }

    read numerator;
    read denominator;

    let simplified = simplify(numerator, denominator);
    say simplified;
end
```