"""
@Author: Cretu Cristian, 913
"""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List


"""
We can take f(x) = x^2 - cos(x)
f'(x) = 2x + sin(x)
f''(x) = 2 + cos(x) > 0 for all x from R => f is convex


Since f'(x) = 2x + sin(x) => f'(x) = 0 <=> 2x + sin(x) = 0
We see that x = 0 is a solution, and by doing the derivative of f'(x) we see that it is the only solution
So x = 0 is the global minimum of f(x)
and f(0) = 0^2 - cos(0) = -1
"""

def f(x):
    # Our initial function
    return x**2 - np.cos(x)


def df(x):
    # Derivative of f(x)
    return 2 * x + np.sin(x)

"""
As a concave function, we can take g(x) = -4x^2 - 8x
g'(x) = -8x - 8
g'(x) = 0 <=> -8x - 8 = 0 <=> x = -1
g''(x) = -8 < 0 for all x from R => g is concave
"""

def g(x):
    return -4*x**2 - 8*x

def dg(x):
    return -8*x - 8


def gradient_descent(x: float, learning_rate: float, iterations: int, function: str) -> Tuple[List[float], List[float]]:
    """
    Returns the x and y values of the gradient descent algorithm

    :param x: initial x value
    :param learning_rate: learning rate
    :param iterations: number of iterations

    :return: x_values, y_values
    """
    x_values = [x]
    y_values = [f(x) if function == 'f' else g(x)]

    for i in range(iterations):
        x = x - learning_rate * (df(x) if function == 'f' else dg(x))
        x_values.append(x)
        y_values.append(f(x) if function == 'f' else g(x))

    return x_values, y_values


def plot_gradient_descent(initial_x: float, learning_rate: float, iterations: int, function: str = 'f', convergence: bool = True):
    x_values, y_values = gradient_descent(initial_x, learning_rate, iterations, function)

    x = np.linspace(-initial_x, initial_x, 400)
    plt.plot(x, f(x), '-r', label='f(x) = x^2 - cos(x)' if function == 'f' else 'g(x) = -4x^2 - 8x')
    plt.scatter(x_values, y_values, c='blue', s=50, label='Gradient Descent Steps')

    print(x)

    title_str = f"{'Convergence' if convergence else 'Divergence'} for η = {learning_rate}"
    plt.title(title_str)
    plt.xlabel('x')
    plt.ylabel('f(x)')
    plt.legend(loc='upper right')
    plt.savefig(f"plot_for_η={learning_rate}.png")
    plt.show()


if __name__ == "__main__":
    INITIAL_X = 0.5
    LEARNING_RATE = 0.1
    ITERATIONS = 100

    # Subpoint a)
    plot_gradient_descent(INITIAL_X, LEARNING_RATE, ITERATIONS)
    # Subpoint b) - we increase the learning rate
    plot_gradient_descent(INITIAL_X, 0.3, ITERATIONS)
    # Subpoint c) - learning rate too large
    plot_gradient_descent(INITIAL_X, 1.5, ITERATIONS, convergence=False)
    # Subpoint d) - concave function
    plot_gradient_descent(-10, 0.05, ITERATIONS, 'g', False)
