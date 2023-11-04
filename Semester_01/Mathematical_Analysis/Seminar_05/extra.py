"""
Extra Homework - Continuous Everywhere, Differentiable Nowhere
@Author: Cretu Cristian, 913
"""
import numpy as np
import matplotlib.pyplot as plt

# Blancmange curve
# https://en.wikipedia.org/wiki/Blancmange_curve

def triangle_wave(x, n):
    """
    Generates a triangle wave that repeats every 2^n units.
    The expression (x * 2**n % 1) creates a sawtooth wave that is then mirrored to create a triangle wave.
    """
    return 2 * abs(2 * (x * 2**n % 1) - 1)


def blancmange_curve(x, iterations = 100):
    """
    Calculates the Blancmange curve for a given x using a specified number of iterations.
    The sum iterates through a series of triangle waves, each with half the y and double the x of the previous.
    """
    return sum((1/2**i) * triangle_wave(x, i) for i in range(iterations))


if __name__ == "__main__":
    x_values = np.linspace(0, 1, 1000)
    y_values = blancmange_curve(x_values)

    plt.plot(x_values, y_values, '-r', label='Blancmange Curve')
    title = 'Blancmange Curve - Continuous Everywhere, Differentiable Nowhere'
    plt.title(title)
    plt.xlabel('x')
    plt.ylabel('Blancmange(x)')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.savefig(f"{title}.png")
    plt.show()
