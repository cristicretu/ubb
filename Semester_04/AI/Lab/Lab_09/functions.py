"""
Benchmark optimization functions for GA testing.
"""
import numpy as np

def f1(x, y):
    """
    Ackley function in 2D form.
    
    Parameters:
    -----------
    x : numpy.ndarray
        x coordinates
    y : numpy.ndarray
        y coordinates
        
    Returns:
    --------
    numpy.ndarray
        Function values at (x,y) points
    """
    return -20 * np.exp(-0.2 * np.sqrt(0.5 * (x**2 + y**2))) - np.exp(0.5 * (np.cos(2 * np.pi * x) + np.cos(2 * np.pi * y))) + np.e + 20

def ackley(x):
    """
    Ackley function for n-dimensional input.
    
    Parameters:
    -----------
    x : numpy.ndarray
        Input vector
        
    Returns:
    --------
    float
        Function value at x
    """
    a = 20
    b = 0.2
    c = 2 * np.pi
    d = len(x)
    sum1 = np.sum(x**2)
    sum2 = np.sum(np.cos(c * x))
    return -a * np.exp(-b * np.sqrt(sum1 / d)) - np.exp(sum2 / d) + a + np.exp(1)

def f2(x, y):
    """
    Bukin function in 2D form.
    
    Parameters:
    -----------
    x : numpy.ndarray
        x coordinates
    y : numpy.ndarray
        y coordinates
        
    Returns:
    --------
    numpy.ndarray
        Function values at (x,y) points
    """
    return 100 * np.sqrt(np.abs(y - 0.01 * x**2) + 0.01 * (x + 10)**2) + 0.01 * np.abs(y + 10)

def bukin(x):
    """
    Bukin function for 2-dimensional input.
    
    Parameters:
    -----------
    x : numpy.ndarray
        Input vector (2D)
        
    Returns:
    --------
    float
        Function value at x
    """
    return 100 * np.sqrt(np.abs(x[1] - 0.01 * x[0]**2) + 0.01 * (x[0] + 10)**2) + 0.01 * np.abs(x[1] + 10) 