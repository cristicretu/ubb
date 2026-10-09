"""
Genetic algorithm operators for real-valued and binary encodings.
"""
# https://stackoverflow.com/questions/38952879/blx-alpha-crossover-what-approach-is-the-right-one
# https://www.linkedin.com/pulse/crossovers-genetic-algorithms-ali-karazmoodeh-tthjf/
import numpy as np

# Real-valued GA operators
def child(p1, p2, alpha=0.5):
    """
    Arithmetic crossover for real-valued encoding.
    
    Parameters:
    -----------
    p1 : numpy.ndarray
        First parent
    p2 : numpy.ndarray
        Second parent
    alpha : float, optional
        Weighting factor, by default 0.5
        
    Returns:
    --------
    numpy.ndarray
        Child individual
    """
    return alpha * p1 + (1 - alpha) * p2

def blx_alpha_crossover(p1, p2, alpha, lower_bound, upper_bound):
    """
    BLX-alpha crossover for real-valued encoding.
    
    Parameters:
    -----------
    p1 : numpy.ndarray
        First parent
    p2 : numpy.ndarray
        Second parent
    alpha : float
        Expansion factor
    lower_bound : numpy.ndarray
        Lower bounds for each dimension
    upper_bound : numpy.ndarray
        Upper bounds for each dimension
        
    Returns:
    --------
    numpy.ndarray
        Child individual
    """
    child = np.zeros_like(p1)
    for i in range(len(p1)):
        d_i = np.abs(p1[i] - p2[i])
        min_i = min(p1[i], p2[i]) - alpha * d_i
        max_i = max(p1[i], p2[i]) + alpha * d_i
        
        # Generate offspring gene and clamp it
        child[i] = np.random.uniform(min_i, max_i)
        child[i] = np.clip(child[i], lower_bound[i], upper_bound[i])
    return child

def gaussian_mutation(individual, mutation_rate=0.1, sigma=0.1):
    """
    Gaussian mutation for real-valued encoding.
    
    Parameters:
    -----------
    individual : numpy.ndarray
        Individual to mutate
    mutation_rate : float, optional
        Probability of mutation, by default 0.1
    sigma : float, optional
        Standard deviation of the Gaussian noise, by default 0.1
        
    Returns:
    --------
    numpy.ndarray
        Mutated individual
    """
    return individual + np.random.normal(0, sigma, len(individual)) * mutation_rate

# Binary GA operators
def binary_one_point_crossover(parent1, parent2):
    """
    One-point crossover for binary encoding.
    
    Parameters:
    -----------
    parent1 : numpy.ndarray
        First parent (binary string)
    parent2 : numpy.ndarray
        Second parent (binary string)
        
    Returns:
    --------
    tuple
        Two children (binary strings)
    """
    cross_point = np.random.randint(1, len(parent1) - 1)
    child1 = np.concatenate([parent1[:cross_point], parent2[cross_point:]])
    child2 = np.concatenate([parent2[:cross_point], parent1[cross_point:]])
    return child1, child2

def binary_two_point_crossover(parent1, parent2):
    """
    Two-point crossover for binary encoding.
    
    Parameters:
    -----------
    parent1 : numpy.ndarray
        First parent (binary string)
    parent2 : numpy.ndarray
        Second parent (binary string)
        
    Returns:
    --------
    tuple
        Two children (binary strings)
    """
    length = len(parent1)
    # Ensure first point < second point
    point1, point2 = sorted(np.random.choice(range(1, length), 2, replace=False))
    
    child1 = np.concatenate([parent1[:point1], parent2[point1:point2], parent1[point2:]])
    child2 = np.concatenate([parent2[:point1], parent1[point1:point2], parent2[point2:]])
    return child1, child2

def bit_flip_mutation(binary_individual, mutation_rate=0.01):
    """
    Bit-flip mutation for binary encoding.
    
    Parameters:
    -----------
    binary_individual : numpy.ndarray
        Individual to mutate (binary string)
    mutation_rate : float, optional
        Probability of flipping each bit, by default 0.01
        
    Returns:
    --------
    numpy.ndarray
        Mutated individual (binary string)
    """
    mutated = binary_individual.copy()
    for i in range(len(mutated)):
        if np.random.random() < mutation_rate:
            mutated[i] = 1 - mutated[i]  # Flip the bit
    return mutated

def binary_to_real(binary_individual, bit_length_per_var, num_vars, lower_bounds, upper_bounds):
    """
    Convert a binary chromosome to real values within specified bounds.
    
    Parameters:
    -----------
    binary_individual : numpy.ndarray
        Binary chromosome
    bit_length_per_var : int
        Number of bits per variable
    num_vars : int
        Number of variables
    lower_bounds : numpy.ndarray
        Lower bounds for each variable
    upper_bounds : numpy.ndarray
        Upper bounds for each variable
        
    Returns:
    --------
    numpy.ndarray
        Real-valued vector
    """
    real_values = np.zeros(num_vars)
    
    for i in range(num_vars):
        # Extract bits for this variable
        start_bit = i * bit_length_per_var
        end_bit = (i + 1) * bit_length_per_var
        var_bits = binary_individual[start_bit:end_bit]
        
        # Convert to decimal (0 to 2^bit_length - 1)
        decimal_val = 0
        for j, bit in enumerate(var_bits):
            decimal_val += bit * (2 ** (bit_length_per_var - j - 1))
        
        # Map to range [lower_bounds[i], upper_bounds[i]]
        mapped_val = lower_bounds[i] + (decimal_val / (2**bit_length_per_var - 1)) * (upper_bounds[i] - lower_bounds[i])
        real_values[i] = mapped_val
        
    return real_values 