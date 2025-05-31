"""
Genetic algorithm implementations for real-valued and binary encodings.
"""
# https://www.linkedin.com/pulse/crossovers-genetic-algorithms-ali-karazmoodeh-tthjf/
import numpy as np
from operators import (
    child, blx_alpha_crossover, gaussian_mutation,
    binary_one_point_crossover, binary_two_point_crossover,
    bit_flip_mutation, binary_to_real
)

def setup_population(pop_size, dim, lower_bound, upper_bound):
    """
    Initialize a real-valued population.
    
    Parameters:
    -----------
    pop_size : int
        Population size
    dim : int
        Dimensionality of individuals
    lower_bound : list or numpy.ndarray
        Lower bounds for each dimension
    upper_bound : list or numpy.ndarray
        Upper bounds for each dimension
        
    Returns:
    --------
    numpy.ndarray
        Initialized population
    """
    return np.random.uniform(lower_bound, upper_bound, (pop_size, dim))

def setup_binary_population(pop_size, bit_length_per_var, num_vars):
    """
    Initialize a binary population.
    
    Parameters:
    -----------
    pop_size : int
        Population size
    bit_length_per_var : int
        Number of bits per variable
    num_vars : int
        Number of variables
        
    Returns:
    --------
    numpy.ndarray
        Initialized binary population
    """
    return np.random.randint(0, 2, (pop_size, num_vars * bit_length_per_var))

def evaluate_population(population, f):
    """
    Evaluate fitness of a population.
    
    Parameters:
    -----------
    population : numpy.ndarray
        Population to evaluate
    f : callable
        Objective function
        
    Returns:
    --------
    numpy.ndarray
        Fitness values
    """
    return np.array([f(individual) for individual in population])

def tournament_selection(population, fitness, tournament_size=2):
    """
    Tournament selection for parent selection.
    
    Parameters:
    -----------
    population : numpy.ndarray
        Current population
    fitness : numpy.ndarray
        Fitness values of the population
    tournament_size : int, optional
        Size of each tournament, by default 2
        
    Returns:
    --------
    list
        Indices of selected parents
    """
    selected = []
    for _ in range(len(population)):
        candidates = np.random.choice(len(population), tournament_size, replace=False)
        best_candidate = candidates[np.argmin(fitness[candidates])]
        selected.append(best_candidate)
    return selected

def run_ga(objective_function, pop_size, dim, lower_bound, upper_bound, generations,
           tournament_size=2, mutation_rate=0.1, sigma=0.1,
           crossover_type="arithmetic", crossover_rate=1.0, alpha=0.5):
    """
    Run a real-valued genetic algorithm.
    
    Parameters:
    -----------
    objective_function : callable
        Function to minimize
    pop_size : int
        Population size
    dim : int
        Problem dimensionality
    lower_bound : list or numpy.ndarray
        Lower bounds for each dimension
    upper_bound : list or numpy.ndarray
        Upper bounds for each dimension
    generations : int
        Number of generations to run
    tournament_size : int, optional
        Tournament size for selection, by default 2
    mutation_rate : float, optional
        Mutation rate, by default 0.1
    sigma : float, optional
        Standard deviation for Gaussian mutation, by default 0.1
    crossover_type : str, optional
        Type of crossover ("arithmetic" or "blx_alpha"), by default "arithmetic"
    crossover_rate : float, optional
        Probability of applying crossover, by default 1.0
    alpha : float, optional
        Parameter for crossover operators, by default 0.5
        
    Returns:
    --------
    tuple
        (best_individual, best_fitness)
    """
    
    # Initialize population
    population = setup_population(pop_size, dim, lower_bound, upper_bound)
    best_fitness = float('inf')
    best_individual = None

    for gen in range(generations):
        # Evaluate current population
        fitness = evaluate_population(population, objective_function)
        
        # Keep track of best solution
        gen_best_idx = np.argmin(fitness)
        if fitness[gen_best_idx] < best_fitness:
            best_fitness = fitness[gen_best_idx]
            best_individual = population[gen_best_idx].copy()

        # Select parents for next generation
        selected_indices = tournament_selection(population, fitness, tournament_size)
        
        # Create next generation
        new_population = []
        while len(new_population) < pop_size:
            idx1, idx2 = np.random.choice(selected_indices, 2, replace=False)
            p1, p2 = population[idx1], population[idx2]
            
            if np.random.random() < crossover_rate:
                if crossover_type == "arithmetic":
                    offspring = child(p1, p2, alpha)
                elif crossover_type == "blx_alpha":
                    offspring = blx_alpha_crossover(p1, p2, alpha, lower_bound, upper_bound)
                else:
                    raise ValueError(f"Unknown crossover type: {crossover_type}")
            else:
                offspring = p1.copy()
            
            offspring = gaussian_mutation(offspring, mutation_rate, sigma)
            
            offspring = np.clip(offspring, lower_bound, upper_bound)
            
            new_population.append(offspring)
        
        population = np.array(new_population)
    
    # Final evaluation
    final_fitness = evaluate_population(population, objective_function)
    final_best_idx = np.argmin(final_fitness)
    
    # Return the best between the final population and the best ever found
    if final_fitness[final_best_idx] < best_fitness:
        best_fitness = final_fitness[final_best_idx]
        best_individual = population[final_best_idx]
    
    print(f"Best individual: {best_individual}")
    print(f"Best fitness: {best_fitness}")
    
    return best_individual, best_fitness

def run_binary_ga(objective_function, pop_size, num_vars, bit_length_per_var, 
                 lower_bounds, upper_bounds, generations,
                 tournament_size=2, mutation_rate=0.01,
                 crossover_type="one_point", crossover_rate=0.8):
    """
    Run a genetic algorithm with binary encoding.
    
    Parameters:
    -----------
    objective_function : callable
        Function to minimize
    pop_size : int
        Population size
    num_vars : int
        Number of variables
    bit_length_per_var : int
        Number of bits per variable
    lower_bounds : list or numpy.ndarray
        Lower bounds for each variable
    upper_bounds : list or numpy.ndarray
        Upper bounds for each variable
    generations : int
        Number of generations to run
    tournament_size : int, optional
        Tournament size for selection, by default 2
    mutation_rate : float, optional
        Bit-flip mutation rate, by default 0.01
    crossover_type : str, optional
        Type of crossover ("one_point" or "two_point"), by default "one_point"
    crossover_rate : float, optional
        Probability of applying crossover, by default 0.8
        
    Returns:
    --------
    tuple
        (best_individual_decoded, best_fitness)
    """
    
    # Initialize population
    population = setup_binary_population(pop_size, bit_length_per_var, num_vars)
    best_fitness = float('inf')
    best_individual = None
    best_decoded = None

    for gen in range(generations):
        # Decode and evaluate
        fitness = []
        for ind in population:
            decoded = binary_to_real(ind, bit_length_per_var, num_vars, lower_bounds, upper_bounds)
            fitness.append(objective_function(decoded))
        fitness = np.array(fitness)
        
        # Keep track of best solution
        gen_best_idx = np.argmin(fitness)
        if fitness[gen_best_idx] < best_fitness:
            best_fitness = fitness[gen_best_idx]
            best_individual = population[gen_best_idx].copy()
            best_decoded = binary_to_real(best_individual, bit_length_per_var, num_vars, 
                                         lower_bounds, upper_bounds)

        # Select parents
        selected_indices = tournament_selection(population, fitness, tournament_size)
        
        # Create next generation
        new_population = []
        while len(new_population) < pop_size:
            # Select two parents
            idx1, idx2 = np.random.choice(selected_indices, 2, replace=False)
            p1, p2 = population[idx1], population[idx2]
            
            # Apply crossover with probability crossover_rate
            if np.random.random() < crossover_rate:
                if crossover_type == "one_point":
                    c1, _ = binary_one_point_crossover(p1, p2)
                elif crossover_type == "two_point":
                    c1, _ = binary_two_point_crossover(p1, p2)
                else:
                    raise ValueError(f"Unknown binary crossover type: {crossover_type}")
            else:
                # If no crossover, clone first parent
                c1 = p1.copy()
            
            # Apply mutation
            c1 = bit_flip_mutation(c1, mutation_rate)
            
            new_population.append(c1)
        
        # Update population
        population = np.array(new_population)
    
    # Final evaluation
    final_decoded = []
    final_fitness = []
    for ind in population:
        decoded = binary_to_real(ind, bit_length_per_var, num_vars, lower_bounds, upper_bounds)
        final_decoded.append(decoded)
        final_fitness.append(objective_function(decoded))
    
    final_best_idx = np.argmin(final_fitness)
    
    # Return the best between the final population and the best ever found
    if final_fitness[final_best_idx] < best_fitness:
        best_fitness = final_fitness[final_best_idx]
        best_decoded = final_decoded[final_best_idx]
    
    print(f"Best individual (decoded): {best_decoded}")
    print(f"Best fitness: {best_fitness}")
    
    return best_decoded, best_fitness 