import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import stats

from functions import ackley, bukin
from visualization import plot_results_boxplot

def run_ackley(pop_size=100, generations=100, crossover_type="arithmetic", 
              crossover_rate=1.0, mutation_rate=0.1, tournament_size=2, alpha=0.5):
    """
    Run Ackley optimization with real-valued encoding.
    
    Parameters:
    -----------
    pop_size : int, optional
        Population size, by default 100
    generations : int, optional
        Number of generations, by default 100
    crossover_type : str, optional
        Type of crossover ("arithmetic" or "blx_alpha"), by default "arithmetic"
    crossover_rate : float, optional
        Probability of applying crossover, by default 1.0
    mutation_rate : float, optional
        Mutation rate, by default 0.1
    tournament_size : int, optional
        Tournament size for selection, by default 2
    alpha : float, optional
        Parameter for crossover operators, by default 0.5
        
    Returns:
    --------
    tuple
        (best_individual, best_fitness)
    """
    from ga import run_ga
    
    dim = 2
    lower_bound = [-32.768, -32.768]
    upper_bound = [32.768, 32.768]
    
    return run_ga(
        objective_function=ackley,
        pop_size=pop_size,
        dim=dim,
        lower_bound=lower_bound,
        upper_bound=upper_bound,
        generations=generations,
        tournament_size=tournament_size,
        mutation_rate=mutation_rate,
        crossover_type=crossover_type,
        crossover_rate=crossover_rate,
        alpha=alpha
    )

def run_bukin(pop_size=100, generations=100, crossover_type="arithmetic", 
             crossover_rate=1.0, mutation_rate=0.1, tournament_size=2, alpha=0.5):
    """
    Run Bukin optimization with real-valued encoding.
    
    Parameters:
    -----------
    pop_size : int, optional
        Population size, by default 100
    generations : int, optional
        Number of generations, by default 100
    crossover_type : str, optional
        Type of crossover ("arithmetic" or "blx_alpha"), by default "arithmetic"
    crossover_rate : float, optional
        Probability of applying crossover, by default 1.0
    mutation_rate : float, optional
        Mutation rate, by default 0.1
    tournament_size : int, optional
        Tournament size for selection, by default 2
    alpha : float, optional
        Parameter for crossover operators, by default 0.5
        
    Returns:
    --------
    tuple
        (best_individual, best_fitness)
    """
    from ga import run_ga
    
    dim = 2
    lower_bound = [-15, -3]
    upper_bound = [5, 3]
    
    return run_ga(
        objective_function=bukin,
        pop_size=pop_size,
        dim=dim,
        lower_bound=lower_bound,
        upper_bound=upper_bound,
        generations=generations,
        tournament_size=tournament_size,
        mutation_rate=mutation_rate,
        crossover_type=crossover_type,
        crossover_rate=crossover_rate,
        alpha=alpha
    )

def run_binary_ackley(pop_size=100, generations=100, bit_length_per_var=16, 
                    crossover_type="one_point", crossover_rate=0.8, mutation_rate=0.01, 
                    tournament_size=2):
    """
    Run Ackley optimization with binary encoding.
    
    Parameters:
    -----------
    pop_size : int, optional
        Population size, by default 100
    generations : int, optional
        Number of generations, by default 100
    bit_length_per_var : int, optional
        Number of bits per variable, by default 16
    crossover_type : str, optional
        Type of crossover ("one_point" or "two_point"), by default "one_point"
    crossover_rate : float, optional
        Probability of applying crossover, by default 0.8
    mutation_rate : float, optional
        Bit-flip mutation rate, by default 0.01
    tournament_size : int, optional
        Tournament size for selection, by default 2
        
    Returns:
    --------
    tuple
        (best_individual_decoded, best_fitness)
    """
    from ga import run_binary_ga
    
    num_vars = 2
    lower_bounds = [-32.768, -32.768]
    upper_bounds = [32.768, 32.768]
    
    return run_binary_ga(
        objective_function=ackley,
        pop_size=pop_size,
        num_vars=num_vars,
        bit_length_per_var=bit_length_per_var,
        lower_bounds=lower_bounds,
        upper_bounds=upper_bounds,
        generations=generations,
        tournament_size=tournament_size,
        mutation_rate=mutation_rate,
        crossover_type=crossover_type,
        crossover_rate=crossover_rate
    )

def run_binary_bukin(pop_size=100, generations=100, bit_length_per_var=16, 
                   crossover_type="one_point", crossover_rate=0.8, mutation_rate=0.01, 
                   tournament_size=2):
    """
    Run Bukin optimization with binary encoding.
    
    Parameters:
    -----------
    pop_size : int, optional
        Population size, by default 100
    generations : int, optional
        Number of generations, by default 100
    bit_length_per_var : int, optional
        Number of bits per variable, by default 16
    crossover_type : str, optional
        Type of crossover ("one_point" or "two_point"), by default "one_point"
    crossover_rate : float, optional
        Probability of applying crossover, by default 0.8
    mutation_rate : float, optional
        Bit-flip mutation rate, by default 0.01
    tournament_size : int, optional
        Tournament size for selection, by default 2
        
    Returns:
    --------
    tuple
        (best_individual_decoded, best_fitness)
    """
    from ga import run_binary_ga
    
    num_vars = 2
    lower_bounds = [-15, -3]
    upper_bounds = [5, 3]
    
    return run_binary_ga(
        objective_function=bukin,
        pop_size=pop_size,
        num_vars=num_vars,
        bit_length_per_var=bit_length_per_var,
        lower_bounds=lower_bounds,
        upper_bounds=upper_bounds,
        generations=generations,
        tournament_size=tournament_size,
        mutation_rate=mutation_rate,
        crossover_type=crossover_type,
        crossover_rate=crossover_rate
    )

def run_experiments(num_runs=30, pop_size=100, generations=100):
    """
    Run experiments with all combinations of functions, encodings, and crossover types.
    
    Parameters:
    -----------
    num_runs : int, optional
        Number of independent runs per configuration, by default 30
    pop_size : int, optional
        Population size, by default 100
    generations : int, optional
        Number of generations, by default 100
        
    Returns:
    --------
    dict
        Results dictionary with statistics for each configuration
    """
    functions = ["ackley", "bukin"]
    encodings = ["real", "binary"]
    real_crossovers = ["arithmetic", "blx_alpha"]
    binary_crossovers = ["one_point", "two_point"]
    
    results = {}
    
    for func_name in functions:
        for encoding in encodings:
            if encoding == "real":
                crossovers = real_crossovers
            else:
                crossovers = binary_crossovers
                
            for crossover in crossovers:
                config_name = f"{func_name}_{encoding}_{crossover}"
                print(f"\n\nRunning {config_name} for {num_runs} runs...")
                
                best_fitnesses = []
                
                for run in range(num_runs):
                    print(f"Run {run+1}/{num_runs}")
                    
                    if func_name == "ackley":
                        if encoding == "real":
                            _, fitness = run_ackley(
                                pop_size=pop_size,
                                generations=generations,
                                crossover_type=crossover,
                                crossover_rate=0.8,
                                mutation_rate=0.1 if crossover == "arithmetic" else 0.01
                            )
                        else:  # binary
                            _, fitness = run_binary_ackley(
                                pop_size=pop_size,
                                generations=generations,
                                crossover_type=crossover,
                                crossover_rate=0.8,
                                mutation_rate=0.01
                            )
                    else:  # bukin
                        if encoding == "real":
                            _, fitness = run_bukin(
                                pop_size=pop_size,
                                generations=generations,
                                crossover_type=crossover,
                                crossover_rate=0.8,
                                mutation_rate=0.1 if crossover == "arithmetic" else 0.01
                            )
                        else:  # binary
                            _, fitness = run_binary_bukin(
                                pop_size=pop_size,
                                generations=generations,
                                crossover_type=crossover,
                                crossover_rate=0.8,
                                mutation_rate=0.01
                            )
                    
                    best_fitnesses.append(fitness)
                
                mean_fitness = np.mean(best_fitnesses)
                std_fitness = np.std(best_fitnesses)
                min_fitness = np.min(best_fitnesses)
                
                results[config_name] = {
                    "best_fitnesses": best_fitnesses,
                    "mean": mean_fitness,
                    "std": std_fitness,
                    "min": min_fitness
                }
                
                print(f"Results for {config_name}:")
                print(f"Best: {min_fitness}")
                print(f"Mean: {mean_fitness}")
                print(f"Std: {std_fitness}")
    
    return results

def perform_statistical_analysis(results):
    """
    Perform statistical analysis on experiment results.
    
    Parameters:
    -----------
    results : dict
        Results dictionary with statistics for each configuration
        
    Returns:
    --------
    None
        Prints analysis results
    """
    functions = ["ackley", "bukin"]
    
    print("\n\nStatistical Analysis")
    print("===================")
    
    for func_name in functions:
        print(f"\nAnalysis for {func_name} function:")
        
        configs = [k for k in results.keys() if k.startswith(func_name)]
        
        data = [results[config]["best_fitnesses"] for config in configs]
        
        f_val, p_val = stats.f_oneway(*data)
        
        print(f"One-way ANOVA: F={f_val:.4f}, p={p_val:.6f}")
        if p_val < 0.05:
            print("There are statistically significant differences between configurations.")
            
            # Perform pairwise tests
            print("\nPairwise comparisons:")
            for i, config1 in enumerate(configs):
                for j, config2 in enumerate(configs):
                    if i < j:  # Avoid duplicate comparisons
                        # Get the data for both configurations
                        data1 = results[config1]["best_fitnesses"]
                        data2 = results[config2]["best_fitnesses"]
                        
                        mean_diff = abs(np.mean(data1) - np.mean(data2))
                        mean_avg = (np.mean(data1) + np.mean(data2)) / 2
                        relative_diff = mean_diff / mean_avg if mean_avg != 0 else mean_diff
                        
                        # Perform t-test
                        try:
                            t_val, t_pval = stats.ttest_ind(data1, data2)
                            print(f"{config1} vs {config2}: t-test t={t_val:.4f}, p={t_pval:.6f}")
                        except Exception as e:
                            print(f"{config1} vs {config2}: t-test failed: {e}")
                        
                        # Also perform Wilcoxon rank-sum test (non-parametric, more robust when data is similar)
                        try:
                            w_val, w_pval = stats.mannwhitneyu(data1, data2)
                            print(f"{config1} vs {config2}: Wilcoxon rank-sum W={w_val:.4f}, p={w_pval:.6f}")
                        except Exception as e:
                            print(f"{config1} vs {config2}: Wilcoxon test failed: {e}")
                        
                        if relative_diff < 0.01:  
                            print(f"  Note: These configurations have very similar results (rel. diff: {relative_diff:.6f}).")
                            print(f"  Non-parametric tests may be more reliable in this case.")
                        
                        print("")  
        else:
            print("No statistically significant differences between configurations.")
            
            print("\nDetailed pairwise comparisons (for information only):")
            for i, config1 in enumerate(configs):
                for j, config2 in enumerate(configs):
                    if i < j:  
                        data1 = results[config1]["best_fitnesses"]
                        data2 = results[config2]["best_fitnesses"]
                        
                        mean_diff = abs(np.mean(data1) - np.mean(data2))
                        mean_avg = (np.mean(data1) + np.mean(data2)) / 2
                        relative_diff = mean_diff / mean_avg if mean_avg != 0 else mean_diff
                        
                        if relative_diff > 0.1:  # 10% difference might be practically significant
                            try:
                                # Use Wilcoxon for possibly more robust results
                                w_val, w_pval = stats.mannwhitneyu(data1, data2)
                                print(f"{config1} vs {config2}: Wilcoxon W={w_val:.4f}, p={w_pval:.6f}")
                                print(f"  Relative difference: {relative_diff:.2%}")
                                print("")
                            except Exception:
                                pass

def print_results_table(results):
    """
    Print a formatted table of experiment results.
    
    Parameters:
    -----------
    results : dict
        Results dictionary with statistics for each configuration
        
    Returns:
    --------
    None
        Prints results table
    """
    print("\n\nSummary of Results")
    print("=================")
    print(f"{'Configuration':<30} {'Best':<10} {'Mean':<10} {'Std':<10}")
    print("-" * 60)
    
    for config, stats in results.items():
        print(f"{config:<30} {stats['min']:<10.6f} {stats['mean']:<10.6f} {stats['std']:<10.6f}") 