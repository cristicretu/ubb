"""
Main script for running GA optimization experiments.

This script evaluates the performance of genetic algorithms with different 
configurations (real/binary encoding, different crossover operators) on benchmark 
optimization functions (Ackley and Bukin).
"""
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
from scipy import stats

from functions import f1, f2, ackley, bukin
from visualization import plot_2d_contour, plot_3d_surface, plot_results_boxplot
from experiment import (
    run_experiments, perform_statistical_analysis, print_results_table,
    run_ackley, run_bukin, run_binary_ackley, run_binary_bukin
)

def main():
    """
    Main function to run the complete benchmark experiment.
    
    1. Plots the benchmark functions
    2. Runs experiments with all configurations
    3. Performs statistical analysis
    4. Visualizes the results
    """
    print("Generating plots...")
    
    ackley_bounds = ([-32.768, 32.768], [-32.768, 32.768])
    plot_2d_contour(f1, ackley_bounds[0], ackley_bounds[1], "Ackley Function Contour")
    plot_3d_surface(f1, ackley_bounds[0], ackley_bounds[1], "Ackley Function Surface")
    
    bukin_bounds = ([-15, 5], [-3, 3])
    plot_2d_contour(f2, bukin_bounds[0], bukin_bounds[1], "Bukin Function Contour")
    plot_3d_surface(f2, bukin_bounds[0], bukin_bounds[1], "Bukin Function Surface")
    
    run_quick_tests = False
    if run_quick_tests:
        print("\nRunning quick tests...")
        
        print("\nAckley with real-valued encoding and arithmetic crossover:")
        run_ackley(crossover_type="arithmetic")
        
        print("\nAckley with real-valued encoding and BLX-alpha crossover:")
        run_ackley(crossover_type="blx_alpha")
        
        print("\nAckley with binary encoding and one-point crossover:")
        run_binary_ackley(crossover_type="one_point")
        
        print("\nAckley with binary encoding and two-point crossover:")
        run_binary_ackley(crossover_type="two_point")
        
        print("\nBukin with real-valued encoding and arithmetic crossover:")
        run_bukin(crossover_type="arithmetic")
        
        print("\nBukin with real-valued encoding and BLX-alpha crossover:")
        run_bukin(crossover_type="blx_alpha")
        
        print("\nBukin with binary encoding and one-point crossover:")
        run_binary_bukin(crossover_type="one_point")
        
        print("\nBukin with binary encoding and two-point crossover:")
        run_binary_bukin(crossover_type="two_point")
    
    run_full_experiments = True
    if run_full_experiments:
        print("\nRunning full experiments...")
        
        num_runs = 30  
        pop_size = 100
        generations = 100
        
        results = run_experiments(num_runs=num_runs, pop_size=pop_size, generations=generations)
        
        perform_statistical_analysis(results)
        
        print_results_table(results)
        
        functions = ["ackley", "bukin"]
        plot_results_boxplot(results, functions)
        
        print("\nExperiments completed successfully!")

if __name__ == "__main__":
    main()





