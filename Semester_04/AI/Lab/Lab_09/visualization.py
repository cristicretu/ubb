"""
Visualization functions for benchmark optimization problems.
"""
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

def plot_2d_contour(f, x_range, y_range, title):
    """
    Create a 2D contour plot of a function.
    
    Parameters:
    -----------
    f : callable
        Function that takes x, y arrays and returns z values
    x_range : tuple
        (min_x, max_x) range for x-axis
    y_range : tuple
        (min_y, max_y) range for y-axis
    title : str
        Plot title
    
    Returns:
    --------
    None
        Saves the plot to a file named "{title}_contour.png"
    """
    x = np.linspace(x_range[0], x_range[1], 100)
    y = np.linspace(y_range[0], y_range[1], 100)
    X, Y = np.meshgrid(x, y)
    Z = f(X, Y)
    plt.figure(figsize=(10, 8))
    plt.contour(X, Y, Z, levels=20, cmap='viridis')
    plt.colorbar(label='f(x,y)')
    plt.title(title)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid(True)
    plt.savefig(f"{title.replace(' ', '_')}_contour.png")
    plt.close()

def plot_3d_surface(f, x_range, y_range, title):
    """
    Create a 3D surface plot of a function.
    
    Parameters:
    -----------
    f : callable
        Function that takes x, y arrays and returns z values
    x_range : tuple
        (min_x, max_x) range for x-axis
    y_range : tuple
        (min_y, max_y) range for y-axis
    title : str
        Plot title
    
    Returns:
    --------
    None
        Saves the plot to a file named "{title}_surface.png"
    """
    x = np.linspace(x_range[0], x_range[1], 100)
    y = np.linspace(y_range[0], y_range[1], 100)
    X, Y = np.meshgrid(x, y)
    Z = f(X, Y)
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(X, Y, Z, cmap='viridis', alpha=0.8)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('f(x,y)')
    ax.set_title(title)
    fig.colorbar(surf, shrink=0.5, aspect=5, label='f(x,y)')
    plt.savefig(f"{title.replace(' ', '_')}_surface.png")
    plt.close()

def plot_results_boxplot(results, functions):
    """
    Create boxplots comparing the performance of different GA configurations.
    
    Parameters:
    -----------
    results : dict
        Dictionary with configuration results
    functions : list
        List of function names
        
    Returns:
    --------
    None
        Saves the plot to a file named "ga_comparison_boxplot.png"
    """
    # Prepare data for plotting
    plot_data = []
    for config, stats in results.items():
        # Extract components from config name, handling variable formats
        parts = config.split('_')
        # The first part is always the function name
        func = parts[0]
        # The second part is always the encoding
        encoding = parts[1]
        # The rest is the crossover type (might contain underscores)
        crossover = '_'.join(parts[2:])
        
        for fitness in stats["best_fitnesses"]:
            plot_data.append({
                "Function": func,
                "Encoding": encoding,
                "Crossover": crossover,
                "Fitness": fitness
            })
    
    df = pd.DataFrame(plot_data)
    
    # Create Config column in the main dataframe before filtering
    df["Config"] = df["Encoding"] + "-" + df["Crossover"]
    
    # Plot boxplots
    plt.figure(figsize=(15, 10))
    for i, func in enumerate(functions):
        plt.subplot(1, 2, i+1)
        # Filter the dataframe after creating the Config column
        func_df = df[df["Function"] == func]
        
        sns.boxplot(x="Config", y="Fitness", data=func_df)
        plt.title(f"{func.capitalize()} Function")
        plt.xticks(rotation=45)
        plt.tight_layout()
    
    plt.savefig("ga_comparison_boxplot.png")
    plt.close() 