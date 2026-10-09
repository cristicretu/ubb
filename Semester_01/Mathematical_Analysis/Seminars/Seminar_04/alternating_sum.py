"""

Calculate numerically the sum of the alternating harmonic series.
And show how rearranging the terms can give a different sum.

Plots the spline interpolation of the sum of the original alternating harmonic series.

Author: Cretu Cristian, 913

"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
import termtables as tt


def original_sum(length_of_terms: int) -> np.float64:
    """
    Calculates the sum of the original alternating harmonic series up to a given number of terms.

    :param length_of_terms: The number of terms to sum up to.

    :return: The sum of the series up to the given number of terms.
    """
    n = np.arange(1, length_of_terms + 1)
    series = (-1) ** (n + 1) / n
    return np.sum(series)


def rearranged_sum(p: int, q: int, length_of_terms: int) -> np.float64:
    """
    Calculates the sum of the rearranged alternating harmonic series up to a given number of terms.

    :param p: The number of positive terms in a row.
    :param q: The number of negative terms in a row.
    :param length_of_terms: The number of terms to sum up to.

    :return: The sum of the series up to the given number of terms.
    """
    terms = []
    current_term = 1

    while current_term <= length_of_terms:
        # Positive terms
        end_positive = min(current_term + p - 1, length_of_terms)
        terms.append(1 / np.arange(current_term, end_positive + 1))
        current_term += p

        if current_term > length_of_terms:
            break

        # Negative terms
        end_negative = min(current_term + q - 1, length_of_terms)
        terms.append(-1 / np.arange(current_term, end_negative + 1))
        current_term += q

    return np.concatenate(terms)


def plot_sum(length_of_terms: int, function_to_plot, title: str, **kwargs):
    """
    Plots the sum of a given function up to a given number of terms.

    :param length_of_terms: The number of terms to sum up to.
    :param function_to_plot: The function to plot the sum of.
    :param title: The title of the plot.
    :param kwargs: Additional keyword arguments for the function_to_plot.

    :return: None
    """
    x_values = np.arange(1, length_of_terms + 1)
    if function_to_plot == original_sum:
            y_values = np.cumsum((-1) ** (x_values + 1) / x_values)
    else:
        sum_values = function_to_plot(**kwargs, length_of_terms=length_of_terms)
        y_values = np.cumsum(sum_values)


    # Spline interpolation
    X_Y_Spline = make_interp_spline(x_values, y_values)
    X_ = np.linspace(x_values.min(), x_values.max(), 10000)
    Y_ = X_Y_Spline(X_)

    plt.figure(figsize=(10, 6))
    plt.plot(X_, Y_, label=f'n up to {length_of_terms}')

    plt.xscale('log')
    # plt.yscale('log')

    plt.xlabel('n')
    plt.ylabel('Sum Value')
    plt.title(title)
    plt.grid(True, which="both", ls="--", c='0.65')
    plt.legend()

    # Save to file
    filename = title.replace(" ", "_") + f'_{length_of_terms}.png'
    # Add p and q to filename if it's a rearranged sum
    if function_to_plot == rearranged_sum:
        filename = filename.replace(".png", f"_p{kwargs['p']}_q{kwargs['q']}.png")
    plt.savefig(filename)
    plt.show()

if __name__ == "__main__":
    lengths = [10000, 20000, 50000]
    p1, q1 = 3, 2
    p2, q2 = 4, 7

    # Calculate and print values
    header = ["Type", f"Value for n={lengths[0]}", f"Value for n={lengths[1]}", f"Value for n={lengths[2]}"]
    rows = [
        ["ln(2)", np.log(2), np.log(2), np.log(2)],
        ["Original sum", original_sum(lengths[0]), original_sum(lengths[1]), original_sum(lengths[2])],
        [f"Rearranged (p={p1}, q={q1})", np.sum(rearranged_sum(p1, q1, lengths[0])), np.sum(rearranged_sum(p1, q1, lengths[1])), np.sum(rearranged_sum(p1, q1, lengths[2]))],
        [f"Rearranged (p={p2}, q={q2})", np.sum(rearranged_sum(p2, q2, lengths[0])), np.sum(rearranged_sum(p2, q2, lengths[1])), np.sum(rearranged_sum(p2, q2, lengths[2]))]
    ]
    print(tt.to_string(rows, header=header))

    # Plot for each length for original sum
    for length in lengths:
        plot_sum(length, original_sum, "Smooth Curve for Original Sum")

    # Plot for each length for rearranged sum
    for length in lengths:
        plot_sum(length, rearranged_sum, f"Smooth Curve for Rearranged Sum; p = {p1} q = {q1}", p=p1, q=q1)

    for length in lengths:
        plot_sum(length, rearranged_sum, f"Smooth Curve for Rearranged Sum; p = {p2} q = {q2}", p=p2, q=q2)
