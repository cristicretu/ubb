from typing import List, Tuple
from itertools import combinations
import math


def find_collinear_points(
    points: List[Tuple[float, float]],
) -> List[List[Tuple[float, float]]]:
    """
    Find all subsets of collinear points from a given list of points.
    Points are collinear if they lie on the same straight line.

    Args:
        points: List of tuples where each tuple contains (x, y) coordinates

    Returns:
        List of lists, where each inner list contains a set of collinear points
    """

    def calculate_slope(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Calculate slope between two points, handling vertical lines."""
        if p1[0] == p2[0]:  # Vertical line
            return float("inf")
        return (p2[1] - p1[1]) / (p2[0] - p1[0])

    def are_collinear(points_subset: List[Tuple[float, float]]) -> bool:
        """Check if a set of points are collinear."""
        if len(points_subset) <= 2:
            return True

        # Calculate slope between first two points
        base_slope = calculate_slope(points_subset[0], points_subset[1])

        # Compare slope between first point and all other points
        for i in range(2, len(points_subset)):
            current_slope = calculate_slope(points_subset[0], points_subset[i])
            if (
                abs(current_slope - base_slope) > 1e-10
            ):  # Use small epsilon for float comparison
                return False
        return True

    collinear_groups = []
    # Check all possible combinations of 3 or more points
    for size in range(3, len(points) + 1):
        for combo in combinations(points, size):
            if are_collinear(list(combo)):
                collinear_groups.append(list(combo))

    return collinear_groups


# Example usage
if __name__ == "__main__":
    # Test with your example points
    points = [(2, 4), (3, 6), (4, 8), (0, 0), (1, 1)]
    collinear_sets = find_collinear_points(points)

    print("Found collinear point sets:")
    for i, point_set in enumerate(collinear_sets, 1):
        print(f"Set {i}: {point_set}")
