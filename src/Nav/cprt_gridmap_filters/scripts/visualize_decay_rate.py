import math
import numpy as np
import matplotlib.pyplot as plt


def visualize_decay_with_cutoff(core_radius, decay_radius, decay_rate, start_value):
    """
    Visualizes the exponential decay of inflation cost with a cutoff at the decay radius.

    Args:
      core_radius (float): The radius within which inflation is at its start value.
      decay_radius (float): The maximum radius of inflation influence.
      decay_rate (float): The rate of exponential decay.
      start_value (float): The inflation cost within the core radius.
    """
    distances = np.linspace(0, decay_radius * 1.2, 200)
    decayed_costs = np.piecewise(
        distances,
        [
            distances <= core_radius,
            (distances > core_radius) & (distances <= decay_radius),
            distances > decay_radius,
        ],
        [
            start_value,
            lambda d: start_value * np.exp(-decay_rate * (d - core_radius)),
            0,
        ],
    )

    plt.figure(figsize=(10, 7))
    plt.plot(distances, decayed_costs)
    plt.xlabel("Distance from Obstacle Center")
    plt.ylabel("Inflation Cost")
    plt.title("Visualization of Exponential Cost Decay with Cutoff")
    plt.axvline(
        core_radius, color="r", linestyle="--", label=f"Core Radius: {core_radius:.2f}"
    )
    plt.axvline(
        decay_radius,
        color="g",
        linestyle="--",
        label=f"Decay Radius: {decay_radius:.2f}",
    )
    plt.hlines(
        0, 0, decay_radius * 1.2, colors="k", linestyles="-", linewidth=0.5
    )  # Indicate zero cost
    plt.scatter(
        core_radius,
        start_value,
        color="r",
        marker="o",
        label=f"Start Value: {start_value:.2f}",
    )
    plt.scatter(
        decay_radius, 0, color="g", marker="o", label=f"Cost at Decay Radius: 0.00"
    )  # Mark cutoff

    plt.grid(True)
    plt.legend()
    plt.ylim(bottom=-0.1)  # Extend y-axis slightly to show zero
    plt.xlim(left=0)
    plt.show()


if __name__ == "__main__":
    print("Visualizing exponential inflation decay with cutoff...")

    core_radius = float(input("Enter the core inflation radius (e.g., 0.5): "))
    decay_radius = float(input("Enter the decay inflation radius (e.g., 1.5): "))
    decay_rate = float(input("Enter the decay rate (e.g., 1.0): "))
    start_value = float(
        input("Enter the starting inflation value within the core radius (e.g., 1.0): ")
    )

    visualize_decay_with_cutoff(core_radius, decay_radius, decay_rate, start_value)
