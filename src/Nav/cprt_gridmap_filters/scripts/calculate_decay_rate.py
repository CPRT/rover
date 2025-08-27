import math


def calculate_decay_rate(core_radius, decay_radius, inflation_value, target_cost):
    """
    Calculates the decay rate for exponential inflation falloff.

    Args:
      core_radius (float): The radius within which inflation is at its maximum.
      decay_radius (float): The maximum radius of inflation influence.
      inflation_value (float): The inflation cost at the edge of the core radius.
      target_cost (float): The desired inflation cost at the decay radius.

    Returns:
      float: The calculated decay rate, or None if inputs are invalid.
    """
    if (
        decay_radius <= core_radius
        or target_cost <= 0
        or target_cost >= inflation_value
    ):
        print("Invalid input parameters for decay rate calculation.")
        return None
    else:
        decay_rate = math.log(inflation_value / target_cost) / (
            decay_radius - core_radius
        )
        return decay_rate


if __name__ == "__main__":
    print("Calculating decay rate for exponential inflation...")

    core_radius = float(input("Enter the core inflation radius (e.g., 0.5): "))
    decay_radius = float(input("Enter the decay inflation radius (e.g., 1.5): "))
    inflation_value = float(
        input("Enter the inflation value at the core radius (e.g., 1.0): ")
    )
    target_cost = float(
        input("Enter the desired target cost at the decay radius (e.g., 0.1): ")
    )

    decay_rate = calculate_decay_rate(
        core_radius, decay_radius, inflation_value, target_cost
    )

    if decay_rate is not None:
        print(f"\nCalculated decay rate: {decay_rate:.4f}")
        print("\nYou can use this value for your 'decayRate_' parameter.")

        # --- Optional: Visualize the decay ---
        import numpy as np
        import matplotlib.pyplot as plt

        distances = np.linspace(0, decay_radius, 100)
        decayed_costs = np.piecewise(
            distances,
            [distances <= core_radius, distances > core_radius],
            [
                inflation_value,
                lambda d: inflation_value * np.exp(-decay_rate * (d - core_radius)),
            ],
        )

        plt.figure(figsize=(8, 6))
        plt.plot(distances, decayed_costs)
        plt.xlabel("Distance from Obstacle Center")
        plt.ylabel("Inflation Cost")
        plt.title("Exponential Cost Decay with Flat Core Cost")
        plt.axvline(
            core_radius,
            color="r",
            linestyle="--",
            label=f"Core Radius: {core_radius:.2f}",
        )
        plt.axvline(
            decay_radius,
            color="g",
            linestyle="--",
            label=f"Decay Radius: {decay_radius:.2f}",
        )
        plt.scatter(
            decay_radius,
            target_cost,
            color="g",
            marker="o",
            label=f"Target Cost: {target_cost:.2f}",
        )
        plt.scatter(
            core_radius,
            inflation_value,
            color="r",
            marker="o",
            label=f"Initial Cost: {inflation_value:.2f}",
        )
        plt.grid(True)
        plt.legend()
        plt.show()
