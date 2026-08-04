#!/usr/bin/env python3
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import csv
import sys
import math


def plot_set(ax: plt.Axes, x_deg: list[int], readings: list[int], title: str):
    x_rad = np.deg2rad(x_deg)

    # --- Model: A cos^2(x - phi) + C ---
    def cos2_model(x, A, B, phi, C):
        return A * np.cos(B * x - phi) ** 2 + C

    # --- Initial parameter guesses ---
    A_guess = np.max(readings) - np.min(readings)
    B_guess = 1
    phi_guess = np.deg2rad(50)
    C_guess = np.min(readings)

    popt, _ = curve_fit(
        cos2_model, x_rad, readings, p0=[A_guess, B_guess, phi_guess, C_guess]
    )

    A_fit, B_fit, phi_fit, C_fit = popt

    # Convert phase to degrees
    phase_deg = np.rad2deg(phi_fit)

    # --- Generate smooth fit curve ---
    x_fit_deg = np.linspace(np.min(x_deg), np.max(x_deg), 1000)
    x_fit_rad = np.deg2rad(x_fit_deg)
    y_fit = cos2_model(x_fit_rad, A_fit, B_fit, phi_fit, C_fit)

    # --- Plot ---
    ax.scatter([i * B_fit for i in x_deg], readings, label="Data")
    ax.plot([i * B_fit for i in x_fit_deg], y_fit, label="Cos² Fit")
    ax.axvline(phase_deg, linestyle="--", label=f"Phase Offset = {phase_deg:.2f}°")

    ax.set_xlabel("Angle (degrees)")
    ax.set_ylabel("Signal")
    ax.set_title(f"{title} - Cosine Squared Fit")
    ax.legend()
    return phase_deg


def plot_file(ax: plt.Axes, filename: str, title: str):
    x_deg = []
    readings = []
    with open(filename, newline="") as file:
        reader = csv.reader(file)
        next(reader, None)
        for row in reader:
            x_deg.append(int(row[0]))
            readings.append(int(row[1]))
    return plot_set(ax, x_deg, readings, title)


fig, ax = plt.subplots(1, 2, figsize=(12, 5))

conc = float(input("Enter concentration (g/mL): "))

phase1 = plot_file(ax[0], sys.argv[1], sys.argv[1].split("_")[1])
phase2 = plot_file(ax[1], sys.argv[2], sys.argv[2].split("_")[1])

props = dict(boxstyle="round", facecolor="grey", alpha=0.15)
ax[0].text(
    0,
    -0.2,
    f"Phase difference: {math.fabs(phase1 - phase2):.2f} deg\nConcentration: {conc:.2f} g/mL\nSpecific rotation: {math.fabs(phase1 - phase2) * 100 / (43.3 * conc):.4f} deg*cm^2/g",
    transform=ax[0].transAxes,
    verticalalignment="top",
    bbox=props,
)
plt.tight_layout()
plt.savefig(f"{sys.argv[3]}.png")

plt.show()
