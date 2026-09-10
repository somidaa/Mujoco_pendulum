import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import matplotlib.pyplot as plt
from src.logger import Logger


def plot_free_dynamics(csv_path, out_path):
    logger = Logger()
    time, q, dq, u, *_ = logger.load_csv(csv_path)

    fig, axes = plt.subplots(2, 1, figsize=(8, 8))
    axes[0].plot(time, q)
    axes[0].set_xlabel("time (s)")
    axes[0].set_ylabel("q (rad)")
    axes[0].grid(True)

    axes[1].plot(time, dq)
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("dq (rad/s)")
    axes[1].grid(True)

    _savefig(fig, out_path)


def plot_energy(csv_path, out_path):
    logger = Logger()

    time, q, dq, u, kinetic_energy, potential_energy, total_energy = logger.load_csv(csv_path)

    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    axes[0].plot(time, kinetic_energy)
    axes[0].set_ylabel("Kinetic Energy")
    axes[0].grid(True)
    axes[1].plot(time, potential_energy)
    axes[1].set_ylabel("Potential Energy")
    axes[1].grid(True)
    axes[2].plot(time, total_energy)
    axes[2].set_xlabel("Time")
    axes[2].set_ylabel("Total Energy")
    axes[2].grid(True)

    _savefig(fig, out_path)


def _savefig(fig, save_path):
    """Save the figure if a path is given (creating the directory first)."""
    if save_path is not None:
        dir_name = os.path.dirname(save_path)
        if dir_name:
            os.makedirs(dir_name, exist_ok=True)
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
