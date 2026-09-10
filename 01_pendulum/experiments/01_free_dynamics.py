"""Milestone 2 / Experiment A: free dynamics (u = 0).

Experiment 2.1 -- canonical free swing from q0 = 45 deg
    -> outputs/csv/free_dynamics.csv
    -> outputs/plots/free_dynamics.png (+ _q, _dq)

Experiment 2.2 -- initial-angle comparison 15 / 30 / 45 / 60 deg
    -> outputs/csv/free_dynamics_<angle>deg.csv
    -> outputs/plots/free_dynamics_initial_angle.png

Also measures and prints the oscillation period of each run, so you
can see that the period grows with amplitude (nonlinear pendulum).
"""
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np  # noqa: E402

from src.model import load_model, create_data  # noqa: E402
from src.simulation import run_simulation  # noqa: E402
from src.logger import DataLogger  # noqa: E402
from src.visualization import (  # noqa: E402
    plot_free_dynamics,
    plot_initial_angle_comparison,
)

OUT_CSV = PROJECT_ROOT / "outputs" / "csv"
DURATION = 10.0
ANGLES_DEG = [15.0, 30.0, 45.0, 60.0]


def measure_period(t, q):
    """Average time between upward zero crossings of q (one per period)."""
    t = np.asarray(t)
    q = np.asarray(q)
    crossings = []
    for i in range(1, len(q)):
        if q[i - 1] < 0.0 <= q[i]:          # crossing 0 moving upward
            frac = -q[i - 1] / (q[i] - q[i - 1])
            crossings.append(t[i - 1] + frac * (t[i] - t[i - 1]))
    if len(crossings) >= 2:
        return float(np.mean(np.diff(crossings)))
    return float("nan")


def main():
    model = load_model()
    data = create_data(model)
    print(f"timestep = {model.opt.timestep} s, "
          f"integrator = {model.opt.integrator}")

    # ---- experiment 2.1: canonical free swing (45 deg) ----
    q0 = np.radians(45.0)
    mujoco_reset(model, data, q0)
    logger = run_simulation(model, data, DURATION, controller=None)
    logger.save_csv(OUT_CSV / "free_dynamics.csv")
    d = logger.to_dict()
    plot_free_dynamics(d["time"], d["q"], d["dq"])
    print(f"free run (45 deg): amplitude = "
          f"{np.abs(d['q']).max():.3f} rad, "
          f"period = {measure_period(d['time'], d['q']):.3f} s")

    # ---- experiment 2.2: initial-angle comparison ----
    series = {}
    for deg in ANGLES_DEG:
        mujoco_reset(model, data, np.radians(deg))
        log = run_simulation(model, data, DURATION, controller=None)
        log.save_csv(OUT_CSV / f"free_dynamics_{deg:g}deg.csv")
        dd = log.to_dict()
        series[f"{deg:g} deg"] = (dd["time"], dd["q"])
        print(f"q0 = {deg:4.0f} deg -> period = "
              f"{measure_period(dd['time'], dd['q']):.3f} s")

    plot_initial_angle_comparison(series)
    print("\nAll free-dynamics outputs written to outputs/csv and outputs/plots.")


def mujoco_reset(model, data, q0):
    """Reset to a clean initial state with the given angle."""
    import mujoco
    mujoco.mj_resetData(model, data)
    data.qpos[0] = q0
    data.qvel[0] = 0.0
    data.ctrl[0] = 0.0
    mujoco.mj_forward(model, data)


if __name__ == "__main__":
    main()
