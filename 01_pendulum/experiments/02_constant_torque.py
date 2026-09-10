"""Milestone 5 / Experiment B: constant torque.

Apply a constant torque u = const and watch the pendulum settle at a
new equilibrium angle q* where the gravity torque balances u:

    m g Lc sin(q*) = u   =>   q* = asin(u / (m g Lc))

The predicted q* is compared with the measured steady-state angle.
"""
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np  # noqa: E402
import mujoco  # noqa: E402

from src.model import load_model, create_data  # noqa: E402
from src.simulation import run_simulation  # noqa: E402
from src.visualization import plot_constant_torque  # noqa: E402

OUT_CSV = PROJECT_ROOT / "outputs" / "csv"
DURATION = 15.0
TORQUES = [0.5, 1.0, 2.0]
CANONICAL = 1.0


def effective_lever(model):
    """Distance from the hinge to the pendulum COM (m)."""
    m = float(np.sum(model.body_mass[1:]))          # exclude worldbody
    com = np.zeros(3)
    for i in range(1, model.nbody):
        com += model.body_mass[i] * model.body_ipos[i]
    return float(np.linalg.norm(com) / m), m


def predicted_equilibrium(u, m, g, lever):
    if u >= m * g * lever:
        return np.pi / 2
    return float(np.arcsin(u / (m * g * lever)))


def main():
    model = load_model()
    data = create_data(model)
    g = -model.opt.gravity[2]
    lever, mass = effective_lever(model)
    print(f"pendulum mass = {mass:.3f} kg, COM distance = {lever:.3f} m")

    series = {}
    for u in TORQUES:
        mujoco.mj_resetData(model, data)
        data.qpos[0] = 0.0
        data.qvel[0] = 0.0
        mujoco.mj_forward(model, data)

        logger = run_simulation(model, data, DURATION,
                                controller=lambda q, dq, t, u=u: u)
        logger.save_csv(OUT_CSV / f"constant_torque_{u:g}.csv")
        d = logger.to_dict()
        series[f"{u:g}"] = (d["time"], d["q"])

        q_meas = float(np.mean(d["q"][-int(2.0 / model.opt.timestep):]))
        q_pred = predicted_equilibrium(u, mass, g, lever)
        print(f"u = {u:4.1f} N*m -> q* measured = {q_meas:6.3f} rad, "
              f"predicted = {q_pred:6.3f} rad")

        if abs(u - CANONICAL) < 1e-12:
            logger.save_csv(OUT_CSV / "constant_torque.csv")

    plot_constant_torque(series)
    print("\nConstant-torque outputs written to outputs/csv and outputs/plots.")


if __name__ == "__main__":
    main()
