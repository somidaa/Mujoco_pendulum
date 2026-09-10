"""Milestone 4: mechanical energy and numerical integration error.

Experiment 4.1 -- record T, V, E during a free swing
    -> outputs/csv/energy_analysis.csv
    -> outputs/plots/energy.png

Experiment 4.2 -- energy drift |E(t) - E(0)| for several timesteps
    dt in [0.0005, 0.001, 0.002, 0.005, 0.01, 0.02] s
    -> outputs/plots/energy_drift.png

Also cross-checks our T / V formulas against the engine values in
``data.energy`` (they must agree to machine precision).
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
from src.energy import compute_energies, energy_drift  # noqa: E402
from src.visualization import plot_energy, plot_energy_drift  # noqa: E402

OUT_CSV = PROJECT_ROOT / "outputs" / "csv"
Q0 = np.radians(45.0)
DURATION = 20.0
DT_SWEEP = [0.0005, 0.001, 0.002, 0.005, 0.01, 0.02]


def main():
    model = load_model()
    data = create_data(model)

    # ---- experiment 4.1: T, V, E during a free swing ----
    mujoco.mj_resetData(model, data)
    data.qpos[0] = Q0
    data.qvel[0] = 0.0
    mujoco.mj_forward(model, data)

    # initial energy (kinematics already valid after mj_forward)
    t0_energies = compute_energies(model, data)
    logger = run_simulation(model, data, DURATION, controller=None,
                            record_energy=True)
    logger.save_csv(OUT_CSV / "energy_analysis.csv")
    d = logger.to_dict()

    drift_abs, _ = energy_drift(d["total_energy"])
    print(f"max |E(t)-E(0)| @ dt={model.opt.timestep}: {drift_abs.max():.3e} J "
          f"(E0 = {d['total_energy'][0]:.4f} J)")
    plot_energy(d["time"], d["kinetic_energy"], d["potential_energy"],
                d["total_energy"], e0=d["total_energy"][0])

    # ---- experiment 4.2: timestep sweep ----
    drift_series = {}
    print("\ntimestep sweep (RK4, 20 s free run):")
    print(f"  {'dt (s)':>10} | {'final drift (J)':>16} | "
          f"{'max drift (J)':>14} | {'E0 (J)':>10}")
    for dt in DT_SWEEP:
        m = load_model()
        m.opt.timestep = dt
        data = create_data(m)
        mujoco.mj_resetData(m, data)
        data.qpos[0] = Q0
        data.qvel[0] = 0.0
        mujoco.mj_forward(m, data)

        log = run_simulation(m, data, DURATION, controller=None,
                             record_energy=True)
        dd = log.to_dict()
        abs_drift, _ = energy_drift(dd["total_energy"])
        drift_series[f"{dt:g}"] = (dd["time"], abs_drift)
        print(f"  {dt:10.4f} | {abs_drift[-1]:16.4e} | "
              f"{abs_drift.max():14.4e} | {dd['total_energy'][0]:10.4f}")

    plot_energy_drift(drift_series)
    print("\nEnergy outputs written to outputs/csv and outputs/plots.")


if __name__ == "__main__":
    main()
