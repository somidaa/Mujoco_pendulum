import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import mujoco

from src.logger import Logger
from src.model import load_model, create_data
from src.simulation import run_simulation
from src.visualization import plot_energy, plot_energy_drift

XML_PATH = PROJECT_ROOT / "model" / "pendulum.xml"
OUT_PATH = PROJECT_ROOT / "outputs" / "csv" / "energy.csv"
FIG_PATH = PROJECT_ROOT / "outputs" / "plots" / "energy.png"

DT_SWEEP = [0.0005, 0.001, 0.002, 0.005, 0.01, 0.02]


def main():
    # 实验 4.1：记录 T、V、E
    model = load_model(str(XML_PATH))
    data = create_data(model)

    data.qpos[0] = np.pi / 4
    data.qvel[0] = 0.0
    data.ctrl[0] = 0.0

    mujoco.mj_forward(model, data)

    logger = Logger()

    while data.time < 10.0:

        logger.record(data)

        mujoco.mj_step(model, data)

    logger.save_csv(str(OUT_PATH))

    print(f"CSV saved to {OUT_PATH}")

    plot_energy(str(OUT_PATH), str(FIG_PATH))

    print(f"Fig saved to {FIG_PATH}")


def timestep_sweep():
    # 实验 4.2：不同 timestep 下自由摆的能量漂移
    series = {}
    print(f"{'dt (s)':>10} | {'max |E-E0| (J)':>16}")

    for dt in DT_SWEEP:
        model = load_model(str(XML_PATH))
        model.opt.timestep = dt
        data = create_data(model)
        data.qpos[0] = np.pi / 4
        data.qvel[0] = 0.0

        logger = Logger()
        run_simulation(model, data, 10.0, logger=logger)

        E = np.array(logger.total_energy)
        drift = np.abs(E - E[0])
        series[f"{dt:g}"] = (np.array(logger.time), drift)
        print(f"{dt:10.4f} | {drift.max():16.4e}")

    plot_energy_drift(series, str(PROJECT_ROOT / "outputs" / "plots" / "energy_drift.png"))


if __name__ == "__main__":
    main()
    timestep_sweep()
