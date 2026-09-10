import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import mujoco

from src.model import load_model, create_data
from src.simulation import run_simulation
from src.logger import Logger
from src.visualization import plot_free_dynamics_comparison

XML_PATH = PROJECT_ROOT / "model" / "pendulum.xml"
OUT_CSV = PROJECT_ROOT / "outputs" / "csv"
OUT_PLOTS = PROJECT_ROOT / "outputs" / "plots"

DURATION = 10.0
ANGLES_DEG = [15.0, 30.0, 45.0, 60.0]


def measure_period(t, q):
    # 向上穿越 q=0 的平均间隔 = 一个周期
    crossings = []
    for i in range(1, len(q)):
        if q[i - 1] < 0 <= q[i]:
            frac = -q[i - 1] / (q[i] - q[i - 1])
            crossings.append(t[i - 1] + frac * (t[i] - t[i - 1]))
    if len(crossings) >= 2:
        return float(np.mean(np.diff(crossings)))
    return float("nan")


def main():
    model = load_model(str(XML_PATH))
    data = create_data(model)

    series = {}
    print(f"{'q0 (deg)':>9} | {'amplitude (rad)':>15} | {'period (s)':>10}")

    for deg in ANGLES_DEG:
        mujoco.mj_resetData(model, data)
        data.qpos[0] = np.radians(deg)
        data.qvel[0] = 0.0

        logger = Logger()
        run_simulation(model, data, DURATION, logger=logger)

        logger.save_csv(str(OUT_CSV / f"free_dynamics_{deg:g}deg.csv"))
        series[f"{deg:g} deg"] = (logger.time, logger.q)

        print(f"{deg:9.0f} | {np.abs(logger.q).max():15.3f} | "
              f"{measure_period(logger.time, logger.q):10.3f}")

    plot_free_dynamics_comparison(series,
                                  str(OUT_PLOTS / "free_dynamics_initial_angle.png"))


if __name__ == "__main__":
    main()
